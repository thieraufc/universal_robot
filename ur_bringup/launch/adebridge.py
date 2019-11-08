#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import json
import yaml
import time
from math import pi, sqrt, radians, atan
from std_msgs.msg import String
from ur_msgs.srv import PoseGoalRequest, JointStateRequest
from moveit_commander.conversions import pose_to_list

ACCEPTABLE_DISTANCE = 0.35 # The threshold for a path's closeness to the center of the base where we will retract instead of going there directly
GROUP_NAME = None
GROUP = None
SWIVEL_MODE = False

# Set up subscribers/publishers
def main():
    global GROUP
    global GROUP_NAME

    GROUP_NAME = 'manipulator'
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ade_bridge', anonymous=True)
    #time.sleep(20) # wait for move group to come up with this hack :(
    GROUP = moveit_commander.MoveGroupCommander(GROUP_NAME)

    add_obstacles()

    rospy.Subscriber("/adebridge_goal_pose", geometry_msgs.msg.Pose, pose_goal_callback)

    # Hand in a pose/joint state, and let this code figure out how to make that happen
    rospy.Subscriber("/adebridge_goal_joint_state", sensor_msgs.msg.JointState, joint_goal_callback)
    rospy.Subscriber("/adebridge_group_name", String, move_group_callback)

    # The above, but as a service (providing blocking functionality with success/fail return state)
    rospy.Service("/adebridge_goal_pose_service", PoseGoalRequest, move_group_service_callback)
    rospy.Service("/adebridge_goal_joint_state_service", JointStateRequest, joint_goal_service_callback)

    publisher = rospy.Publisher('/adebridge_current_pose', geometry_msgs.msg.Pose, queue_size=3)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        publisher.publish(GROUP.get_current_pose().pose)
        rate.sleep()


def add_obstacles():
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2) # Hack via https://answers.ros.org/question/209030/moveit-planningsceneinterface-addbox-not-showing-in-rviz/
    table_pose = geometry_msgs.msg.PoseStamped()
    pillar_pose = geometry_msgs.msg.PoseStamped()
    wall_pose = geometry_msgs.msg.PoseStamped()
    effector_pose = geometry_msgs.msg.PoseStamped()
    camera_pose = geometry_msgs.msg.PoseStamped()

    robot = moveit_commander.RobotCommander()
    table_pose.header.frame_id  = robot.get_planning_frame(); table_pose.pose.position.x =   0;   table_pose.pose.position.y  = 0;   table_pose.pose.position.z = 0;
    pillar_pose.header.frame_id = robot.get_planning_frame(); pillar_pose.pose.position.x = .68;  pillar_pose.pose.position.y = .9;  pillar_pose.pose.position.z = 0;
    wall_pose.header.frame_id   = robot.get_planning_frame(); wall_pose.pose.position.x =   -.72; wall_pose.pose.position.y   = .07; wall_pose.pose.position.z = 0;

    camera_pose.header.frame_id = "ee_link"
    camera_pose.pose.orientation.w = 1
    camera_pose.pose.position.z = .07
    scene.add_box("camera", camera_pose, size=(0.05, 0.18, 0.05))

    effector_pose.header.frame_id = "ee_link"
    effector_pose.pose.orientation.w = 1
    effector_pose.pose.position.x = .1
    scene.add_box("effector", effector_pose, size=(.155, .18, .08))

    rospy.sleep(2) # Same hack as above
    scene.attach_box("ee_link", "effector")
    scene.attach_box("ee_link", "camera")

    table_pose.pose.orientation.w = 1; pillar_pose.pose.orientation.w = 1; wall_pose.pose.orientation.w = 1;

    scene.add_box("table", table_pose, (2, 2, 0))
#    scene.add_box("pillar", pillar_pose, (.6, .6, 2))
#    scene.add_box("wall", wall_pose, (.2, 2, 2))


def retract():
    global GROUP
    joint_goal = GROUP.get_current_joint_values()

    # Get the shoulder/elbow in pretty much the right area
    joint_goal[1] = -radians(90)
    joint_goal[2] = -radians(90)
    GROUP.go(joint_goal, wait=True)
    GROUP.stop()
    time.sleep(2)

    # Get the wrist in pretty much the right area
    joint_goal[3] =  radians(0)
    joint_goal[4] =  radians(90)
    joint_goal[5] =  radians(0)
    GROUP.go(joint_goal, wait=True)
    GROUP.stop()
    time.sleep(2)

    # Get to where we actually want to be
    joint_goal[1] = -radians(75)
    joint_goal[2] = -radians(150)
    joint_goal[3] =  radians(0)
    joint_goal[4] =  radians(90)
    joint_goal[5] =  radians(0)
    GROUP.go(joint_goal, wait=True)
    GROUP.stop()


def unretract():
    global GROUP
    joint_goal = GROUP.get_current_joint_values()
    joint_goal[3] =  radians(0)
    joint_goal[4] =  radians(90)
    joint_goal[5] =  radians(0)
    GROUP.go(joint_goal, wait=True)
    GROUP.stop()
    time.sleep(2)


def in_coordinate_range(check, a, b):
    xsafe = min(a[0], b[0]) <= check[0] <= max(a[0], b[0])
    ysafe = min(a[1], b[1]) <= check[1] <= max(a[1], b[1])
    return xsafe and ysafe


def swivel_to_point(goal):
    global GROUP
    joint_goal = GROUP.get_current_joint_values()
    if goal.x is not 0:
        goal_degree = atan(goal.y/goal.x) # Calculate the angle to achieve to be pointing at the goal
    else:
        goal_degree = pi/2 # Avoid divide by 0 error: if x is zero we're at 90 degrees

    if goal.x < 0:
            # If goal.x is negative, we're off by pi
            goal_degree += pi
    goal_degree += pi # We're also off by pi because 0 degrees on the UR5 is on the negative x axis, but tan assumes it's the positive x axis.
    joint_goal[0] = goal_degree
    rospy.loginfo("\ngoal.x         | %f\ngoal.y         | %f\ngoal_degree    | %f" % (goal.x, goal.y, goal_degree))
    GROUP.go(joint_goal)
    time.sleep(2)


def arm_needs_to_swivel(goal):
    # We treat the direct path as a slope and try to find how close it gets to the origin.
    # The closest point at which this occurs will be at 90deg from the slope.
    # The distance between this generated point and the origin is the closest distance
    # we get to the origin. If this value is less than a constant value, we are getting
    # too close and will return True.
    # Note that we're only looking at this on the x/y: this imaginary safety zone is an
    # infinite height, so don't bother dealing with z.
    
    global ACCEPTABLE_DISTANCE
    global GROUP
    position = GROUP.get_current_pose().pose.position # Get the current position

    slope = (position.y-goal.y)/(position.x-goal.x) # Get linear trajectory as a slope
    normalized_slope = 1/slope # Find the slope that is 90deg off of this slope

    point_x = None
    point_y = None
    try:
        point_x = ((slope*position.x)+position.y)/(normalized_slope-slope) # Solve for x-component intersection of slope and normalized slope from origin
        point_y = -(normalized_slope*(position.y-slope*position.x))/(slope-normalized_slope) # As above, for y

    except OverflowError as err:
        # We're at basically an infinite distance away. That's big enough to not be within the range, probably.
        return False

    if point_x is not None and point_y is not None:
        # This is a real point-- let's find how close it is
        distance = sqrt(point_x**2 + point_y**2)
        if distance < ACCEPTABLE_DISTANCE:
            # Ok, so the point is too close... However, these lines are infinite and our path is not.
            # If the point isn't actually on our line, we don't need to care. Return the result of 
            # whether or not we're in that point.
            return in_coordinate_range((point_x, point_y), (position.x, position.y), (goal.x, goal.y))
        
        #If we're here, the point isn't too close: return True, we're safe.
        return True


def move_group_service_callback(data):
    return move_group_callback(data.pose)


def joint_goal_service_callback(data):
    return joint_goal_callback(data.joint_state)


def move_group_callback(data):
    global GROUP
    global GROUP_NAME

    if GROUP is None or GROUP_NAME is not data.data:
        rospy.loginfo("GROUP is being updated by callback")
        GROUP_NAME = data.data
        GROUP = moveit_commander.MoveGroupCommander(GROUP_NAME)
        time.sleep(1) # Wait for everything else to catch up


def pose_goal_callback(data):
    global GROUP
    global SWIVEL_MODE

    pose_goal = geometry_msgs.msg.Pose()
    rospy.loginfo("data recieved: "+json.dumps(yaml.load(str(data))))
    pose_goal.orientation = data.orientation
    pose_goal.position = data.position

    if SWIVEL_MODE and arm_needs_to_swivel(pose_goal.position):
        retract()
        rospy.loginfo("Retracted")
        time.sleep(2)
        swivel_to_point(pose_goal.position)
        rospy.loginfo("Swiveled")
        unretract()
        rospy.loginfo("Unretracted")

    GROUP.set_pose_target(pose_goal)
    
    plan = GROUP.go(wait=True)
    if plan is False:
        rospy.loginfo("Planning failed")
        return False

    GROUP.stop()
    GROUP.clear_pose_targets()
    return True


def joint_goal_callback(data):
    global GROUP
    if SWIVEL_MODE:
        retract()

    joint_goal = GROUP.get_current_joint_values()
    for i in range(0, len(data.position)):
        joint_goal[i] = data.position[i]

    plan = GROUP.go(joint_goal, wait=True)
    if plan is False:
        rospy.loginfo("Planning failed")
        return False

    GROUP.stop()
    time.sleep(2)
    return True


if __name__ == '__main__':
    main()
