#!/usr/bin/env python

def main():

    group = moveit_commander.MoveGroupCommander('manipulator')
    
    print("Go to the starting point.")
    raw_input("When ready to save this as the starting point of the path, press any key.")

    point1 = group.get_current_joint_values()

    print("Saved that point.")
    print("Go to the end point.")
    raw_input("When ready to save this as the ending point of the path, press any key.")

    point2 = group.get_current_joint_values()

    print("Done!")
    print("Showing that path.")

    group.go(point1)
    plan = group.plan(point2)
    if not plan.joint_trajectory.points:
        rospy.logerror("Planning failed")
        return

    result = group.execute(plan, Wait=True)
    group.stop()

    name = raw_input("To save that path, type a name for it now: ")
    if len(name) is 0:
        print("Not saved.")
        return

    rp = rospkg.RosPack()
    file_path = os.path.join(rp.get_path('ur_bringup'), 'saved_paths', name)
    with open(file_path, 'w') as savefile:
        yaml.dump(plan, savefile, default_flow_style=True)

    

if __name__ == '__main__':
    main()
