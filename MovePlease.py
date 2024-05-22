#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv

def read_pose_from_csv(file_path):
    poses = []
    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            # Assuming CSV columns named 'x', 'y', 'z'
            x, y, z = float(row['x']), float(row['y']), float(row['z'])
            poses.append([x, y, z])
    return poses

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_move_group_from_csv', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Load poses from CSV
    file_path = '/projects/PathPlanning/Scaled_Downsample_optimized_path.csv'
    poses = read_pose_from_csv(file_path)

    # Set and execute each pose
    for pose_data in poses:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0  # Static orientation
        pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = pose_data
        move_group.set_pose_target(pose_goal)

        # Plan and execute
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        rospy.sleep(1)  # Optional delay

if __name__ == '__main__':
    main()
