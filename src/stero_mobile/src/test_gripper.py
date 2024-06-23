#!/usr/bin/env python3
import sys
import rospy
from time import sleep
import geometry_msgs.msg
import moveit_commander

joint_default = [0.15, 0.2, -1.335, -0.2, 1.924, -1.57, 1.369, 0.0]

def move_to_point(move_group, point_x, point_y, point_z):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = point_x
    pose_goal.position.y = point_y
    pose_goal.position.z = point_z
    pose_goal.orientation.w = 1.0

    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()


def homing(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal = joint_default
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    
    
def switch_gripper(gripper_group, open_gripper, goal_x, goal_y):
    goal = gripper_group.get_current_joint_values()
    goal[0] = goal_x
    goal[1] = goal_y
    gripper_group.go(goal, wait=open_gripper)
    gripper_group.stop()


def main():
    rospy.init_node("testing_our_project", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    state = 10
    sleep(1)

    move_group = moveit_commander.MoveGroupCommander("arm_torso")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    
    while(True):
        if state == 10:
            print("Moving to point...")
            move_to_point(move_group, 0.5, -0.4, 0.5)
            state = 20
            
        if state == 20:
            print("Closing the gripper...")
            switch_gripper(gripper_group, False, 0.01, 0.01)
            print("The gripper is closed.")
            sleep(3)
            state = 30
            
        if state == 30:
            print("Opening the gripper...")
            switch_gripper(gripper_group, True, 0.04, 0.04)
            print("The gripper is open.")
            state = 40
            
        if state == 40:
            print("Homing...")
            homing(move_group)
            print("Done:)")
            break

if __name__ == "__main__":
    main()
