#!/usr/bin/env python3
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from move_base_msgs.msg import MoveBaseActionResult
from gazebo_msgs.srv import GetLinkState
from time import sleep
import sys
import moveit_commander
import moveit_msgs.msg
import PyKDL
import math
import tf2_geometry_msgs

state = 0


table_pos = { 'table_0': PyKDL.Vector(-7.32247, -1.481084, 0.265),
              'table_1': PyKDL.Vector(-4.45497, 0.962041, 0.415),
              'table_2': PyKDL.Vector(-1.336559, 3.2311, 0.39)}
              
object_pos = { 'table_0': PyKDL.Vector(-4.3722, 2.8982, 0.625),
              'table_1': PyKDL.Vector(-7.11527, 0.2852, 0.875),
              'table_2': PyKDL.Vector(0.434879, 1.05841, 0.805)}


joint_default = [0.15, 0.2, -1.335, -0.2, 1.924, -1.57, 1.369, 0.0]
joint_straight = [0.18, 0.07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

FRAME_ID = 'map'
SCENE = moveit_commander.PlanningSceneInterface()


def get_table_coords(table_name):
    if table_name not in table_pos.keys():
        print('ERROR: Table does not exist.')
        return None, None, None, None
    
    return table_pos[table_name].x(), table_pos[table_name].y()


def goto(x, y):
    pose = Pose()
    orientations = quaternion_from_euler(0, 0, 0) #tu nie do konca dziala:/
    pose.position.x = x
    pose.position.y = y
    pose.orientation.x = orientations[0]
    pose.orientation.y = orientations[1]
    pose.orientation.z = orientations[2]
    pose.orientation.w = orientations[3]

    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose = pose
    return msg


def homing(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal = joint_default
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    
def straight_arm(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal = joint_straight
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    
def move_arm(move_group, pose_goal):
    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
def switch_gripper(gripper_group, open_gripper, goal_x, goal_y):
    goal = gripper_group.get_current_joint_values()
    goal[0] = goal_x
    goal[1] = goal_y
    gripper_group.go(goal, wait=open_gripper)
    sleep(3)
    gripper_group.stop()


def get_object_coords(object_name):
    objects_links = object_pos[object_name]
    pose_goal = Pose()
    pose_goal.position.x = objects_links.x()
    pose_goal.position.y = objects_links.y()
    pose_goal.position.z = objects_links.z()
    return pose_goal
    

def resultCallback(result):
    result_text = result.status.text
    if result_text == "Goal reached.":
    	global state
    	state += 10
    print(result.status.text)
    return


def main():
    global state
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('projekt3_stero')
    sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult ,resultCallback)
    sleep(1)
    print("Simulation started :)")
    rate = rospy.Rate(10)

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("arm_torso")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    move_group.set_num_planning_attempts(45)


    while not rospy.is_shutdown():
        if state == 0:
            print("Going to table 0.")
            x, y = get_table_coords('table_0')
            msg = goto(x, y)
            pub.publish(msg)
            state = 10
            
        if state == 10:
            print("Waiting...")
            
        if state == 20:
            print("Taking the object from table 0.")
            object_pose = get_object_coords('table_0')
            straight_arm(move_group)
            move_arm(move_group, object_pose)
            switch_gripper(gripper_group, False, 0.01, 0.01)
            homing(move_group)
            state = 30
            
        if state == 30:
            print("Going to table 1.")
            goal_x, goal_y = get_table_coords('table_1')
            msg = goto(goal_x, goal_y)
            pub.publish(msg)
            state = 40
            
        if state == 40:
            print("Waiting...")
            
        if state == 50:
            switch_gripper(gripper_group, True, 0.04, 0.04)
            print("Taking the object from table 1.")
            object_pose = get_object_coords('table_1')
            straight_arm(move_group)
            move_arm(move_group, object_pose)
            switch_gripper(gripper_group, False, 0.01, 0.01)
            homing(move_group)
            state = 60
            
        if state == 60:
            print("Going to table 2.")
            goal_x, goal_y = get_table_coords('table_2')
            msg = goto(goal_x, goal_y)
            pub.publish(msg)
            state = 70
            
        if state == 70:
            print("Waiting...")
            
        if state == 80:
            switch_gripper(gripper_group, True, 0.04, 0.04)
            print("Taking the object from table 2.")
            object_pose = get_object_coords('table_2')
            straight_arm(move_group)
            move_arm(move_group, object_pose)
            switch_gripper(gripper_group, False, 0.01, 0.01)
            homing(move_group)
            break

        
        rate.sleep() 
    

if __name__ == "__main__":
    main()
