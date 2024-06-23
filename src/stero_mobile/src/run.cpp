#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <global_planner/planner_core.h>
#include <rotate_recovery/rotate_recovery.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>


float position_x, position_y, orientation_x, orientation_y, orientation_z, orientation_w;
bool is_moving = false;
bool goal_received = false;
geometry_msgs::Twist velocity;
std::vector<geometry_msgs::PoseStamped> route;
geometry_msgs::PoseStamped start_point, end_point;

void goal_callbackback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    end_point = *msg;
    goal_received = true;
}

void odom_callback (const nav_msgs::Odometry& msg)
{
    position_x = msg.pose.pose.position.x;
    position_y = msg.pose.pose.position.y;
    orientation_x = msg.pose.pose.orientation.x;
    orientation_y = msg.pose.pose.orientation.y;
    orientation_z = msg.pose.pose.orientation.z;
    orientation_w = msg.pose.pose.orientation.w;
}

void plan_route(global_planner::GlobalPlanner &my_global_planner, base_local_planner::TrajectoryPlannerROS &my_trajectory_planner)
{
    start_point.header.frame_id = "map";
    start_point.pose.position.x = position_x;
    start_point.pose.position.y = position_y;
    start_point.pose.position.z = 0.0;
    start_point.pose.orientation.x = orientation_x;
    start_point.pose.orientation.y = orientation_y;
    start_point.pose.orientation.z = orientation_z;
    start_point.pose.orientation.w = orientation_w;
    my_global_planner.makePlan(start_point, end_point, route);
    my_global_planner.publishPlan(route);
    my_trajectory_planner.setPlan(route);
    goal_received = false;
    is_moving = true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stero_mobile_init_node");
    ros::NodeHandle nh;
    
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener my_tf(buffer);
    costmap_2d::Costmap2DROS global_costmap("my_global_costmap", buffer);
    costmap_2d::Costmap2DROS local_costmap("my_local_costmap", buffer);
    base_local_planner::TrajectoryPlannerROS my_trajectory_planner;
    rotate_recovery::RotateRecovery my_rotate_recovery;
    global_planner::GlobalPlanner my_global_planner("my_global_planner", global_costmap.getCostmap(), "map");
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goal_callbackback);
    ros::Subscriber odom_sub = nh.subscribe("/mobile_base_controller/odom", 10, odom_callback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("nav_vel", 1000);

    my_trajectory_planner.initialize("my_trajectory_planner", &buffer, &local_costmap);
    my_rotate_recovery.initialize("my_rotate_recovery", &buffer, &global_costmap, &local_costmap);

    ros::Rate loopRate(10.0);
    loopRate.sleep();
    
    while(ros::ok)
    {
        if(my_trajectory_planner.isGoalReached())
            is_moving = false;
        else if(is_moving)
        {
            if(my_trajectory_planner.computeVelocityCommands(velocity))
                vel_pub.publish(velocity);
            else
                my_rotate_recovery.runBehavior();
        }
        if(goal_received)
            plan_route(my_global_planner, my_trajectory_planner);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
