#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

geometry_msgs::PoseStamped soap_msg;
int pose_first = 0;
void pose_soap_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_first++;
    soap_msg = *msg;
    ROS_WARN("i have got the 666");

    ros::AsyncSpinner spinner(1);//ROS多线程订阅消息，这里开了一个线程
    spinner.start();//线程启动
    ROS_WARN("i have got the 999");
    static const std::string PLANNING_GROUP = "manipulator_i5";
    ROS_WARN("i have got the 100");
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
    visual_tools.deleteAllMarkers();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75; // above head of PR2，文字的位置放在了PR2的头上

    geometry_msgs::Pose target_pose1;
    geometry_msgs::PoseStamped wrist3_pose;
    wrist3_pose.header.frame_id = "wrist3_Link";
    //wrist3_pose.stamp_ = ros::time(0);

    tf::TransformListener listener;
    ROS_WARN("i have got the 888");

}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle n;
    //ros::Publisher pub = nh.advertise<std_msgs::String>("/move_group/goal", 1000);
    ros::Subscriber sub = n.subscribe("/dope/pose_soap", 10, pose_soap_callback);
     ROS_WARN("i have got the 100");

    ros::AsyncSpinner spinner(1);//ROS多线程订阅消息，这里开了一个线程
    spinner.start();//线程启动
    ROS_WARN("i have got the 999");
    static const std::string PLANNING_GROUP = "manipulator_i5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
    visual_tools.deleteAllMarkers();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75; // above head of PR2，文字的位置放在了PR2的头上

    geometry_msgs::Pose target_pose1;
    geometry_msgs::PoseStamped wrist3_pose;
    wrist3_pose.header.frame_id = "wrist3_Link";
    //wrist3_pose.stamp_ = ros::time(0);

    tf::TransformListener listener;
    ROS_WARN("i have got the 888");
    while(ros::ok())
    {
        if(pose_first==1)
        {
            ROS_WARN("i have got the 777");
            try
            {
                listener.transformPose("base_Link", soap_msg, wrist3_pose);
            }
            catch (tf::TransformException ex)
            {
                ROS_WARN("transfrom exception : %s",ex.what());
            }
            target_pose1.orientation.w = 1.0;
            target_pose1.position.x = wrist3_pose.pose.position.x;
            target_pose1.position.y = wrist3_pose.pose.position.y;
            target_pose1.position.z = wrist3_pose.pose.position.z;
            move_group.setPoseTarget(target_pose1);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
            visual_tools.publishAxisLabeled(target_pose1, "pose1");
            visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools.trigger();
            visual_tools.prompt("next step");

            if(success)
            {
                printf("plan success and execution!");
                move_group.move();
            }
            else
                printf("plan error!");
            
        }
        ros::spin();
    }


    return 0;
}