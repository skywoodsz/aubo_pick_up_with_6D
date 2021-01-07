#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
/*
功能： tf转换
订阅：/dope/pose_soap 目标物体基于相机坐标系位姿
发布：/dope/wrist3_pose 目标物体基于世界坐标系位姿
	  visualization_marker RVIZ 目标物体可视化
*/
class camera_tf
{
public:
    camera_tf()
    {
        sub_ = n_.subscribe("/dope/pose_soap",10,&camera_tf::poseCallback,this);
        pub_ = n_.advertise<geometry_msgs::PoseStamped>("/dope/wrist3_pose",10);
        marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        /*  
        while (ros::ok())
        {
            poseCallback_test();
        }
        */
        
    }
    ~camera_tf(){}
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_msg = *msg;
        pose_msg.header.stamp=ros::Time(0);
        //pose_msg.header.frame_id="camera_link";
        try
            {
                listener_.transformPose("world", pose_msg, wrist3_pose_);
            }
        catch (tf::TransformException ex)
            {
                ROS_WARN("transfrom exception : %s",ex.what());
            }
        pub_.publish(wrist3_pose_);

        points.header.frame_id = wrist3_pose_.header.frame_id;
        points.header.stamp =  ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.position = wrist3_pose_.pose.position;
        points.pose.orientation = wrist3_pose_.pose.orientation;

        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 1.0;
        points.scale.y = 1.0;
        points.color.g = 1.0f;
        points.color.a = 1.0;
        marker_pub.publish(points);


    }
	// just debug test
    void poseCallback_test()
    {
        tf::Quaternion q;
        q.setRPY(3.14,0,-1.57);       //radian
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "camera_color_optical_frame";
        pose_msg.header.stamp = ros::Time(0);
        pose_msg.pose.position.x = -0.02; //-0.44
        pose_msg.pose.position.y = 0.07; //-0.08
        pose_msg.pose.position.z = 0.6;  //0.33
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();


        try
            {
                listener_.transformPose("world", pose_msg, wrist3_pose_);
            }
        catch (tf::TransformException ex)
            {
                ROS_WARN("transfrom exception : %s",ex.what());
            }
        pub_.publish(wrist3_pose_);

    }

public:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher marker_pub;
    visualization_msgs::Marker points;
    tf::TransformListener listener_;
    geometry_msgs::PoseStamped wrist3_pose_;
    
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_camera");
    camera_tf camera_tf_test;
    ros::spin();
    return 0;
}