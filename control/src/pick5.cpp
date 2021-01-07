/*
功能：根据目标位姿进行抓取
订阅：/dope/wrist3_pose		经过tf转换后的目标位姿
发布：/gripper_open_close	gripper抓取控制
*/


#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>
#include <std_msgs/Int32.h>

bool get_msg = false;
geometry_msgs::Pose target_pose;

void pose_soap_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    get_msg = true;
    geometry_msgs::PoseStamped pose = *msg;
    target_pose = pose.pose;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle node_handle;
  ros::Subscriber sub = node_handle.subscribe("/dope/wrist3_pose", 10, pose_soap_callback);
  ros::Publisher pub = node_handle.advertise<std_msgs::Int32>("/gripper_open_close",10);

  //define the command of gripprt
  std_msgs::Int32 gripper_open_close;
  gripper_open_close.data = 2;
  pub.publish(gripper_open_close);
  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 创建movegroup
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPoseReferenceFrame("world");//设置全局参考系
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // 可视化
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.2;
  visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();//uodata visual

  //添加车面 防止撞击车面
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = "ground_car";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.5;
  primitive.dimensions[1] = 2.0;
  primitive.dimensions[2] = 0.05;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = -1.0;
  box_pose.position.z = 0.4;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

    
  //获取初始位姿
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO("wait for the traget pose");
  while(!get_msg)
  {
      ;
  }

  //第一次运动
  tf::Quaternion quat;
  tf::quaternionMsgToTF(target_pose.orientation, quat);  
  double roll, pitch, yaw;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

  tf::Quaternion q;
  //q.setRPY(3.14,pitch,yaw+1.57);       //radian
  q.setRPY(3.14,pitch,yaw);

  geometry_msgs::Pose target_pose1;
  target_pose1 = target_pose;
  //target_pose1.position.y -= 0.05;
  target_pose1.position.x += 0.02;
  target_pose1.position.y -= 0.02;
  target_pose1.position.z += 0.15;
  
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();
  


  move_group.setPoseTarget(target_pose1); //添加pose空间

  moveit::planning_interface::MoveGroupInterface::Plan my_plan; //规划
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");

  //过程可视化
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
  // Parameter 1 (trajectory_): path information
  // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("Press 'next'1 in the RvizVisualToolsGui window to start the demo"); // 阻塞

  //执行
  if(success)
  {
    move_group.execute(my_plan);
    // gripper_open_close.data = 1; // cloth
    // pub.publish(gripper_open_close);
  }

  //第二次执行 
  target_pose1.position.z -= 0.08;
  move_group.setPoseTarget(target_pose1); //添加pose空间
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success)
  {
    move_group.execute(my_plan);
    gripper_open_close.data = 1; // cloth
    pub.publish(gripper_open_close);
  }
  //第third次执行 
  target_pose1.position.z += 0.03;
  move_group.setPoseTarget(target_pose1); //添加pose空间
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success)
  {
    move_group.execute(my_plan);
    gripper_open_close.data = 1; // cloth
    pub.publish(gripper_open_close);
  }


  //place
  target_pose1.position.x = 0.00633134284375; // 车面左上角位置
  target_pose1.position.y = -0.640539597811;
  target_pose1.position.z = 0.684433345497;

  target_pose1.position.z += 0.02;
  move_group.setPoseTarget(target_pose1); //添加pose空间

  
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");

  //过程可视化
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "pose2");
  visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
  // Parameter 1 (trajectory_): path information
  // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next'2 in the RvizVisualToolsGui window to start the demo"); // 阻塞

  //执行
  if(success)
  {
    move_group.execute(my_plan);
   gripper_open_close.data = 2; // close
    pub.publish(gripper_open_close);
    // gripper_open_close.data = 1; // cloth
    // pub.publish(gripper_open_close);
  }

  
    

  //回归原姿态
  move_group.setJointValueTarget(joint_group_positions); //设置joint空间
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  if(success)
  {
    move_group.execute(my_plan);
  }

    return 0;
}