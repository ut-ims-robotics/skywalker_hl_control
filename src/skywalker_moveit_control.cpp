#include "ros/ros.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/GetMotionPlan.h"
#include "moveit_msgs/Constraints.h"
#include "moveit_msgs/JointConstraint.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "moveit_msgs/PlanningScene.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include "pluginlib/class_loader.h"
#include <string>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit/move_group_interface/move_group_interface.h>

#include <boost/scoped_ptr.hpp>

class skywalker_moveit_control
{
private:
  sensor_msgs::JointState _current_Joint;         //current joint state for better preview/ updates with new IK
  moveit_msgs::GetPositionIK _request_ik;         //whole IK request variable. some pre-filled, some filled during the function call 
  moveit_msgs::GetMotionPlan _request_trajectory; //whole traj request. seems to be not used...
  // moveit_msgs::JointConstraint _goalPosition;  //what the fuck???...

  ros::NodeHandle _nh;                            //node handle
  ros::ServiceClient _client_ik;                  //clients for service requests. will send the actual request, when it is made
  ros::ServiceClient _client_trajectory;
public:
  skywalker_moveit_control(ros::NodeHandle nh, std::string group_name)
  {
    _nh = nh;
    _client_ik = _nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    _client_trajectory = _nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");

    //pre-filling the IK request with constant and initial data
    _current_Joint = sensor_msgs::JointState();
    _request_ik.request.ik_request.robot_state.joint_state = _current_Joint;
    _request_ik.request.ik_request.group_name = group_name;    
    _request_ik.request.ik_request.pose_stamped = geometry_msgs::PoseStamped();
    _request_ik.request.ik_request.pose_stamped.header.frame_id = "base_footprint";
    _request_ik.request.ik_request.robot_state.is_diff = true;
  }
  ~skywalker_moveit_control();

  sensor_msgs::JointState IK_compute(geometry_msgs::PoseStamped);

  //work for future: make a traj preview request
};



skywalker_moveit_control::~skywalker_moveit_control()
{
}

sensor_msgs::JointState skywalker_moveit_control::IK_compute(geometry_msgs::PoseStamped end_effector_position)
{
  //filling last pieces of request
  _request_ik.request.ik_request.pose_stamped = end_effector_position;
  //ROS_INFO_STREAM(_request_ik.request);
  _request_ik.request.ik_request.pose_stamped.header.frame_id = "base_footprint";
  
  if (_client_ik.call(_request_ik))//sending the request
  {
    //update of current robot state for future
    _request_ik.request.ik_request.robot_state.joint_state = _request_ik.response.solution.joint_state;
    std::cout << _request_ik.response.error_code << std::endl;
  }
  //give me the IK for use
  return _request_ik.request.ik_request.robot_state.joint_state;
}

//receiving the callback for desired end-effector pose
geometry_msgs::PoseStamped end_effector_pos;
void eeCb(geometry_msgs::PoseStamped msg)
{
  end_effector_pos = msg;
}

bool toPlan = false;
void planCb(std_msgs::Bool msg)
{
  toPlan = msg.data;
}

int main(int argc, char **argv)
{
  //initiating all regular ros and moveit stuff
  ros::init(argc, argv, "skywalker_moveit_control");
  ros::NodeHandle _nh;
  static const std::string PLANNING_GROUP = "ur5_arm";

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber eeSub = _nh.subscribe(
    "unity/end_effector_goal",1000, eeCb);

  ros::Subscriber toplanCb = _nh.subscribe(
    "unity/to_plan",1, planCb); //for now querry better be 1

  ros::Publisher isSentPub = _nh.advertise<std_msgs::Bool>("/toSend",10);

  ros::Publisher joint_pub = _nh.advertise<sensor_msgs::JointState>("/querry_joint",10);

  moveit::planning_interface::MoveGroupInterface
    move_group_interface(PLANNING_GROUP);
  robot_model_loader::RobotModelLoader
    robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr&
    robot_model = robot_model_loader.getModel();

  /* Create a RobotState and JointModelGroup
  to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(
    new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group =
    robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene(
    new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().
    setToDefaultValues(joint_model_group, "ready");

  boost::scoped_ptr<pluginlib::ClassLoader<
    planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (!_nh.getParam("/move_group/planning_pipelines/ompl/planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, _nh.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls : classes)
      ss << cls << " ";
    ROS_ERROR_STREAM("yException while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
  
  ros::Rate loop_rate(10);
  sensor_msgs::JointState joint_to_send = sensor_msgs::JointState(); //just a joint message for a latter use after IK request

  skywalker_moveit_control skywalker_moveit_control_object = skywalker_moveit_control(_nh, PLANNING_GROUP);

  end_effector_pos.pose.position.x = 0.4;
  end_effector_pos.pose.position.y = 0.3;
  end_effector_pos.pose.position.z = 0.8;
  end_effector_pos.pose.orientation.x = 0;
  end_effector_pos.pose.orientation.y = 0;
  end_effector_pos.pose.orientation.z = 0;
  end_effector_pos.pose.orientation.w = 1;
  end_effector_pos.header.frame_id = "ur53_base_link";

  while(ros::ok())
  {
    joint_to_send = skywalker_moveit_control_object.IK_compute(end_effector_pos);
    joint_pub.publish(joint_to_send);

    ROS_INFO_STREAM(joint_to_send);

    //TODO
    //check for to_plan var

    if(toPlan)
    {
      ROS_INFO_STREAM("Time to execute.");
      ROS_INFO_STREAM(joint_to_send);
      moveit::core::RobotState goal_state(robot_model);
      std::vector<double> joint_value;

      moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions = joint_to_send.position;
      move_group_interface.setJointValueTarget(joint_group_positions);
      //change to actually have a trajectory as a preview. 
      bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
      {
          move_group_interface.execute(my_plan);
      }

      std_msgs::Bool isSent;
      isSent.data = false;
      isSentPub.publish(isSent);
    }

    loop_rate.sleep();
  }

  return 0;
}