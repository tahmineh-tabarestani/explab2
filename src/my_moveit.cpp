#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/SetBool.h>

bool reach(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  // 1st pose  
  group.setStartStateToCurrentState();
  std::vector<double> joint_values;
  joint_values = {0.78, 0, 0, 0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 1 " << std::endl;
  sleep(2.0);

  // 2nd pose  
  group.setStartStateToCurrentState();
  joint_values = {2.35, 0, 0, 0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 2 " << std::endl;
  sleep(2.0);

  // 3rd pose  
  group.setStartStateToCurrentState();
  joint_values = {3.07, -0.90, -1.69, -1.80};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 3 " << std::endl;
  sleep(2.0);

  // 4th pose  
  group.setStartStateToCurrentState();
  joint_values = {3.07, -0.90, -1.69, 4.0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 4 " << std::endl;
  sleep(2.0);

  // 5th pose  
  group.setStartStateToCurrentState();
  joint_values = {-1.57, 0, 0, 0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 5 " << std::endl;
  sleep(2.0);

  resp.message = "ok";
  return true;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_moveit");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return(0);
}
  
  
  

