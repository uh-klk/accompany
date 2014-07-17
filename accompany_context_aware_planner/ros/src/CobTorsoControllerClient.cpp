#include "accompany_context_aware_planner/CobTorsoControllerClient.h"

void UH_CobTorsoControllerClient::init()
{
  ptrMoveTorsoClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh, "/torso_controller/follow_joint_trajectory/", true);

  ROS_INFO("Waiting for the torso action server to come up.");
  if (ptrMoveTorsoClient->waitForServer(ros::Duration(2.0) ) )
      ROS_INFO("Torso action server is up.");

}

void UH_CobTorsoControllerClient::sendGoal(float torso_lower_tilt, float torso_pan, float torso_upper_tilt)
{
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.joint_names.push_back("torso_lower_neck_tilt_joint");
  goal.trajectory.joint_names.push_back("torso_pan_joint");
  goal.trajectory.joint_names.push_back("torso_upper_neck_tilt_joint");

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.push_back(torso_lower_tilt);
  goal.trajectory.points[0].positions.push_back(torso_pan);
  goal.trajectory.points[0].positions.push_back(torso_upper_tilt);

  // 0 -0.1 0  left
  // 0  0.1 0  right

  goal.trajectory.points[0].velocities.push_back(0.0);
  goal.trajectory.points[0].velocities.push_back(0.0);
  goal.trajectory.points[0].velocities.push_back(0.0);

  goal.trajectory.points[0].time_from_start =ros::Duration(1.0);

  goal.goal_time_tolerance=ros::Duration(0);

  ptrMoveTorsoClient->sendGoal(goal);

  /*ptrMoveTorsoClient->waitForResult();
  if(ptrMoveTorsoClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reached its target.");
  else
    ROS_INFO("The torso failed to rotate for some reason");*/
}

/*

int main(int argc, char **argv)
{

  ros::init(argc, argv, "PublisherAndListener");
  ros::NodeHandle n;

  UH_CobTorsoControllerClient torsoControllerClient(n);

  torsoControllerClient.init();
  torsoControllerClient.sendGoal(0,0.2,0);
  ros::Duration(3).sleep();
  torsoControllerClient.sendGoal(0,-0.2,0);
  ros::Duration(3).sleep();
  torsoControllerClient.sendGoal(0,0,0);
/*
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> MoveTorsoClient;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ptrMoveTorsoClient=NULL;
  ptrMoveTorsoClient = new MoveTorsoClient("/torso_controller/follow_joint_trajectory/", true);

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.joint_names.push_back("torso_lower_neck_tilt_joint");
  goal.trajectory.joint_names.push_back("torso_pan_joint");
  goal.trajectory.joint_names.push_back("torso_upper_neck_tilt_joint");

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.push_back(-0.1);
  goal.trajectory.points[0].positions.push_back(0.0);
  goal.trajectory.points[0].positions.push_back(-0.15);

  // 0 -1 0  left
  // 0  1 0  right

  goal.trajectory.points[0].velocities.push_back(0.0);
  goal.trajectory.points[0].velocities.push_back(0.0);
  goal.trajectory.points[0].velocities.push_back(0.0);

  goal.trajectory.points[0].time_from_start =ros::Duration(3.0);

  goal.goal_time_tolerance=ros::Duration(0);

  ros::Rate poll_rate(100);

  while(!ptrMoveTorsoClient->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the torso action server to come up");
    }

  ptrMoveTorsoClient->sendGoal(goal);

  ptrMoveTorsoClient->waitForResult();
  if(ptrMoveTorsoClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reached its target.");
  else
    ROS_INFO("The torso failed to rotate for some reason");
*//*
  ros::spin();

  return 0;
}
*/
