#include "accompany_context_aware_planner/CobBaseControllerClient.h"

void UH_CobBaseControllerClient::init()
{
  ptrTransformListener = new tf::TransformListener();

  velPub = nh.advertise<geometry_msgs::Twist>("/base_controller/command_safe",10);  //for sending direct command to the base controller

  ptrMoveBaseClient = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(nh,"move_base", true); //for sending goal action

  //wait for the action server to come up
  ROS_INFO("Waiting for the move_base action server to come up");
  if (ptrMoveBaseClient->waitForServer(ros::Duration(2.0)))
    ROS_INFO("move_base action server is up");
}

Pose UH_CobBaseControllerClient::getRobPose() //Calculate robot pose in map coordinate frame
{
  //stored the robot's origin in base_link frame
  ROS_INFO("UH_CobBaseControllerClient::getRobPose()");
  tf::Stamped<tf::Pose> robotOrigin = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(0),
                                                                     tf::Point(0.0, 0.0, 0.0)),
                                                            ros::Time(0)/*ros::Time::now()*/,
                                                            "base_link"); //base_link origin is the robot coordinate frame
  // create a PoseStamped variable to store the StampedPost TF
  geometry_msgs::PoseStamped map_frame; // create a map_frame to store robotOrigin in map frame
  geometry_msgs::PoseStamped base_link_frame; // create a base_link_frame to store robotOrigin in base_link frame
  tf::poseStampedTFToMsg(robotOrigin, base_link_frame); //stored the robot coordinate in base_link frame
  try
  {
    ptrTransformListener->transformPose("map", base_link_frame, map_frame); //listen for base_link to map transform, then transform the robot coordinate to map coordinate frame
    //ROS_INFO("Robot coordinate in map frame: (%.2f, %.2f, %.2f)",map_frame.pose.position.x, map_frame.pose.position.y, radian2degree(tf::getYaw(map_frame.pose.orientation)));
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform robot origin from \"base_link\" to \"map\": %s", ex.what());
  }

  Pose robotLocation(map_frame.pose.position.x, map_frame.pose.position.y, tf::getYaw(map_frame.pose.orientation)); // stored the current robot pose

  return robotLocation;
}

void UH_CobBaseControllerClient::changeRobotHeading(float deltaTheta)
{
  geometry_msgs::Twist cmd;

  cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

  if (deltaTheta!=0)
  {
    float angularVel = 0.5*((sqrt(deltaTheta*deltaTheta))/degree2radian(180));
    //calculate the desire angular velocity --- might overshoot depending on the tracker refresh rate and robot battery charge

    if ((angularVel < 0.3) && (angularVel >0.2))
      angularVel=0.3;
    else if ((angularVel<=0.2) && (angularVel>=0.1))
        angularVel=0.2;
    else if (angularVel< 0.1)
        angularVel=0.2;

    cmd.angular.z = angularVel*(sqrt(deltaTheta*deltaTheta)/deltaTheta); ////calculate the desire direction
    ROS_INFO("%f", cmd.angular.z);

    //cmd.angular.z = 0.5*(sqrt(deltaTheta*deltaTheta)/deltaTheta);
  }

  velPub.publish(cmd);
}

void UH_CobBaseControllerClient::sendPoseTargetInMapFrame(Pose robPose)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = robPose.x;
  goal.target_pose.pose.position.y = robPose.y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robPose.orientation);

  ROS_INFO("Robot orientation =  %f", radian2degree(robPose.orientation));
  ROS_INFO("Robot delta to target = %f", radian2degree(robPose.orientation) );

  ptrMoveBaseClient->sendGoal(goal);
  ptrMoveBaseClient->waitForResult();

  if(ptrMoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reached its target.");
  else
    ROS_INFO("The base failed to rotate for some reason");
}
/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "PublisherAndListener");

  ros::NodeHandle n;

  UH_CobBaseControllerClient baseControllerClient(n);

  baseControllerClient.init();

  baseControllerClient.changeRobotHeading(0.3);

  Pose temPose(0,0,0);

  //baseControllerClient.sendPoseTargetInMapFrame(temPose);

  ros::spin();
  return 0;

}
*/
