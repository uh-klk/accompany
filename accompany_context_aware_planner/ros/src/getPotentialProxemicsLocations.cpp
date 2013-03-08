#include "accompany_context_aware_planner/GetPotentialProxemicsLocations.h"
#include <cstdlib>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread.hpp>

float degree2radian(float degree)
{
  float radian;
  float pi = 4.0 * std::atan2(1.0, 1.0);

  return radian = pi * degree / 180.0;
}

void gotoTarget(float x, float y, float theta)
{

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  move_base_msgs::MoveBaseGoal goal;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the move_base action server to come up");

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";//"base_footprint"; //"base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  ac.sendGoal(goal);
  ROS_INFO("waiting for goal result" );
  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, robot reached the goal");
  }
  else
  {
    ROS_INFO("robot failed to reach goal some reason");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");

  if (argc != 4)
  {
    ROS_INFO("usage: getPotentialProxemicsLocations X(m) Y(m) Orientation(deg)");

    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient
                     client =
                         n.serviceClient<accompany_context_aware_planner::GetPotentialProxemicsLocations> (
                                                                                                           "get_potential_proxemics_locations");
  accompany_context_aware_planner::GetPotentialProxemicsLocations srv;

  float X = atof(argv[1]);
  float Y = atof(argv[2]);
  float deg = atof(argv[3]);

  srv.request.header.frame_id = "map";
  srv.request.header.stamp = ros::Time::now();
  srv.request.userId = 1;
  srv.request.userPosture = 1;
  srv.request.userPose.orientation = tf::createQuaternionMsgFromYaw(degree2radian(deg)); //create Quaternion Msg from Yaw
  tf::pointTFToMsg(tf::Point(X, Y, 0.0), srv.request.userPose.position); //convertTF to Msg then store in the server request container
  srv.request.robotGenericTaskId = 1;

  if (client.call(srv))
  {
    for (unsigned i = 0; i < srv.response.targetPoses.size(); i++)
    {
      /*
       srv.response.targetPoses[i].header.seq
       srv.response.targetPoses[i].header.stamp
       srv.response.targetPoses[i].header.frame_id
       srv.response.targetPoses[i].pose.position.x
       srv.response.targetPoses[i].pose.position.y
       srv.response.targetPoses[i].pose.position.z
       */

      ROS_INFO("Request is:  x=%f, y=%f, z=%f yaw = %f",
          srv.request.userPose.position.x,
          srv.request.userPose.position.y,
          srv.request.userPose.position.z,
          tf::getYaw(srv.request.userPose.orientation));

      ROS_INFO("MsgSeq = %d, time =  %2f, coordinate frame = %s ",
          srv.response.targetPoses[i].header.seq,
          (ros::Time::now().toSec()-srv.response.targetPoses[i].header.stamp.toSec()),
          srv.response.targetPoses[i].header.frame_id.c_str());

      ROS_INFO("response is: x=%f, y=%f, z=%f yaw = %f",
          srv.response.targetPoses[i].pose.position.x,
          srv.response.targetPoses[i].pose.position.y,
          srv.response.targetPoses[i].pose.position.z,
          tf::getYaw(srv.response.targetPoses[i].pose.orientation));

      gotoTarget(srv.response.targetPoses[i].pose.position.x, srv.response.targetPoses[i].pose.position.y,
                 tf::getYaw(srv.response.targetPoses[i].pose.orientation));

    }

  }
  else
  {
    ROS_ERROR("Failed to call service getPotentialProcemicsLocations");
    return 1;
  }

  return 0;
}
