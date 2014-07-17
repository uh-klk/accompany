#ifndef COBTORSOCONTROLLERCLIENT_H
#define COBTORSOCONTROLLERCLIENT_H

#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class UH_CobTorsoControllerClient
{
public:
  UH_CobTorsoControllerClient(){}
  void init(void);
  void sendGoal(float, float, float);

private:
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ptrMoveTorsoClient;
};


#endif
