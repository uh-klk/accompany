#ifndef COB_BASE_CONTROLLER_CLIENT_H
#define COB_BASE_CONTROLLER_CLIENT_H

#include "ros/ros.h"

#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

 struct Pose
  {
    float x; //in [m]
    float y; //in [m]
    float orientation; //in [rad]

    Pose()
    {
      x = 0;
      y = 0;
      orientation = 0;
    }

    Pose(float x_, float y_, float orientation_)
    {
      x = x_;
      y = y_;
      orientation = orientation_;
    }
  };

class UH_CobBaseControllerClient
{
public:
  UH_CobBaseControllerClient(){}
  void init(void);
  Pose getRobPose();
  void changeRobotHeading(float);
  void sendPoseTargetInMapFrame(Pose);



  float degree2radian(float degree)
  {
    float radian;
    float pi = 4.0 * std::atan2(1.0, 1.0);

    return radian = pi * degree / 180.0;
  }
  float radian2degree(float radian)
  {
    float degree;
    float pi = 4.0 * std::atan2(1.0, 1.0);

    return degree = 180 * radian / pi;
  }


private:
  ros::NodeHandle nh;
  tf::TransformListener *ptrTransformListener;
  ros::Publisher velPub;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ptrMoveBaseClient;
};

#endif
