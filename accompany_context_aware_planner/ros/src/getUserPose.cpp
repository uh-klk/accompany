#include "ros/ros.h"
#include "std_msgs/String.h"
#include <accompany_uva_msg/TrackedHumans.h>

#include <std_msgs/ColorRGBA.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <cob_light/SetLightModeAction.h>
#include <cob_light/SetLightMode.h>


#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <vector>

/* MySQL Connector/C++ specific headers */
#include <cppconn/driver.h>
#include <cppconn/connection.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include <cppconn/metadata.h>
#include <cppconn/resultset_metadata.h>
#include <cppconn/exception.h>
#include <cppconn/warning.h>

#include <sensor_msgs/Joy.h>


tf::TransformListener *ptrListener=NULL;
ros::Publisher *ptrVelPub=NULL;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ptrMoveBaseClient=NULL;

int light_on =0;

using namespace std;
using namespace sql;

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

 struct Bearing
 {
   float distance;
   float orientation;

   Bearing()
   {
     distance = 0.0;
     orientation = 0.0;
   }
 };



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
bool isRobotFree(void)
{
  string sql;
  string expValue ="1"; //lock or free
  string actualValue;
  bool status = false;

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect("tcp://localhost:3306", "rhUser", "waterloo"); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema("Accompany"); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  //determine if the current session user is in the Living Room
  sql = "SELECT value from Accompany.Sensors where sensorId = 1001";
  cout << sql << endl;
  result = stmt->executeQuery(sql);

  if (result->next())
  {
    actualValue = result->getString("value");

    if (! expValue.compare(actualValue))
    {
      cout<<"oneeeeeeeeeeee"<<endl;
      status = true;
    }
    else
    {
      cout<<"Zeeero"<<endl;
      cout<<expValue<<" != "<<actualValue<<endl;

      status = false;
    }
  }
  else
  {
    cout<<"Error with the following statement: "<<sql<<endl;

    status = false;
  }

  delete result;
  delete stmt;
  con -> close();
  delete con;

  return status;
}
Pose getRobotPoseInMapFrame()
{
  //Calculate robot pose in map coordinate frame
  //    stored the robot's origin in base_link frame
  tf::Stamped<tf::Pose> robotOrigin = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(0),
                                                                     tf::Point(0.0, 0.0, 0.0)),
                                                            ros::Time(0),
                                                            "base_link"); //base_link origin is the robot coordinate frame
  // create a PoseStamped variable to store the StampedPost TF
  geometry_msgs::PoseStamped map_frame; // create a map_frame to store robotOrigin in map frame
  geometry_msgs::PoseStamped base_link_frame; // create a base_link_frame to store robotOrigin in base_link frame
  tf::poseStampedTFToMsg(robotOrigin, base_link_frame); //stored the robot coordinate in base_link frame
  try
  {
    ptrListener->transformPose("map", base_link_frame, map_frame); //listen for base_link to map transform, then transform the robot coordinate to map coordinate frame

    ROS_INFO("Robot coordinate in map frame: (%.2f, %.2f, %.2f)",
             map_frame.pose.position.x, map_frame.pose.position.y, radian2degree(tf::getYaw(map_frame.pose.orientation)));
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform robot origin from \"base_link\" to \"map\": %s", ex.what());
  }

  Pose robotLocation(map_frame.pose.position.x, map_frame.pose.position.y, tf::getYaw(map_frame.pose.orientation)); // stored the current robot pose

  return robotLocation;
}


float calRobotRotationAngle(Pose robPose, Pose humPose)
{

   float d_y = humPose.y-robPose.y;
   float d_x = humPose.x-robPose.x;
   float theta_tar = atan2(d_y, d_x);

   std::cout<<radian2degree(theta_tar)<<endl;

   return theta_tar;
}
float calDistanceToHuman(Pose robPose, Pose humPose)
 {

   float d_y = humPose.y-robPose.y;
   float d_x = humPose.x-robPose.x;

  return sqrt((d_x*d_x)+(d_y*d_y));
 }

int iSeeYouSeeingMe(Pose robPoseInRobotFrame, Pose humPoseInRobotFrame)
{
  float deltaTheta = 0;
  float distance = 9999;

  geometry_msgs::Twist cmd;

  cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

  deltaTheta = calRobotRotationAngle(robPoseInRobotFrame, humPoseInRobotFrame);
  distance = calDistanceToHuman(robPoseInRobotFrame, humPoseInRobotFrame);

  cout<<"Distance = "<< distance <<", Angle = "<< radian2degree(deltaTheta) <<endl;

  //turns toward the user when the user entered the robot's social zone and he is more than 10 degree off the robot's heading and when it is safe for the robot to do so.
  if ( (distance <= 3.6) && (distance >=0.5) && (( radian2degree(deltaTheta) >= 10) || (radian2degree(deltaTheta) <= -10))  && isRobotFree())
  {
    float angularVel = 0.5*((sqrt(deltaTheta*deltaTheta))/degree2radian(180));
    cout<<angularVel<<endl;

    if ((angularVel < 0.3) && (angularVel >0.2))
      angularVel=0.3;
    else if ((angularVel<=0.2) && (angularVel>=0.1))
        angularVel=0.2;
    else if (angularVel< 0.1)
        angularVel=0.1;

    cmd.angular.z = angularVel*(sqrt(deltaTheta*deltaTheta)/deltaTheta); //calculate the desire angular velocity
    cout<<cmd.angular.z<<endl;
    ptrVelPub->publish(cmd); //send command to the based controller, this method provide a smoother tracking behaviour
  }
  else
    {
      ptrVelPub->publish(cmd);  //always zero
    }

  return 1;
}

void chatterCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg)
{
  geometry_msgs::PointStamped userPointInMap;

  for (int i=0; i< msg->trackedHumans.size(); ++i)
  {
    if (msg->trackedHumans[i].specialFlag == 1) //specialFlag indicate the most reliable tracked person.
    {
      try // transform to robot egocentric coordinate frame (i.e. base_link)
      {
        //wait for /base_link to /camera_frame transform to be available for the specific time stamped
        bool condition =  ptrListener->waitForTransform("/base_link",
                                      msg->trackedHumans[i].location.header.frame_id,
                                      msg->trackedHumans[i].location.header.stamp,
                                      ros::Duration(5.0));
        //transform the tracked human's coordinate from /camera_frame to /base_link frame
        ptrListener->transformPoint("/base_link",
                                msg->trackedHumans[i].location,
                                userPointInMap);

        ROS_INFO("User is at (%f, %f) of the robot coordinate frame ",userPointInMap.point.x, userPointInMap.point.y);

        Pose userPoseInRobotFrame(userPointInMap.point.x, userPointInMap.point.y, 0);
        Pose robotPoseInRobotFrame(0,0,0); //egocentric

        iSeeYouSeeingMe(robotPoseInRobotFrame,userPoseInRobotFrame);

      }
      catch (tf::TransformException e)
      {
        ROS_ERROR("Received an exception trying to transform user location from \"camera_frame\" to \"map\" : %s", e.what());
      }
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "PublisherAndListener");
  ros::NodeHandle n;

  /*//for publishing to joystick
  ros::Publisher keyboard_pub = n.advertise<sensor_msgs::Joy>("joy",1);

  sensor_msgs::Joy msg;
  msg.axes.resize(6);
  msg.buttons.resize(12);

  // a -up
  msg.buttons[5]=1;
  msg.buttons[6]=1;
  msg.axes[4]=-1;
  keyboard_pub.publish(msg);
  */

  ptrListener = new tf::TransformListener();
  ptrMoveBaseClient = new MoveBaseClient("move_base", true);

  //wait for the action server to come up
  while(!ptrMoveBaseClient->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Publisher VelPub = n.advertise<geometry_msgs::Twist>("/base_controller/command_safe",1);  //for sending direct command to the base controller
  ros::Subscriber sub = n.subscribe("/trackedHumans", 1, chatterCallback); // stored only the most recent
  ptrVelPub = &VelPub;


  ros::spin();

  return 0;
}
