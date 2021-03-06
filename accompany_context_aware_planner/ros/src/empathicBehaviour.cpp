#include "accompany_context_aware_planner/CobLightControllerClient.h"
#include "accompany_context_aware_planner/CobTorsoControllerClient.h"
#include "accompany_context_aware_planner/CobBaseControllerClient.h"

#include <accompany_uva_msg/TrackedHumans.h>

#include <cob_leg_detection/TrackedHumans.h>

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
using namespace sql;

using namespace std;

class EmpathicBehaviour
{
public:

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

  struct TrackedData
  {
    bool dataFlag;
    float distanceToHuman;
    Pose pose;

    TrackedData()
    {
      dataFlag = 0;
      distanceToHuman = 999;
    }

  };

  EmpathicBehaviour(){};
  ~EmpathicBehaviour(){};
  void init(ros::NodeHandle nodeHandle, string dbhost, string user, string password, string database);
  int ISeeYouSeeingMe(Pose robPoseInRobotFrame, Pose humPoseInRobotFrame);
  void BaseMovement(float distance, float deltaTheta, int Flag_IsRobotFree);
  void TorsoMovement(float deltaTheta, int torsoTargetDirection, int Flag_IsRobotFree);
  void ResetRobotInitInt(float deltaTheta);
  void SetRobotInitInt(float deltaTheta);
  void SelectHumansTrackersData();

  void OmniCamBasedHumansTrackerCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg);
  void LaserBasedHumansTrackerCallback(const cob_leg_detection::TrackedHumans::ConstPtr& msg);

  float calRobotRotationAngle(Pose robPose, Pose humPose)
  {

    float d_y = humPose.y - robPose.y;
    float d_x = humPose.x - robPose.x;
    float theta_tar = atan2(d_y, d_x);

    //std::cout<<radian2degree(theta_tar)<<endl;

    return theta_tar;
  }
  float calDistanceToHuman(Pose robPose, Pose humPose)
  {

    float d_y = humPose.y - robPose.y;
    float d_x = humPose.x - robPose.x;

    return sqrt((d_x * d_x) + (d_y * d_y));
  }
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
    string expValue = "1"; //lock or free
    string actualValue;
    bool status = false;

    Driver *driver;
    Connection *con;
    Statement *stmt;
    ResultSet *result;

    driver = get_driver_instance();
    //con = driver->connect("tcp://localhost:3306", "rhUser", "waterloo"); // create a database connection using the Driver
    con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
    con->setAutoCommit(0); // turn off the autocommit
    //con->setSchema("Accompany"); // select appropriate database schema
    con->setSchema(DATABASE); // select appropriate database schema
    stmt = con->createStatement(); // create a statement object

    //determine if the robot is free
    sql = "SELECT value from Accompany.Sensors where sensorId = 1001";
    //cout << sql << endl;
    result = stmt->executeQuery(sql);

    if (result->next())
    {
      actualValue = result->getString("value");

      if (!expValue.compare(actualValue)) //1 is free
      {
        //cout<<"Robot is Free"<<endl;
        status = true;
      }
      else
      {
        //cout<<"Robot is Not Free"<<endl;
        status = false;
      }
    }
    else
    {
      cout << "Error with the following statement: " << sql << endl;
      status = false;
    }

    delete result;
    delete stmt;
    con->close();
    delete con;

    return status;
  }

  Pose getRobotPoseInRobotFrame()
  {
    return robotPoseInRobotFrame;
  }

  Pose getTrackedHumanPoseInRobotFrame()
  {
    return trackedHumanPoseInRobotFrame;
  }

private:
  ros::NodeHandle nh;

  string DBHOST;
  string USER;
  string PASSWORD;
  string DATABASE;


  //tf::TransformListener transformListener;

  std::vector<ros::Time> timer_RobotInitInt;
  int update_timer;

  tf::TransformListener *ptrListener;

  UH_CobLightControllerClient lightControllerClient;
  UH_CobTorsoControllerClient torsoControllerClient;
  UH_CobBaseControllerClient baseControllerClient;

  ros::Subscriber trackedHumansSubscriber;
  ros::Subscriber trackedHumansLaserSubscriber;

  Pose robotPoseInRobotFrame; //egocentric
  Pose trackedHumanPoseInRobotFrame;

  float robotRadius;          //need to manually change this variable based on the robot
  float personRadius;
  float personalDistance; //meter
  float robotInteractionZone;

  TrackedData cameraTrackedData;
  TrackedData laserTrackedData;
  ros::Time lastDataProcessed;

  ros::Time lastDatabaseUpdateTime;

  int lightActivated; //flag to set the robot led to green when its facing the user
  int torsoDirection; //user location relative to the based_link frame i.e. +ve is at left side, -ve is at the right side
  std_msgs::ColorRGBA blue, red, yellow, green, white;
};

void EmpathicBehaviour::init(ros::NodeHandle nodeHandle, string dbhost, string user, string password, string database)
{

  nh = nodeHandle;

  // stored the database information
  DBHOST = dbhost;
  USER = user;
  PASSWORD = password;
  DATABASE = database;

  lightControllerClient.init();
  torsoControllerClient.init();
  baseControllerClient.init();

  //subscribe to omni-cam based humans tracker
  trackedHumansSubscriber = nh.subscribe("/trackedHumans", 1, &EmpathicBehaviour::OmniCamBasedHumansTrackerCallback,
                                         this); // stored only the most recent
  //subscribe laser based humans tracker
  trackedHumansLaserSubscriber = nh.subscribe("/leg_detection/detected_humans_laser", 1,
                                              &EmpathicBehaviour::LaserBasedHumansTrackerCallback, this); // stored only the most recent

  ptrListener = new tf::TransformListener(); //Transform listener for transforming user's coordinate to robot's base_link frame

  //Initialisation
  robotPoseInRobotFrame.x = 0.0;
  robotPoseInRobotFrame.y = 0.0;
  robotPoseInRobotFrame.orientation = 0.0;

  trackedHumanPoseInRobotFrame.x = 0.0;
  trackedHumanPoseInRobotFrame.y = 0.0;
  trackedHumanPoseInRobotFrame.orientation = 0.0;

  robotRadius = 0.55;          //need to manually change this variable based on the robot
  personRadius = 0.2;
  personalDistance = 1.2; //meter
  robotInteractionZone = personalDistance + (personRadius + robotRadius);

  lightActivated = 0;
  torsoDirection = 0;

  lastDataProcessed = ros::Time::now();

  lastDatabaseUpdateTime = ros::Time::now();

  //Define LED colour
  blue.r = 0; blue.g = 0; blue.b = 1; blue.a = 1;
  red.r = 1; red.g = 0; red.b = 0; red.a = 1;
  green.r = 0; green.g = 1; green.b = 0; green.a = 1;
  yellow.r = 0.4; yellow.g = 1; yellow.b = 0; yellow.a = 1;
  white.r = 0.3; white.g = 1; white.b = 0.3; white.a = 1;
}

int EmpathicBehaviour::ISeeYouSeeingMe(Pose robPoseInRobotFrame, Pose humPoseInRobotFrame)
{
  float deltaTheta = 0;
  float distance = 9999;
  int torsoTargetDirection = 0;

  geometry_msgs::Twist cmd;
  cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

  deltaTheta = calRobotRotationAngle(robPoseInRobotFrame, humPoseInRobotFrame);
  distance = calDistanceToHuman(robPoseInRobotFrame, humPoseInRobotFrame);

  torsoTargetDirection = (int)(sqrt(deltaTheta * deltaTheta) / deltaTheta); //get the direction in +1 or -1

  bool Flag_IsRobotFree = isRobotFree(); //check if scheduler has releasing control

  TorsoMovement(deltaTheta, torsoTargetDirection, Flag_IsRobotFree);

  BaseMovement(distance, deltaTheta, Flag_IsRobotFree);

  SetRobotInitInt(deltaTheta);

  return 1;
}

void EmpathicBehaviour::BaseMovement(float distance, float deltaTheta, int Flag_IsRobotFree)
{
  //turns toward the user when the user enter the robot's social zone and is more than 10 degree off the robot's heading and it is safe for the robot to do so.
  if ((distance >= 0.8) && (radian2degree(sqrt(deltaTheta * deltaTheta)) >= 5) && Flag_IsRobotFree)
  {
    if (lightActivated == 0)
    {
      //lightControllerClient.setLight(red, 2);
      lightActivated = 1;
    }
    baseControllerClient.changeRobotHeading(deltaTheta);
  }
  else if (((distance < 0.8) || (radian2degree(sqrt(deltaTheta * deltaTheta)) < 5)) && (lightActivated == 1))
  {
    lightControllerClient.setLight(green, 1);
    lightActivated = 0;
    baseControllerClient.changeRobotHeading(0);
  }
}

void EmpathicBehaviour::TorsoMovement(float deltaTheta, int torsoTargetDirection, int Flag_IsRobotFree)
{

  if ((radian2degree(sqrt(deltaTheta * deltaTheta)) > 5) && (torsoDirection != torsoTargetDirection) && Flag_IsRobotFree)
  { //turn the torso to the torsoTargetDirection
    ROS_INFO("Sending Torso command... %f", deltaTheta); //if the torso is not facing the user then move
    torsoControllerClient.sendGoal(0, -1 * torsoTargetDirection * 0.4, 0); //torso -ve is to the left
    torsoDirection = torsoTargetDirection;
  }
  else if ( ((radian2degree(sqrt(deltaTheta * deltaTheta)) <= 5) && (torsoDirection != 0)) && Flag_IsRobotFree )
  { // if user is in front of the robot, turn the torso to the front
    torsoTargetDirection = 0;
    torsoControllerClient.sendGoal(0.0, torsoTargetDirection, 0.0);
    torsoDirection = torsoTargetDirection;
  }

}

void EmpathicBehaviour::ResetRobotInitInt(float deltaTheta)
{
  //user is outside of social space, initialise variable
  if (radian2degree(sqrt(deltaTheta * deltaTheta)) > 5)
  {
    timer_RobotInitInt.clear(); //reset the moving window
    Driver *driver;
    Connection *con;
    Statement *stmt;
    string sql;

    driver = get_driver_instance();
    con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
    con->setAutoCommit(0); // turn off the autocommit
    con->setSchema(DATABASE); // select appropriate database schema
    stmt = con->createStatement(); // create a statement object

    //MYSQL entry to database
    //register the user is not interested to interact with the robot
    ROS_INFO("set Robot Initiate Interaction Flag 0");
    sql = "UPDATE Sensors SET value = '0' WHERE sensorId = 1002";
    stmt->executeUpdate(sql);

    delete stmt;
    con->close();
    delete con;
    lastDatabaseUpdateTime = ros::Time::now();
  }
}


void EmpathicBehaviour::SetRobotInitInt(float deltaTheta)
{
  int update_timer = ros::Time::now().toSec() - lastDatabaseUpdateTime.toSec();

  //if user is robot's in social space, store current time
  if ((radian2degree(sqrt(deltaTheta * deltaTheta)) <= 5))
  {
    timer_RobotInitInt.push_back(ros::Time::now()); //store current time in the moving window

    if (timer_RobotInitInt.size() > 2)
    {
      if (((timer_RobotInitInt.back().toSec() - timer_RobotInitInt.front().toSec()) > 2) && (update_timer > 1))
      {
        Driver *driver;
        Connection *con;
        Statement *stmt;
        string sql;

        driver = get_driver_instance();
        con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
        con->setAutoCommit(0); // turn off the autocommit
        con->setSchema(DATABASE); // select appropriate database schema
        stmt = con->createStatement(); // create a statement object

        //TODO::MYSQL entry to database
        ROS_INFO("set Robot Initiate Interaction Flag 1");
        //register the user is interested to interact with the robot
        sql = "UPDATE Sensors SET value = '1' WHERE sensorId = 1002";
        stmt->executeUpdate(sql);

        delete stmt;
        con->close();
        delete con;

        lastDatabaseUpdateTime = ros::Time::now();

        //delete all the old data that is more than 2 sec.
        while ((timer_RobotInitInt.back().toSec() - timer_RobotInitInt.front().toSec()) > 2)
        {
          timer_RobotInitInt.erase(timer_RobotInitInt.begin());
          //cout << "Delete entry that are over 2sec." << "size is " << timer_RobotInitInt.size() << endl;
        }
      }
    }
  }

}

void EmpathicBehaviour::SelectHumansTrackersData()
{ //TODO:: Only process laser data that its less or equal than 1.5m
          //else use vision data if its between 1.5 and 3.6 m

  if (laserTrackedData.dataFlag == 1)
  {
    trackedHumanPoseInRobotFrame = laserTrackedData.pose; //retrieve the latest data
    laserTrackedData.dataFlag = 0;
    cameraTrackedData.dataFlag = 0;
    lastDataProcessed = ros::Time::now() + ros::Duration(0.2); //stop the robot after 0.2 sec. if no data is available
    ISeeYouSeeingMe(getRobotPoseInRobotFrame(), getTrackedHumanPoseInRobotFrame()); //perform the behaviour based on the retrieve data
  }
  else if (cameraTrackedData.dataFlag == 1)
  {
    trackedHumanPoseInRobotFrame = cameraTrackedData.pose;
    laserTrackedData.dataFlag = 0;
    cameraTrackedData.dataFlag = 0;
    lastDataProcessed = ros::Time::now() + ros::Duration(0.2);
    ISeeYouSeeingMe(getRobotPoseInRobotFrame(), getTrackedHumanPoseInRobotFrame());
  }
  else if (ros::Time::now() > lastDataProcessed) //Using vel to drive the robot, therefore need to stop the robot else it keeps going after reaching the target.
  {
    trackedHumanPoseInRobotFrame = robotPoseInRobotFrame; // Therefore if no data was detected for more than 3 second, we setup the robot stop moving, TODO:: turn the robot back to its old (prior to tracking) pose.
    lastDataProcessed = ros::Time::now() + ros::Duration(7 * 24 * 60 * 60); //The robot will not run this loop again until new data is available.
    cout << "In Process Tracker - stop if " << endl;

    ResetRobotInitInt(180);  //reset robot initiated interaction since user is not being detected
    TorsoMovement(0, 1, isRobotFree());


  }

}

void EmpathicBehaviour::LaserBasedHumansTrackerCallback(const cob_leg_detection::TrackedHumans::ConstPtr& msg)
{
  geometry_msgs::PointStamped userPointInBaseLink;
  float min_dist = 10; //10meter
  float tempDistance = 10;
  laserTrackedData.dataFlag = 0;

  Pose userPoseInRobotFrame(0, 0, 0);
  Pose closestUserPoseInRobotFrame(0, 0, 0);

  try
  { // transform to robot egocentric coordinate frame (i.e. base_link)
    if (msg->trackedHumans.size() > 0)
    {

      //wait for transform from /base_link to /map_frame to be available
      if (ptrListener->waitForTransform("/base_link", msg->trackedHumans[0].location.header.frame_id,
                                        msg->trackedHumans[0].location.header.stamp, ros::Duration(5.0)))
      {
        //Select the closest target to the robot
        for (unsigned int i = 0; i < msg->trackedHumans.size(); ++i)
        {
          //determine the tracked person's coordinate in /base_link frame
          ptrListener->transformPoint("/base_link", msg->trackedHumans[i].location, userPointInBaseLink);

          userPoseInRobotFrame.x = userPointInBaseLink.point.x;
          userPoseInRobotFrame.y = userPointInBaseLink.point.y;
          userPoseInRobotFrame.orientation = 0;

          tempDistance = calDistanceToHuman(robotPoseInRobotFrame, userPoseInRobotFrame);

          if (tempDistance < min_dist) //Stored the coordinate of the closest person
          {
            min_dist = tempDistance;
            closestUserPoseInRobotFrame = userPoseInRobotFrame;
          }
        }

        if (min_dist <= robotInteractionZone) //only use the closest tracked person if they are 1.8m or less from the robot
        {
          laserTrackedData.pose = closestUserPoseInRobotFrame;
          laserTrackedData.distanceToHuman = min_dist;
          laserTrackedData.dataFlag = 1;

          //cout<<"Distance from robot is "<<laserTrackedData.distanceToHuman <<endl;
        }
      }
    }
  }
  catch (tf::TransformException e)
  {
    ROS_ERROR(
        "Received an exception trying to transform laserTracker person data from \"camera_frame\" to \"base_link\" : %s",
        e.what());
  }
}

void EmpathicBehaviour::OmniCamBasedHumansTrackerCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg)
{
  geometry_msgs::PointStamped userPointInBaseLink;

  //Search and processed the most reliable data of the detected human
  for (unsigned int i = 0; i < msg->trackedHumans.size(); ++i)
  {
    if (msg->trackedHumans[i].specialFlag == 1) //specialFlag indicates the most reliable tracked person
    {
      try
      {
        //wait for transform from /base_link to /camera_frame to be available
        if (ptrListener->waitForTransform("/base_link", msg->trackedHumans[i].location.header.frame_id,
                                          msg->trackedHumans[i].location.header.stamp, ros::Duration(5.0)))
        {
          //determine the tracked person's coordinate in /base_link frame
          ptrListener->transformPoint("/base_link", msg->trackedHumans[i].location, userPointInBaseLink);

          Pose userPoseInRobotFrame(userPointInBaseLink.point.x, userPointInBaseLink.point.y, 0);

          cameraTrackedData.distanceToHuman = calDistanceToHuman(robotPoseInRobotFrame, userPoseInRobotFrame);

          //if ((cameraTrackedData.distanceToHuman > 1.8) && (cameraTrackedData.distanceToHuman <= 3.6)) //if user is in the robot's social zone
          if (cameraTrackedData.distanceToHuman <= robotInteractionZone) //if user is in the robot's social zone
          {
            cameraTrackedData.pose = userPoseInRobotFrame;
            cameraTrackedData.dataFlag = 1;
            //cout<<"Distance from robot is "<<cameraTrackedData.distanceToHuman <<endl;
          }
        }
        else
          cout << "Make sure both publishers' PC time are synchronise" << endl;
      }
      catch (tf::TransformException e)
      {
        ROS_ERROR(
            "Received an exception trying to transform cameraTracker person data from \"camera_frame\" to \"base_link\" : %s",
            e.what());
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EmpathicBehaviour");
  ros::NodeHandle nodeHandle;

  //Declare MySQL server variables that can be modified by launch file or command line.
  std::string DBHOST;
  std::string USER;
  std::string PASSWORD;
  std::string DATABASE;

  //Initialize node parameters from launch file or command line.
  //Use a private node handle so hat multiple instances of the node can be run simultaneously while using different parameters.
  ros::NodeHandle private_node_handle_("~");

  //Default MySQL setting for local host
  private_node_handle_.param("DBHOST",          DBHOST,         string("tcp://localhost:3306"));
  private_node_handle_.param("USER",            USER,           string("rhUser"));
  private_node_handle_.param("PASSWORD",        PASSWORD,       string("waterloo"));
  private_node_handle_.param("DATABASE",        DATABASE,       string("Accompany"));

  EmpathicBehaviour myEmpathicBehaviour;
  myEmpathicBehaviour.init(nodeHandle, DBHOST, USER, PASSWORD, DATABASE);

  //  robotInteractionZone = personalDistance + (personRadius + robotRadius);

  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    myEmpathicBehaviour.SelectHumansTrackersData();
    r.sleep();
  }

  ros::spin();

  return 0;
}
