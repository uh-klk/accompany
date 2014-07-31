#include "accompany_context_aware_planner/CobLightControllerClient.h"
#include "accompany_context_aware_planner/CobTorsoControllerClient.h"
#include "accompany_context_aware_planner/CobBaseControllerClient.h"

#include <accompany_uva_msg/TrackedHumans.h>

#include <cob_leg_detection/TrackedHumans.h>

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
    bool distanceToHuman;
    Pose pose;

    TrackedData()
    {
      dataFlag = 0;
      distanceToHuman = 999;
    }

  };

  EmpathicBehaviour(ros::NodeHandle n):nh(n){}
  void init();
  int ISeeYouSeeingMe(Pose robPoseInRobotFrame, Pose humPoseInRobotFrame);
  void HumansTrackerCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg);
  void HumansTrackerLaserCallback(const cob_leg_detection::TrackedHumans::ConstPtr& msg);

  void ProcessHumansTrackersData()
  { //check if laser got data and Laser detected people that is less or equal than 1.5m
    //else check for vision got data and detected people between 1.5 and 3.6 m

    if (laserTrackedData.dataFlag == 1) {
       trackedHumanPoseInRobotFrame = laserTrackedData.pose;
       laserTrackedData.dataFlag = 0;
       cameraTrackedData.dataFlag = 0;
       lastDataProcessed = ros::Time::now() + ros::Duration(3);
     }
     else if (cameraTrackedData.dataFlag == 1) {
       trackedHumanPoseInRobotFrame = cameraTrackedData.pose;
       laserTrackedData.dataFlag = 0;
       cameraTrackedData.dataFlag = 0;
       lastDataProcessed = ros::Time::now() + ros::Duration(3);
     }
     else if (ros::Time::now() > lastDataProcessed ){ //Using vel to drive the robot, therefore need to stop the robot else it keeps going after reaching the target.
       trackedHumanPoseInRobotFrame = robotPoseInRobotFrame; // Therefore if no data was detected for more than 3 second, we setup the robot stop moving, TODO:: turn the robot back to its old (prior to tracking) pose.
       lastDataProcessed = ros::Time::now() + ros::Duration(7*24*60*60); //The robot will not run this loop again until it has been reactivated.
       cout<<"In Process Tracker - stop if "<<endl;
     }
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

  Pose getRobotPoseInRobotFrame() {return robotPoseInRobotFrame;}

  Pose getTrackedHumanPoseInRobotFrame(){return trackedHumanPoseInRobotFrame;}

private:
    ros::NodeHandle nh;

    //tf::TransformListener transformListener;


    tf::TransformListener *ptrListener;

    UH_CobLightControllerClient lightControllerClient;
    UH_CobTorsoControllerClient torsoControllerClient;
    UH_CobBaseControllerClient baseControllerClient;

    ros::Subscriber trackedHumansSubscriber;
    ros::Subscriber trackedHumansLaserSubscriber;

    Pose robotPoseInRobotFrame; //egocentric
    Pose trackedHumanPoseInRobotFrame;

    TrackedData cameraTrackedData;
    TrackedData laserTrackedData;
    ros::Time lastDataProcessed;

    int lightActivated; //flag to set the robot led to green when its facing the user
    int torsoDirection;      //user location relative to the based_link frame i.e. +ve is at left side, -ve is at the right side
    std_msgs::ColorRGBA blue, red, yellow, green, white;
};

void EmpathicBehaviour::init()
{
  lightControllerClient.init();
  torsoControllerClient.init();
  baseControllerClient.init();

  //omni-cam based humans tracker
  trackedHumansSubscriber = nh.subscribe("/trackedHumans", 1, &EmpathicBehaviour::HumansTrackerCallback, this); // stored only the most recent
  //laser based humans tracker
  trackedHumansLaserSubscriber = nh.subscribe("/leg_detection/detected_humans_laser", 1, &EmpathicBehaviour::HumansTrackerLaserCallback, this); // stored only the most recent

  ptrListener = new tf::TransformListener(); //Transform listerner for transforming user coordinate to robot's base_link

  robotPoseInRobotFrame.x = 0.0;
  robotPoseInRobotFrame.y = 0.0;
  robotPoseInRobotFrame.orientation = 0.0;

  trackedHumanPoseInRobotFrame.x = 0.0;
  trackedHumanPoseInRobotFrame.y = 0.0;
  trackedHumanPoseInRobotFrame.orientation = 0.0;

  lightActivated = 0;
  torsoDirection = 0;

  lastDataProcessed = ros::Time::now();

  blue.r =0;     blue.g =0;    blue.b =1;      blue.a = 1;
  red.r =1;      red.g =0;     red.b =0;       red.a = 1;
  green.r=0;   green.g =1;     green.b =0;     green.a=1;
  yellow.r =0.4; yellow.g =1;  yellow.b =0;    yellow.a =1;
  white.r =0.3;  white.g =1;   white.b =0.3;   white.a =1;
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
  cout<<"Distance = "<< distance <<", Angle = "<< radian2degree(deltaTheta) <<endl;

  torsoTargetDirection = (int) (sqrt(deltaTheta*deltaTheta)/deltaTheta);

  //Torso
  if ((distance <= 3.6) &&
      (radian2degree(sqrt(deltaTheta*deltaTheta)) > 5) &&
      (torsoDirection != torsoTargetDirection) &&
      isRobotFree())
  {     //turn the torso to the torsoTargetDirection
    ROS_INFO("Sending Torso command... %f", deltaTheta); //if the torso is not facing the user then move
    //torsoControllerClient.cancelAllPreviousGoals();
    torsoControllerClient.sendGoal(0, -1*torsoTargetDirection*0.2, 0); //torso -ve is to the left
    torsoDirection = torsoTargetDirection;
  }
  else if (((distance>3.6) || (radian2degree(sqrt(deltaTheta*deltaTheta))<= 5) ) &&
           (torsoDirection != 0) &&
           isRobotFree())
  { // if user is outside social zone or infront of the robot, turn the torso back to the front
      torsoTargetDirection = 0;
      //torsoControllerClient.cancelAllPreviousGoals();
      torsoControllerClient.sendGoal(0.0,0.0,0.0);
      torsoDirection = torsoTargetDirection;
  }


  //turns toward the user when the user enter the robot's social zone and is more than 10 degree off the robot's heading and it is safe for the robot to do so.
  if ( (distance <= 3.6) && (distance >= 0.8) &&
       (radian2degree(sqrt(deltaTheta*deltaTheta)) >= 18) && isRobotFree())
  {
    if (lightActivated == 0)
    {
      lightControllerClient.setLight(red,2);
      lightActivated = 1;
    }
    baseControllerClient.changeRobotHeading(deltaTheta);
    ROS_INFO("ROBOT IS MOVING");
  }
  else if ( ((distance > 3.6) || (distance < 0.8) || (radian2degree(sqrt(deltaTheta*deltaTheta)) <  10)) && (lightActivated == 1) )
  {
    lightControllerClient.setLight(green,1);
    lightActivated = 0;

    baseControllerClient.changeRobotHeading(0);
    cout<<"STOP - Target Reached/You are standing too close to me/You are out of my social space! -I SEE YOU SEEING ME"<<endl;
  }

  return 1;
}

// /leg_detection/detected_humans_laser  cob_leg_detection/TrackedHumans

void EmpathicBehaviour::HumansTrackerLaserCallback(const cob_leg_detection::TrackedHumans::ConstPtr& msg)
{
  geometry_msgs::PointStamped userPointInBaseLink;
  float min_dist = 10; //10meter
  float tempDistance = 10;
  laserTrackedData.dataFlag = 0;

  Pose userPoseInRobotFrame(0, 0, 0);
  Pose closestUserPoseInRobotFrame(0, 0, 0);

  try { // transform to robot egocentric coordinate frame (i.e. base_link)
    //wait for /base_link to /map_frame transform to be available for the specific time stamped
    if (msg->trackedHumans.size() > 0) {
      if (ptrListener->waitForTransform("/base_link",
                                        msg->trackedHumans[0].location.header.frame_id,
                                        msg->trackedHumans[0].location.header.stamp,
                                        ros::Duration(5.0)))
      {
        for (unsigned int i = 0; i< msg->trackedHumans.size(); ++i)
        {
          //transform the tracked human's coordinate from /map_frame to /base_link frame
          ptrListener->transformPoint("/base_link",
                                      msg->trackedHumans[i].location,
                                      userPointInBaseLink);
          userPoseInRobotFrame.x = userPointInBaseLink.point.x;
          userPoseInRobotFrame.y = userPointInBaseLink.point.y;
          userPoseInRobotFrame.orientation = 0;

          tempDistance = calDistanceToHuman(robotPoseInRobotFrame, userPoseInRobotFrame);

          //find the closest detected person
          if (tempDistance < min_dist)
          {
            min_dist = tempDistance;
            closestUserPoseInRobotFrame = userPoseInRobotFrame;
          }
        }

        if (min_dist <= 1.8)  //and only use the closest laser tracked person if the person is less than 1.8m from the robot
        {
          laserTrackedData.pose = closestUserPoseInRobotFrame;
          laserTrackedData.distanceToHuman = min_dist;
          laserTrackedData.dataFlag = 1;
        }
      }
    }
  }
  catch (tf::TransformException e) {
    ROS_ERROR("Received an exception trying to transform user location from \"camera_frame\" to \"map\" : %s", e.what());
  }
}

void EmpathicBehaviour::HumansTrackerCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg)
{
  geometry_msgs::PointStamped userPointInBaseLink;

  for (unsigned int i=0; i< msg->trackedHumans.size(); ++i)
  {
    if (msg->trackedHumans[i].specialFlag == 1) //specialFlag indicate the most reliable tracked person.
    {
      try // transform to robot egocentric coordinate frame (i.e. base_link)
      {
        //wait for /base_link to /camera_frame transform to be available for the specific time stamped
        if (ptrListener->waitForTransform("/base_link",
                                          msg->trackedHumans[i].location.header.frame_id,
                                          msg->trackedHumans[i].location.header.stamp,
                                          ros::Duration(5.0)))
        {
          //transform the tracked human's coordinate from /camera_frame to /base_link frame
          ptrListener->transformPoint("/base_link",
                                      msg->trackedHumans[i].location,
                                      userPointInBaseLink);
          //ROS_INFO("humansTrackerCallback_ after transform");
          //ROS_INFO("User is at (%f, %f) of the robot coordinate frame ",userPointInMap.point.x, userPointInMap.point.y);

          Pose userPoseInRobotFrame(userPointInBaseLink.point.x, userPointInBaseLink.point.y, 0);

          cameraTrackedData.pose = userPoseInRobotFrame;
          cameraTrackedData.distanceToHuman = calDistanceToHuman(robotPoseInRobotFrame, userPoseInRobotFrame);
          cameraTrackedData.dataFlag = 1;
          //iSeeYouSeeingMe(robotPoseInRobotFrame,userPoseInRobotFrame) ;//todo: create a function to processs both data and run this if its >1.5
        }
      }
      catch (tf::TransformException e)
      {
        ROS_ERROR("Received an exception trying to transform user location from \"camera_frame\" to \"base_link\" : %s", e.what());
      }
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "EmpathicBehaviour");
  ros::NodeHandle n;

  EmpathicBehaviour myEmpathicBehaviour(n);

  myEmpathicBehaviour.init();

  ros::Rate r(100);
  while (ros::ok()) {

    ros::spinOnce();
    myEmpathicBehaviour.ProcessHumansTrackersData();
    myEmpathicBehaviour.ISeeYouSeeingMe(myEmpathicBehaviour.getRobotPoseInRobotFrame(), myEmpathicBehaviour.getTrackedHumanPoseInRobotFrame());
    r.sleep();
  }


  ros::spin();

  return 0;
}
