#include "accompany_context_aware_planner/CobLightControllerClient.h"
#include "accompany_context_aware_planner/CobTorsoControllerClient.h"
#include "accompany_context_aware_planner/CobBaseControllerClient.h"

#include <accompany_uva_msg/TrackedHumans.h>

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
  EmpathicBehaviour(ros::NodeHandle n):nh(n){}
  void init();
  int iSeeYouSeeingMe(Pose robPoseInRobotFrame, Pose humPoseInRobotFrame);
  void humansTrackerCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg);
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

private:
    ros::NodeHandle nh;

    //tf::TransformListener transformListener;


    tf::TransformListener *ptrListener;

    UH_CobLightControllerClient lightControllerClient;
    UH_CobTorsoControllerClient torsoControllerClient;
    UH_CobBaseControllerClient baseControllerClient;

    int lightActivated;

    int sign;

    ros::Subscriber trackedHumansSubscriber;
};

void EmpathicBehaviour::init()
{
  lightControllerClient.init();
  torsoControllerClient.init();
  baseControllerClient.init();
  sign = 0;
  trackedHumansSubscriber = nh.subscribe("/trackedHumans", 1, &EmpathicBehaviour::humansTrackerCallback, this); // stored only the most recent

  ptrListener = new tf::TransformListener();

  lightActivated = 0;
}

int EmpathicBehaviour::iSeeYouSeeingMe(Pose robPoseInRobotFrame, Pose humPoseInRobotFrame)
{
  float deltaTheta = 0;
  float distance = 9999;

  geometry_msgs::Twist cmd;

  std_msgs::ColorRGBA blue, red, yellow, green, white;

  blue.r =0;     blue.g =0;    blue.b =1;      blue.a = 1;
  red.r =1;      red.g =0;     red.b =0;       red.a = 1;
  green.r=0;   green.g =1;     green.b =0;     green.a=1;
  yellow.r =0.4; yellow.g =1;  yellow.b =0;    yellow.a =1;
  white.r =0.3;  white.g =1;   white.b =0.3;   white.a =1;

  cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

  deltaTheta = calRobotRotationAngle(robPoseInRobotFrame, humPoseInRobotFrame);
  distance = calDistanceToHuman(robPoseInRobotFrame, humPoseInRobotFrame);
  cout<<"Distance = "<< distance <<", Angle = "<< radian2degree(deltaTheta) <<endl;

  //turns toward the user when the user entered the robot's social zone and he is more than 10 degree off the robot's heading and when it is safe for the robot to do so.
  if ( (distance <= 3.6) && (distance >=0.5) && ( radian2degree(sqrt(deltaTheta*deltaTheta) ) >= 30)  && isRobotFree())
  {
    if (lightActivated == 0)
    {
      lightActivated = 1;
      lightControllerClient.setLight(red,2);
    }
    baseControllerClient.changeRobotHeading(deltaTheta);
  }
  else
  {
    if (lightActivated == 1) //future: wait for robot to stop then publish green LED
    {
      lightControllerClient.setLight(green,1);
      lightActivated = 0;
    }
    baseControllerClient.changeRobotHeading(0);
    cout<<"STOP - Target Reached!"<<endl;
  }

  //Torso
  if ( (distance <= 3.6)  && ( radian2degree(sqrt(deltaTheta*deltaTheta) ) < 30) && ( radian2degree(sqrt(deltaTheta*deltaTheta) ) > 5) && isRobotFree() )
  {
    ROS_INFO("sending torso command %f", deltaTheta);
    if (sign == 0) {
        sign = sqrt(deltaTheta*deltaTheta)/deltaTheta;
        torsoControllerClient.sendGoal(0,-1*0.15*sign, 0);
    }
    else if (sign != (sqrt(deltaTheta*deltaTheta)/deltaTheta)) {
        sign = (sqrt(deltaTheta*deltaTheta)/deltaTheta);
        torsoControllerClient.sendGoal(0,-1*0.15*sign,0); //torso -ve is to the left
    }

  }
  else if ( (distance <= 3.6)  && ( radian2degree(sqrt(deltaTheta*deltaTheta) ) <= 5) && ( radian2degree(sqrt(deltaTheta*deltaTheta) ) >= 0) && isRobotFree() ) {
    if (sign != 0) {
      sign = 0;
      torsoControllerClient.sendGoal(0.0,0.0,0.0);
    }

  }


  return 1;

}

void EmpathicBehaviour::humansTrackerCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg)
{
  geometry_msgs::PointStamped userPointInMap;

  for (int i=0; i< msg->trackedHumans.size(); ++i)
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
                                      userPointInMap);
          //ROS_INFO("humansTrackerCallback_ after transform");
          //ROS_INFO("User is at (%f, %f) of the robot coordinate frame ",userPointInMap.point.x, userPointInMap.point.y);

          Pose userPoseInRobotFrame(userPointInMap.point.x, userPointInMap.point.y, 0);
          Pose robotPoseInRobotFrame(0,0,0); //egocentric

          iSeeYouSeeingMe(robotPoseInRobotFrame,userPoseInRobotFrame);
        }
        else
          ROS_INFO("No suitable transform from base_link to camera_frame can be found.");
      }
      catch (tf::TransformException e)
      {
        ROS_ERROR("Received an exception trying to transform user location from \"camera_frame\" to \"map\" : %s", e.what());

        if (lightActivated == 1) //future: wait for robot to stop then publish green LED
        {
          baseControllerClient.changeRobotHeading(0);
          std_msgs::ColorRGBA green;
          green.r=0;   green.g =1;     green.b =0;     green.a=1;
          lightControllerClient.setLight(green,1);
          lightActivated = 0;
         }
         cout<<"STOPPING THE ROBOT!"<<endl;
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


  ros::spin();
  return 0;
}
