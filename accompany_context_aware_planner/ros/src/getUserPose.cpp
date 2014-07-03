#include "ros/ros.h"
#include "std_msgs/String.h"
#include <accompany_uva_msg/TrackedHumans.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

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

tf::TransformListener *ptrListener=NULL;

using namespace std;
using namespace sql;

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


//Calculate the angle for the robot to rotate
//Check if robot is free / safe to move
//moves the robot

void chatterCallback(const accompany_uva_msg::TrackedHumans::ConstPtr& msg)
{
  geometry_msgs::PointStamped newLocation;

  ROS_INFO("\n Size of array is %d ",msg->trackedHumans.size());

  for (int i=0; i< msg->trackedHumans.size(); ++i)
  {
    ROS_INFO(" Special flag is %d",msg->trackedHumans[i].specialFlag);

    if (msg->trackedHumans[i].specialFlag == 1)
    {
      ROS_INFO("Frame id is %s",msg->trackedHumans[i].location.header.frame_id.c_str());

      try// transform to map coordinate system
      {
        ptrListener->waitForTransform("/map",
                                      msg->trackedHumans[i].location.header.frame_id,
                                      ros::Time::now(),
                                      ros::Duration(10.0));

        ptrListener->transformPoint("/map",
                                msg->trackedHumans[i].location,
                                newLocation);

        ROS_INFO("Output x=%f , y=%f ",newLocation.point.x, newLocation.point.y);
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

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ptrListener = new tf::TransformListener();

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  isRobotFree();
  //ros::Subscriber sub = n.subscribe("/trackedHumans", 1000, chatterCallback);




  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
