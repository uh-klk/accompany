#include "accompany_context_aware_planner/proxemics.h"

template<class T>
  T Proxemics::convertFromMeterToPixelCoordinates(const Pose& pose)
  {
    T val;
    val.x = (pose.x - map_origin_.x) / map_resolution_;
    val.y = (pose.y - map_origin_.y) / map_resolution_;
    return val;
  }

template<class T>
  string Proxemics::to_string(const T& t)
  {
    stringstream ss;
    ss << t;
    return ss.str();
  }

float Proxemics::degree2radian(float degree)
{
  float radian;
  float pi = 4.0 * std::atan2(1.0, 1.0);

  return radian = pi * degree / 180.0;
}

float Proxemics::radian2degree(float radian)
{
  float degree;
  float pi = 4.0 * std::atan2(1.0, 1.0);

  return degree = 180 * radian / pi;
}

/*
 * Temp function for Y1 context aware proxemics i.e. sofa position
 */
void Proxemics::getPotentialProxemicsLocations_Sofa_Y1(void) //retrieves current user (who)
{

  string sql;
  int currentUserId;
  string currentUserPosture;
  int currentUserLocationId;

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  //con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object


  sql = "SELECT sc.SessionUser FROM SessionControl sc";
  result = stmt->executeQuery(sql);

  while (result->next())
    currentUserId = result->getInt("SessionUser");

  cout << "**********Current user id is = " << currentUserId << endl;

  sql = "SELECT P.poseType from Pose P, Users U WHERE P.poseId = U.poseId AND U.userId = ";
  sql += to_string(currentUserId);
  cout << sql << endl;
  result = stmt->executeQuery(sql);
  while (result->next())
    currentUserPosture = result->getString("poseType");

  cout << "**********Current user posture is = " << currentUserPosture << endl;

  //if user is in living room then the sensors trigger must be user's action

  sql = "SELECT L.locationId from Locations L where L.name = 'Living Room'";
  cout << sql << endl;
  result = stmt->executeQuery(sql);
  while (result->next())
    currentUserLocationId = result->getInt("locationId");
  cout << "**********Current user location id is = " << currentUserLocationId << endl;

  sql = "call spCheckUserLocation( ";
  sql += to_string(currentUserId);
  sql += ",";
  sql += to_string(currentUserLocationId);
  sql += ",";
  sql += "@res)";
  cout << sql << endl;

  if (stmt->execute(sql)) // return row count, if rowcnt >= 1 user is in Living Room
  {
    delete result;
    delete stmt;
    con -> close();
    delete con;

    //driver = get_driver_instance();
    con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
    con->setAutoCommit(0); // turn off the autocommit
    con->setSchema(DATABASE); // select appropriate database schema
    stmt = con->createStatement(); // create a statement object

    cout << "user is in living room" << endl; //check which sofa the user is sitting

    sql = "SELECT * FROM Sensors S where S.name like 'Sofa%' ";
    cout << sql << endl;
    cout << "test!!!!" << endl;

    result = stmt->executeQuery(sql);

    while (result->next())
      if ((int)result->getDouble("value")) //check if sensor is on
      {
        switch (result->getInt("sensorId"))
        {
          case 15:
            cout << "sofa 15 is occupied" << endl;
            //Pose robot1TargetPose(3.613, 1.29, degree2radian(40));
            sql = "UPDATE Locations SET Locations.xCoord = ";
            sql += to_string(3.613);
            sql += ", ";
            sql += "Locations.yCoord = ";
            sql += to_string(1.29);
            sql += ", ";
            sql += "Locations.orientation = ";
            sql += to_string(40);
            sql += " WHERE Locations.locationId = 999";
            cout << sql << endl;
            stmt->executeUpdate(sql);
            break;
          case 16:
            cout << "sofa 16 is occupied" << endl;
            //robotTargetPose(4.158, 0.952, degree2radian(49));
            sql = "UPDATE Locations SET Locations.xCoord = ";
            sql += to_string(4.158);
            sql += ", ";
            sql += "Locations.yCoord = ";
            sql += to_string(0.952);
            sql += ", ";
            sql += "Locations.orientation = ";
            sql += to_string(49);
            sql += " WHERE Locations.locationId = 999";
            cout << sql << endl;
            stmt->executeUpdate(sql);
            break;
          case 17:
            cout << "sofa 17 is occupied" << endl;
            //robotTargetPose(4.158, 0.952, degree2radian(42));
            sql = "UPDATE Locations SET Locations.xCoord = ";
            sql += to_string(4.158);
            sql += ", ";
            sql += "Locations.yCoord = ";
            sql += to_string(0.952);
            sql += ", ";
            sql += "Locations.orientation = ";
            sql += to_string(42);
            sql += " WHERE Locations.locationId = 999";
            cout << sql << endl;
            stmt->executeUpdate(sql);
            break;
          case 18:
            cout << "sofa 18 is occupied" << endl;
            //robotTargetPose(4.158, 0.952, degree2radian(7));
            sql = "UPDATE Locations SET Locations.xCoord = ";
            sql += to_string(4.158);
            sql += ", ";
            sql += "Locations.yCoord = ";
            sql += to_string(0.952);
            sql += ", ";
            sql += "Locations.orientation = ";
            sql += to_string(7);
            sql += " WHERE Locations.locationId = 999";
            cout << sql << endl;
            stmt->executeUpdate(sql);
            break;
          case 19:
            cout << "sofa 19 is occupied" << endl;
            //robotTargetPose(4.257, 0.835, degree2radian(0));
            sql = "UPDATE Locations SET Locations.xCoord = ";
            sql += to_string(4.257);
            sql += ", ";
            sql += "Locations.yCoord = ";
            sql += to_string(0.835);
            sql += ", ";
            sql += "Locations.orientation = ";
            sql += to_string(0);
            sql += " WHERE Locations.locationId = 999";
            cout << sql << endl;
            stmt->executeUpdate(sql);
            break;
          default:
            break;
        }
      }
  }
  else
    cout << "not in location";

  delete result;
  delete stmt;
  con -> close();
  delete con;

}

/******************************************************************************
 * This function return the radius of the user at the specified angle in user's frame
 * The user is modelled as an ellipse therefore have shorter radius in front than sides
 *
 *
 ******************************************************************************/
float Proxemics::getUserRadius(float thetaInRadian, float halfShoulderWidth, float halfChestDepth)
{
  float a, b, rad, userRadius;

  a = halfChestDepth; //a is x-axis in[m]
  b = halfShoulderWidth;//b is y-axis in[m]

  rad = thetaInRadian;

  //Polar form relative to center formula from see http://en.wikipedia.org/wiki/Ellipse#Polar_form_relative_to_center
  //use to calculate the radius
  userRadius = (a * b / sqrt(pow(b * cos(rad), 2) + pow(a * sin(rad), 2)));

  return userRadius;
}

/******************************************************************************
 This will be the main function for Proxemics.

 The request data from the client to determine potential proxemics locations are:
 req.header.seq      - can be used to stored id of the client and be return
 to the client for identification etc. if needed
 (needed to be verify if this is necessary)
 req.header.stamp    - time stamped when the request is created i.e. ros::Time::now()
 req.header.frame_id - the user coordinate's reference frame i.e. "map"

 req.userId          - the user's Id in the database
 req.userPosture     - the user's posture i.e. standing/seating

 //User's coordinate i.e. in map's coordinate frame in meter
 req.userPose.position.x      - the x coordinate of the user
 req.userPose.position.y      - the y coordinate of the user
 req.userPose.position.z      - the z coordinate of the user

 // User's orientation in Quaternion. the heading direction of the user can be extract
 // by using tf::getYaw(req.userPose.orientation) function which return yaw in radian
 req.userPose.orientation.x
 req.userPose.orientation.y
 req.userPose.orientation.z
 req.userPose.orientation.w

 //Type of task the robot is going to perform using this information
 req.robotGenericTaskId       - the type of task the robot is going to perform using this proxemics

 ******************************************************************************/
bool Proxemics::getPotentialProxemicsLocations(
                                               accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                                               accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res)
{
  Pose targetPose;
  Bearing proxemics_bearing;
  Bearing ranked_bearing[21];

  //test
  //getCurrentUserId();

  //1. Process the request data from the client.
  //ROS_INFO("MsgSeq = %d, time =  %li, coordinate frame = %s ",req.header.seq, static_cast<long>(ros::Time::now().toNSec()-req.header.stamp.toNSec()), req.header.frame_id.c_str());
  ROS_INFO("MsgSeq = %d, time = %2f, coordinate frame = %s ",req.header.seq, (ros::Time::now().toSec()-req.header.stamp.toSec()), req.header.frame_id.c_str());
  ROS_INFO("userId = %d, userPosture = %d", req.userId, req.userPosture);

  Pose personLocation(req.userPose.position.x, req.userPose.position.y, tf::getYaw(req.userPose.orientation));

  ROS_INFO("request proxemics for user pose: x=%f, y=%f, z=%f yaw = %f", req.userPose.position.x, req.userPose.position.y, req.userPose.position.z, tf::getYaw(req.userPose.orientation));
  ROS_INFO("robotGenericTask: %d", req.robotGenericTaskId);

  // Calculate robot pose in map coordinate frame
  //stored the robot's origin in base_link frame
  tf::Stamped<tf::Pose> robotOrigin = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(0),
                                                                     tf::Point(0.0, 0.0, 0.0)), ros::Time(0),
                                                            "base_link"); //base_link origin is the robot coordinate frame
  //create a PoseStamped variable to store the StampedPost TF
  geometry_msgs::PoseStamped map_frame; // create a map_frame to store robotOrigin in map frame
  geometry_msgs::PoseStamped base_link_frame; // create a base_link_frame to store robotOrigin in base_link frame
  tf::poseStampedTFToMsg(robotOrigin, base_link_frame); //stored the robot coordinate in base_link frame
  try
  {
    listener.transformPose("map", base_link_frame, map_frame); //listen for base_link to map transform, then transform the robot coordinate to map coordinate frame

    ROS_INFO("robot origin in base_link frame: (%.2f, %.2f. %.2f), in map frame: (%.2f, %.2f, %.2f)",
        base_link_frame.pose.position.x, base_link_frame.pose.position.y, radian2degree(tf::getYaw(base_link_frame.pose.orientation)),
        map_frame.pose.position.x, map_frame.pose.position.y, radian2degree(tf::getYaw(map_frame.pose.orientation)));
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform robot origin from \"base_link\" to \"map\": %s", ex.what());
  }

  Pose robotLocation(map_frame.pose.position.x, map_frame.pose.position.y, tf::getYaw(map_frame.pose.orientation)); // stored the current robot pose


  //2. Retreives user's preference
  proxemics_bearing = retrieveProxemicsPreferences(req.userId, req.robotGenericTaskId);



  retrieveProxemicsPreferences_ranking(req.userId, req.robotGenericTaskId, ranked_bearing);
  for (int j=0; j<21; j++)
  {
    if (ranked_bearing[j].orientation != 999)
      ROS_INFO("Ranked Bearing %d orientation is %f distance is %f",j, ranked_bearing[j].orientation, ranked_bearing[j].distance);
  }

  //To do
  //multiple poses
  //search based on distance
  //search based on orientation

//-- for loop here to calculate each bearing
  //3. Calculate all the target poses from the database, where distance and orientation can be obtain from step2
  targetPose = calculateRobotPoseFromProxemicsPreference(req.userPose, proxemics_bearing);
  // To do
  //multiple poses

  //4. Eliminate target poses that could not be occupied by the robot based on static map (i.e. too close to obstacle or on obstacle)
  //   Eliminate target poses that could not be reach by the robot based on static map
  bool validApproach = validApproachPosition(personLocation, robotLocation, targetPose);

  //5. Elimimate target poses that are not in the same location as the user (i.e. user is in living room, therefore all potential robot poses have to be in the living room for HRI)
  tf::Stamped<tf::Pose> p;
  if (validApproach == true)
  {
    p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                       tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");
    ROS_INFO("The approach position is valid.");
  }
  else
  {
    //return robot current pose as temporary solution
    p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(tf::getYaw(map_frame.pose.orientation)),
                                       tf::Point(map_frame.pose.position.x, map_frame.pose.position.y, 0.0)),
                              ros::Time::now(), "map");
    ROS_INFO("The approach position is invalid.");
  }
//---for loop end here


  //6. Compile the respond message for the client.
  geometry_msgs::PoseStamped pose; //create a PoseStamped variable to store the StampedPost TF
  tf::poseStampedTFToMsg(p, pose); //convert the PoseStamped data into message format and store in pose
  res.targetPoses.push_back(pose); //push the pose message into the respond vector to be send back to the client

  ROS_INFO("Sending request out");
  /*The respond vector data from the server are in the following format and contain a list of potential target locations.
   These potential target poses needed to be varify in the real environment to take into account of dynamic obstacles etc.

   res.targetPoses[i].seq
   res.targetPoses[i].stamp
   res.targetPoses[i].frame_id

   res.targetPoses[i].pose.x
   res.targetPoses[i].pose.y
   res.targetPoses[i].pose.z

   res.targetPoses[i].orientation.x
   res.targetPoses[i].orientation.y
   res.targetPoses[i].orientation.z
   res.targetPoses[i].orientation.w
   */

  //res.sum = req.a + req.b;publishLocalPlan
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

/******************************************************************************
 Retrieve user's proxemics preferences from the database

 *******************************************************************************/
Proxemics::Bearing Proxemics::retrieveProxemicsPreferences(int userId, int robotGenericTaskId)
{
  string test, temp1, temp2;
  stringstream out;

  Bearing bearing;


  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  test = "SELECT * FROM UserProxemicPreferences where UserProxemicPreferences.userId = ";
  test += to_string(userId);
  test += " and UserProxemicPreferences.robotGenericTaskId = ";
  test += to_string(robotGenericTaskId);
  cout << test << endl;
  result = stmt->executeQuery(test);

  test = "SELECT * FROM Accompany.Proxemics where Accompany.Proxemics.proxemicId = ";

  while (result->next())
    test += result -> getString("proxemicId");

  cout << test << endl;
  result = stmt->executeQuery(test);

  test = "SELECT * FROM RobotApproachDistance where RobotApproachDistance.robotApproachDistanceId = ";

  while (result->next())
  {
    temp1 = result -> getString("robotApproachDistanceId");
    temp2 = result -> getString("robotApproachOrientationId");
  }

  test += temp1;
  cout << test << endl;
  result = stmt->executeQuery(test);

  while (result->next())
    bearing.distance = (float)result -> getDouble("distance");


  test = "SELECT * FROM RobotApproachOrientation where RobotApproachOrientation.robotApproachOrientationId = ";
  test += temp2;
  cout << test << endl;
  result = stmt->executeQuery(test);

  while (result->next())
    bearing.orientation = degree2radian(result -> getDouble("orientation"));

  cout << "Distance = " << bearing.distance << " Orientation = " << radian2degree(bearing.orientation) << endl;

  /* Simulated user's pose. The actual user's pose will be provided by the caller*/
  // geometry_msgs::Pose userPose;
  // userPose.orientation = tf::createQuaternionMsgFromYaw( degree2radian(87.6) ); //create Quaternion Msg from Yaw
  // tf::pointTFToMsg(tf::Point(1, 1, 0), userPose.position);

  delete result;
  delete stmt;
  con -> close();
  delete con;

  return bearing;
}

/******************************************************************************
 Retrieve user's proxemics preferences from the database

 *******************************************************************************/
void Proxemics::retrieveProxemicsPreferences_ranking(int userId, int robotGenericTaskId, Bearing* rankedBearing)
{
  string test, temp1, temp2;

  int priority = 0;
  int k=0;

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  ProcTable procOrientationInfo[8];

  for (int h=0; h<21; h++)
    {
      rankedBearing[h].orientation  = 999;
      rankedBearing[h].distance  = 999;
    }

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  test = "SELECT * FROM UserProxemicPreferences where UserProxemicPreferences.userId = ";
  test += to_string(userId);
  test += " and UserProxemicPreferences.robotGenericTaskId = ";
  test += to_string(robotGenericTaskId);
  cout << test << endl;
  result = stmt->executeQuery(test);

  test = "SELECT * FROM Accompany.Proxemics where Accompany.Proxemics.proxemicId = ";
  while (result->next())
    test += result -> getString("proxemicId");

  cout << test << endl;
  result = stmt->executeQuery(test);

  while (result->next())
  {
    temp1 = result -> getString("robotApproachDistanceId");
    temp2 = result -> getString("robotApproachOrientationId");

  }

  test = "SELECT * FROM RobotApproachDistance where RobotApproachDistance.robotApproachDistanceId = ";
  test += temp1;
  cout << test << endl;
  result = stmt->executeQuery(test);
  while (result->next())
  {
    rankedBearing[0].distance = (float) result -> getDouble("distance");
    priority = result ->getInt("priority");
  }

  test = "SELECT * FROM RobotApproachOrientation where RobotApproachOrientation.robotApproachOrientationId = ";
  test += temp2;
  cout << test << endl;
  result = stmt->executeQuery(test);
  while (result->next())
    rankedBearing[0].orientation = (float) (result -> getDouble("orientation"));

  cout << "Distance = " << rankedBearing[0].distance << " Orientation = " << rankedBearing[0].orientation << endl;

  test = "SELECT * FROM RobotApproachOrientation";
  result = stmt->executeQuery(test);

  while(result->next())
  {
    procOrientationInfo[k].robotApproachOrientationId = result->getInt("robotApproachOrientationId");
    procOrientationInfo[k].orientation = result->getInt("orientation");
    procOrientationInfo[k].priority = result->getInt("priority");
    k++;
  }

  k = 1;

  float distance = 0;

  for (int h = priority; h<=3; h++)
  {
    if (h == 1)
      distance = 0.825;
    else
      if (h == 2)
        distance = 1.5;
      else
        if (h == 3)
          distance = 2;

    if (k<7) //perform the first cycle of ranking then reuse for different distances
    {
      if (rankedBearing[0].orientation == 0) //then check id 1, then check RobotApproachOrientation id 2, 3 or 3, 2 depending on user's handedness or current robot location //front
      //search for next bearing that fall on the same side as the preferred bearing
      {
        for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
          for (int i=0; i<8; i++) //number of data to search
            if ((procOrientationInfo[i].orientation <= 0) && (procOrientationInfo[i].orientation > -140)) //right side, angle between -1 to -140
              if (procOrientationInfo[i].priority == j)
                if (procOrientationInfo[i].orientation != rankedBearing[0].orientation) //&& (procOrientationInfo[i].distance != rankedBearing[0].distance))
                {
                  rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                  rankedBearing[k].distance = distance;
                  k++;
                }

        //search for next bearing that fall on the different side as the preferred bearing
        for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
          for (int i=0; i<8; i++) //number of data to search
            if ((procOrientationInfo[i].orientation > 0) && (procOrientationInfo[i].orientation < 140)) //left side, angle between 1 to 140
              if (procOrientationInfo[i].priority == j)
                  if (procOrientationInfo[i].orientation != rankedBearing[0].orientation)
                  {
                    rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                    rankedBearing[k].distance = distance;
                    k++;
                  }
      }

      //if (rankedBearing[0].orientation == 180)

      if (rankedBearing[0].orientation > 0) // then check id 2,4,6 //left
      {
        //search for next bearing that fall on the different side as the preferred bearing
        for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
          for (int i=0; i<8; i++) //number of data to search
            if ((procOrientationInfo[i].orientation >= 0) && (procOrientationInfo[i].orientation < 140)) //left side, angle between 1 to 140
              if (procOrientationInfo[i].priority == j)
                  if (procOrientationInfo[i].orientation != rankedBearing[0].orientation)
                  {
                    rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                    rankedBearing[k].distance = distance;
                    k++;
                  }

        //search for next bearing that fall on the same side as the preferred bearing
        for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
          for (int i=0; i<8; i++) //number of data to search
            if ((procOrientationInfo[i].orientation < 0) && (procOrientationInfo[i].orientation > -140)) //right side, angle between -1 to -140
              if (procOrientationInfo[i].priority == j)
                if (procOrientationInfo[i].orientation != rankedBearing[0].orientation) //&& (procOrientationInfo[i].distance != rankedBearing[0].distance))
                {
                  rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                  rankedBearing[k].distance = distance;
                  k++;
                }
      }

      if (rankedBearing[0].orientation < 0) // then check id 3,5,7 //right side
      {
        //search for next bearing that fall on the same side as the preferred bearing
        for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
          for (int i=0; i<8; i++) //number of data to search
            if ((procOrientationInfo[i].orientation <= 0) && (procOrientationInfo[i].orientation > -140)) //right side, angle between -1 to -140
              if (procOrientationInfo[i].priority == j)
                if (procOrientationInfo[i].orientation != rankedBearing[0].orientation) //&& (procOrientationInfo[i].distance != rankedBearing[0].distance))
                {
                  rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                  rankedBearing[k].distance = distance;
                  k++;
                }


        //search for next bearing that fall on the different side as the preferred bearing
        for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
          for (int i=0; i<8; i++) //number of data to search
            if ((procOrientationInfo[i].orientation > 0) && (procOrientationInfo[i].orientation < 140)) //left side, angle between 1 to 140
              if (procOrientationInfo[i].priority == j)
                  if (procOrientationInfo[i].orientation != rankedBearing[0].orientation)
                  {
                    rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                    rankedBearing[k].distance = distance;
                    k++;
                  }
      }
    }
  else
    for (int i=0; i<7; i++)
    {
      rankedBearing[k].orientation = rankedBearing[k-7].orientation;
      rankedBearing[k].distance = distance;
      k++;
    }
  }

  for (int j=0; j<21; j++)
  {
    if (rankedBearing[j].orientation != 999)
      cout<<"Ranked Bearing "<<j<<" orientation is "<<rankedBearing[j].orientation<<", distance is "<<rankedBearing[j].distance<< endl;
  }
  /* Simulated user's pose. The actual user's pose will be provided by the caller*/
  // geometry_msgs::Pose userPose;
  // userPose.orientation = tf::createQuaternionMsgFromYaw( degree2radian(87.6) ); //create Quaternion Msg from Yaw
  // tf::pointTFToMsg(tf::Point(1, 1, 0), userPose.position);

  delete result;
  delete stmt;
  con -> close();
  delete con;

//  return bearing;
}

/*first calculate the possible pose then eliminate and rearranged based on closest distance.

id = 3

if id.orientation = 0, then check id 1, then check RobotApproachOrientation id 2, 3 or 3, 2 depending on user's handedness or current robot location //front

if id.orientation > 0, then check id 2,4,6 //left


if id.orientation < 0, then check id 3,5,7 //right side
  stored the current bearing
  search for next bearing


  for (j=1; j<=4, j++)  //priority
    for (i=0; i<8; i++) //number of data to search
      if orientation < 0 orientation > -140 //right side, angle between -1 to -140
        if priority == j && id !=id.orientation
          stored current bearing.
          break;

  for (j=1; j<=4, j++)
    for (i=0; i<8; i++)
     if orientation > 0 orientation < 140 //left side, angle between 1 to 140
        if priority == j
          stored current bearing.
          break;


if id.orientation = (+/-) 180 //back



1,2,3 for orientation, for another orientation
  id is 3 = front right default = -45, -90,     0,   +45, +90         D = 1.5m
                                = -45, -90,     0,   +45, +90         D = 2m

  id is 5 = side right default  = -90, -45,  -135,  0
//id is

if orientation = 0, then check id 1, then check RobotApproachOrientation id 2, 3 or 3, 2 depending on user's handedness or current robot location //front


if orientation >=0, then check id 2,4,6 //left

if orientation <=0, then check id 3,5,7 //right side

if orientation = (+/-) 180 //back
*/
/******************************************************************************
 Calculate the the robot's target pose relative to the user's pose, based on
 the user's proxemics preference

 userPose: is the user pose (x,y,z, Quaternion).

 prefOrientation: is the user prefered robot's approach direction with respect
 to the user's coordinate frame in radian.

 prefDistance is: the user preferred robot's approach distance with respect
 to the user's coordinate frame in meter.
 ******************************************************************************/
Proxemics::Pose Proxemics::calculateRobotPoseFromProxemicsPreference(geometry_msgs::Pose &userPose, Bearing prefBearing)
{
  float x_tar, y_tar, theta_tar, x_usr, y_usr, theta_usr, d_x, d_y;
  float x_tar_temp, y_tar_temp;
  float prefDistance;
  float prefOrientation;
  Pose targetPose;

  prefDistance = prefBearing.distance;
  prefOrientation = prefBearing.orientation;

  ROS_INFO("Calculate proxemics targets for user's at: x=%f, y=%f, yaw = %f",
      userPose.position.x, userPose.position.y, radian2degree(tf::getYaw(userPose.orientation)));

  // Determines the robot target coordinate in user's coordinate frame
  x_tar = prefDistance * cos(prefOrientation);
  y_tar = prefDistance * sin(prefOrientation);

  //Determines the robot target coordinate in map's coordinate frame
  //1. Retrieve user's coordinate in map's coordinate frame
  x_usr = userPose.position.x;
  y_usr = userPose.position.y;
  theta_usr = tf::getYaw(userPose.orientation);

  //2.Calculate the robot target's relative to the user, taking into account the orientation of the user in map frame.
  //i.e. Rotation
  x_tar_temp = x_tar * cos(theta_usr) - y_tar * sin(theta_usr);
  y_tar_temp = x_tar * sin(theta_usr) + y_tar * cos(theta_usr);

  //3. Calculate the robot target's position in map coordinate frame.
  //i.e. Translation
  x_tar = x_usr + x_tar_temp;
  y_tar = y_usr + y_tar_temp;

  //4. Calculate the robot target's orientation in map coordinate frame.
  d_x = x_usr - x_tar;
  d_y = y_usr - y_tar;
  theta_tar = atan2(d_y, d_x);

  ROS_INFO("Robot proxemics target is: x=%f, y=%f, theta = %f",x_tar, y_tar, radian2degree(theta_tar));

  targetPose.x = x_tar;
  targetPose.y = y_tar;
  targetPose.orientation = theta_tar; //in [rad]

  return targetPose;
}

/******************************************************************************
 - Eliminate target poses that could not be occupied by the robot based on static map
 (i.e. too close to obstacle or on obstacle)
 - Eliminate target poses that could not be reach by the robot based on static map

 ******************************************************************************/
bool Proxemics::validApproachPosition(Pose personLocation, Pose robotLocation, Pose potentialApproachPose)
{
  // convert coordinates to pixels
  cv::Point potentialApproachPosePixel = convertFromMeterToPixelCoordinates<cv::Point> (potentialApproachPose);
  cv::Point personLocationPixel = convertFromMeterToPixelCoordinates<cv::Point> (personLocation);

  // copy expanded map
  cv::Mat expanded_map_with_person = expanded_map_.clone();

  // draw person into map as obstacle
  int personRadiusPixel = (int)((personRadius + robotRadius) / map_resolution_);
  //std::cout << "cx:" << center.x << "  cy:" << center.y << "  mx:" << map_origin_.x << "  my:" << map_origin_.y << "  resolution:" << map_resolution_ << std::endl;
  cv::circle(expanded_map_with_person, personLocationPixel, personRadiusPixel, cv::Scalar(0, 0, 0, 0), -1);

  // display new inflated map
  //cv::imshow("inflated map", expanded_map_with_person);
  //cv::waitKey(10);

  // find the individual connected areas
  std::vector<std::vector<cv::Point> > contours; // first index=contour index;  second index=point index within contour
  //std::vector<cv::Vec4i> hierarchy;
  cv::Mat expanded_map_with_person_copy = expanded_map_with_person.clone();
  cv::findContours(expanded_map_with_person_copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  // display found contours
  cv::drawContours(expanded_map_with_person, contours, -1, cv::Scalar(128, 128, 128, 128), 2);
  cv::circle(expanded_map_with_person, convertFromMeterToPixelCoordinates<cv::Point> (robotLocation), 3,
             cv::Scalar(100, 100, 100, 100), -1); //robot position
  cv::circle(expanded_map_with_person, potentialApproachPosePixel, 3, cv::Scalar(200, 200, 200, 200), -1); //approach position
  //cv::line(expanded_map_with_person, convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(Pose(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
  //cv::line(map_, convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(Pose(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
  cv::imshow("contour areas", expanded_map_with_person);
  cv::waitKey(100);

  // Eliminate poses that could not be reach by the robot based on static map
  // i.e. check whether potentialApproachPose and robotLocation are in the same area (=same contour)
  int contourIndexRobot = -1;
  int contourIndexPotentialApproachPose = -1;
  for (unsigned int i = 0; i < contours.size(); i++)
  {
    if (0 <= cv::pointPolygonTest(contours[i], convertFromMeterToPixelCoordinates<cv::Point2f> (potentialApproachPose),
                                  false))
      contourIndexPotentialApproachPose = i;
    if (0 <= cv::pointPolygonTest(contours[i], convertFromMeterToPixelCoordinates<cv::Point2f> (robotLocation), false))
      contourIndexRobot = i;
  }
  std::cout << "contourIndexPotentialApproachPose=" << contourIndexPotentialApproachPose << "  contourIndexRobot="
      << contourIndexRobot << std::endl;
  if (contourIndexRobot != contourIndexPotentialApproachPose || (contourIndexRobot == -1
      && contourIndexPotentialApproachPose == -1))
    return false;

  // Eliminate poses that are not in the same location as the user (i.e. user is in living room, therefore all potential robot poses have to be in the living room for HRI)
  // check whether there is an obstacle in direct line of sight between personLocation and potentialApproachPose
  double dx = personLocationPixel.x - potentialApproachPosePixel.x;
  double dy = personLocationPixel.y - potentialApproachPosePixel.y;
  double interpolationSteps = 0.;
  if (dx >= dy) // normalize
  {
    interpolationSteps = dx;
    dy /= dx;
    dx = 1.;
  }
  else
  {
    interpolationSteps = dy;
    dx /= dy;
    dy = 1.;
  }

  for (int i = 0; i < interpolationSteps; i++)
  {
    if (map_.at<unsigned char> (potentialApproachPosePixel.y + i * dy, potentialApproachPosePixel.x + i * dx) == 0) // if there is an obstacle in line of sight (map(y,x)==0)
      return false;
  }

  return true;
}

/******************************************************************************
 This function copies the received map into opencv's format  and creates an
 inflated version of the map with robot radius

 ******************************************************************************/
void Proxemics::updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
{
  // copy properties
  map_resolution_ = map_msg->info.resolution;
  map_origin_ = cv::Point2d(map_msg->info.origin.position.x, map_msg->info.origin.position.y);

  // create empty copy of map
  map_ = 255 * cv::Mat::ones(map_msg->info.height, map_msg->info.width, CV_8UC1);

  // copy real static map into cv::Mat element-wise
  for (unsigned int v = 0, i = 0; v < map_msg->info.height; v++)
  {
    for (unsigned int u = 0; u < map_msg->info.width; u++, i++)
    {
      if (map_msg->data[i] != 0)
        map_.at<unsigned char> (v, u) = 0;
    }
  }

  // create the inflated map
  int iterations = (int)(robotRadius / map_resolution_);
  //std::cout << "iterations=" << iterations << std::endl;
  cv::erode(map_, expanded_map_, cv::Mat(), cv::Point(-1, -1), iterations);

  // display maps
  //            cv::imshow("blown up map", expanded_map_);
  //            cv::imshow("map", map_);
  //            cv::waitKey(10);


  ROS_INFO("Map received.");
}
