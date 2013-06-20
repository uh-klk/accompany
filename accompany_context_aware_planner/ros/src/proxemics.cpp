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

  //Polar form relative to centre formula from see http://en.wikipedia.org/wiki/Ellipse#Polar_form_relative_to_center
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

 The respond vector data from the server are in the following format and contain a list of potential target locations.
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

 ******************************************************************************/
bool Proxemics::getPotentialProxemicsLocations(
                                               accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                                               accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res)
{
  Pose targetPose, robotLocation;
  Bearing proxemics_bearing;
  Bearing ranked_bearing[21];

  float x, y, orientation;
  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;
  string sql;

  int locationId, ValidRobotLocation;

  int validDataCount = 0;
  tf::Stamped<tf::Pose> p;
  geometry_msgs::PoseStamped pose; //create a PoseStamped variable to store the StampedPost TF

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object


  SQL_error = false; //reset the SQL error flag to false before begin processing

  //1. Process the request data from the client.
  //ROS_INFO("MsgSeq = %d, time =  %li, coordinate frame = %s ",req.header.seq, static_cast<long>(ros::Time::now().toNSec()-req.header.stamp.toNSec()), req.header.frame_id.c_str());
  ROS_INFO("MsgSeq = %d, time = %2f, coordinate frame = %s ",req.header.seq, (ros::Time::now().toSec()-req.header.stamp.toSec()), req.header.frame_id.c_str());
  ROS_INFO("userId = %d, userPosture = %d", req.userId, req.userPosture);

  Pose personLocation(req.userPose.position.x, req.userPose.position.y, tf::getYaw(req.userPose.orientation));

  ROS_INFO("request proxemics for user pose: x=%f, y=%f, z=%f yaw = %f", req.userPose.position.x, req.userPose.position.y, req.userPose.position.z, tf::getYaw(req.userPose.orientation));
  ROS_INFO("robotGenericTask: %d", req.robotGenericTaskId);


  //if user is sitting
  /*if (req.userPosture == 2)    //sitting on sofa
  {
    //getsensorlocation id and check if user is in locationid

      getPotentialProxemicsLocations_Sofa(req,res);



    if (SQL_error == true)
    {
      cout<<"SQL_error!!!"<<endl;
      return true;
    }
  }
  */
  if(getPotentialProxemicsLocations_ExceptionCase(req,res) == true)
  {
    cout<<"Done processing ExceptionCase."<<endl;
    //cout<<SQL_error<<endl;
  }
  else if (req.userPosture == 1) //standing
  {
    SQL_error = false;  //reset it to false incase it was set by getPotentialProxemicsLocations_ExceptionCase
    if(getPotentialProxemicsLocations_Standing(req,res) == false)
      cout<<"Error!"<<endl;

  }
  //check if there are data in it.
  /*if ((validDataCount>0) && (SQL_error != true))
  {
    cout<<validDataCount<<" valid pose(s) found."<<endl;
    cout<<"Sending pose(s) out."<<endl;
    return true;
  }
  else if (req.userPosture!=2)
  {
    cout<<"No valid target pose was found."<<endl;
    return true;
  }

  cout<<"Found an exceptionCaseProxemicPose."<<endl;*/

  return true;
}
/************************************************************************************************/
bool Proxemics::getPotentialProxemicsLocations_Standing(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                                             accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res)
{
Pose targetPose, robotLocation;
Bearing proxemics_bearing,ranked_bearing[21];

Driver *driver;
Connection *con;
Statement *stmt;
ResultSet *result;
string sql;

int locationId, ValidRobotLocation;
float temp_x, temp_y, temp_orientation;

int validDataCount = 0;

tf::Stamped<tf::Pose> p;
geometry_msgs::PoseStamped pose; //create a PoseStamped variable to store the StampedPost TF

Pose personLocation(req.userPose.position.x, req.userPose.position.y, tf::getYaw(req.userPose.orientation));

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  //retrieved user's locationId
  sql  = "SELECT locationId FROM Users WHERE userId = ";
  sql += to_string(req.userId);
  cout<<sql<<endl;
  result = stmt->executeQuery(sql);
  if (result->next())
  {
    locationId = result->getInt("locationId");
  }
  else
  {
    SQL_error = true;
    cout<<"SQL_error!!!, no user's locationId found."<<endl;
    return false;
  }

  //check if locationId has valid robot location
  sql = "SELECT ValidRobotLocation FROM Locations WHERE locationId = ";
  sql += to_string(locationId);
  cout<<sql<<endl;
  result = stmt->executeQuery(sql);
  if (result->next())
  {
    ValidRobotLocation = result->getInt("ValidRobotLocation");
  }
  else
  {
    SQL_error = true;
    cout<<"SQL_error!!!, no ValidRobotLocation found."<<endl;
    return false;
  }

  if (ValidRobotLocation)
  {
    robotLocation = getRobotPose();

    //2. Retrieves user's preference - create a prioritise list of all possible bearing based on user's preference

    retrieveProxemicsPreferences_ranking(req.userId, req.robotGenericTaskId, ranked_bearing);
    if (SQL_error == true)
    {
      cout<<"Error after retrieveProxemicsPreferences_ranking."<<endl;
      return false; //true;
    }
    //To do //search based on distance  //search based on orientation
    for(int i=0; i<21; i++)
    {
      if (ranked_bearing[i].orientation == 999)
        i = 21;     //21 possible approach target' poses, ignoring the back approach.
      else
      { //calculate each bearing
        proxemics_bearing = ranked_bearing[i];
        //ROS_INFO("Ranked Bearing %d orientation is %f distance is %f",i, radian2degree(proxemics_bearing.orientation), proxemics_bearing.distance);

        //3. Calculate all the target poses from the database, where distance and orientation can be obtain from step2
        targetPose = calculateRobotPoseFromProxemicsPreference(req.userPose, proxemics_bearing);
        if (SQL_error == true)
          return false; //true;

        //4. Eliminate target poses that could not be occupied by the robot based on static map (i.e. too close to obstacle or on obstacle)
        //   Eliminate target poses that could not be reach by the robot based on static map
        //5. Elimimate target poses that are not in the same location as the user (i.e. user is in living room, therefore all potential robot poses have to be in the living room for HRI)
        bool validApproach = validApproachPosition(personLocation, robotLocation, targetPose);
        if (validApproach == true)
        {
          p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                             tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");
          ROS_INFO("Ranked Bearing %d orientation is %f distance is %f",i, radian2degree(proxemics_bearing.orientation), proxemics_bearing.distance);
          ROS_INFO("The approach position is valid+++++++++++++++++++++++++++++++++++++++++++++++++");

          //6. Compile the respond message for the client.
          tf::poseStampedTFToMsg(p, pose); //convert the PoseStamped data into message format and store in pose
          res.targetPoses.push_back(pose); //push the pose message into the respond vector to be send back to the client

          validDataCount++;
        }
        else
        {
          ROS_INFO("Ranked Bearing %d orientation is %f distance is %f",i, radian2degree(proxemics_bearing.orientation), proxemics_bearing.distance);
          ROS_INFO("The approach position is invalid-----------------------------------------------");
        }
      }
    }
  }
  else
  {
    while (ValidRobotLocation == 0)
    {
      sql = "SELECT closestValidRobotLocation FROM Locations WHERE locationId = ";
      sql += to_string(locationId);
      result = stmt->executeQuery(sql);
      if(result->next())
      {
        locationId = result->getInt("closestValidRobotLocation");
        sql  = "SELECT * FROM Locations WHERE locationId = ";
        sql += to_string (locationId);
        result = stmt->executeQuery(sql);
        while(result->next())
        {
          temp_x = result->getDouble("xCoord");
          temp_y = result->getDouble("yCoord");
          temp_orientation = result->getDouble("orientation");
          ValidRobotLocation = result->getInt("ValidRobotLocation");
          locationId = result->getDouble("closestValidRobotLocation");
        }
      }
      else
      {
        SQL_error = true;
        cout<<"SQL_error!!!, no ValidRobotLocation can be found."<<endl;
        return false;
      }
    }
    targetPose.x = temp_x;
    targetPose.y = temp_y;
    targetPose.orientation = degree2radian(temp_orientation);
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                                       tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");
    tf::poseStampedTFToMsg(p, pose);    //Convert the PoseStamped data into message format and store in pose.
    res.targetPoses.push_back(pose);    //Store the pose in the respond message for the client.
    validDataCount++;
  }

  if (validDataCount>0)
  {
    cout<<validDataCount<<" valid pose(s) found."<<endl;
    cout<<"Sending pose(s) out."<<endl;
    return true;
  }

}


bool Proxemics::getPotentialProxemicsLocations_ExceptionCase(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                            accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res)
{
  int userLocationId;
  Pose targetPose;
  geometry_msgs::PoseStamped pose; //create a PoseStamped variable to store the StampedPost TF
  int ValidRobotLocation, closestValidRobotLocation;

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  string sql;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  sql = "SELECT * FROM Users where userId =  ";
  sql += to_string(req.userId);
  cout<<sql<<endl;
  result = stmt->executeQuery(sql);
  if (result->next())
  {
    userLocationId = result->getInt("locationId");
  }
  else
  {
    cout<<"Error retrieving user's locationId from userId in Users table."<<endl;
    SQL_error = true;

    delete result;
    delete stmt;
    con->close();
    delete con;
    return false;
  }

  /*
   SELECT
     e.*
   FROM
     ExceptionCaseProxemicsPose e
     INNER JOIN SensorBasedProxemicsPreferences p ON p.exceptionCaseProxemicsPoseId = e.exceptionCaseProxemicsPoseId
     INNER JOIN Sensors s ON s.sensorID = p.sensorID
   WHERE
     s.locationId = 14 AND (
        s.status = 'On' OR
        s.status = 'Open' OR
        s.status = 'Occupied')
   */
  sql = "SELECT \
           COUNT(e.environmentId) as `count` \
         FROM \
           ExceptionCaseProxemicsPose e \
           INNER JOIN SensorBasedProxemicsPreferences p ON p.exceptionCaseProxemicsPoseId = e.exceptionCaseProxemicsPoseId \
           INNER JOIN Sensors s ON s.sensorId = p.sensorId \
         WHERE \
           s.locationId = ";
   sql += to_string(userLocationId);
   sql += " AND ( \
         s.status = 'On' OR \
         s.status = 'Open' OR \
         s.status = 'Occupied')";
   cout<<sql<<endl;
   result = stmt->executeQuery(sql);     //Search for the pose of the proxemicsPoseId in the current environment.
   if (result->next())
   {
     int count = result->getInt("count");

     if (count == 0)
     {
       cout<<"Sensor based proxemics were not found in ExceptionCaseProxemicsPose table, switching to general proxemics processing."<<endl;
       SQL_error = true;
       delete result;
       delete stmt;
       con->close();
       delete con;
       return false;
     }
     else if (count == 1)
     {
       sql = " SELECT \
                 e.* \
               FROM \
                 ExceptionCaseProxemicsPose e \
                 INNER JOIN SensorBasedProxemicsPreferences p ON p.exceptionCaseProxemicsPoseId = e.exceptionCaseProxemicsPoseId \
                 INNER JOIN Sensors s ON s.sensorId = p.sensorId \
               WHERE \
                 s.locationId = ";
         sql += to_string(userLocationId);
         sql += " AND ( s.status = 'On' OR \
                        s.status = 'Open' OR \
                        s.status = 'Occupied')";
         cout<<sql<<endl;
         result = stmt->executeQuery(sql);     //Search for the pose of the proxemicsPoseId in the current environment.
         if (result->next())
         {
           targetPose.x=result->getDouble("x");
           targetPose.y=result->getDouble("y");
           targetPose.orientation=degree2radian(result->getDouble("orientation"));
           tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                                                    tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");
           tf::poseStampedTFToMsg(p, pose);    //Convert the PoseStamped data into message format and store in pose.
           res.targetPoses.push_back(pose);    //Store the pose in the respond message for the client.
           cout<<"Done processing - Found user triggering sensor based proxemics."<<endl;
         }
         else
         {
           cout<<"Sensor based proxemics were not found in ExceptionCaseProxemicsPose table, switching to general proxemics processing."<<endl;
           SQL_error = true;
           delete result;
           delete stmt;
           con->close();
           delete con;
           return false;
         }

     }
     else if (count >=1 )
     {
       sql = "SELECT * FROM Locations WHERE locationId = ";
       sql+= to_string(userLocationId);
       cout<<sql<<endl;
       result = stmt->executeQuery(sql);
       if (result->next())
       {
         ValidRobotLocation = result->getInt("ValidRobotLocation");
         while (!ValidRobotLocation)     //find a valid robot location near the sofa locationId
         {
           closestValidRobotLocation = result->getInt("closestValidRobotLocation");
           sql = "SELECT * FROM Locations WHERE locationId = ";
           sql+= to_string(closestValidRobotLocation);
           cout<<sql<<endl;
           result = stmt->executeQuery(sql);
           if (result->next())
           {
             ValidRobotLocation = result->getInt("ValidRobotLocation");
           }
           else
           {
             SQL_error = true;
             cout<<"SQL_error!!!, no ValidRobotLocation can be found."<<endl;
             return false;
           }
         }
         targetPose.x=result->getDouble("xCoord");
         targetPose.y=result->getDouble("yCoord");
         targetPose.orientation=degree2radian(result->getDouble("orientation"));
         tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                                                  tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");
         tf::poseStampedTFToMsg(p, pose);    //Convert the PoseStamped data into message format and store in pose.
         res.targetPoses.push_back(pose);    //Store the pose in the respond message for the client.
         cout<<"Done processing user on sofa in living room"<<endl;
         delete result;
         delete stmt;
         con -> close();
         delete con;
         return true;
       }
       else
       {
         SQL_error = true;
         cout<<"Sofa's locationId not found."<<endl;
         delete result;
         delete stmt;
         con -> close();
         delete con;
         return false;
       }
     }
   }
   else
   {
     cout<<"Sensor based proxemics were not found in ExceptionCaseProxemicsPose table, switching to general proxemics processing."<<endl;
     SQL_error = true;
     delete result;
     delete stmt;
     con->close();
     delete con;
     return false;
   }

  return true;
}
/******************************************************************************
 * Context-aware Proxemics for sofa, regardless of user
 ******************************************************************************/
void Proxemics::getPotentialProxemicsLocations_Sofa(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                                                    accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res) //retrieves current user (who)
{
  string sql;
  int firstSensorId, lastSensorId;
  string currentUserPosture;
  Pose targetPose;
  geometry_msgs::PoseStamped pose; //create a PoseStamped variable to store the StampedPost TF
  int proxemicsPoseId = 0; //default for multiple sofa activations.
  int sensorId, sensor_locationId;

  int locationId, ValidRobotLocation, closestValidRobotLocation;
  int experimentalLocationId; //1 = UHRH, 5 = IPA Kitchen
  int i=0;

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  //Determine the experimental location for this user from the SessionControl table
  sql = "SELECT * FROM SessionControl where SessionUser =  ";
  sql += to_string(req.userId);
  cout<<sql<<endl;
  result = stmt->executeQuery(sql);
  if (result->next())
  {
    experimentalLocationId = result->getInt("ExperimentalLocationId");
  }
  else
  {
    cout<<"Error retrieving ExperimentalLocationId from SessionControl table."<<endl;
    SQL_error = true;

    delete result;
    delete stmt;
    con->close();
    delete con;
    return;
  }

  int numberOfSofa;
  int sofaId[10];

  switch (experimentalLocationId) {
    case 1: //Sofa in RH
      //lastSensorId = 19;
      //firstSensorId = 15;
      numberOfSofa = 5;
      sofaId[0]=15;
      sofaId[1]=16;
      sofaId[2]=17;
      sofaId[3]=18;
      sofaId[4]=19;
      break;
    case 2:  //Heerlen
      numberOfSofa = 3;
      sofaId[0] = 301; 
      sofaId[1] = 304;
      sofaId[2] = 305;
      break;
    case 4: //IPA Kitchen
      //lastSensorId = 811;
      //firstSensorId = 810;
      numberOfSofa = 2;
      sofaId[0]=810;
      sofaId[1]=811;
      break;
    default:
      cout<<"experimentalLocationId for "<<experimentalLocationId<<"is not set in this program"<<endl;
      return;
  }

  //check if user is in same location as sensors
  sql = "SELECT locationId FROM Sensors where sensorId = ";
  /*sql+= to_string(lastSensorId);
  sql+= " and sensorId >= ";
  sql+= to_string(firstSensorId);
  sql+= " and value = 1";*/
  sql+= to_string(sofaId[0]);
  for (i=1; i<numberOfSofa; i++)
  {
	  sql+= " || sensorId = ";
	  sql+= to_string(sofaId[i]);
  }

  cout<<sql<<endl;
  result = stmt->executeQuery(sql);     //Search for the number of occupied sofa.
  if (result->next())
  {
    sensor_locationId = result->getInt("locationId");
  }
  else
  {
    cout<<"Could not determine locationId of sensor id "<<firstSensorId<<endl;
    SQL_error = true;
    delete result;
    delete stmt;
    con->close();
    delete con;
    return;
  }

  sql = "SELECT locationId FROM Users where userId = ";
  sql+= to_string(req.userId);
  cout<<sql<<endl;
  result = stmt->executeQuery(sql);     //Search for the number of occupied sofa.
  if (result->next())
  {
    if (sensor_locationId != result->getInt("locationId"))
    {
      cout<<"User id "<<req.userId<<" is not in the same location as the sensors id "<<firstSensorId<<" !"<<endl;
      SQL_error = true;
      delete result;
      delete stmt;
      con->close();
      delete con;
      return;
    }
  }
  else
  {
    cout<<"Could not determine locationId of user id "<<req.userId<<endl;
    SQL_error = true;
    delete result;
    delete stmt;
    con->close();
    delete con;
    return;
  }

/*sql = "SELECT COUNT(*) FROM Sensors where sensorId <= ";
  sql+= to_string(lastSensorId);
  sql+= " and sensorId >= ";
  sql+= to_string(firstSensorId);
  sql+= " and value = 1";
*/

  sql = "SELECT COUNT(*) FROM Sensors where ( ";
  sql += " sensorId = ";
  sql+= to_string(sofaId[0]);
  for (i=1; i<numberOfSofa; i++)
  {
	  sql += " or sensorId = ";
	  sql+= to_string(sofaId[i]);
  }
  sql+= ") and value = 1";

  cout<<sql<<endl;
  result = stmt->executeQuery(sql);     //Search for the number of occupied sofa.
  if (result->next())
  {
    if (result->getInt("COUNT(*)") == 1)  //If only one sofa is occupied
    {
/*    sql = "SELECT * FROM Sensors where sensorId <= ";
      sql+= to_string(lastSensorId);
      sql+= " and sensorId >= ";
      sql+= to_string(firstSensorId);
      sql+= " and value = 1";
*/
      sql = "SELECT * FROM Sensors where ( ";
      sql += " sensorId = ";
      sql+= to_string(sofaId[0]);

      for (i=1; i<numberOfSofa; i++)
   	  {
   		  sql += " or sensorId = ";
   		  sql+= to_string(sofaId[i]);
   	  }
      sql+= ") and value = 1";

      cout<<sql<<endl;
      result = stmt->executeQuery(sql); //Which Sofa is occupied?
      if (result->next())
      {
        sensorId = result->getInt("sensorId");
      }
      else
      {
        cout<<"No sofa sensors are activated now."<<endl;
        delete result;
        delete stmt;
        con->close();
        delete con;
        return;
      }

      sql  = "SELECT * FROM SensorBasedProxemicsPreferences where sensorId = ";
      sql += to_string(sensorId);
      cout<<sql<<endl;
      result = stmt->executeQuery(sql);       //Search for the preferred proxemicsPoseId for the sofa using its Id.
      if (result->next())
      {
        proxemicsPoseId = result->getInt("exceptionCaseProxemicsPoseId");
      }
      else
      {
        cout<<"exceptionCaseProxemicsPoseId for sensorId "<<sensorId<<" was not found in the SensorBasedProxemicsPreferences table."<<endl;
        SQL_error = true;
        delete result;
        delete stmt;
        con->close();
        delete con;
        return;
      }

      sql = "SELECT * FROM ExceptionCaseProxemicsPose where exceptionCaseProxemicsPoseId = ";
      sql += to_string(proxemicsPoseId);
      cout<<sql<<endl;
      result = stmt->executeQuery(sql);     //Search for the pose of the proxemicsPoseId in the current environment.
      if (result->next())
      {
        targetPose.x=result->getDouble("x");
        targetPose.y=result->getDouble("y");
        targetPose.orientation=degree2radian(result->getDouble("orientation"));
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                                                 tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");
        tf::poseStampedTFToMsg(p, pose);    //Convert the PoseStamped data into message format and store in pose.
        res.targetPoses.push_back(pose);    //Store the pose in the respond message for the client.
        cout<<"Done processing user on sofa in living room"<<endl;
      }
      else
      {
        cout<<"Pose for exceptionCaseProxemicsPoseId = "<<proxemicsPoseId<<" was not found for environmentId "<<experimentalLocationId<<" in the ExceptionCaseProxemicsPose table."<<endl;
        SQL_error = true;
        delete result;
        delete stmt;
        con->close();
        delete con;
        return;
      }
    }
    else // more than one sofa is activated
    {
      cout<<"More than one sofa is activated."<<endl;

      //sql = "SELECT locationId FROM Sensors where sensorId <= 19 and sensorId >= 15 and value = 1"; //Sofa
/*    sql = "SELECT * FROM Sensors where sensorId <= ";
      sql+= to_string(lastSensorId);
      sql+= " and sensorId >= ";
      sql+= to_string(firstSensorId);
      sql+= " and value = 1";
*/
      sql = "SELECT locationId FROM Sensors where ( ";
      sql += " sensorId = ";
      sql+= to_string(sofaId[0]);

      for (i=1; i<numberOfSofa; i++)
      {
   	sql += " or sensorId = ";
   	sql+= to_string(sofaId[i]);
      }
      sql+= ") and value = 1";

      cout<<sql<<endl;
      result = stmt->executeQuery(sql);
      if (result->next())
      {
        locationId = result->getInt("locationId"); //For now anyone will do
      }
      else
      {
        SQL_error = true;
        cout<<"SQL_error!!!, sofa are no longer activated."<<endl;

        delete result;
        delete stmt;
        con -> close();
        delete con;
        return;
      }
      //check if locationId is reachable, if so then use the coordinate
      sql = "SELECT * FROM Locations WHERE locationId = ";
      sql+= to_string(locationId);
      cout<<sql<<endl;
      result = stmt->executeQuery(sql);
      if (result->next())
      {
        ValidRobotLocation = result->getInt("ValidRobotLocation");
        while (!ValidRobotLocation)     //find a valid robot location near the sofa locationId
        {
          closestValidRobotLocation = result->getInt("closestValidRobotLocation");
          sql = "SELECT * FROM Locations WHERE locationId = ";
          sql+= to_string(closestValidRobotLocation);
          cout<<sql<<endl;
          result = stmt->executeQuery(sql);
          if (result->next())
          {
            ValidRobotLocation = result->getInt("ValidRobotLocation");
          }
          else
          {
            SQL_error = true;
            cout<<"SQL_error!!!, no ValidRobotLocation can be found."<<endl;
            return;
          }
        }

        targetPose.x=result->getDouble("xCoord");
        targetPose.y=result->getDouble("yCoord");
        targetPose.orientation=degree2radian(result->getDouble("orientation"));
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                                        tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");
        tf::poseStampedTFToMsg(p, pose);    //Convert the PoseStamped data into message format and store in pose.
        res.targetPoses.push_back(pose);    //Store the pose in the respond message for the client.
        cout<<"Done processing user on sofa in living room"<<endl;
        delete result;
        delete stmt;
        con -> close();
        delete con;
        return;
      }
      else
      {
        SQL_error = true;
        cout<<"Sofa's locationId not found."<<endl;
        delete result;
        delete stmt;
        con -> close();
        delete con;
        return;
      }
    }
  }
  else
  {
    SQL_error = true;
    cout<<"SQL_error!!!, sofa are no longer activated."<<endl;
    delete result;
    delete stmt;
    con -> close();
    delete con;
    return;
  }
}

/******************************************************************************
*
*  Context-aware Proxemics for user in the Kitchen, regardless of user
*
*
*****************************************************************************/
void Proxemics::getPotentialProxemicsLocations_Kitchen(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
    accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res)
{
  string sql;
  string currentUserPosture;
  Pose targetPose;
  geometry_msgs::PoseStamped pose; //create a PoseStamped variable to store the StampedPost TF
  int proxemicsPoseId=0;
  int locationId=0; //kitchen

  int experimentalLocationId; //1 = UHRH, 5 = IPA Kitchen

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  if (SQL_error != true)
  {
    sql = "SELECT * FROM SessionControl ";
    result = stmt->executeQuery(sql);
    if (result->next())
    {
      experimentalLocationId = result->getInt("ExperimentalLocationId");

      switch (experimentalLocationId)
      {
        case 1:   //UH Robot House
          locationId = 10;
          break;

        case 2:   //HZ Apartment
          locationId = 610;
          break;

        case 3:   //Madopa Apartment
          locationId = 710;
          break;

        case 4:   //IPA Kitchen
          locationId = 810;
          break;

        case 5:   //Stuttgard Apartment
          locationId = 502;
          break;

        default:
          cout<<"ExperimentalLocationId not found "<<endl;
          SQL_error = true;
          break;
      }
    }
    else
    {
      cout<<"Error retrieving ExperimentalLocationId from SessionControl table."<<endl;
      SQL_error = true;
    }
  }

  if (SQL_error != true)
  {
    sql  = "SELECT * FROM LocationBasedProxemicsPreferences where locationId = ";
    sql += to_string(locationId);
    cout<<sql<<endl;
    result = stmt->executeQuery(sql);       //Search for the preferred proxemicsPoseId for Kitchen.
    if (result->next())
    {
      proxemicsPoseId = result->getInt("exceptionCaseProxemicsPoseId");
    }
    else
    {
      cout<<"exceptionCaseProxemicsPoseId for Location "<<locationId<<" was not found in the LocationBasedProxemicsPreferences table."<<endl;
      SQL_error = true;
    }
  }

  if (SQL_error != true)
  {
    sql  = "SELECT * FROM ExceptionCaseProxemicsPose where exceptionCaseProxemicsPoseId = ";
    sql += to_string(proxemicsPoseId);
    sql += " and environmentId = ";
    sql += to_string(experimentalLocationId);
    cout<<sql<<endl;
    result = stmt->executeQuery(sql);     //Search for the pose of the proxemicsPoseId in the current environment.
    if (result->next())
    {
      targetPose.x=result->getDouble("x");
      targetPose.y=result->getDouble("y");
      targetPose.orientation=degree2radian(result->getDouble("orientation"));
      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(targetPose.orientation),
                                                  tf::Point(targetPose.x, targetPose.y, 0.0)), ros::Time::now(), "map");

      tf::poseStampedTFToMsg(p, pose);    //Convert the PoseStamped data into message format and store in pose.
      res.targetPoses.push_back(pose);    //Store the pose in the respond message for the client.
    }
    else
    {
      cout<<"Pose for exceptionCaseProxemicsPoseId = "<<proxemicsPoseId<<" was not found for environmentId "<< experimentalLocationId<<" in the ExceptionCaseProxemicsPose table."<<endl;
      SQL_error = true;
    }
  }

  delete result;
  delete stmt;
  con -> close();
  delete con;

}
/******************************************************************************
 Retrieve user's proxemics preferences from the database

 *******************************************************************************/
Proxemics::Bearing Proxemics::retrieveProxemicsPreferences(int userId, int robotGenericTaskId)
{
  string sql, robotApproachDistanceId, robotApproachOrientationId;
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

  sql  = "SELECT * FROM UserProxemicsPreferences where userId = ";
  sql += to_string(userId);
  sql += " and robotGenericTaskId = ";
  sql += to_string(robotGenericTaskId);
  cout << sql<< endl;
  result = stmt->executeQuery(sql);     //retrieved user's proxemics preferences based on robot's generic task
  if (result->next())
  {
    sql  = "SELECT * FROM Proxemics where proxemicsId = ";
    sql += result->getString("proxemicsId");
    cout<<sql<< endl;
    result = stmt->executeQuery(sql);   //retrieved the proxemics baring from proxemicsId
    if (result->next())
    {
      robotApproachDistanceId = result -> getString("robotApproachDistanceId");
      robotApproachOrientationId = result -> getString("robotApproachOrientationId");

      sql  = "SELECT * FROM RobotApproachDistance where robotApproachDistanceId = ";
      sql += robotApproachDistanceId;
      cout << sql<< endl;
      result = stmt->executeQuery(sql); //retrieved the actual distance from the robotApproachDistanceId
      if (result->next())
      {
        bearing.distance = (float)result -> getDouble("distance");

        sql  = "SELECT * FROM RobotApproachOrientation where robotApproachOrientationId = ";
        sql += robotApproachOrientationId;
        cout << sql<< endl;
        result = stmt->executeQuery(sql);       //retrieved the actual orientation from robotApproachOrientationId
        if (result->next())
        {
          bearing.orientation = degree2radian(result -> getDouble("orientation"));

          cout << "Distance = " << bearing.distance << " Orientation = " << radian2degree(bearing.orientation) << endl;
        }
        else
        {
          cout<<"Error with the following statement: "<<sql<<endl;
          SQL_error = true;
        }
      }
      else
      {
        cout<<"Error with the following statement: "<<sql<<endl;
        SQL_error = true;
      }
    }
    else
    {
      cout<<"Error with the following statement: "<<sql<<endl;
      SQL_error = true;
    }
  }
  else
    {
      cout<<"Error with the following statement: "<<sql<<endl;
      SQL_error = true;
    }

  delete result;
  delete stmt;
  con -> close();
  delete con;

  return bearing;
}
/************************************************************************************************/
bool Proxemics::isUserIn(int userId, string locationName)
{
  string sql;
  int locationId;

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  //determine if the current session user is in the Living Room
  sql = "SELECT L.locationId from Locations L where L.name = '";
  sql += locationName;
  sql += "'";
  cout << sql << endl;
  result = stmt->executeQuery(sql);
  if (result->next())
  {
    locationId = result->getInt("locationId");

    cout << "**********Current user location id is = " << locationId << endl;

    sql = "call spCheckUserLocation( ";
    sql += to_string(userId);
    sql += ",";
    sql += to_string(locationId);
    sql += ",";
    sql += "@res)";
    cout << sql << endl;
    if (stmt->execute(sql)) // return row , if rowcnt >= 1 user is in Living Room
    {
      delete result;
      delete stmt;
      con->close();
      delete con;  //this is to avoid issue where the "spCheckUserLocation" not returning when rowcnt = null?
      cout << "User is in the "<<locationName <<endl; //check which sofa the user is sitting

      return true;
    }
    else
      {
        delete result;
        delete stmt;
        con->close();
        delete con;  //this is to avoid issue where the "spCheckUserLocation" not returning when rowcnt = null?
        cout << "User is not in the "<<locationName<<endl; //check which sofa the user is sitting

        return false; //user is not in living room
      }
  }
  else
  {
    cout<<"No locationId for "<<locationName<<" was found in the Locations table."<<endl;
    SQL_error = true;
    return false;
  }
}
/******************************************************************************
 Retrieve user's proxemics preferences from the database

 *******************************************************************************/
Proxemics::BearingWithPriority Proxemics::retrieveProxemicsPreferencesWithPriority(int userId, int robotGenericTaskId)
{
  string sql, robotApproachDistanceId, robotApproachOrientationId;
  stringstream out;

  BearingWithPriority bearing;

  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  sql  = "SELECT * FROM UserProxemicsPreferences where userId = ";
  sql += to_string(userId);
  sql += " and robotGenericTaskId = ";
  sql += to_string(robotGenericTaskId);
  cout << sql<< endl;
  result = stmt->executeQuery(sql);     //retrieved user's proxemics preferences based on robot's generic task
  if (result->next())
  {
    sql  = "SELECT * FROM Proxemics where proxemicsId = ";
    sql += result->getString("proxemicsId");
    cout<<sql<< endl;
    result = stmt->executeQuery(sql);   //retrieved the proxemics baring from proxemicsId
    if (result->next())
    {
      robotApproachDistanceId = result -> getString("robotApproachDistanceId");
      robotApproachOrientationId = result -> getString("robotApproachOrientationId");

      sql  = "SELECT * FROM RobotApproachDistance where robotApproachDistanceId = ";
      sql += robotApproachDistanceId;
      cout << sql<< endl;
      result = stmt->executeQuery(sql); //retrieved the actual distance from the robotApproachDistanceId
      if (result->next())
      {
        bearing.distance.distance = (float)result -> getDouble("distance");
        bearing.distance.priority = result -> getInt("priority");

        sql  = "SELECT * FROM RobotApproachOrientation where robotApproachOrientationId = ";
        sql += robotApproachOrientationId;
        cout << sql<< endl;
        result = stmt->executeQuery(sql);       //retrieved the actual orientation from robotApproachOrientationId
        if (result->next())
        {
          bearing.orientation.orientation = degree2radian(result -> getDouble("orientation"));
          bearing.orientation.priority = result -> getInt("priority");

          cout<< "Distance = " << bearing.distance.distance << " Orientation = " << radian2degree(bearing.orientation.orientation) << endl;
        }
        else
        {
          cout<<"Error with the following statement: "<<sql<<endl;
          SQL_error = true;
        }
      }
      else
      {
        cout<<"Error with the following statement: "<<sql<<endl;
        SQL_error = true;
      }
    }
    else
    {
      cout<<"Error with the following statement: "<<sql<<endl;
      SQL_error = true;
    }
  }
  else
    {
      cout<<"Error with the following statement: "<<sql<<endl;
      SQL_error = true;
    }

  delete result;
  delete stmt;
  con -> close();
  delete con;

  return bearing;
}

/******************************************************************************
 Retrieve user's proxemics preferences with ranking

 *******************************************************************************/
void Proxemics::retrieveProxemicsPreferences_ranking(int userId, int robotGenericTaskId, Bearing* rankedBearing)
{
  string  temp1, temp2, robotApproachDistanceId, robotApproachOrientationId;

  BearingWithPriority bearingWithPriority;

  for (int h=0; h<21; h++) //init the arrays
    {
      rankedBearing[h].orientation  = 999;
      rankedBearing[h].distance  = 999;
    }

  //search the database to retrieve the preferred bearing with their respective priority
  bearingWithPriority = retrieveProxemicsPreferencesWithPriority(userId, robotGenericTaskId);
  if (SQL_error == true)
    return;
  rankedBearing[0].distance = bearingWithPriority.distance.distance;
  rankedBearing[0].orientation = bearingWithPriority.orientation.orientation;

  rankDistanceBasedPriority(bearingWithPriority.distance.priority, rankedBearing);

}
/*****************************************************************************
 *
 * Ranked the robot target poses based on user's preferred distance as main priority
 * starting from their preferred bearing (distance, orientation) and around them based
 * on the orientation priority.
 *
 *
 ****************************************************************************/
void Proxemics::rankDistanceBasedPriority( int distancePriority, Bearing* rankedBearing)
{

  ProcTable procOrientationInfo[8];
  int i;

  string sql;
  Driver *driver;
  Connection *con;
  Statement *stmt;
  ResultSet *result;

  driver = get_driver_instance();
  con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  con->setAutoCommit(0); // turn off the autocommit
  con->setSchema(DATABASE); // select appropriate database schema
  stmt = con->createStatement(); // create a statement object

  //retrieves the RobotApproachOrientation ranking list
  sql = "SELECT * FROM RobotApproachOrientation";
  result = stmt->executeQuery(sql);

  for (i=0; i<=7; i++)
  {
    if (result->next())
    {
      procOrientationInfo[i].robotApproachOrientationId = result->getInt("robotApproachOrientationId");
      procOrientationInfo[i].orientation = degree2radian(result->getInt("orientation"));
      procOrientationInfo[i].priority = result->getInt("priority");
    }
    else if (i == 0)
      {
        cout<<"Error executing "<<sql<<" statement."<<endl;
        SQL_error = true;
        return;
      }
      else
      {
        cout<<"Error retrieving (the "<<i<<"th) result from "<<sql<<" statement."<<endl;
        SQL_error = true;
        return;
      }
   }


  int k = 1;
  float distance = 0;

  if (distancePriority == 1)
    distance = 0.825; //read from database
  else if (distancePriority == 2)
    distance = 1.5;
  else if (distancePriority == 3)
    distance = 2;

  //perform the first ranking cycle around the user's preferred distance, based on their preferred bearing( distance, orientation).
  if (rankedBearing[0].orientation == degree2radian(0)) //then check id 1, then check RobotApproachOrientation id 2, 3 or 3, 2 depending on user's handedness or current robot location //front
  {
    //search for next bearing that fall on right side
    for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
      for (int i=0; i<8; i++) //number of data to search
        if ((procOrientationInfo[i].orientation <= degree2radian(0)) && (procOrientationInfo[i].orientation > degree2radian(-140))) //right side, angle between -1 to -140
          if (procOrientationInfo[i].priority == j)
            if (procOrientationInfo[i].orientation != rankedBearing[0].orientation) //&& (procOrientationInfo[i].distance != rankedBearing[0].distance))
            {
              rankedBearing[k].orientation = procOrientationInfo[i].orientation;
              rankedBearing[k].distance = distance;
              k++;
            }
    //search for next bearing that fall on a different side
    for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
      for (int i=0; i<8; i++) //number of data to search
        if ((procOrientationInfo[i].orientation > degree2radian(0)) && (procOrientationInfo[i].orientation < degree2radian(140))) //left side, angle between 1 to 140
            if (procOrientationInfo[i].priority == j)
              if (procOrientationInfo[i].orientation != rankedBearing[0].orientation)
              {
                rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                rankedBearing[k].distance = distance;
                k++;
              }
  }   //if user's preferred bearing is to the left side
  else if (rankedBearing[0].orientation > degree2radian(0)) // then check id 2,4,6
  {
    //search for next bearing that fall on the different side as the preferred bearing
    for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
      for (int i=0; i<8; i++) //number of data to search
        if ((procOrientationInfo[i].orientation >= degree2radian(0)) && (procOrientationInfo[i].orientation < degree2radian(140))) //left side, angle between 1 to 140
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
        if ((procOrientationInfo[i].orientation < degree2radian(0)) && (procOrientationInfo[i].orientation > degree2radian(-140))) //right side, angle between -1 to -140
          if (procOrientationInfo[i].priority == j)
            if (procOrientationInfo[i].orientation != rankedBearing[0].orientation) //&& (procOrientationInfo[i].distance != rankedBearing[0].distance))
            {
              rankedBearing[k].orientation = procOrientationInfo[i].orientation;
              rankedBearing[k].distance = distance;
              k++;
            }
  }   //if user's preferred bearing is to the right side
  else if (rankedBearing[0].orientation < degree2radian(0)) // then check id 3,5,7 //right side
  {
    //search for next bearing that fall on the same side as the preferred bearing
    for (int j=1; j<=4; j++)  //priority, ignore the back i.e. 5
      for (int i=0; i<8; i++) //number of data to search
        if ((procOrientationInfo[i].orientation <= degree2radian(0)) && (procOrientationInfo[i].orientation > degree2radian(-140))) //right side, angle between -1 to -140
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
        if ((procOrientationInfo[i].orientation > degree2radian(0)) && (procOrientationInfo[i].orientation < degree2radian(140))) //left side, angle between 1 to 140
          if (procOrientationInfo[i].priority == j)
              if (procOrientationInfo[i].orientation != rankedBearing[0].orientation)
              {
                rankedBearing[k].orientation = procOrientationInfo[i].orientation;
                rankedBearing[k].distance = distance;
                k++;
              }
  }

  //Replicated the same orientation ranking for two other distances based on their priority
  for(int i=1; i<=3; i++)
  {
    if (distancePriority != i) //if the orientation ranking haven't being done for this distance then do it
    {
      if (i == 1)
        distance = 0.825; //read from database
      else if (i == 2)
        distance = 1.5;
      else if (i == 3)
        distance = 2;

      for (int j=0; j<7; j++) //use the new rank list and replicate for other distances
         {
           rankedBearing[k].orientation = rankedBearing[k-7].orientation;
           rankedBearing[k].distance = distance;
           k++;
         }
    }
  }

  for (int j=0; j<21; j++) //display all the rankedBearing
  {
    if (rankedBearing[j].orientation != 999)
      cout<<"Ranked Bearing "<<j<<" orientation is "<<radian2degree(rankedBearing[j].orientation)<<", distance is "<<rankedBearing[j].distance<< endl;
  }

  delete result;
  delete stmt;
  con -> close();
  delete con;

}

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

//To Display the results
 
  cv::imshow("contour areas", expanded_map_with_person);

  cv::Mat expanded_map_with_person_flip;
  cv::flip(expanded_map_with_person, expanded_map_with_person_flip,0);
  cv::imshow("contour areas flip",expanded_map_with_person_flip);

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

//  display maps
    cv::imshow("blown up map", expanded_map_);
    cv::imshow("map", map_);
    cv::waitKey(10);


  ROS_INFO("Map received.");
}

Proxemics::Pose Proxemics::getRobotPose()
{
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

  return robotLocation;
}
