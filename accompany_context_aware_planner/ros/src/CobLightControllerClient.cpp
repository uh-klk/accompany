#include "accompany_context_aware_planner/CobLightControllerClient.h"

void UH_CobLightControllerClient::init()
{

  ptrSetLightModeClient = new actionlib::SimpleActionClient<cob_light::SetLightModeAction> (nh, "/light_controller/set_lightmode/", true);
  ROS_INFO("Waiting for the Light action server to come up.");
  if (ptrSetLightModeClient->waitForServer(ros::Duration(2.0) ) )
      ROS_INFO("Light action server is up.");
}

void UH_CobLightControllerClient::setLight(std_msgs::ColorRGBA colour, uint mode)
{
  cob_light::SetLightModeGoal goal;

  goal.mode.mode = mode;//cob_light::LightMode::FLASH; //cob_light::LightMode::NONE;
  goal.mode.color = colour;
  goal.mode.frequency = 1;  // in Hz
  goal.mode.timeout = 0;    // in s, requested mode will be executed for max timout s.  default is 0 and meens no timeout.
  goal.mode.pulses = 1000;  // spezifies the amount of pulses which will be executed. eg: mode = flash, pulses = 2. Means the light will flash two times
  //goal.mode.priority = 0; // priority [-20,20] default = 0. Modes with same or higher priorities will be executed.

  ROS_INFO("Sending goal");
  ptrSetLightModeClient->sendGoal(goal);
  ROS_INFO("Waiting for resut");
  ptrSetLightModeClient->waitForResult();

  if (ptrSetLightModeClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The robot has flash warning signal and is ready to move.");
  }
}

/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "PublisherAndListener");

  ros::NodeHandle n;
  UH_CobLightControllerClient lightControllerClient(n);


  std_msgs::ColorRGBA blue;
  blue.r = 0;     blue.g = 0;    blue.b = 1;      blue.a = 1;

  lightControllerClient.init();
  lightControllerClient.sendGoal(blue, 1);

  ros::spin();

  return 0;
}
*/
