#ifndef COBLIGHTCONTROLLERCLIENT_H
#define COBLIGHTCONTROLLERCLIENT_H

#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <cob_light/SetLightModeAction.h>

/* ================================================================================
 MSG: cob_light/LightMode
 uint8 mode
 uint8 NONE = 0                  # will turn everything off
 uint8 STATIC = 1                # will change the LEDs to \"color\"
 uint8 FLASH = 2                 # will change the LEDs frequently with \"frequency\" from \"color\" to black
 uint8 BREATH = 3                # will change the LEDs smoothly with \"frequency\" from \"color\" to black
 uint8 BREATH_COLOR = 4  # will change the LEDs smoothly with \"frequency\" from \"color\" to black and flips color in time
 uint8 FADE_COLOR = 5    # will fade the colors in rainbow

 std_msgs/ColorRGBA color #the color which will be used
 float32 frequency               # in Hz
 float32 timeout                 # in s, requested mode will be executed for max timout s.
                                                 # default is 0 and meens no timeout.
 int32 pulses                    # spezifies the amount of pulses which will be executed.
                                                 # eg: mode = flash, pulses = 2. Meens the light will flash two times
 int8 priority                   # priority [-20,20] default = 0. Modes with same or higher priorities will be executed.
 ================================================================================*/
/*
 * cob_light::LightMode::FLASH
  blue.r = 0;     blue.g = 0;    blue.b = 1;      blue.a = 1;
  red.r  = 1;     red.g = 0;     red.b = 0;       red.a = 1;
  green.r = 0;    green.g = 1;   green.b = 0;     green.a = 1;
  yellow.r = 0.4; yellow.g = 1;  yellow.b = 0;    yellow.a = 1;
  white.r = 0.3;  white.g = 1;   white.b = 0.3;   white.a = 1;
*/

class UH_CobLightControllerClient
{
public:
  //UH_CobLightControllerClient(ros::NodeHandle n):nh(n)
  UH_CobLightControllerClient()
  {

  }

  void init();
  void setLight(std_msgs::ColorRGBA colour, uint mode);

private:
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<cob_light::SetLightModeAction> *ptrSetLightModeClient;
  std_msgs::ColorRGBA blue, red, yellow, green, white;
};


#endif
