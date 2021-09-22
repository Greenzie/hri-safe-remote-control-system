/*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * ROS Includes
 */
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt32.h"

/**
 * System Includes
 */
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/resource.h>

/**
 * Includes
 */
#include <hri_safe_remote_control_system/VscProcess.h>
#include <hri_safe_remote_control_system/JoystickHandler.h>
#include <hri_safe_remote_control_system/VehicleInterface.h>
#include <hri_safe_remote_control_system/VehicleMessages.h>

using namespace hri_safe_remote_control_system;

VscProcess::VscProcess() : myEStopState(0)
{
  ros::NodeHandle nh("~");
  if (nh.getParam("port", serial_port_))
  {
    ROS_INFO("Serial Port updated to:  %s", serial_port_.c_str());
  }

  if (nh.getParam("serial_speed", serial_speed_))
  {
    ROS_INFO("Serial Port Speed updated to:  %i", serial_speed_);
  }

  /* Open VSC Interface */
  vscInterface = vsc_initialize(serial_port_.c_str(), serial_speed_);
  if (vscInterface == NULL)
  {
    ROS_FATAL("Cannot open serial port! (%s, %i)", serial_port_.c_str(), serial_speed_);
  }
  else
  {
    ROS_INFO("Connected to VSC on %s : %i", serial_port_.c_str(), serial_speed_);
  }

  // Attempt to Set priority
  bool set_priority = false;
  if (nh.getParam("set_priority", set_priority))
  {
    ROS_INFO("Set priority updated to:  %i", set_priority);
  }

  if (set_priority)
  {
    if (setpriority(PRIO_PROCESS, 0, -19) == -1)
    {
      ROS_ERROR("UNABLE TO SET PRIORITY OF PROCESS! (%i, %s)", errno, strerror(errno));
    }
  }

  // Create Message Handlers
  joystickHandler = new JoystickHandler();

  // EStop callback
  estopServ = rosNode.advertiseService("safety/service/send_emergency_stop", &VscProcess::EmergencyStop, this);

  // KeyValue callbacks
  keyValueServ = rosNode.advertiseService("safety/service/key_value", &VscProcess::KeyValue, this);
  keyStringServ = rosNode.advertiseService("safety/service/key_string", &VscProcess::KeyString, this);

  // Publish Emergency Stop Status
  estopPub = rosNode.advertise<std_msgs::UInt32>("safety/emergency_stop", 10);
  
  // Publish Remote Status
  vscModePub = rosNode.advertise<std_msgs::UInt32>("safety/vsc/mode", 10);
  batteryLevelPub = rosNode.advertise<std_msgs::UInt32>("safety/src/battery/level", 10);
  batteryChargingPub = rosNode.advertise<std_msgs::Bool>("safety/src/battery/is_charging", 10);
  vscConnectionStrengthPub = rosNode.advertise<std_msgs::UInt32>("safety/vsc/connection_strength", 10);
  srcConnectionStrengthPub = rosNode.advertise<std_msgs::UInt32>("safety/src/connection_strength", 10);

  // Subscribe for SRC actions
  vibrateSrcSub = rosNode.subscribe("/src_vibrate", 1, &VscProcess::receivedVibration, this);
  displaySrcOnSub = rosNode.subscribe("/src_display_mode_on", 1, &VscProcess::receivedDisplayOnCommand, this);
  displaySrcOffSub = rosNode.subscribe("/src_display_mode_off", 1, &VscProcess::receivedDisplayOffCommand, this);

  // Main Loop Timer Callback
  mainLoopTimer = rosNode.createTimer(ros::Duration(1.0 / VSC_INTERFACE_RATE), &VscProcess::processOneLoop, this);

  // Init last time to now
  lastDataRx = ros::Time::now();

  // Clear all error counters
  memset(&errorCounts, 0, sizeof(errorCounts));
}

VscProcess::~VscProcess()
{
  // before destroying, reset the SRC display for next time
  std_msgs::EmptyConstPtr clear_msg;
  receivedDisplayOffCommand(clear_msg);

  // Destroy vscInterface
  vsc_cleanup(vscInterface);

  if (joystickHandler)
  {
    delete joystickHandler;
  }
}

void VscProcess::receivedVibration(const std_msgs::Bool msg)
{
  bool received_msg = msg.data;
  if (received_msg == true)
  {
    vsc_send_user_feedback(vscInterface, VSC_USER_BOTH_MOTOR_INTENSITY, MOTOR_CONTROL_INTENSITY_HIGH);
  }
}

void VscProcess::checkCharacterLimit(const hri_safe_remote_control_system::SrcDisplay& msg)
{
  // Check if display message is above MAXCHARACTERS for a row in SRC display
  if (msg.displayrow1.size() > msg.MAXCHARACTERS)
  {
    ROS_WARN("Maximum characters limit reached for display row 1. Please enter upto 20 characters.");
  }
  if (msg.displayrow2.size() > msg.MAXCHARACTERS)
  {
    ROS_WARN("Maximum characters limit reached for display row 2. Please enter upto 20 characters.");
  }
  if (msg.displayrow3.size() > msg.MAXCHARACTERS)
  {
    ROS_WARN("Maximum characters limit reached for display row 3. Please enter upto 20 characters.");
  }
  if (msg.displayrow4.size() > msg.MAXCHARACTERS)
  {
    ROS_WARN("Maximum characters limit reached for display row 4. Please enter upto 20 characters.");
  }
}

void VscProcess::receivedDisplayOnCommand(const hri_safe_remote_control_system::SrcDisplay& msg)
{
  // Turn on custom display mode
  vsc_send_user_feedback(vscInterface, VSC_USER_DISPLAY_MODE, DISPLAY_MODE_CUSTOM_TEXT);

  // Checks if the display messages are below the MAXCHARACTERS limit
  checkCharacterLimit(msg);

  // Update Display messages
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_1, msg.displayrow1.c_str());
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_2, msg.displayrow2.c_str());
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_3, msg.displayrow3.c_str());
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_4, msg.displayrow4.c_str());
}

void VscProcess::receivedDisplayOffCommand(const std_msgs::EmptyConstPtr& msg)
{
  hri_safe_remote_control_system::SrcDisplay clear_msg;
  clear_msg.displayrow1 = clear_msg.displayrow2 = clear_msg.displayrow3 = clear_msg.displayrow4 = "";
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_1, clear_msg.displayrow1.c_str());
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_2, clear_msg.displayrow2.c_str());
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_3, clear_msg.displayrow3.c_str());
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_4, clear_msg.displayrow4.c_str());
  vsc_send_user_feedback(vscInterface, VSC_USER_DISPLAY_MODE, DISPLAY_MODE_STANDARD);
}

bool VscProcess::EmergencyStop(EmergencyStop::Request& req, EmergencyStop::Response& res)
{
  myEStopState = (uint32_t)req.EmergencyStop;

  ROS_WARN("VscProcess::EmergencyStop: to 0x%x", myEStopState);

  return true;
}

bool VscProcess::KeyValue(KeyValue::Request& req, KeyValue::Response& res)
{
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback(vscInterface, req.Key, req.Value);

  ROS_INFO("VscProcess::KeyValue: 0x%x, 0x%x", req.Key, req.Value);

  return true;
}

bool VscProcess::KeyString(KeyString::Request& req, KeyString::Response& res)
{
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback_string(vscInterface, req.Key, req.Value.c_str());

  ROS_INFO("VscProcess::KeyValue: 0x%x, %s", req.Key, req.Value.c_str());

  return true;
}

void VscProcess::processOneLoop(const ros::TimerEvent&)
{
  // Send heartbeat message to vehicle in every state
  vsc_send_heartbeat(vscInterface, myEStopState);

  // Check for new data from vehicle in every state
  readFromVehicle();
}

int VscProcess::handleHeartbeatMsg(VscMsgType& recvMsg)
{
  int retVal = 0;

  if (recvMsg.msg.length == sizeof(HeartbeatMsgType))
  {
    ROS_DEBUG("Received Heartbeat from VSC");

    HeartbeatMsgType* msgPtr = (HeartbeatMsgType*)recvMsg.msg.data;

    // Publish Values
    uIntRosPub(&vscModePub, msgPtr->VscMode);
    uIntRosPub(&estopPub, msgPtr->EStopStatus);

    if (msgPtr->EStopStatus > 0)
    {
      ROS_WARN_THROTTLE(5.0, "Received ESTOP from the vehicle!!! 0x%x", msgPtr->EStopStatus);
    }
  }
  else
  {
    ROS_WARN("RECEIVED HEARTBEAT WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
             (unsigned int)sizeof(HeartbeatMsgType),
             recvMsg.msg.length);
    retVal = 1;
  }

  return retVal;
}

void VscProcess::uIntRosPub(ros::Publisher* pubPtr, uint32_t value)
{
  std_msgs::UInt32 ros_uint;
  ros_uint.data = value ;
  pubPtr->publish(ros_uint);
}

int VscProcess::handleRemoteStatusMsg(VscMsgType& recvMsg)
{
  int retVal = 0;

  if (recvMsg.msg.length == sizeof(RemoteStatusMsgType))
  {
    ROS_DEBUG("Received Remote Status Msg from VSC");

    RemoteStatusMsgType* msgPtr = (RemoteStatusMsgType*)recvMsg.msg.data;

    // Publish Status Values
    std_msgs::Bool ros_bool;
    ros_bool.data = msgPtr->battery_charging ;
    batteryChargingPub.publish(ros_bool);

    uIntRosPub(&batteryLevelPub, msgPtr->battery_level);
    uIntRosPub(&vscConnectionStrengthPub, msgPtr->connection_strength_vsc);
    uIntRosPub(&srcConnectionStrengthPub, msgPtr->connection_strength_src);
  }
  else
  {
    ROS_WARN("RECEIVED REMOTE STATUS WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
             (unsigned int)sizeof(HeartbeatMsgType),
             recvMsg.msg.length);
    retVal = 1;
  }

  return retVal;
}

void VscProcess::readFromVehicle()
{
  VscMsgType recvMsg;

  /* Read all messages */
  while (vsc_read_next_msg(vscInterface, &recvMsg) > 0)
  {
    /* Read next Vsc Message */
    switch (recvMsg.msg.msgType)
    {
      case MSG_VSC_HEARTBEAT:
        if (handleHeartbeatMsg(recvMsg) == 0)
        {
          lastDataRx = ros::Time::now();
        }
        break;
      case MSG_VSC_JOYSTICK:
        if (joystickHandler->handleNewMsg(recvMsg) == 0)
        {
          lastDataRx = ros::Time::now();
        }
        break;
      case MSG_USER_REMOTE_STATUS:
        if(handleRemoteStatusMsg(recvMsg) == 0)
        {
          lastDataRx = ros::Time::now();
        }
        break; 
      case MSG_VSC_NMEA_STRING:
        //			handleGpsMsg(&recvMsg);
        break;
      case MSG_USER_FEEDBACK:
        //			handleFeedbackMsg(&recvMsg);
        break;
      default:
        errorCounts.invalidRxMsgCount++;
        break;
    }
  }

  // Log warning when no data is received
  ros::Duration noDataDuration = ros::Time::now() - lastDataRx;
  if (noDataDuration > ros::Duration(.25))
  {
    ROS_WARN_THROTTLE(.5, "No Data Received in %i.%09i seconds", noDataDuration.sec, noDataDuration.nsec);
    ROS_WARN_STREAM("Attempting to reconnect to the VSC...");
    /* Open VSC Interface */
    vscInterface = vsc_initialize(serial_port_.c_str(), serial_speed_);
    if (vscInterface == NULL)
    {
      ROS_FATAL("Cannot open serial port! (%s, %i)", serial_port_.c_str(), serial_speed_);
    }
    else
    {
      ROS_INFO("Connected to VSC on %s : %i", serial_port_.c_str(), serial_speed_);
    }
    // On fail, we expect a crash at the moment. This will mean that the node respawns
  }
}
