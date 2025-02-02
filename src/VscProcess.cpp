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

  if (nh.getParam("reconnect_time", reconnect_time_))
  {
    ROS_INFO("Reconnect Time updated to:  %fs", reconnect_time_);
  }

  /* Open VSC Interface */
  vscInterface = vsc_initialize(serial_port_.c_str(), serial_speed_);
  if (vscInterface == NULL)
  {
    ROS_ERROR("Cannot open serial port! (%s, %i)", serial_port_.c_str(), serial_speed_);
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

  // Grab VSC Settings
  readSettings();

  // Create Message Handlers
  joystickHandler = new JoystickHandler();

  // EStop callback
  estopServ = rosNode.advertiseService("safety/service/send_emergency_stop", &VscProcess::EmergencyStop, this);

  // KeyValue callbacks
  keyValueServ = rosNode.advertiseService("safety/service/key_value", &VscProcess::KeyValue, this);
  keyStringServ = rosNode.advertiseService("safety/service/key_string", &VscProcess::KeyString, this);

  vscSettingServ = rosNode.advertiseService("vsc/settings", &VscProcess::vscSettingsSrv, this);

  // Publish Emergency Stop Status
  estopPub = rosNode.advertise<std_msgs::UInt32>("safety/emergency_stop", 10);
  
  // Publish Vsc Health
  srcHealthPub = rosNode.advertise<hri_safe_remote_control_system::SrcHealth>("safety/health_status", 10);

  // Subscribe for SRC actions
  vibrateSrcSub = rosNode.subscribe("/src_vibrate", 1, &VscProcess::receivedVibration, this);
  displaySrcOnSub1 = rosNode.subscribe("/src_display_mode_on_1", 1, &VscProcess::receivedDisplayOnCommand1, this);
  displaySrcOnSub2 = rosNode.subscribe("/src_display_mode_on_2", 1, &VscProcess::receivedDisplayOnCommand2, this);
  displaySrcOnSub3 = rosNode.subscribe("/src_display_mode_on_3", 1, &VscProcess::receivedDisplayOnCommand3, this);
  displaySrcOnSub4 = rosNode.subscribe("/src_display_mode_on_4", 1, &VscProcess::receivedDisplayOnCommand4, this);
  displaySrcOffSub = rosNode.subscribe("/src_display_mode_off", 1, &VscProcess::receivedDisplayOffCommand, this);

  // Main Loop Timer Callback
  mainLoopTimer = rosNode.createTimer(ros::Duration(1.0 / VSC_INTERFACE_RATE), &VscProcess::processOneLoop, this);

  // Init last time to now
  lastDataRx = ros::Time::now();
  lastRemoteStatusRxTime = lastDataRx;
  lastReconnectAttempt = lastDataRx;

  // Clear all error counters
  memset(&errorCounts, 0, sizeof(errorCounts));
}

VscProcess::~VscProcess()
{
  // before destroying, reset the SRC display for next time
  std_msgs::EmptyConstPtr clear_msg;
  receivedDisplayOffCommand(clear_msg);

  if (vscInterface != NULL)
  {
    // Destroy vscInterface
    vsc_cleanup(vscInterface);
  }
  if (joystickHandler)
  {
    delete joystickHandler;
  }
}

void VscProcess::receivedVibration(const std_msgs::Bool msg)
{
  bool received_msg = msg.data;
  if (received_msg == true && vscInterface != NULL)
  {
    vsc_send_user_feedback(vscInterface, VSC_USER_BOTH_MOTOR_INTENSITY, MOTOR_CONTROL_INTENSITY_HIGH);
  }
}

void VscProcess::receivedDisplayOnCommand1(const std_msgs::StringConstPtr& msg)
{
  if (vscInterface == NULL)
  {
    return;
  }
  // Turn on custom display mode
  vsc_send_user_feedback(vscInterface, VSC_USER_DISPLAY_MODE, DISPLAY_MODE_CUSTOM_TEXT);

  // Update Display message
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_1, msg->data.c_str());
}

void VscProcess::receivedDisplayOnCommand2(const std_msgs::StringConstPtr& msg)
{
  if (vscInterface == NULL)
  {
    return;
  }
  // Turn on custom display mode
  vsc_send_user_feedback(vscInterface, VSC_USER_DISPLAY_MODE, DISPLAY_MODE_CUSTOM_TEXT);

  // Update Display message
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_2, msg->data.c_str());
}

void VscProcess::receivedDisplayOnCommand3(const std_msgs::StringConstPtr& msg)
{
  if (vscInterface == NULL)
  {
    return;
  }
  // Turn on custom display mode
  vsc_send_user_feedback(vscInterface, VSC_USER_DISPLAY_MODE, DISPLAY_MODE_CUSTOM_TEXT);

  // Update Display message
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_3, msg->data.c_str());
}

void VscProcess::receivedDisplayOnCommand4(const std_msgs::StringConstPtr& msg)
{
  if (vscInterface == NULL)
  {
    return;
  }
  // Turn on custom display mode
  vsc_send_user_feedback(vscInterface, VSC_USER_DISPLAY_MODE, DISPLAY_MODE_CUSTOM_TEXT);

  // Update Display message
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_4, msg->data.c_str());
}

void VscProcess::receivedDisplayOffCommand(const std_msgs::EmptyConstPtr& msg)
{
  if (vscInterface == NULL)
  {
    return;
  }

  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_1, "");
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_2, "");
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_3, "");
  vsc_send_user_feedback_string(vscInterface, VSC_USER_DISPLAY_ROW_4, "");
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
  if (vscInterface == NULL)
  {
    return false;
  }
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback(vscInterface, req.Key, req.Value);

  ROS_INFO("VscProcess::KeyValue: 0x%x, 0x%x", req.Key, req.Value);

  return true;
}

bool VscProcess::KeyString(KeyString::Request& req, KeyString::Response& res)
{
  if (vscInterface == NULL)
  {
    return false;
  }
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback_string(vscInterface, req.Key, req.Value.c_str());

  ROS_INFO("VscProcess::KeyValue: 0x%x, %s", req.Key, req.Value.c_str());

  return true;
}

bool VscProcess::vscSettingsSrv(GetVscSettings::Request  &req, GetVscSettings::Response &res)
{
  res.srv_ready = srv_ready;
  res.serial_number = serial;
  res.firmware_version = firmware;
  res.radio_power_db = radio_power_db;

  return true;
}

void VscProcess::processOneLoop(const ros::TimerEvent&)
{
  if (vscInterface != NULL)
  {
    // Send heartbeat message to vehicle in every state
    vsc_send_heartbeat(vscInterface, myEStopState);
  }

  // Check for new data from vehicle in every state
  readFromVehicle();

  if (have_serial && have_radio_power_db && have_firmware && !srv_ready)
  {
    srv_ready = true;
    ROS_INFO("Settings Grabbed from VSC, service is ready..");
  }
  else if (!have_serial || !have_radio_power_db || !have_firmware)
  {
    readSettings();
  }
}

int VscProcess::handleHeartbeatMsg(VscMsgType& recvMsg)
{
  int retVal = 0;

  if (recvMsg.msg.length == sizeof(HeartbeatMsgType))
  {
    ROS_DEBUG("Received Heartbeat from VSC");

    HeartbeatMsgType* msgPtr = (HeartbeatMsgType*)recvMsg.msg.data;
    latest_vsc_mode_ = msgPtr->VscMode ;

    // Publish Values
    std_msgs::UInt32 estopValue;
    estopValue.data = msgPtr->EStopStatus;
    estopPub.publish(estopValue);

    // Static values allow us to detect when the state changes
    static bool prev_estop_vehicle = false;
    static bool prev_estop_src = false;
    static bool prev_estop_any = false;

    bool estop_vehicle = (msgPtr->EStopStatus >> 2) & 0x01;
    bool estop_src = msgPtr->EStopStatus & 0x01;
    bool estop_any = msgPtr->EStopStatus > 0;

    // Print a ROS message when any ESTOP becomes active
    if (estop_any && !prev_estop_any)
    {
      ROS_WARN("VscProcess: ESTOP now ACTIVE!!! 0x%x", msgPtr->EStopStatus);
    }

    // Print a ROS message when the Vehicle ESTOP becomes active or inactive
    if (estop_vehicle && !prev_estop_vehicle)
    {
      ROS_WARN("VscProcess: Received ESTOP on vehicle is now ACTIVE!!! 0x%x", msgPtr->EStopStatus);
    }
    else if (!estop_vehicle && prev_estop_vehicle)
    {
      ROS_WARN("VscProcess: Received ESTOP on vehicle is NO LONGER ACTIVE 0x%x", msgPtr->EStopStatus);
    }

    // Print a ROS message when the SRC ESTOP becomes active or inactive
    if (estop_src && !prev_estop_src)
    {
      ROS_WARN("VscProcess: Received ESTOP on SRC is now ACTIVE!!! 0x%x", msgPtr->EStopStatus);
    }
    else if (!estop_src && prev_estop_src)
    {
      ROS_WARN("VscProcess: Received ESTOP on SRC is NO LONGER ACTIVE 0x%x", msgPtr->EStopStatus);
    }

    // Print a ROS message if all ESTOPs are now clear
    if (!estop_any && prev_estop_any)
    {
      ROS_WARN("VscProcess: All ESTOPS now inactive 0x%x", msgPtr->EStopStatus);
    }

    // Updated previous values values
    prev_estop_vehicle = estop_vehicle;
    prev_estop_src = estop_src;
    prev_estop_any = estop_any;
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

int VscProcess::handleRemoteStatusMsg(VscMsgType& recvMsg)
{
  int retVal = 0;

  if (recvMsg.msg.length == (sizeof(*srcHealthMsg)-sizeof(latest_vsc_mode_)))
  {
    ROS_DEBUG("Received Remote Status Msg from VSC");
    lastRemoteStatusRxTime = ros::Time::now();

    // Publish Status Values
    srcHealthMsg = (SrcHealth*)recvMsg.msg.data;
    srcHealthMsg->vsc_mode = latest_vsc_mode_;
    srcHealthPub.publish(*srcHealthMsg);
  }
  else
  {
    ROS_WARN("RECEIVED REMOTE STATUS WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
             (unsigned int)(sizeof(*srcHealthMsg)-sizeof(latest_vsc_mode_)),
             recvMsg.msg.length);
    retVal = 1;
  }

  return retVal;
}

int VscProcess::handleGetSettingInt(VscMsgType& recvMsg)
{
  int retVal = 0;
  if (recvMsg.msg.length == (sizeof(uint8_t) + sizeof(int32_t)))
  {
    std::stringstream ss;
    for (size_t i = 0; i < recvMsg.msg.length + VSC_HEADER_OVERHEAD + VSC_FOOTER_OVERHEAD; i++)
    {
        ss << "\\x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(recvMsg.msg.buffer[i]);
    }

    if (recvMsg.msg.data[0] == VSC_SETUP_KEY_RADIO_POWER_LEVEL)
    {
      std::stringstream ss;
      for (size_t i = 0; i < recvMsg.msg.length; i++)
      {
          ss << "\\x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(recvMsg.msg.data[i]);
      }
      
      radio_power_db = recvMsg.msg.data[1] | (recvMsg.msg.data[2] << 8) | (recvMsg.msg.data[3] << 16) | (recvMsg.msg.data[4] << 24);

      have_radio_power_db = true;
      ROS_INFO("Acquired VSC Radio Power Level");
    }
  }
  else
  {
    ROS_WARN("RECEIVED SETTING MESSAGE WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
             (unsigned int)(sizeof(uint8_t) + sizeof(int32_t)),
             recvMsg.msg.length);
    retVal = 1;
  }

  return retVal;
}

int VscProcess::handleGetSettingString(VscMsgType& recvMsg)
{
  int retVal = 0;

  if (recvMsg.msg.length == (sizeof(uint8_t) + VSC_SETTING_STRING_LENGTH))
  {
    std::stringstream ss;
    for (size_t i = 0; i < recvMsg.msg.length + VSC_HEADER_OVERHEAD + VSC_FOOTER_OVERHEAD; i++)
    {
        ss << "\\x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(recvMsg.msg.buffer[i]);
    }

    if (recvMsg.msg.data[0] == VSC_SETUP_KEY_SERIAL)
    {
      serial.assign(reinterpret_cast<const char*>(recvMsg.msg.data+1), VSC_SETTING_SERIAL_LENGTH);

      have_serial = true;
      ROS_INFO("Acquired VSC Firmware Version");
    }
    else if (recvMsg.msg.data[0] == VSC_SETUP_KEY_FIRMWARE)
    {
      firmware.assign(reinterpret_cast<const char*>(recvMsg.msg.data+1), VSC_SETTING_FIRMWARE_LENGTH);

      have_firmware = true;
      ROS_INFO("Acquired VSC Firmware Version");
    }
  }
  else
  {
    ROS_WARN("RECEIVED SETTING MESSAGE WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
             (unsigned int)(sizeof(uint8_t) + VSC_SETTING_STRING_LENGTH),
             recvMsg.msg.length);
    retVal = 1;
  }

  return retVal;
}

void VscProcess::readFromVehicle()
{
  VscMsgType recvMsg;

  if (vscInterface != NULL)
  {
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
        case MSG_VSC_REMOTE_STATUS:
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
        case MSG_SETUP_KEY_INT_2:
          //			handleGetSettingInt2(&recvMsg);
          break;
        case MSG_SETUP_KEY_INT_1:
          if(handleGetSettingInt(recvMsg) == 0)
          {
            lastDataRx = ros::Time::now();
          }
          break;
        case MSG_SETUP_KEY_STRING_1:
          if(handleGetSettingString(recvMsg) == 0)
          {
            lastDataRx = ros::Time::now();
          }
          break;
        default:
          ROS_WARN("Received Invalid Message of type: %02x", recvMsg.msg.msgType);
          errorCounts.invalidRxMsgCount++;
          break;
      }
    }
  }

  // Log warning when no data is received
  ros::Time curr_time = ros::Time::now();
  ros::Duration noDataDuration = curr_time - lastDataRx;
  ros::Duration reconnectTimeout = curr_time - lastReconnectAttempt;
  if (noDataDuration > ros::Duration(.25) && reconnectTimeout > ros::Duration(reconnect_time_))
  {
    ROS_WARN_THROTTLE(.5, "No Data Received in %i.%09i seconds", noDataDuration.sec, noDataDuration.nsec);
    ROS_WARN_STREAM("Attempting to reconnect to the VSC...");
    
    lastReconnectAttempt = curr_time;

    /* Open VSC Interface */
    vscInterface = vsc_initialize(serial_port_.c_str(), serial_speed_);
    if (vscInterface == NULL)
    {
      ROS_ERROR_THROTTLE(0.5,"Cannot open serial port! (%s, %i)", serial_port_.c_str(), serial_speed_);
    }
    else
    {
      ROS_INFO("Connected to VSC on %s : %i", serial_port_.c_str(), serial_speed_);
    }

  }
  ros::Duration noRemoteStatusDuration = curr_time - lastRemoteStatusRxTime;
  if (vscInterface != NULL && noRemoteStatusDuration > ros::Duration(2.0))
  {
    ROS_WARN_THROTTLE(.5, "No Remote Status Received in %i.%09i seconds", noRemoteStatusDuration.sec, noRemoteStatusDuration.nsec);
    ROS_WARN_THROTTLE(.5, "Attempting to send the control rate msg to VSC...");
    uint8_t enableMessage = 1;
    uint16_t milliSecondInterval = 1000;
    /* Enable Remote Status Messages */
    vsc_send_control_msg_rate(vscInterface, MSG_VSC_REMOTE_STATUS, enableMessage, milliSecondInterval);
    // reset rx time to allow time for VSC to send a message
    lastRemoteStatusRxTime = curr_time;   
  }
}

void VscProcess::readSettings()
{
  if (vscInterface != NULL)
  {
    vsc_scm_target_set(vscInterface, 0);
    vsc_scm_target_get(vscInterface);

    vsc_setup_unlock(vscInterface);
    vsc_get_setting(vscInterface, VSC_SETUP_KEY_RADIO_POWER_LEVEL);
    vsc_get_setting(vscInterface, VSC_SETUP_KEY_SERIAL);
    vsc_get_setting(vscInterface, VSC_SETUP_KEY_FIRMWARE);

    vsc_setup_unlock(vscInterface);
    vsc_get_setting_int(vscInterface, VSC_SETUP_KEY_RADIO_POWER_LEVEL);
    vsc_get_setting_int(vscInterface, VSC_SETUP_KEY_SERIAL);
    vsc_get_setting_int(vscInterface, VSC_SETUP_KEY_FIRMWARE);

    vsc_setup_unlock(vscInterface);
    vsc_get_setting_string(vscInterface, VSC_SETUP_KEY_RADIO_POWER_LEVEL);
    vsc_get_setting_string(vscInterface, VSC_SETUP_KEY_SERIAL);
    vsc_get_setting_string(vscInterface, VSC_SETUP_KEY_FIRMWARE);
  }
}
