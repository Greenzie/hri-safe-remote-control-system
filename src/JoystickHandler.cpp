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

#include <VehicleMessages.h>
#include <JoystickHandler.h>

using namespace hri_safe_remote_control_system;

JoystickHandler::JoystickHandler()
{
  // Joystick Pub
  rawLeftPub = rosNode.advertise<sensor_msgs::Joy>("/joy", 10);

  // Boolean param to check if we need to reconfigure /joy messages
  ros::param::get("~useArrowsAsAxes", useArrowsAsAxes);
}

JoystickHandler::~JoystickHandler()
{
}

int32_t JoystickHandler::getStickValue(JoystickType joystick)
{
  int32_t magnitude = (joystick.magnitude << 2) + joystick.mag_lsb;

  if (joystick.neutral_status == STATUS_SET)
  {
    return 0;
  }
  else if (joystick.negative_status == STATUS_SET)
  {
    return -1 * magnitude;
  }
  else if (joystick.positive_status == STATUS_SET)
  {
    return magnitude;
  }

  // Error case
  return 0;
}

int32_t JoystickHandler::getButtonValue(uint8_t button)
{
  if (button == STATUS_SET)
  {
    return 1;
  }

  // Error case
  return 0;
}

uint32_t JoystickHandler::handleNewMsg(const VscMsgType& incomingMsg)
{
  int retval = 0;

  if (incomingMsg.msg.length == sizeof(JoystickMsgType))
  {
    JoystickMsgType* joyMsg = (JoystickMsgType*)incomingMsg.msg.data;

    // Broadcast Left Joystick
    sensor_msgs::Joy sendLeftMsg;

    sendLeftMsg.header.stamp = ros::Time::now();
    sendLeftMsg.header.frame_id = "/srcs";

    sendLeftMsg.axes.push_back((float)getStickValue(joyMsg->leftX));
    sendLeftMsg.axes.push_back((float)getStickValue(joyMsg->leftY));
    sendLeftMsg.axes.push_back((float)getStickValue(joyMsg->leftZ));

    sendLeftMsg.axes.push_back((float)getStickValue(joyMsg->rightX));
    sendLeftMsg.axes.push_back((float)getStickValue(joyMsg->rightY));
    sendLeftMsg.axes.push_back((float)getStickValue(joyMsg->rightZ));

    if (useArrowsAsAxes)
    {
      if (joyMsg->leftSwitch.first == 1)
      {
        sendLeftMsg.axes.push_back(-(float)getButtonValue(joyMsg->leftSwitch.first));
      }
      if (joyMsg->leftSwitch.third == 1)
      {
        sendLeftMsg.axes.push_back((float)getButtonValue(joyMsg->leftSwitch.third));
      }
      if (joyMsg->leftSwitch.first == 0 && joyMsg->leftSwitch.third == 0)
      {
        sendLeftMsg.axes.push_back(0);
      }

      if (joyMsg->leftSwitch.home == 1)
      {
        sendLeftMsg.axes.push_back(-(float)getButtonValue(joyMsg->leftSwitch.home));
      }
      if (joyMsg->leftSwitch.second == 1)
      {
        sendLeftMsg.axes.push_back((float)getButtonValue(joyMsg->leftSwitch.second));
      }
      if (joyMsg->leftSwitch.home == 0 && joyMsg->leftSwitch.second == 0)
      {
        sendLeftMsg.axes.push_back(0);
      }
    }
    else
    {
      sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.home));
      sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.first));
      sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.second));
      sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.third));
    }

    sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.home));
    sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.first));
    sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.second));
    sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.third));

    rawLeftPub.publish(sendLeftMsg);
  }
  else
  {
    retval = -1;

    ROS_WARN("RECEIVED PTZ COMMANDS WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
             (unsigned int)sizeof(JoystickMsgType),
             incomingMsg.msg.length);
  }

  return retval;
}
