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

#ifndef __VSC_PROCESS_INCLUDED__
#define __VSC_PROCESS_INCLUDED__

/**
 * ROS Includes
 */
#include "ros/ros.h"
#include "hri_safe_remote_control_system/EmergencyStop.h"
#include "hri_safe_remote_control_system/KeyValue.h"
#include "hri_safe_remote_control_system/KeyString.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <hri_safe_remote_control_system/SrcDisplay.h>


/**
 * HRI_COMMON Includes
 */
#include "MsgHandler.h"
#include "VehicleMessages.h"
#include "VehicleInterface.h"

namespace hri_safe_remote_control_system {

	// Diagnostics
	struct ErrorCounterType {
		uint32_t sendErrorCount;
		uint32_t invalidRxMsgCount;
	};

	/**
	 * Local Definitions
	 */
	const unsigned int VSC_INTERFACE_RATE = 50; /* 50 Hz */
	const unsigned int VSC_HEARTBEAT_RATE = 20; /* 20 Hz */

	class VscProcess {
	   public:
		  VscProcess();
		  ~VscProcess();

		  // Main loop
		  void processOneLoop(const ros::TimerEvent&);

		  // ROS Callback's
		  bool EmergencyStop(EmergencyStop::Request &req, EmergencyStop::Response &res);
		  bool KeyValue(KeyValue::Request &req, KeyValue::Response &res);
		  bool KeyString(KeyString::Request &req, KeyString::Response &res);

		  void receivedVibration(const std_msgs::Bool msg);
		  void receivedDisplayOnCommand(const hri_safe_remote_control_system::SrcDisplay& msg);
		  void receivedDisplayOffCommand(const std_msgs::EmptyConstPtr& msg);
		  void checkCharacterLimit(const hri_safe_remote_control_system::SrcDisplay& msg);

	   private:

		  void readFromVehicle();
		  int handleHeartbeatMsg(VscMsgType& recvMsg);
		  hri_safe_remote_control_system::SrcDisplay prev_msg_;

		  // Local State
		  uint32_t 				myEStopState;
		  ErrorCounterType 		errorCounts;

		  // ROS
		  ros::NodeHandle 		rosNode;
		  ros::Timer 	  		mainLoopTimer;
		  ros::ServiceServer    estopServ, keyValueServ, keyStringServ;
		  ros::Publisher		estopPub;
		  ros::Subscriber 		vibrateSrcSub;
		  ros::Subscriber 		displaySrcOnSub;
		  ros::Subscriber 		displaySrcOffSub;
		  ros::Time 			lastDataRx, lastTxTime;

		  // Message Handlers
		  MsgHandler			*joystickHandler;

		  /* File descriptor for VSC Interface */
		  VscInterfaceType		*vscInterface;

	};

} // namespace


#endif
