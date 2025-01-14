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

#ifndef __JOYSTICK_HANDLER_INCLUDED__
#define __JOYSTICK_HANDLER_INCLUDED__

/**
 * Includes
 */
#include "ros/ros.h"

#include "VehicleMessages.h"
#include "MsgHandler.h"

namespace hri_safe_remote_control_system
{
/**
 *
 */
class JoystickHandler : public MsgHandler
{
public:
  JoystickHandler();
  ~JoystickHandler();

  uint32_t handleNewMsg(const VscMsgType& incomingMsg);

private:
  int32_t getStickValue(JoystickType joystick);
  int32_t getButtonValue(uint8_t button);

  ros::NodeHandle rosNode;
  ros::Publisher rawLeftPub, rawRightPub;
  bool useArrowsAsAxes{ false };
};

}  // namespace hri_safe_remote_control_system

#endif
