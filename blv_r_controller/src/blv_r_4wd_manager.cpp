/**************************************************************************
MIT License

Copyright (c) 2025 Panasonic Advanced Technology Development Co.,Ltd.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
***************************************************************************/
/*!
 * @file blv_r_4wd_manager.cpp
 * @date 2024/10/01
 * @brief   OM BLV_R motor driver 管理クラス for 4 wheel drive
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "ros/ros.h"
#include "robot_driver_fw/mecanum_robot_driver_ros.h"
#include "robot_driver_fw/mecanum_robot_driver.h"
#include "blv_r_controller/blv_r_4wd_manager.h"
#include "blv_r_controller/blv_r_4wd_controller.h"

namespace robot_driver_fw
{
Blvr4wdManager::Blvr4wdManager(ros::NodeHandle &nh) : BlvrManager(nh)
{
  getConfig(nh);
}

Blvr4wdManager::~Blvr4wdManager()
{
}

void Blvr4wdManager::checkParam()
{
}

void Blvr4wdManager::getConfig(ros::NodeHandle &)
{
  checkParam();
}

int32_t Blvr4wdManager::init(MotorDriverIf::InitParam &param)
{
  ROS_DEBUG("call Blvr4wdManager::init()");

  int32_t ret;

  controller_ = new Blvr4wdController(nh_);
  if (controller_ == nullptr)
  {
    ROS_ERROR("create Blvr4wdController object failed");
    return RobotDriverErr::INTERNAL_ERR;
  }

  ret = dynamic_cast<Blvr4wdController*>(controller_)->init();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("init failed (%d)", ret);
  }
  else
  {
    controller_->setAcceleration(param.vel_acc, param.vel_dec);
  }

  return ret;
}

void Blvr4wdManager::setMotorRPM(double fl, double fr, double rl, double rr)
{
  ROS_DEBUG("call setMotorRPM (fl:%f, fr:%f, rl:%f, rr:%f)", fl, fr, rl, rr);

  int32_t ret;
  ret = dynamic_cast<Blvr4wdController*>(controller_)->setMotorRpm(fl, fr, rl, rr);
  if (ret != RobotDriverErr::NOERR)
  {
    if (ret == RobotDriverErr::BUSY)
    {
      ROS_WARN("Motor is busy (%d)", ret);
    }
    else if (ret == RobotDriverErr::BAD_STATE)
    {
      ROS_DEBUG("Motor is not running (%d)", ret);
    }
    else
    {
      ROS_ERROR("setMotorRpm failed (%d)", ret);
    }
  }
}

void Blvr4wdManager::setMotorPOS(double fl, double fr, double rl, double rr)
{
  ROS_DEBUG("call setMotorPOS (fl:%f, fr:%f, rl:%f, rr:%f)", fl, fr, rl, rr);

  int32_t ret;
  ret = dynamic_cast<Blvr4wdController*>(controller_)->setMotorPos(fl, fr, rl, rr);
  if (ret != RobotDriverErr::NOERR)
  {
    if (ret == RobotDriverErr::BUSY)
    {
      ROS_WARN("Motor is busy (%d)", ret);
    }
    else if (ret == RobotDriverErr::BAD_STATE)
    {
      ROS_DEBUG("Motor is not running (%d)", ret);
    }
    else
    {
      ROS_ERROR("setMotorPos failed (%d)", ret);
    }
  }
}

void Blvr4wdManager::encoderPollingTimerCallback()
{
  int32_t ret;
  double fl;
  double fr;
  double rl;
  double rr;
  ret = dynamic_cast<Blvr4wdController*>(controller_)->encoderPolling(fl, fr, rl, rr);
  if (ret != RobotDriverErr::NOERR)
  {
    if (ret == RobotDriverErr::BAD_STATE)
    {
      ROS_DEBUG("Motor is not running (%d)", ret);
    }
    else
    {
      ROS_ERROR("encoderPolling failed (%d)", ret);
    }
  }
  else
  {
    if (mecanum_encoder_callback_ != nullptr)
    {
      mecanum_encoder_callback_(fl, fr, rl, rr);
    }
  }
}
};  // namespace robot_driver_fw
