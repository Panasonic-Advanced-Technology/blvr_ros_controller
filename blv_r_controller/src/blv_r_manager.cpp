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
 * @file blv_r_manager.cpp
 * @date 2024/02/06
 * @brief   OM BLV_R motor driver 管理クラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "ros/ros.h"
#include "robot_driver_fw/robot_driver_ros.h"
#include "robot_driver_fw/robot_driver.h"
#include "blv_r_controller/blv_r_manager.h"
#include "blv_r_controller/blv_r_controller.h"

namespace robot_driver_fw
{
BlvrManager::BlvrManager(ros::NodeHandle &nh) :
nh_(nh)
{
  getConfig(nh);
}

BlvrManager::~BlvrManager()
{
}

void BlvrManager::checkParam()
{
}

void BlvrManager::getConfig(ros::NodeHandle &)
{
  checkParam();
}

int32_t BlvrManager::init(MotorDriverIf::InitParam &param)
{
  ROS_DEBUG("call BlvrManager::init()");

  int32_t ret;

  controller_ = new BlvrController(nh_);
  if (controller_ == nullptr)
  {
    ROS_ERROR("create BlvrController object failed");
    return RobotDriverErr::INTERNAL_ERR;
  }

  ret = controller_->init();
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

int32_t BlvrManager::fini()
{
  int32_t ret;
  ret = controller_->fini();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("fini failed (%d)", ret);
  }
  return ret;
}

int32_t BlvrManager::reset()
{
  int32_t ret;
  ret = controller_->reset();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("reset failed (%d)", ret);
  }
  return ret;
}

int32_t BlvrManager::emgStop()
{
  int32_t ret;
  ret = controller_->emgStop();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("emgStop failed (%d)", ret);
  }
  return ret;
}

void BlvrManager::setMotorRPM(double left, double right)
{
  ROS_DEBUG("call setMotorRPM (left:%f, right:%f)", left, right);

  int32_t ret;
  ret = controller_->setMotorRpm(left, right);
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

void BlvrManager::setMotorPOS(double left, double right)
{
  ROS_DEBUG("call setMotorPOS (left:%f, right:%f)", left, right);

  int32_t ret;
  ret = controller_->setMotorPos(left, right);
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

int32_t BlvrManager::configSetPos(float vel, float acc, float dec)
{
  int32_t ret;
  ret = controller_->configSetPosition(vel, acc, dec);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("configSetPos failed (%d)", ret);
  }
  return ret;
}

int32_t BlvrManager::getMotorStatus(std::vector<robot_driver_msgs::MotorStatus> &status)
{
  int32_t ret;
  status.resize(0);
  ret = controller_->getStatus(status);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("getStatus failed (%d)", ret);
  }
  return ret;
}

int32_t BlvrManager::controlMotorExcitation(bool state)
{
  int32_t ret;
  ret = controller_->controlMotorExcitation(state);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("controlMotorExcitation failed (%d)", ret);
  }
  return ret;
}

void BlvrManager::encoderPollingTimerCallback()
{
  int32_t ret;
  double left;
  double right;
  ret = controller_->encoderPolling(left, right);
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
    if (encoder_callback_ != nullptr)
    {
      encoder_callback_(left, right);
    }
  }
}

void BlvrManager::statusPollingTimerCallback()
{
  int32_t ret;
  std::vector<robot_driver_msgs::MotorStatus> list;
  ret = controller_->statusPolling(list);
  if (ret != RobotDriverErr::NOERR)
  {
    if (ret == RobotDriverErr::BAD_STATE)
    {
      ROS_DEBUG("Motor is not running (%d)", ret);
    }
    else
    {
      ROS_ERROR("statusPolling failed (%d)", ret);
    }
  }
  else
  {
    if (status_callback_ != nullptr)
    {
      status_callback_(list);
    }
  }
}

};  // namespace robot_driver_fw
