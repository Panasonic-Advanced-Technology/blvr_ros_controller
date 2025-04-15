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
 * @file mecanum_robot_driver.cpp
 * @date 2024/04/02
 * @brief   robot driver frameworkクラス メカナムホイール拡張
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "robot_driver_fw/mecanum_robot_driver.h"
#include "robot_driver_fw/mecanum_motor_driver_if.h"
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

namespace robot_driver_fw
{
  MecanumRobotDriver::~MecanumRobotDriver()
  {}

  int32_t MecanumRobotDriver::init()
  {
    int32_t ret;

    wheel_base_ = nh_.param(ROBOT_DRIVER_KEY_STR_WHEEL_BASE, ROBOT_DRIVER_DEFAULT_WHEELBASE);

    ret = checkParam();
    if (ret != RobotDriverErr::NOERR)
    {
      return ret;
    }
    if (!isPositiveNumber(ROBOT_DRIVER_KEY_STR_WHEEL_BASE, wheel_base_))
    {
      return RobotDriverErr::BAD_PARAM;
    }

    mecanum_odom_.setWheelParams(tread_width_, wheel_base_, wheel_size_ / 2.0);
    mecanum_odom_.init();

    MotorDriverIf::InitParam param;
    param.vel_acc = vel_acc_;
    param.vel_dec = vel_dec_;
    ret = mdif_->init(param);

    return ret;
  }

  void MecanumRobotDriver::setTwist(double vx, double vy, double omega)
  {
    ROS_DEBUG("setTwist called vx=%f, vy=%f, omega=%f", vx, vy, omega);
    double fl_rpm, fr_rpm, rl_rpm, rr_rpm;

    fl_rpm  = convMpsToRpm(vx - vy - ((tread_width_ + wheel_base_) / 2.0) * omega);
    fr_rpm  = convMpsToRpm(vx + vy + ((tread_width_ + wheel_base_) / 2.0) * omega);
    rl_rpm  = convMpsToRpm(vx + vy - ((tread_width_ + wheel_base_) / 2.0) * omega);
    rr_rpm  = convMpsToRpm(vx - vy + ((tread_width_ + wheel_base_) / 2.0) * omega);

    dynamic_cast<MecanumMotorDriverIf*>(mdif_)->setMotorRPM(fl_rpm, fr_rpm, rl_rpm, rr_rpm);
  }

  void MecanumRobotDriver::setPos(double x, double y, double yaw)
  {
    int32_t ret;
    double fl_r, fr_r, rl_r, rr_r;
    ROS_DEBUG("setPos called x=%f, y=%f, yaw=%f", x, y, yaw);

    if ((x != 0) || (y != 0))
    {
      fl_r = convMToR(x - y);
      fr_r = convMToR(x + y);
      rl_r = convMToR(x + y);
      rr_r = convMToR(x - y);

      // 速度パラメータ設定
      ret = mdif_->configSetPos(pos_linear_vel_, pos_linear_acc_, pos_linear_dec_);
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_ERROR("Linear configSetPos failed (%d)", ret);
        return;
      }
    }
    else if (yaw != 0)
    {
      fl_r = convMToR(-1.0 * ((tread_width_ + wheel_base_) / 2.0) * yaw);
      fr_r = convMToR(((tread_width_ + wheel_base_) / 2.0) * yaw);
      rl_r = convMToR(-1.0 * ((tread_width_ + wheel_base_) / 2.0) * yaw);
      rr_r = convMToR(((tread_width_ + wheel_base_) / 2.0) * yaw);

      // 速度パラメータ設定
      ret = mdif_->configSetPos(pos_angular_vel_, pos_angular_acc_, pos_angular_dec_);
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_ERROR("Angular configSetPos failed (%d)", ret);
        return;
      }
    }
    else
    {
      fl_r = 0;
      fr_r = 0;
      rl_r = 0;
      rr_r = 0;
    }

    dynamic_cast<MecanumMotorDriverIf*>(mdif_)->setMotorPOS(fl_r, fr_r, rl_r, rr_r);
  }

  bool MecanumRobotDriver::updateOdometry(double fl, double fr, double rl, double rr, nav_msgs::Odometry &odom)
  {
    ros::Time now = ros::Time::now();

    if (!mecanum_odom_.update(convRpmToRadps(fl), convRpmToRadps(fr), convRpmToRadps(rl), convRpmToRadps(rr), now))
    {
      ROS_WARN("update failed");
      mecanum_odom_.init();
      return false;
    }
    else
    {
      geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(mecanum_odom_.getYaw());
      odom.header.stamp = now;
      odom.pose.pose.position.x = mecanum_odom_.getX();
      odom.pose.pose.position.y = mecanum_odom_.getY();
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation = quat;
      odom.twist.twist.linear.x = mecanum_odom_.getVX();
      odom.twist.twist.linear.y = mecanum_odom_.getVY();
      odom.twist.twist.linear.z = 0;
      odom.twist.twist.angular.x = 0;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.z = mecanum_odom_.getW();
    }

    return true;
  }

};  // namespace robot_driver_fw
