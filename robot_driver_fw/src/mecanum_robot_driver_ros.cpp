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
 * @file mecanum_robot_driver_ros.cpp
 * @date 2024/04/02
 * @brief   robot driver ROS クラス メカナムホイール拡張
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "robot_driver_fw/mecanum_robot_driver_ros.h"
#include "robot_driver_fw/mecanum_robot_driver.h"

namespace robot_driver_fw
{
  MecanumRobotDriverRos::~MecanumRobotDriverRos()
  {
    if (rb_driver_ != nullptr)
    {
      delete(rb_driver_);
    }
  }

  int32_t MecanumRobotDriverRos::init()
  {
    if (initialized_ == true)
    {
      ROS_ERROR("aleady initialized");
      return RobotDriverErr::BAD_STATE;
    }

    // create subscriber
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MecanumRobotDriverRos::cmdVelCallback, this);
    cmd_pos_sub_ = nh_.subscribe("cmd_pos", 1, &MecanumRobotDriverRos::cmdPosCallback, this);

    // set callback
    dynamic_cast<MecanumMotorDriverIf*>(mdif_)->setMecanumEncoderCallback(&MecanumRobotDriverRos::encoderCallback, this);

    rb_driver_ = new MecanumRobotDriver(nh_, mdif_);
    if (rb_driver_ == nullptr)
    {
      ROS_ERROR("create MecanumRobotDriver object failed");
      return RobotDriverErr::INTERNAL_ERR;
    }

    int32_t ret;
    ret = dynamic_cast<MecanumRobotDriver*>(rb_driver_)->init();
    if (ret == RobotDriverErr::NOERR)
    {
      initialized_ = true;
    }
    else
    {
      ROS_ERROR("init failed");
    }
    return ret;
  }

  void MecanumRobotDriverRos::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (velocity_update_)
    {
      dynamic_cast<MecanumRobotDriver*>(rb_driver_)->setTwist(msg->linear.x, msg->linear.y, msg->angular.z);
      velocity_update_ = false;
    }
  }

  void MecanumRobotDriverRos::cmdPosCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (velocity_update_)
    {
      dynamic_cast<MecanumRobotDriver*>(rb_driver_)->setPos(msg->linear.x, msg->linear.y, msg->angular.z);
      velocity_update_ = false;
    }
  }

  void MecanumRobotDriverRos::encoderCallback(double fl, double fr, double rl, double rr)
  {
    ROS_DEBUG("encoderCallback(fl=%f, fr=%f, rl=%f, rr=%f)", fl, fr, rl, rr);

    // odometry
    nav_msgs::Odometry odom;
    odom.header.frame_id = frame_id_.c_str();
    odom.child_frame_id = child_frame_id_.c_str();
    dynamic_cast<MecanumRobotDriver*>(rb_driver_)->updateOdometry(fl, fr, rl, rr, odom);

    // publish
    odometry_pub_.publish(odom);
  }

};  // namespace robot_driver_fw
