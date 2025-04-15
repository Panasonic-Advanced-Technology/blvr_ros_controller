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
 * @file mecanum_odometry.cpp
 * @date 2024/03/04
 * @brief   メカナムホイール用オドメトリ
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "robot_driver_fw/mecanum_odometry.h"

namespace robot_driver_fw
{
  MecanumOdometry::MecanumOdometry() :
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  timestamp_(0.0),
  vx_(0.0),
  vy_(0.0),
  w_(0.0)
  {}

  MecanumOdometry::~MecanumOdometry()
  {}

  void MecanumOdometry::init()
  {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    timestamp_ = ros::Time::now();
    vx_ = 0.0;
    vy_ = 0.0;
    w_ = 0.0;
  }

  void MecanumOdometry::setWheelParams(double tread_width, double wheel_base, double wheel_radius)
  {
    tread_width_ = tread_width;
    wheel_base_ = wheel_base;
    wheel_radius_ = wheel_radius;
  }

  bool MecanumOdometry::update(double fl, double fr, double rl, double rr, ros::Time &time)
  {
    vx_ = (fl + fr + rl + rr) * wheel_radius_ / 4.0;
    vy_ = (-fl + fr + rl - rr) * wheel_radius_ / 4.0;
    w_ = (-fl + fr - rl + rr) * wheel_radius_ / (4.0 * ((tread_width_ + wheel_base_) / 2.0));

    double dt = (time - timestamp_).toSec();
    double dx = (vx_ * cos(yaw_) - vy_ * sin(yaw_)) * dt;
    double dy = (vx_ * sin(yaw_) + vy_ * cos(yaw_)) * dt;
    double dth = w_ * dt;

    x_ += dx;
    y_ += dy;
    yaw_ += dth;
    timestamp_ = time;

    return true;
  }

  double MecanumOdometry::getX()
  {
    return x_;
  }

  double MecanumOdometry::getY()
  {
    return y_;
  }

  double MecanumOdometry::getYaw()
  {
    return yaw_;
  }

  double MecanumOdometry::getVX()
  {
    return vx_;
  }

  double MecanumOdometry::getVY()
  {
    return vy_;
  }

  double MecanumOdometry::getW()
  {
    return w_;
  }
};  // namespace robot_driver_fw
