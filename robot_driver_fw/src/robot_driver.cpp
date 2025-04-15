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
 * @file robot_driver.cpp
 * @date 2023/09/20
 * @brief   robot driver frameworkクラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "robot_driver_fw/robot_driver.h"
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

namespace robot_driver_fw
{
  std::string convRobotDriverErrToStr(RobotDriverErr code)
  {
    std::string msg;
    switch (code)
    {
      case RobotDriverErr::NOERR:
        msg = "NOERR";
        break;
      case RobotDriverErr::NOT_FOUND:
        msg = "NOT_FOUND";
        break;
      case RobotDriverErr::NOT_PERM:
        msg = "NOT_PERM";
        break;
      case RobotDriverErr::NOT_SUPPORT:
        msg = "NOT_SUPPORT";
        break;
      case RobotDriverErr::BAD_PARAM:
        msg = "BAD_PARAM";
        break;
      case RobotDriverErr::BUSY:
        msg = "BUSY";
        break;
      case RobotDriverErr::BAD_STATE:
        msg = "BAD_STATE";
        break;
      case RobotDriverErr::TIMEOUT:
        msg = "TIMEOUT";
        break;
      case RobotDriverErr::INTERNAL_ERR:
        msg = "INTERNAL_ERR";
        break;
      default:
        msg = "UNKNOWN";
        break;
    }
    return msg;
  }

  RobotDriver::RobotDriver(ros::NodeHandle &nh, MotorDriverIf *mdif) :
  nh_(nh),
  mdif_(mdif)
  {
    getConfig(nh);
  }

  RobotDriver::~RobotDriver()
  {}

  void RobotDriver::getConfig(ros::NodeHandle &nh)
  {
    gear_ratio_ = nh.param(ROBOT_DRIVER_KEY_STR_GEAR_RATIO, ROBOT_DRIVER_DEFAULT_GEARRATIO);
    wheel_size_ = nh.param(ROBOT_DRIVER_KEY_STR_WHEEL_SIZE, ROBOT_DRIVER_DEFAULT_WHEEL);
    tread_width_ = nh.param(ROBOT_DRIVER_KEY_STR_TREAD_WIDTH, ROBOT_DRIVER_DEFAULT_TREAD);
    vel_acc_ = convMpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_ACC_IN_VEL, ROBOT_DRIVER_DEFAULT_ACC_IN_VEL));
    vel_dec_ = convMpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_DEC_IN_VEL, ROBOT_DRIVER_DEFAULT_DEC_IN_VEL));
    pos_linear_vel_ = convMpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_LIN_VEL_POS, ROBOT_DRIVER_DEFAULT_LINEAR_VEL));
    pos_linear_acc_ = convMpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_LIN_ACC_POS, ROBOT_DRIVER_DEFAULT_LINEAR_ACC));
    pos_linear_dec_ = convMpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_LIN_DEC_POS, ROBOT_DRIVER_DEFAULT_LINEAR_DEC));
    pos_angular_vel_ =
      convTurnRadpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_ANG_VEL_POS, ROBOT_DRIVER_DEFAULT_ANGULAR_VEL));
    pos_angular_acc_ =
      convTurnRadpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_ANG_ACC_POS, ROBOT_DRIVER_DEFAULT_ANGULAR_ACC));
    pos_angular_dec_ =
      convTurnRadpsToRpm(nh.param(ROBOT_DRIVER_KEY_STR_ANG_DEC_POS, ROBOT_DRIVER_DEFAULT_ANGULAR_DEC));
  }

  bool RobotDriver::isPositiveNumber(const char *str, double val)
  {
    if (val <= 0)
    {
      ROS_ERROR("%s is invalid.(%lf <= 0)", str, val);
      return false;
    }
    return true;
  }

  bool RobotDriver::isPositiveNumber(const char *str, float val)
  {
    if (val <= 0)
    {
      ROS_ERROR("%s is invalid.(%lf <= 0)", str, val);
      return false;
    }
    return true;
  }

  int32_t RobotDriver::checkParam()
  {
    bool ret = true;
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_GEAR_RATIO, gear_ratio_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_WHEEL_SIZE, wheel_size_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_TREAD_WIDTH, tread_width_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_ACC_IN_VEL, vel_acc_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_DEC_IN_VEL, vel_dec_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_LIN_VEL_POS, pos_linear_vel_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_LIN_ACC_POS, pos_linear_acc_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_LIN_DEC_POS, pos_linear_dec_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_ANG_VEL_POS, pos_angular_vel_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_ANG_ACC_POS, pos_angular_acc_);
    ret &= isPositiveNumber(ROBOT_DRIVER_KEY_STR_ANG_DEC_POS, pos_angular_dec_);
    if (ret)
    {
      return RobotDriverErr::NOERR;
    }
    else
    {
      return RobotDriverErr::BAD_PARAM;
    }
  }

  int32_t RobotDriver::init()
  {
    int32_t ret;

    ret = checkParam();
    if (ret != RobotDriverErr::NOERR)
    {
      return ret;
    }

    odom_.setWheelParams(tread_width_, wheel_size_ / 2.0, wheel_size_ / 2.0);
    odom_.init(ros::Time::now());

    MotorDriverIf::InitParam param;
    param.vel_acc = vel_acc_;
    param.vel_dec = vel_dec_;
    ret = mdif_->init(param);

    return ret;
  }

  int32_t RobotDriver::fini()
  {
    int32_t ret;
    ret = mdif_->fini();

    return ret;
  }

  int32_t RobotDriver::reset()
  {
    int32_t ret;
    ret = mdif_->reset();

    return ret;
  }

  void RobotDriver::setTwist(double v, double omega)
  {
    ROS_DEBUG("setTwist called v=%f, omega=%f", v, omega);
    double left_rpm, right_rpm;
    double x_rpm, w_rpm;

    // convert
    x_rpm = convMpsToRpm(v);
    w_rpm = convTurnRadpsToRpm(omega);
    left_rpm = x_rpm - w_rpm;
    right_rpm = x_rpm + w_rpm;
    ROS_DEBUG("left_rpm=%f, right_rpm=%f", left_rpm, right_rpm);

    mdif_->setMotorRPM(left_rpm, right_rpm);
  }

  void RobotDriver::setPos(double x, double yaw)
  {
    int32_t ret;
    double left_rotation, right_rotation;

    // convert
    if (x != 0)
    {
      left_rotation = convMToR(x);
      right_rotation = left_rotation;

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
      right_rotation = convRadToR(yaw);
      left_rotation = -1.0 * right_rotation;

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
      // cancel
      right_rotation = 0;
      left_rotation = 0;
    }
    ROS_DEBUG("left_rotation=%f, right_rotation=%f", left_rotation, right_rotation);

    mdif_->setMotorPOS(left_rotation, right_rotation);
  }

  void RobotDriver::configSetPos(float vel, float acc, float dec, float a_vel, float a_acc, float a_dec)
  {
    // convert
    // m/s --> r/M
    pos_linear_vel_ = convMpsToRpm(vel);
    // m/s^2 --> r/Ms
    pos_linear_acc_ = convMpsToRpm(acc);
    pos_linear_dec_ = convMpsToRpm(dec);
    // rad/s --> r/M
    pos_angular_vel_ = convTurnRadpsToRpm(a_vel);
    // rad/s^2 --> r/Ms
    pos_angular_acc_ = convTurnRadpsToRpm(a_acc);
    pos_angular_dec_ = convTurnRadpsToRpm(a_dec);
  }

  int32_t RobotDriver::getMotorStatus(std::vector<robot_driver_msgs::MotorStatus> &status)
  {
    return mdif_->getMotorStatus(status);
  }

  int32_t RobotDriver::controlMotorExcitation(bool state)
  {
    return mdif_->controlMotorExcitation(state);
  }

  void RobotDriver::encoderPollingTimerCallback(const ros::TimerEvent&)
  {
    mdif_->encoderPollingTimerCallback();
  }

  void RobotDriver::statusPollingTimerCallback(const ros::TimerEvent&)
  {
    mdif_->statusPollingTimerCallback();
  }

  bool RobotDriver::updateOdometry(double left, double right, nav_msgs::Odometry &odom)
  {
    ros::Time now = ros::Time::now();
#if USE_MOTOR_POSITION
    if (!odom_.update(left * 2 * M_PI / gear_ratio_, right * 2 * M_PI / gear_ratio_, now))
#else
    if (!odom_.updateByRPM(left / gear_ratio_, right / gear_ratio_, now))
#endif
    {
      ROS_WARN("updateByRPM failed");
      return false;
    }
    else
    {
      tf2::Quaternion quat;
      odom.header.stamp = now;
      odom.pose.pose.position.x = odom_.getX();
      odom.pose.pose.position.y = odom_.getY();
      odom.pose.pose.position.z = 0;
      quat.setRPY(0, 0, odom_.getHeading());
      odom.pose.pose.orientation.x = quat.getX();
      odom.pose.pose.orientation.y = quat.getY();
      odom.pose.pose.orientation.z = quat.getZ();
      odom.pose.pose.orientation.w = quat.getW();
      odom.twist.twist.linear.x = odom_.getLinear();
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.linear.z = 0;
      odom.twist.twist.angular.x = 0;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.z = odom_.getAngular();
    }
    return true;
  }

  bool RobotDriver::isMatchStatus(std::vector<robot_driver_msgs::MotorStatus> list, RobotDriverStatus status,
                                  int &number)
  {
    for (int i = 0; i < static_cast<int>(list.size()); i++)
    {
      if (list[i].status == status)
      {
        number = i;
        return true;
      }
    }
    number = -1;
    return false;
  }

  bool RobotDriver::isMatchStatus(std::vector<robot_driver_msgs::MotorStatus> list, RobotDriverStatus status)
  {
    int n;
    return isMatchStatus(list, status, n);
  }

  bool RobotDriver::isMatchErrorStatus(std::vector<robot_driver_msgs::MotorStatus> list, int &number)
  {
    for (int i = 0; i < static_cast<int>(list.size()); i++)
    {
      if (((list[i].status == RobotDriverStatus::ERRSTOP) || (list[i].status == RobotDriverStatus::ERRSTOPPING)) &&
          (list[i].err_code != 0))
      {
        number = i;
        return true;
      }
    }
    number = -1;
    return false;
  }

  void RobotDriver::confirmStatus(std::vector<robot_driver_msgs::MotorStatus> status,
                                  bool &emg_status, bool &e_stop, std::string &detail)
  {
    int index;
    if (isMatchErrorStatus(status, index))
    {
      e_stop = true;
      if (index >= 0)
      {
        detail = status[index].err_detail;
      }
      emg_status = false;
      if (isMatchStatus(status, RobotDriverStatus::RUNNING) ||
          isMatchStatus(status, RobotDriverStatus::POSITIONING) ||
          isMatchStatus(status, RobotDriverStatus::STANDBY))
      {
        // エラー発生のため緊急停止実施
        emergencyStop();
      }
    }
    else
    {
      e_stop = false;
      detail.clear();
      if (isMatchStatus(status, RobotDriverStatus::EMGSTOP) || isMatchStatus(status, RobotDriverStatus::EMGSTOPPING))
      {
        emg_status = true;
      }
      else
      {
        emg_status = false;
      }
    }
  }

  void RobotDriver::emergencyStop()
  {
    mdif_->emgStop();
  }
};  // namespace robot_driver_fw
