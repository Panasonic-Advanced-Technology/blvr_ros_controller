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
 * @file robot_driver_ros.cpp
 * @date 2023/09/20
 * @brief   robot driver ROS クラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "robot_driver_fw/robot_driver_ros.h"
#include "robot_driver_fw/robot_driver.h"
#include "robot_driver_msgs/ConfigSetPos.h"
#include "robot_driver_msgs/MotorError.h"
#include "robot_driver_msgs/MotorStatusComplex.h"

namespace robot_driver_fw
{
  RobotDriverRos::RobotDriverRos(ros::NodeHandle &nh, MotorDriverIf *mdif) :
  nh_(nh),
  mdif_(mdif),
  initialized_(false),
  velocity_update_(false),
  estop_status_(status::NONE),
  emg_status_(status::NONE)
  {
    getConfig();

    // create publisher
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10, true);
    e_stop_pub_ = nh_.advertise<robot_driver_msgs::MotorError>("estop", 1, true);
    emg_status_pub_ = nh_.advertise<std_msgs::Bool>("emg_status", 1, true);
    status_pub_ = nh_.advertise<robot_driver_msgs::MotorStatusComplex>("status", 1, true);

    // create service
    reset_srv_ = nh_.advertiseService("reset", &RobotDriverRos::resetServiceCallback, this);
    config_set_pos_srv_ = nh_.advertiseService("config_set_pos", &RobotDriverRos::configSetPositionCallback, this);
    get_status_srv_ = nh_.advertiseService("get_status", &RobotDriverRos::getStatusCallback, this);
    control_excitation_srv_ = nh_.advertiseService("control_excitation",
                                                   &RobotDriverRos::controlExcitationCallback, this);

    // set callback
    mdif->setStatusCallback(&RobotDriverRos::statusCallback, this);
  }

  RobotDriverRos::~RobotDriverRos()
  {
    if (rb_driver_ != nullptr)
    {
      delete(rb_driver_);
    }
  }

  void RobotDriverRos::getConfig()
  {
    velocity_updating_interval_sec_ = nh_.param(ROBOT_DRIVER_KEY_STR_VELOCITY_UPDATING_INTERVAL,
                                                ROBOT_DRIVER_VELOCITY_UPDATING_INTERVAL_SEC);
    encoder_polling_interval_sec_ = nh_.param(ROBOT_DRIVER_KEY_STR_ENCODER_POLLING_INTERVAL,
                                              ROBOT_DRIVER_ENCODER_POLLING_INTERVAL_SEC);
    status_polling_interval_sec_ = nh_.param(ROBOT_DRIVER_KEY_STR_STATUS_POLLING_INTERVAL,
                                             ROBOT_DRIVER_STATUS_POLLING_INTERVAL_SEC);
    std::string default_frame_id = ROBOT_DRIVER_ODOM_FRAME_ID_DEFAULT;
    frame_id_ = nh_.param(ROBOT_DRIVER_KEY_STR_FRAME_ID, default_frame_id);
    std::string default_child_frame_id = ROBOT_DRIVER_ODOM_CHILD_FRAME_ID_DEFAULT;
    child_frame_id_ = nh_.param(ROBOT_DRIVER_KEY_STR_CHILD_FRAME_ID, default_child_frame_id);
  }

  int32_t RobotDriverRos::mainLoop()
  {
    if (initialized_ == false)
    {
      ROS_ERROR("not initialized");
      return RobotDriverErr::BAD_STATE;
    }

    int32_t ret = RobotDriverErr::NOERR;

    // create timer
    if (velocity_updating_interval_sec_ > 0)
    {
      velocity_updating_timer_ = nh_.createTimer(ros::Duration(velocity_updating_interval_sec_),
                                                 &RobotDriverRos::velocityUpdatingTimerCallback, this);
    }
    if (encoder_polling_interval_sec_ > 0)
    {
      encoder_polling_timer_ = nh_.createTimer(ros::Duration(encoder_polling_interval_sec_),
                                               &RobotDriver::encoderPollingTimerCallback, rb_driver_);
    }
    if (status_polling_interval_sec_ > 0)
    {
      status_polling_timer_ = nh_.createTimer(ros::Duration(status_polling_interval_sec_),
                                              &RobotDriver::statusPollingTimerCallback, rb_driver_);
    }

    ros::spin();

    return ret;
  }

  int32_t RobotDriverRos::init()
  {
    if (initialized_ == true)
    {
      ROS_ERROR("aleady initialized");
      return RobotDriverErr::BAD_STATE;
    }

    // create subscriber
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &RobotDriverRos::cmdVelCallback, this);
    cmd_pos_sub_ = nh_.subscribe("cmd_pos", 1, &RobotDriverRos::cmdPosCallback, this);

    // set callback
    mdif_->setEncoderCallback(&RobotDriverRos::encoderCallback, this);

    rb_driver_ = new RobotDriver(nh_, mdif_);
    if (rb_driver_ == nullptr)
    {
      ROS_ERROR("create RobotDriver object failed");
      return RobotDriverErr::INTERNAL_ERR;
    }

    int32_t ret;
    ret = rb_driver_->init();
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

  int32_t RobotDriverRos::fini()
  {
    if (initialized_ == false)
    {
      ROS_ERROR("not initialized");
      return RobotDriverErr::BAD_STATE;
    }
    return rb_driver_->fini();
  }

  int32_t RobotDriverRos::reset()
  {
    if (initialized_ == false)
    {
      ROS_ERROR("not initialized");
      return RobotDriverErr::BAD_STATE;
    }
    return rb_driver_->reset();
  }

  bool RobotDriverRos::resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    int32_t ret;
    ret = reset();
    if (ret != RobotDriverErr::NOERR)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  bool RobotDriverRos::configSetPositionCallback(robot_driver_msgs::ConfigSetPos::Request &req,
                                                 robot_driver_msgs::ConfigSetPos::Response &res)
  {
    rb_driver_->configSetPos(req.linear.velocity, req.linear.acceleration, req.linear.deceleration,
                            req.angular.velocity, req.angular.acceleration, req.angular.deceleration);
    res.result = RobotDriverErr::NOERR;
    return true;
  }

  bool RobotDriverRos::getStatusCallback(robot_driver_msgs::GetMotorStatus::Request &,
                                         robot_driver_msgs::GetMotorStatus::Response &res)
  {
    int32_t ret;
    ret = rb_driver_->getMotorStatus(res.status);
    res.result = ret;
    return true;
  }

  bool RobotDriverRos::controlExcitationCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    int32_t ret;
    ret = rb_driver_->controlMotorExcitation(req.data);
    if (ret != RobotDriverErr::NOERR)
    {
      res.success = false;
      res.message = convRobotDriverErrToStr(static_cast<RobotDriverErr>(ret));
    }
    else
    {
      res.success = true;
      res.message.clear();
    }

    return true;
  }

  void RobotDriverRos::velocityUpdatingTimerCallback(const ros::TimerEvent&)
  {
    velocity_update_ = true;
  }

  void RobotDriverRos::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (velocity_update_)
    {
      rb_driver_->setTwist(msg->linear.x, msg->angular.z);
      velocity_update_ = false;
    }
  }

  void RobotDriverRos::cmdPosCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (velocity_update_)
    {
      rb_driver_->setPos(msg->linear.x, msg->angular.z);
      velocity_update_ = false;
    }
  }

  void RobotDriverRos::encoderCallback(double left, double right)
  {
    ROS_DEBUG("encoderCallback(left=%f, right=%f)", left, right);

    // odometry
    nav_msgs::Odometry odom;
    odom.header.frame_id = frame_id_.c_str();
    odom.child_frame_id = child_frame_id_.c_str();
    if (rb_driver_->updateOdometry(left, right, odom))
    {
      // publish
      odometry_pub_.publish(odom);
    }
  }

  void RobotDriverRos::statusCallback(std::vector<robot_driver_msgs::MotorStatus> status)
  {
    ROS_DEBUG("statusCallback(status[%lu])", status.size());
    for (int i = 0; i < static_cast<int>(status.size()); i++)
    {
      ROS_DEBUG("[%d]status=%d, sub_status=%d, err_code=%d, err_detail=%s",
                i, status[i].status, status[i].sub_status, status[i].err_code, status[i].err_detail.c_str());
    }

    std_msgs::Bool emg_status;
    robot_driver_msgs::MotorError e_stop;
    bool emg;
    bool err;
    std::string detail;
    rb_driver_->confirmStatus(status, emg, err, detail);

    // publish
    if (estop_status_ != CONV2STATUS(err))
    {
      e_stop.status = err;
      e_stop.detail = detail;
      e_stop_pub_.publish(e_stop);
      estop_status_ = CONV2STATUS(err);
    }
    if (emg_status_ != CONV2STATUS(emg))
    {
      emg_status.data = emg;
      emg_status_pub_.publish(emg_status);
      emg_status_ = CONV2STATUS(emg);
    }

    // publish status
    robot_driver_msgs::MotorStatusComplex msg;
    msg.status = status;
    msg.estop = err;
    msg.emg_status = emg;
    status_pub_.publish(msg);
  }

};  // namespace robot_driver_fw
