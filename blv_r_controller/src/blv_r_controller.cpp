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
 * @file blv_r_controller.cpp
 * @date 2024/02/06
 * @brief   OM BLV_R motor driver 制御クラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include <string>
#include <modbus/modbus.h>
#include "blv_r_controller/blv_r_controller.h"
#include "robot_driver_fw/robot_driver.h"
#include "robot_driver_fw/robot_status.h"

namespace robot_driver_fw
{
BlvrController::BlvrController(ros::NodeHandle &nh) :
nh_(nh),
modbus_(nullptr),
direction_(true),
status_(RobotDriverStatus::NONE),
excitation_status_(false),
manual_excitation_off_(false)
{
  getConfig(nh);
}

BlvrController::~BlvrController()
{
  (void)fini();
}

void BlvrController::checkParam()
{
  if ((left_target_id_ < BLV_R_MODBUS_SLAVE_ADDR_MIN) || (left_target_id_ > BLV_R_MODBUS_SLAVE_ADDR_MAX))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.",
             BLV_R_LEFT_MOTOR_ID_PARAM_STR, left_target_id_, BLV_R_LEFT_MOTOR_DEFAULT_ID);
    left_target_id_ = BLV_R_LEFT_MOTOR_DEFAULT_ID;
  }
  if ((right_target_id_ < BLV_R_MODBUS_SLAVE_ADDR_MIN) || (right_target_id_ > BLV_R_MODBUS_SLAVE_ADDR_MAX))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.",
             BLV_R_RIGHT_MOTOR_ID_PARAM_STR, right_target_id_, BLV_R_RIGHT_MOTOR_DEFAULT_ID);
    right_target_id_ = BLV_R_RIGHT_MOTOR_DEFAULT_ID;
  }
  if (left_target_id_ == right_target_id_)
  {
    ROS_WARN("%s and %s is same id. Correct %s %d to %d, %s %d to %d.",
             BLV_R_LEFT_MOTOR_ID_PARAM_STR, BLV_R_RIGHT_MOTOR_ID_PARAM_STR,
             BLV_R_LEFT_MOTOR_ID_PARAM_STR, left_target_id_, BLV_R_LEFT_MOTOR_DEFAULT_ID,
             BLV_R_RIGHT_MOTOR_ID_PARAM_STR, right_target_id_, BLV_R_RIGHT_MOTOR_DEFAULT_ID);
    left_target_id_ = BLV_R_LEFT_MOTOR_DEFAULT_ID;
    right_target_id_ = BLV_R_RIGHT_MOTOR_DEFAULT_ID;
  }
  if ((global_id_ < BLV_R_MODBUS_GLOBAL_ID_MIN) || (global_id_ > BLV_R_MODBUS_GLOBAL_ID_MAX))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.", BLV_R_GLOBAL_ID_PARAM_STR, global_id_, BLV_R_GLOBAL_DEFAULT_ID);
    global_id_ = BLV_R_GLOBAL_DEFAULT_ID;
  }
  if ((global_id_ == left_target_id_) || (global_id_ == right_target_id_))
  {
    ROS_WARN("%s and %s or %s is same id. Correct %s %d to %d, %s %d to %d, %s %d to %d.",
             BLV_R_GLOBAL_ID_PARAM_STR, BLV_R_LEFT_MOTOR_ID_PARAM_STR, BLV_R_RIGHT_MOTOR_ID_PARAM_STR,
             BLV_R_GLOBAL_ID_PARAM_STR, global_id_, BLV_R_GLOBAL_DEFAULT_ID,
             BLV_R_LEFT_MOTOR_ID_PARAM_STR, left_target_id_, BLV_R_LEFT_MOTOR_DEFAULT_ID,
             BLV_R_RIGHT_MOTOR_ID_PARAM_STR, right_target_id_, BLV_R_RIGHT_MOTOR_DEFAULT_ID);
    global_id_ = BLV_R_GLOBAL_DEFAULT_ID;
    left_target_id_ = BLV_R_LEFT_MOTOR_DEFAULT_ID;
    right_target_id_ = BLV_R_RIGHT_MOTOR_DEFAULT_ID;
  }
  if ((baudrate_ != MODBUS_WRAPPER_BAUDRATE_2400) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_4800) &&
      (baudrate_ != MODBUS_WRAPPER_BAUDRATE_9600) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_19200) &&
      (baudrate_ != MODBUS_WRAPPER_BAUDRATE_38400) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_57600) &&
      (baudrate_ != MODBUS_WRAPPER_BAUDRATE_115200) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_230400))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.", BLV_R_BAUDRATE_STR, baudrate_, BLV_R_MODBUS_BAUDRATE_DEFAULT);
    baudrate_ = BLV_R_MODBUS_BAUDRATE_DEFAULT;
  }
  if ((parity_ != MODBUS_WRAPPER_PARITY_NONE) && (parity_ != MODBUS_WRAPPER_PARITY_EVEN) &&
      (parity_ != MODBUS_WRAPPER_PARITY_ODD))
  {
    ROS_WARN("%s is invalid. Correct %s to %s.", BLV_R_PARITY_STR, parity_.c_str(), BLV_R_MODBUS_PARITY_DEFAULT);
    parity_ = BLV_R_MODBUS_PARITY_DEFAULT;
  }
  if ((stopbit_ != MODBUS_WRAPPER_STOPBIT_DEFAULT) && (stopbit_ != MODBUS_WRAPPER_STOPBIT_EXT))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.", BLV_R_STOP_BIT_STR, stopbit_, MODBUS_WRAPPER_STOPBIT_DEFAULT);
    stopbit_ = MODBUS_WRAPPER_STOPBIT_DEFAULT;
  }
  if ((torque_ < BLV_R_VALUE_TORQUE_MIN) || (torque_ > BLV_R_VALUE_TORQUE_MAX))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.", BLV_R_TORQUE_IN_VEL_STR, torque_, BLV_R_VALUE_TORQUE_DEFAULT);
    torque_ = BLV_R_VALUE_TORQUE_DEFAULT;
  }
  if ((pos_torque_ < BLV_R_VALUE_TORQUE_MIN) || (pos_torque_ > BLV_R_VALUE_TORQUE_MAX))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.", BLV_R_TORQUE_IN_POS_STR, pos_torque_, BLV_R_VALUE_TORQUE_DEFAULT);
    pos_torque_ = BLV_R_VALUE_TORQUE_DEFAULT;
  }
  if (velocity_updating_interval_sec_ > 0)
  {
    if (set_rpm_timeout_sec_ < velocity_updating_interval_sec_)
    {
      ROS_WARN("%s is too short.(< %s) Correct %lf to %lf.",
                BLV_R_SET_RPM_TIMEOUT_STR, ROBOT_DRIVER_KEY_STR_VELOCITY_UPDATING_INTERVAL,
                set_rpm_timeout_sec_, velocity_updating_interval_sec_);
      set_rpm_timeout_sec_ = velocity_updating_interval_sec_;
    }
  }
  else
  {
    if (set_rpm_timeout_sec_ <= 0)
    {
      ROS_WARN("%s is invalid.(<= 0) Correct %lf to %lf.",
               BLV_R_SET_RPM_TIMEOUT_STR, set_rpm_timeout_sec_, BLV_R_SET_RPM_TIMEOUT_SEC_DEFAULT);
      set_rpm_timeout_sec_ = BLV_R_SET_RPM_TIMEOUT_SEC_DEFAULT;
    }
  }
  uint32_t val;
  val = set_rpm_timeout_sec_ * 1000 * BLV_R_DDD_LIFETIME_BY_TIMEOUT_RATE;
  if (val > UINT16_MAX)
  {
    ROS_WARN("%s is too long for setting Direct Data Drive Life Time. Life time is unavailable.",
             BLV_R_SET_RPM_TIMEOUT_STR);
    ddd_lifetime_ms_ = UINT16_MAX;  // Life Time 無効
  }
  else
  {
    ddd_lifetime_ms_ = val;
  }
  if (status_polling_interval_sec_ > 0)
  {
    if (excitation_off_sec_ < status_polling_interval_sec_)
    {
      ROS_WARN("%s is too short.(< %s) Correct %lf to %lf.",
                BLV_R_EXCITATION_OFF_SEC_STR, ROBOT_DRIVER_KEY_STR_STATUS_POLLING_INTERVAL,
                excitation_off_sec_, status_polling_interval_sec_);
      excitation_off_sec_ = status_polling_interval_sec_;
    }
  }
  else
  {
    if (excitation_off_sec_ <= 0)
    {
      ROS_WARN("%s is invalid.(<= 0) Correct %lf to %lf.",
               BLV_R_EXCITATION_OFF_SEC_STR, excitation_off_sec_, BLV_R_EXCITATION_OFF_DEFAULT_SEC);
      excitation_off_sec_ = BLV_R_EXCITATION_OFF_DEFAULT_SEC;
    }
  }
  if (excitation_off_sec_ < set_rpm_timeout_sec_)
  {
    ROS_WARN("%s is too short.(< %s) Correct %lf to %lf.",
             BLV_R_EXCITATION_OFF_SEC_STR, BLV_R_SET_RPM_TIMEOUT_STR, excitation_off_sec_, set_rpm_timeout_sec_);
    excitation_off_sec_ = set_rpm_timeout_sec_;
  }
}

void BlvrController::getConfig(ros::NodeHandle &nh)
{
  std::string default_dev = BLV_R_MODBUS_DEVICE_DEFAULT;
  tty_dev_path_ = nh.param(BLV_R_MODBUS_DEVPATH_PARAM_STR, default_dev);
  left_target_id_ = nh.param(BLV_R_LEFT_MOTOR_ID_PARAM_STR, BLV_R_LEFT_MOTOR_DEFAULT_ID);
  right_target_id_ = nh.param(BLV_R_RIGHT_MOTOR_ID_PARAM_STR, BLV_R_RIGHT_MOTOR_DEFAULT_ID);
  global_id_ = nh.param(BLV_R_GLOBAL_ID_PARAM_STR, BLV_R_GLOBAL_DEFAULT_ID);
  left_local_id_ = BLV_R_LEFT_LOCAL_DEFAULT_ID;
  right_local_id_ = BLV_R_RIGHT_LOCAL_DEFAULT_ID;
  left_forward_rotation_is_positive_ = nh.param(BLV_R_LEFT_MOTOR_FORWARD_POSITIVE_STR, BLV_R_LEFT_DIRECTION_POSITIVE);
  right_forward_rotation_is_positive_ =
                                    nh.param(BLV_R_RIGHT_MOTOR_FORWARD_POSITIVE_STR, BLV_R_RIGHT_DIRECTION_POSITIVE);
  baudrate_ = nh.param(BLV_R_BAUDRATE_STR, BLV_R_MODBUS_BAUDRATE_DEFAULT);
  std::string default_parity = BLV_R_MODBUS_PARITY_DEFAULT;
  parity_ = nh.param(BLV_R_PARITY_STR, default_parity);
  stopbit_ = nh.param(BLV_R_STOP_BIT_STR, MODBUS_WRAPPER_STOPBIT_DEFAULT);
  torque_ = nh.param(BLV_R_TORQUE_IN_VEL_STR, BLV_R_VALUE_TORQUE_DEFAULT);
  pos_torque_ = nh.param(BLV_R_TORQUE_IN_POS_STR, BLV_R_VALUE_TORQUE_DEFAULT);
  set_rpm_timeout_sec_ = nh.param(BLV_R_SET_RPM_TIMEOUT_STR, BLV_R_SET_RPM_TIMEOUT_SEC_DEFAULT);
  auto_excitation_off_ = nh.param(BLV_R_AUTO_EXCITATION_OFF_STR, BLV_R_AUTO_EXCITATION_OFF_DEFAULT);
  excitation_off_sec_ = nh.param(BLV_R_EXCITATION_OFF_SEC_STR, BLV_R_EXCITATION_OFF_DEFAULT_SEC);

  velocity_updating_interval_sec_ = nh.param(ROBOT_DRIVER_KEY_STR_VELOCITY_UPDATING_INTERVAL,
                                             ROBOT_DRIVER_VELOCITY_UPDATING_INTERVAL_SEC);
  status_polling_interval_sec_ = nh.param(ROBOT_DRIVER_KEY_STR_STATUS_POLLING_INTERVAL,
                                          ROBOT_DRIVER_STATUS_POLLING_INTERVAL_SEC);

  checkParam();
}

int32_t BlvrController::modbusInit()
{
  ROS_DEBUG("call BlvrController::modbusInit()");

  int32_t ret;
  modbus_ = new BlvrModbusIdShare();
  if (modbus_)
  {
    ret = modbus_->init(tty_dev_path_, baudrate_, parity_, stopbit_);
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("modbus init failed(%d)", ret);
      return ret;
    }
    ret = modbus_->configIdShareMode(global_id_, left_local_id_, right_local_id_);
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("modbus configIdShareMode failed(%d)", ret);
      return ret;
    }
    ret = initializeIdShareMode();
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("initializeIdShareMode failed(%d)", ret);
      return ret;
    }
  }
  else
  {
    ROS_ERROR("BlvrModbusIdShare class instance allocate failed");
    return RobotDriverErr::INTERNAL_ERR;
  }

  return ret;
}

int32_t BlvrController::init()
{
  int32_t ret;

  if (status_ != RobotDriverStatus::NONE)
  {
    ROS_ERROR("already initialized");
    return RobotDriverErr::BAD_STATE;
  }

  status_ = RobotDriverStatus::INITIALIZING;

  // timer
  if (!motor_stop_timer_.isValid())
  {
    motor_stop_timer_ =
      nh_.createTimer(ros::Duration(set_rpm_timeout_sec_), &BlvrController::motorStopTimerCallback, this, true, false);
  }
  if (!excitation_off_timer_.isValid())
  {
    excitation_off_timer_ =
      nh_.createTimer(ros::Duration(excitation_off_sec_),
                      &BlvrController::excitationOffTimerCallback, this, true, false);
  }
  ret = modbusInit();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("modbusInit failed(%d)", ret);
    fini();
    return ret;
  }

  // alarm reset
  ret = resetAlarm();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("resetAlarm failed(%d)", ret);
    fini();
    return ret;
  }

  // clear
  ret = clearDeviation();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("clearDeviation failed(%d)", ret);
    fini();
    return ret;
  }

  if (!auto_excitation_off_ && !manual_excitation_off_)
  {
    // s-on
    ret = startMotor();
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("startMotor failed(%d)", ret);
      fini();
      return ret;
    }
  }

  status_ = RobotDriverStatus::STANDBY;
  return ret;
}

int32_t BlvrController::fini()
{
  int32_t ret;

  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_ERROR("not initialized or already finalized");
    return RobotDriverErr::BAD_STATE;
  }

  int32_t prev_status = status_;
  status_ = RobotDriverStatus::FINALIZING;

  if (modbus_ != nullptr)
  {
    if (prev_status != RobotDriverStatus::INITIALIZING)
    {
      // stop
      ret = stopMotor();
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_ERROR("stop failed(%d)", ret);
      }

      // s-off
      ret = turnOffMotorExcitation();
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_ERROR("s-off failed(%d)", ret);
      }

      // reset communication
      ret = resetComm();
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_ERROR("reset communication failed(%d)", ret);
      }
    }

    modbus_->fini();
    delete modbus_;
    modbus_ = nullptr;
  }
  status_ = RobotDriverStatus::NONE;

  return RobotDriverErr::NOERR;
}

int32_t BlvrController::reset()
{
  int32_t ret;

  if (status_ != RobotDriverStatus::NONE)
  {
    ret = fini();
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("fini() in reset failed");
      return ret;
    }

    // wait reset communication
    usleep(BLV_R_WAIT_COMM_RESET_US);
  }

  ret = init();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("init() in reset failed");
    return ret;
  }

  return RobotDriverErr::NOERR;
}

int32_t BlvrController::emgStop()
{
  int32_t ret = RobotDriverErr::NOERR;
  if ((status_ == RobotDriverStatus::RUNNING) || (status_ == RobotDriverStatus::POSITIONING) ||
      (status_ == RobotDriverStatus::EMGSTOPPING) || (status_ == RobotDriverStatus::ERRSTOPPING))
  {
    // stop
    ret = stopMotor();
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("stop at EMGSTOP failed(%d)", ret);
      status_ = RobotDriverStatus::EMGSTOPPING;
      return ret;
    }
  }

  if ((status_ == RobotDriverStatus::STANDBY) || (status_ == RobotDriverStatus::RUNNING) ||
      (status_ == RobotDriverStatus::POSITIONING) ||
      (status_ == RobotDriverStatus::EMGSTOPPING) || (status_ == RobotDriverStatus::ERRSTOPPING))
  {
    status_ = RobotDriverStatus::EMGSTOP;
  }

  return ret;
}

int32_t BlvrController::getStatus(std::vector<robot_driver_msgs::MotorStatus> &list)
{
  int32_t ret;
  robot_driver_msgs::MotorStatus status;
  uint16_t l_errcode = 0;
  uint16_t r_errcode = 0;
  bool l_move;
  bool r_move;

  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_WARN_THROTTLE(BLV_R_LOG_THROTTLE_SEC, "Controller is not active.");
    createMotorStatusMsg(status, BLV_R_LEFT_MOTOR_NAME, 0, false);
    list.emplace_back(status);
    createMotorStatusMsg(status, BLV_R_RIGHT_MOTOR_NAME, 0, false);
    list.emplace_back(status);

    return RobotDriverErr::BAD_STATE;
  }

  ret = getMotorStatus();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("getMotorStatus failed (%d)", ret);
    // ここでreturnせずに以下で最近の値をつめる
  }

  (void)checkAlarmStatus(l_errcode, r_errcode);
  (void)checkMoveStatus(l_move, r_move);

  createMotorStatusMsg(status, BLV_R_LEFT_MOTOR_NAME, l_errcode, l_move);
  list.emplace_back(status);

  createMotorStatusMsg(status, BLV_R_RIGHT_MOTOR_NAME, r_errcode, r_move);
  list.emplace_back(status);

  return ret;
}

int32_t BlvrController::controlMotorExcitation(bool state)
{
  int32_t ret = RobotDriverErr::NOERR;

  if ((status_ == RobotDriverStatus::NONE) ||
      (status_ == RobotDriverStatus::INITIALIZING) || (status_ == RobotDriverStatus::FINALIZING))
  {
    ROS_WARN("Controller is not active.");
    ret = RobotDriverErr::BAD_STATE;
  }
  else
  {
    if (auto_excitation_off_)
    {
      ROS_INFO("Manual excitation setting is not supported because configured auto excitation-off.");
      ret = RobotDriverErr::NOT_SUPPORT;
    }
    else
    {
      manual_excitation_off_ = !state;

      if (!manual_excitation_off_ && !excitation_status_)
      {
        // s-on
        ret = startMotor();
        if (ret != RobotDriverErr::NOERR)
        {
          ROS_ERROR("startMotor in %s failed(%d)", __func__, ret);
        }
      }
    }
  }

  return ret;
}

int32_t BlvrController::correctVelocityValue(int32_t value)
{
  if ((value >= BLV_R_VALUE_VELOCITY_MIN) && (value <= BLV_R_VALUE_VELOCITY_MAX))
  {
    return value;
  }
  else
  {
    if (value < BLV_R_VALUE_VELOCITY_MIN)
    {
      ROS_WARN("velocitiy is too small (%d)", value);
      return BLV_R_VALUE_VELOCITY_MIN;
    }
    else
    {
      ROS_WARN("velocitiy is too large (%d)", value);
      return BLV_R_VALUE_VELOCITY_MAX;
    }
  }
}

int32_t BlvrController::correctAccelerationValue(int32_t value)
{
  if ((value >= BLV_R_VALUE_ACCELERATION_MIN) && (value <= BLV_R_VALUE_ACCELERATION_MAX))
  {
    return value;
  }
  else
  {
    if (value < BLV_R_VALUE_ACCELERATION_MIN)
    {
      ROS_WARN("acceleration is too small (%d)", value);
      return BLV_R_VALUE_ACCELERATION_MIN;
    }
    else
    {
      ROS_WARN("acceleration is too large (%d)", value);
      return BLV_R_VALUE_ACCELERATION_MAX;
    }
  }
}

int32_t BlvrController::initializeIdShareMode()
{
  ROS_DEBUG("call BlvrController::initializeIdShareMode()");

  int32_t ret;

  BLV_R_MODBUS_WAIT;

  uint16_t val[] = {0, static_cast<uint16_t>(global_id_), 0, BLV_R_SHARE_CTRL_NUM_2WD, 0, 0};
  // Left
  val[BLV_R_VALUE_IDSHARE_REGI_NUM - 1] = left_local_id_;
  ret = modbus_->setMultiRegisters(left_target_id_, BLV_R_REGI_ADDR_GLOBAL_ID, BLV_R_VALUE_IDSHARE_REGI_NUM, val);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("initialize Left ID share control failed");
    return ret;
  }

  BLV_R_MODBUS_WAIT;

  // Right
  val[BLV_R_VALUE_IDSHARE_REGI_NUM - 1] = right_local_id_;
  ret = modbus_->setMultiRegisters(right_target_id_, BLV_R_REGI_ADDR_GLOBAL_ID, BLV_R_VALUE_IDSHARE_REGI_NUM, val);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("initialize Right ID share control failed");
    return ret;
  }

  return ret;
}

int32_t BlvrController::startMotor()
{
  int32_t ret;

  BLV_R_MODBUS_WAIT;

  std::vector<uint16_t> list {0, BLV_R_MASK_S_ON_BIT};
  ret = modbus_->idShareSetRegistersSameData(global_id_, BLV_R_SHARE_WRITE_DRIVER_IN, BLV_R_VALUE_DREGI_NUM, list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("set S-ON failed");
    return ret;
  }

  // wait s-on
  usleep(BLV_R_WAIT_MOTOR_S_ON_US);

  excitation_status_ = true;

  return ret;
}

int32_t BlvrController::stopMotor()
{
  int32_t ret;

  BLV_R_MODBUS_WAIT;

  // STOP
  std::vector<uint16_t> list {0, BLV_R_VALUE_IMMEDIATE_STOP};
  ret = modbus_->idShareSetRegistersSameData(global_id_, BLV_R_SHARE_WRITE_MOTOR_STOP, BLV_R_VALUE_DREGI_NUM, list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("set STOP failed");
    return ret;
  }

  if ((status_ == RobotDriverStatus::RUNNING) ||
      (status_ == RobotDriverStatus::ERRSTOPPING) || (status_ == RobotDriverStatus::EMGSTOPPING))
  {
    // ライフタイム無効化
    list[0] = 0;
    list[1] = 0;
    ret = modbus_->idShareSetRegistersSameData(global_id_, BLV_R_SHARE_WRITE_DDD_TRIG, BLV_R_VALUE_DREGI_NUM, list);
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("set DDD life time failed");
      return ret;
    }
    // タイマー解除
    motor_stop_timer_.stop();
  }

  // wait motor stopping
  usleep(BLV_R_WAIT_MOTOR_STOP_US);

  return ret;
}

int32_t BlvrController::turnOffMotorExcitation()
{
  int32_t ret;

  BLV_R_MODBUS_WAIT;

  std::vector<uint16_t> list {0, 0};
  ret = modbus_->idShareSetRegistersSameData(global_id_, BLV_R_SHARE_WRITE_DRIVER_IN, BLV_R_VALUE_DREGI_NUM, list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("set S-OFF failed");
    return ret;
  }

  excitation_status_ = false;

  return ret;
}

int32_t BlvrController::resetAlarm()
{
  int32_t ret;

  BLV_R_MODBUS_WAIT;

  std::vector<uint16_t> list {0, BLV_R_VALUE_MENTE_AUTO};
  ret = modbus_->idShareSetRegistersSameData(global_id_, BLV_R_SHARE_WRITE_ALM_RST, BLV_R_VALUE_DREGI_NUM, list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("reset alaram failed");
    return ret;
  }

  return ret;
}

int32_t BlvrController::clearDeviation()
{
  int32_t ret;

  BLV_R_MODBUS_WAIT;

  std::vector<uint16_t> list {0, BLV_R_MASK_CLR_BIT};
  ret = modbus_->idShareSetRegistersSameData(global_id_, BLV_R_SHARE_WRITE_DRIVER_IN, BLV_R_VALUE_DREGI_NUM, list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("set CLR failed");
    return ret;
  }

  return ret;
}

int32_t BlvrController::resetComm()
{
  int32_t ret;

  BLV_R_MODBUS_WAIT;

  std::vector<uint16_t> list {0, BLV_R_VALUE_MENTE_AUTO};
  ret = modbus_->idShareSetRegistersSameData(global_id_, BLV_R_SHARE_WRITE_COMM_RST, BLV_R_VALUE_DREGI_NUM, list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("reset communication failed");
    return ret;
  }

  return ret;
}

int32_t BlvrController::changeVel(int32_t left, int32_t right)
{
  ROS_DEBUG("call changeVel (L:%d[rpm], R:%d[rmp])", left, right);

  int32_t ret;
  std::vector<uint16_t> l_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_VEL,
    0,
    0,
    static_cast<uint16_t>((left & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(left & 0x0000FFFF),
    static_cast<uint16_t>((acceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(acceleration_ & 0x0000FFFF),
    static_cast<uint16_t>((deceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(deceleration_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(torque_),
    static_cast<uint16_t>((left != 0) ? ddd_lifetime_ms_ : 0),
    BLV_R_VALUE_DDD_TRIG
  };

  std::vector<uint16_t> r_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_VEL,
    0,
    0,
    static_cast<uint16_t>((right & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(right & 0x0000FFFF),
    static_cast<uint16_t>((acceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(acceleration_ & 0x0000FFFF),
    static_cast<uint16_t>((deceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(deceleration_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(torque_),
    static_cast<uint16_t>((right != 0) ? ddd_lifetime_ms_ : 0),
    BLV_R_VALUE_DDD_TRIG
  };

  ret = modbus_->idShareSetRegisters(global_id_, BLV_R_SHARE_WRITE_DDD_METHOD, BLV_R_VALUE_DDD_REGI_NUM,
                                      l_list, r_list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareSetRegisters (direct data drive) failed(%d)", ret);
  }
  return ret;
}

int32_t BlvrController::setZeroVel()
{
  ROS_DEBUG("call BlvrController::setZeroVel()");

  return (changeVel(0, 0));
}

int32_t BlvrController::getMotorStatus()
{
  int32_t ret;
  std::vector<uint16_t> l_res;
  std::vector<uint16_t> r_res;
  ret = modbus_->idShareGetRegisters(global_id_, BLV_R_SHARE_READ_ALM, BLV_R_VALUE_STATUS_REGI_NUM, l_res, r_res);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareGetRegisters (get motor status) failed");
    return ret;
  }

  left_alarm_code_ = l_res[BLV_R_SHARE_READ_ALM + 1];
  right_alarm_code_ = r_res[BLV_R_SHARE_READ_ALM + 1];
  left_comm_err_code_ = l_res[BLV_R_SHARE_READ_COMM_ERR + 1];
  right_comm_err_code_ = r_res[BLV_R_SHARE_READ_COMM_ERR + 1];
  left_stop_status_ = ((l_res[BLV_R_SHARE_READ_IO_STATUS_1 + 1] & BLV_R_MASK_EMG_STOP_BITS) != 0);
  right_stop_status_ = ((r_res[BLV_R_SHARE_READ_IO_STATUS_1 + 1] & BLV_R_MASK_EMG_STOP_BITS) != 0);
  left_move_status_ = ((l_res[BLV_R_SHARE_READ_IO_STATUS_5 + 1] & BLV_R_MASK_MOVE_BIT) != 0);
  right_move_status_ = ((r_res[BLV_R_SHARE_READ_IO_STATUS_5 + 1] & BLV_R_MASK_MOVE_BIT) != 0);

  return RobotDriverErr::NOERR;
}

int32_t BlvrController::getEncodeInfo()
{
  int32_t ret;
  std::vector<uint16_t> l_res;
  std::vector<uint16_t> r_res;
  int32_t addr = BLV_R_SHARE_READ_DETECT_POS;
  ret = modbus_->idShareGetRegisters(global_id_, addr, BLV_R_VALUE_ENCODE_REGI_NUM, l_res, r_res);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareGetRegisters (get encode info) failed");
    return ret;
  }

  left_pos_ = l_res[BLV_R_SHARE_READ_DETECT_POS - addr] << UINT16_WIDTH;
  left_pos_ += l_res[BLV_R_SHARE_READ_DETECT_POS - addr + 1];
  right_pos_ = r_res[BLV_R_SHARE_READ_DETECT_POS - addr] << UINT16_WIDTH;
  right_pos_ += r_res[BLV_R_SHARE_READ_DETECT_POS - addr + 1];
  left_vel_ = l_res[BLV_R_SHARE_READ_DETECT_VEL - addr] << UINT16_WIDTH;
  left_vel_ += l_res[BLV_R_SHARE_READ_DETECT_VEL - addr + 1];
  right_vel_ = r_res[BLV_R_SHARE_READ_DETECT_VEL - addr] << UINT16_WIDTH;
  right_vel_ += r_res[BLV_R_SHARE_READ_DETECT_VEL - addr + 1];

  return RobotDriverErr::NOERR;
}

bool BlvrController::checkAlarmStatus(uint16_t &left, uint16_t &right)
{
  left = left_alarm_code_;
  right = right_alarm_code_;
  return ((left_alarm_code_ != 0) || (right_alarm_code_ != 0));
}

bool BlvrController::checkEmgStopStatus()
{
  return (left_stop_status_ || right_stop_status_);
}

bool BlvrController::checkMoveStatus(bool &left, bool &right)
{
  left = left_move_status_;
  right = right_move_status_;
  return (left_move_status_ || right_move_status_);
}

bool BlvrController::checkMoveStatus()
{
  return (left_move_status_ || right_move_status_);
}

void BlvrController::getActualVelocity(int32_t &left, int32_t &right)
{
  left = left_vel_;
  right = right_vel_;

  return;
}

int32_t BlvrController::setPosition(int32_t left, int32_t right)
{
  ROS_DEBUG("call setPosition (L:%d, R:%d)", left, right);

  int32_t ret;
  std::vector<uint16_t> l_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_POS,
    static_cast<uint16_t>((left & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(left & 0x0000FFFF),
    static_cast<uint16_t>((pos_vel_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(pos_vel_ & 0x0000FFFF),
    static_cast<uint16_t>((pos_acc_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(pos_acc_ & 0x0000FFFF),
    static_cast<uint16_t>((pos_dec_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(pos_dec_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(pos_torque_),
    0,
    BLV_R_VALUE_DDD_TRIG
  };

  std::vector<uint16_t> r_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_POS,
    static_cast<uint16_t>((right & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(right & 0x0000FFFF),
    static_cast<uint16_t>((pos_vel_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(pos_vel_ & 0x0000FFFF),
    static_cast<uint16_t>((pos_acc_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(pos_acc_ & 0x0000FFFF),
    static_cast<uint16_t>((pos_dec_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(pos_dec_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(pos_torque_),
    0,
    BLV_R_VALUE_DDD_TRIG
  };

  ret = modbus_->idShareSetRegisters(global_id_, BLV_R_SHARE_WRITE_DDD_METHOD, BLV_R_VALUE_DDD_REGI_NUM,
                                      l_list, r_list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareSetRegisters (direct data drive) failed(%d)", ret);
  }
  return ret;
}

int32_t BlvrController::configSetPosition(float vel, float acc, float dec)
{
  // rpm --> 0.01rpm
  pos_vel_ = correctVelocityValue(static_cast<int>(vel * BLV_R_VALUE_VEL_UNIT_CONVERSION));
  // rpm/s --> 0.01rpm/s
  pos_acc_ = correctAccelerationValue(static_cast<int>(acc * BLV_R_VALUE_VEL_UNIT_CONVERSION));
  pos_dec_ = correctAccelerationValue(static_cast<int>(dec * BLV_R_VALUE_VEL_UNIT_CONVERSION));

  return RobotDriverErr::NOERR;
}

void BlvrController::setAcceleration(float acc, float dec)
{
  // rpm/s --> 0.01rpm/s
  acceleration_ = correctAccelerationValue(static_cast<int>(acc * BLV_R_VALUE_VEL_UNIT_CONVERSION));
  deceleration_ = correctAccelerationValue(static_cast<int>(dec * BLV_R_VALUE_VEL_UNIT_CONVERSION));
}

int32_t BlvrController::setMotorRpm(double left, double right)
{
  int32_t ret;

  if ((status_ != RobotDriverStatus::STANDBY) && (status_ != RobotDriverStatus::RUNNING))
  {
    if (status_ == RobotDriverStatus::POSITIONING)
    {
      ret = getMotorStatus();
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_WARN("call setMotorRpm but motor communication error");
        return ret;
      }

      if (checkMoveStatus())
      {
        ROS_DEBUG("call setMotorRpm but motor is busy");
        return RobotDriverErr::BUSY;
      }
      status_ = RobotDriverStatus::STANDBY;
    }
    else
    {
      ROS_DEBUG("call setMotorRpm but motor is not running");
      return RobotDriverErr::BAD_STATE;
    }
  }
  ROS_DEBUG("call setMotorRpm (L:%f[rpm], R:%f[rmp])", left, right);

  excitation_off_timer_.stop();
  if (!excitation_status_)
  {
    ret = startMotor();
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("startMotor in %s failed(%d)", __func__, ret);
      return ret;
    }
  }

  int32_t l_sign = left_forward_rotation_is_positive_ ? 1 : -1;
  int32_t r_sign = right_forward_rotation_is_positive_ ? 1 : -1;
  ret = changeVel(correctVelocityValue(static_cast<int32_t>(left * BLV_R_VALUE_VEL_UNIT_CONVERSION) * l_sign),
                  correctVelocityValue(static_cast<int32_t>(right * BLV_R_VALUE_VEL_UNIT_CONVERSION) * r_sign));
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("changeVel failed(%d)", ret);
    return ret;
  }

  status_ = RobotDriverStatus::RUNNING;

  motor_stop_timer_.stop();
  motor_stop_timer_.setPeriod(ros::Duration(set_rpm_timeout_sec_));
  motor_stop_timer_.start();

  return RobotDriverErr::NOERR;
}

int32_t BlvrController::setMotorPos(double left, double right)
{
  int32_t ret;

  if (((status_ != RobotDriverStatus::STANDBY) && (status_ != RobotDriverStatus::POSITIONING)) ||
      ((status_ == RobotDriverStatus::POSITIONING) && ((left != 0) || (right != 0))))
  {
    if ((status_ == RobotDriverStatus::RUNNING) || (status_ == RobotDriverStatus::POSITIONING))
    {
      ret = getMotorStatus();
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_WARN("call setMotorPos but motor communication error");
        return ret;
      }

      if (checkMoveStatus())
      {
        ROS_DEBUG("call setMotorPos but motor is busy");
        return RobotDriverErr::BUSY;
      }
      if (status_ == RobotDriverStatus::RUNNING)
      {
        motor_stop_timer_.stop();
      }
      status_ = RobotDriverStatus::STANDBY;
    }
    else
    {
      ROS_DEBUG("call setMotorPos but motor is not running");
      return RobotDriverErr::BAD_STATE;
    }
  }
  ROS_DEBUG("call setMotorPos (L:%f, R:%f)", left, right);

  excitation_off_timer_.stop();
  if (!excitation_status_)
  {
    ret = startMotor();
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("startMotor in %s failed(%d)", __func__, ret);
      return ret;
    }
  }

  int32_t l_sign = left_forward_rotation_is_positive_ ? 1 : -1;
  int32_t r_sign = right_forward_rotation_is_positive_ ? 1 : -1;
  ret = setPosition(static_cast<int32_t>(left * BLV_R_VALUE_PULSE_PER_ROTATION * l_sign),
                    static_cast<int32_t>(right * BLV_R_VALUE_PULSE_PER_ROTATION * r_sign));
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("set moving position failed(%d)", ret);
    return ret;
  }

  status_ = RobotDriverStatus::POSITIONING;

  return RobotDriverErr::NOERR;
}

int32_t BlvrController::encoderPolling(double &left, double &right)
{
  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_WARN_THROTTLE(BLV_R_LOG_THROTTLE_SEC, "Controller is not active.");
    return RobotDriverErr::BAD_STATE;
  }

  int32_t ret;
  int32_t l_vel;
  int32_t r_vel;

  // read encode info
  ret = getEncodeInfo();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_WARN("getEncodeInfo failed(%d)", ret);
    return ret;
  }

  getActualVelocity(l_vel, r_vel);
  ROS_DEBUG("actual velocity (L:%d, R:%d)", l_vel, r_vel);

  // result
  int32_t l_sign = left_forward_rotation_is_positive_ ? 1 : -1;
  int32_t r_sign = right_forward_rotation_is_positive_ ? 1 : -1;
  left = static_cast<double>(l_vel * l_sign) / BLV_R_VALUE_VEL_UNIT_CONVERSION;
  right = static_cast<double>(r_vel * r_sign) / BLV_R_VALUE_VEL_UNIT_CONVERSION;

  return RobotDriverErr::NOERR;
}

int32_t BlvrController::statusPolling(std::vector<robot_driver_msgs::MotorStatus> &list)
{
  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_WARN_THROTTLE(BLV_R_LOG_THROTTLE_SEC, "Controller is not active.");
    return RobotDriverErr::BAD_STATE;
  }

  int32_t ret;
  uint16_t l_errcode = 0;
  uint16_t r_errcode = 0;

  ret = getMotorStatus();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("getMotorStatus falied(%d)", ret);
    return ret;
  }

  if (checkAlarmStatus(l_errcode, r_errcode))
  {
    if ((status_ != RobotDriverStatus::ERRSTOP) && (status_ != RobotDriverStatus::ERRSTOPPING))
    {
      ROS_ERROR("alarm occurred (L:%2xh, R:%2xh)", l_errcode, r_errcode);
    }

    if ((status_ == RobotDriverStatus::RUNNING) || (status_ == RobotDriverStatus::POSITIONING) ||
        (status_ == RobotDriverStatus::ERRSTOPPING) || (status_ == RobotDriverStatus::EMGSTOPPING))
    {
      // stop
      ret = stopMotor();
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_WARN("stopMotor failed(%d)", ret);
        // 停止に失敗した場合はステータスをエラー停止移行中にし、次回処理で再度、停止を試みる
        status_ = RobotDriverStatus::ERRSTOPPING;
      }
      else
      {
        status_ = RobotDriverStatus::ERRSTOP;
      }
    }
    else
    {
      status_ = RobotDriverStatus::ERRSTOP;
    }
  }
  else
  {
    if (checkEmgStopStatus())
    {
      ret = emgStop();
      if (ret != RobotDriverErr::NOERR)
      {
        ROS_WARN("emgStop falied(%d)", ret);
        // 停止に失敗した場合はステータスが非常停止移行中となり、次回処理で再度、停止を試みる
      }
    }
    else
    {
      if ((status_ == RobotDriverStatus::EMGSTOP) || (status_ == RobotDriverStatus::ERRSTOP) ||
          (status_ == RobotDriverStatus::ERRSTOPPING) || (status_ == RobotDriverStatus::EMGSTOPPING))
      {
        status_ = RobotDriverStatus::STANDBY;
      }
    }
  }

  bool l_move;
  bool r_move;
  bool is_moving = checkMoveStatus(l_move, r_move);
  if (auto_excitation_off_ || manual_excitation_off_)
  {
    if (is_moving)
    {
      excitation_off_timer_.stop();
    }
    else
    {
      if (excitation_status_ && !excitation_off_timer_.hasStarted())
      {
        excitation_off_timer_.setPeriod(ros::Duration(excitation_off_sec_));
        excitation_off_timer_.start();
      }
    }
  }
  else
  {
    excitation_off_timer_.stop();
  }

  robot_driver_msgs::MotorStatus status;
  createMotorStatusMsg(status, BLV_R_LEFT_MOTOR_NAME, l_errcode, l_move);
  list.emplace_back(status);

  createMotorStatusMsg(status, BLV_R_RIGHT_MOTOR_NAME, r_errcode, r_move);
  list.emplace_back(status);

  return RobotDriverErr::NOERR;
}

void BlvrController::motorStopTimerCallback(const ros::TimerEvent&)
{
  int32_t ret;
  if ((status_ == RobotDriverStatus::RUNNING) ||
      (status_ == RobotDriverStatus::ERRSTOPPING) || (status_ == RobotDriverStatus::EMGSTOPPING))
  {
    // zero set
    ret = setZeroVel();
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_WARN("setZeroVel failed(%d)", ret);
    }
    status_ = RobotDriverStatus::STANDBY;
  }
}

void BlvrController::excitationOffTimerCallback(const ros::TimerEvent&)
{
  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_DEBUG_THROTTLE(BLV_R_LOG_THROTTLE_SEC, "Controller is not active.");
    return;
  }

  int ret;
  if ((excitation_status_) && (auto_excitation_off_ || manual_excitation_off_))
  {
    ret = stopMotor();
    if (ret == RobotDriverErr::NOERR)
    {
      ret = turnOffMotorExcitation();
      if (ret == RobotDriverErr::NOERR)
      {
        return;
      }
      else
      {
        ROS_ERROR("turnOffMotorExcitation in %s failed(%d)", __func__, ret);
      }
    }
    else
    {
      ROS_ERROR("stopMotor in %s failed(%d)", __func__, ret);
    }
    // リトライ
    excitation_off_timer_.stop();
    excitation_off_timer_.setPeriod(ros::Duration(excitation_off_sec_));
    excitation_off_timer_.start();
  }
}

void BlvrController::createMotorStatusMsg(robot_driver_msgs::MotorStatus &status, const std::string &name, int32_t err_code,
                                          bool move_status)
{
  char buf[BLV_R_DETAIL_ERROR_MESSAGE_LENGTH] = {0};
  status.name = name;
  status.status = status_;
  status.sub_status = (move_status ? RobotDriverErr::BUSY : RobotDriverErr::NOERR);
  status.err_code = err_code;
  if (err_code != 0)
  {
    std::snprintf(buf, sizeof(buf), "(%s)%2xh", name.c_str(), err_code);
    status.err_detail = std::string(buf);
  }
  else
  {
    status.err_detail.clear();
  }
}

};  // namespace robot_driver_fw
