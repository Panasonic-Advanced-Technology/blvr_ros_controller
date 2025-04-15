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
********************/
/*!
 * @file blv_r_4wd_controller.cpp
 * @date 2024/10/02
 * @brief   OM BLV_R motor driver 制御クラス for 4 wheel drive
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include <string>
#include <modbus/modbus.h>
#include "blv_r_controller/blv_r_4wd_controller.h"
#include "robot_driver_fw/mecanum_robot_driver.h"
#include "robot_driver_fw/robot_status.h"

namespace robot_driver_fw
{
Blvr4wdController::Blvr4wdController(ros::NodeHandle &nh) :
BlvrController(nh)
{
  getConfig(nh);
}

Blvr4wdController::~Blvr4wdController()
{
  (void)fini();
}

void Blvr4wdController::checkParam()
{
  if ((rear_left_target_id_ < BLV_R_MODBUS_SLAVE_ADDR_MIN) || (rear_left_target_id_ > BLV_R_MODBUS_SLAVE_ADDR_MAX))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.",
             BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR, rear_left_target_id_, BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID);
    rear_left_target_id_ = BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID;
  }
  if ((rear_right_target_id_ < BLV_R_MODBUS_SLAVE_ADDR_MIN) || (rear_right_target_id_ > BLV_R_MODBUS_SLAVE_ADDR_MAX))
  {
    ROS_WARN("%s is invalid. Correct %d to %d.",
             BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR, rear_right_target_id_, BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID);
    rear_right_target_id_ = BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID;
  }
  if ((global_id_ == rear_left_target_id_) || (global_id_ == rear_right_target_id_))
  {
    ROS_WARN("%s and %s or %s is same id. Correct %s %d to %d, %s %d to %d, %s %d to %d.",
             BLV_R_GLOBAL_ID_PARAM_STR, BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR, BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR,
             BLV_R_GLOBAL_ID_PARAM_STR, global_id_, BLV_R_GLOBAL_DEFAULT_ID,
             BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR, rear_left_target_id_, BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID,
             BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR, rear_right_target_id_, BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID);
    global_id_ = BLV_R_GLOBAL_DEFAULT_ID;
    rear_left_target_id_ = BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID;
    rear_right_target_id_ = BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID;
  }
  if (rear_left_target_id_ == rear_right_target_id_)
  {
    ROS_WARN("%s and %s is same id. Correct %s %d to %d, %s %d to %d.",
             BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR, BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR,
             BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR, rear_left_target_id_, BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID,
             BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR, rear_right_target_id_, BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID);
    rear_left_target_id_ = BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID;
    rear_right_target_id_ = BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID;
  }
  if ((rear_left_target_id_ == left_target_id_) || (rear_left_target_id_ == right_target_id_) ||
      (rear_right_target_id_ == left_target_id_) || (rear_right_target_id_ == right_target_id_))
  {
    ROS_WARN("Modbus id is duplicated. Correct %s %d to %d, %s %d to %d, %s %d to %d, %s %d to %d.",
             BLV_R_LEFT_MOTOR_ID_PARAM_STR, left_target_id_, BLV_R_LEFT_MOTOR_DEFAULT_ID,
             BLV_R_RIGHT_MOTOR_ID_PARAM_STR, right_target_id_, BLV_R_RIGHT_MOTOR_DEFAULT_ID,
             BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR, rear_left_target_id_, BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID,
             BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR, rear_right_target_id_, BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID);
    left_target_id_ = BLV_R_LEFT_MOTOR_DEFAULT_ID;
    right_target_id_ = BLV_R_RIGHT_MOTOR_DEFAULT_ID;
    rear_left_target_id_ = BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID;
    rear_right_target_id_ = BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID;
  }
}

void Blvr4wdController::getConfig(ros::NodeHandle &nh)
{
  rear_left_target_id_ = nh.param(BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR, BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID);
  rear_right_target_id_ = nh.param(BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR, BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID);
  rear_left_local_id_ = BLV_R_REAR_LEFT_LOCAL_DEFAULT_ID;
  rear_right_local_id_ = BLV_R_REAR_RIGHT_LOCAL_DEFAULT_ID;

  checkParam();
}

int32_t Blvr4wdController::modbusInit()
{
  ROS_DEBUG("call Blvr4wdController::modbusInit()");

  int32_t ret;
  modbus_ = new Blvr4wdModbusIdShare();
  if (modbus_)
  {
    ret = dynamic_cast<Blvr4wdModbusIdShare*>(modbus_)->init(tty_dev_path_, baudrate_, parity_, stopbit_);
    if (ret != RobotDriverErr::NOERR)
    {
      ROS_ERROR("modbus init failed(%d)", ret);
      return ret;
    }
    ret = dynamic_cast<Blvr4wdModbusIdShare*>(modbus_)->configIdShareMode(global_id_,
                                     left_local_id_, right_local_id_, rear_left_local_id_, rear_right_local_id_);
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

int32_t Blvr4wdController::getStatus(std::vector<robot_driver_msgs::MotorStatus> &list)
{
  int32_t ret;
  robot_driver_msgs::MotorStatus status;
  uint16_t fl_errcode = 0;
  uint16_t fr_errcode = 0;
  uint16_t rl_errcode = 0;
  uint16_t rr_errcode = 0;
  bool fl_move;
  bool fr_move;
  bool rl_move;
  bool rr_move;

  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_WARN_THROTTLE(BLV_R_LOG_THROTTLE_SEC, "Controller is not active.");
    createMotorStatusMsg(status, BLV_R_LEFT_MOTOR_NAME, 0, false);
    list.emplace_back(status);
    createMotorStatusMsg(status, BLV_R_RIGHT_MOTOR_NAME, 0, false);
    list.emplace_back(status);
    createMotorStatusMsg(status, BLV_R_REAR_LEFT_MOTOR_NAME, 0, false);
    list.emplace_back(status);
    createMotorStatusMsg(status, BLV_R_REAR_RIGHT_MOTOR_NAME, 0, false);
    list.emplace_back(status);

    return RobotDriverErr::BAD_STATE;
  }

  ret = getMotorStatus();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("getMotorStatus failed (%d)", ret);
    // ここでreturnせずに以下で最近の値をつめる
  }

  (void)checkAlarmStatus(fl_errcode, fr_errcode, rl_errcode, rr_errcode);
  (void)checkMoveStatus(fl_move, fr_move, rl_move, rr_move);

  createMotorStatusMsg(status, BLV_R_LEFT_MOTOR_NAME, fl_errcode, fl_move);
  list.emplace_back(status);
  createMotorStatusMsg(status, BLV_R_RIGHT_MOTOR_NAME, fr_errcode, fr_move);
  list.emplace_back(status);
  createMotorStatusMsg(status, BLV_R_REAR_LEFT_MOTOR_NAME, rl_errcode, rl_move);
  list.emplace_back(status);
  createMotorStatusMsg(status, BLV_R_REAR_RIGHT_MOTOR_NAME, rr_errcode, rr_move);
  list.emplace_back(status);

  return ret;
}

int32_t Blvr4wdController::initializeIdShareMode()
{
  ROS_DEBUG("call Blvr4wdController::initializeIdShareMode()");

  int32_t ret;

  BLV_R_MODBUS_WAIT;

  uint16_t val[] = {0, static_cast<uint16_t>(global_id_), 0, BLV_R_SHARE_CTRL_NUM_4WD, 0, 0};
  // FrontLeft
  val[BLV_R_VALUE_IDSHARE_REGI_NUM - 1] = left_local_id_;
  ret = modbus_->setMultiRegisters(left_target_id_, BLV_R_REGI_ADDR_GLOBAL_ID, BLV_R_VALUE_IDSHARE_REGI_NUM, val);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("initialize FrontLeft ID share control failed");
    return ret;
  }

  BLV_R_MODBUS_WAIT;

  // FrontRight
  val[BLV_R_VALUE_IDSHARE_REGI_NUM - 1] = right_local_id_;
  ret = modbus_->setMultiRegisters(right_target_id_, BLV_R_REGI_ADDR_GLOBAL_ID, BLV_R_VALUE_IDSHARE_REGI_NUM, val);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("initialize FrontRight ID share control failed");
    return ret;
  }

  BLV_R_MODBUS_WAIT;

  // RearLeft
  val[BLV_R_VALUE_IDSHARE_REGI_NUM - 1] = rear_left_local_id_;
  ret = modbus_->setMultiRegisters(rear_left_target_id_, BLV_R_REGI_ADDR_GLOBAL_ID, BLV_R_VALUE_IDSHARE_REGI_NUM, val);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("initialize RearLeft ID share control failed");
    return ret;
  }

  BLV_R_MODBUS_WAIT;

  // FrontRight
  val[BLV_R_VALUE_IDSHARE_REGI_NUM - 1] = rear_right_local_id_;
  ret = modbus_->setMultiRegisters(rear_right_target_id_, BLV_R_REGI_ADDR_GLOBAL_ID, BLV_R_VALUE_IDSHARE_REGI_NUM, val);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("initialize RearRight ID share control failed");
    return ret;
  }

  return ret;
}

int32_t Blvr4wdController::changeVel(int32_t fl, int32_t fr, int32_t rl, int32_t rr)
{
  ROS_DEBUG("call changeVel (FL:%d[rpm], FR:%d[rmp], RL:%d[rpm], RR:%d[rpm])", fl, fr, rl, rr);

  int32_t ret;
  std::vector<uint16_t> fl_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_VEL,
    0,
    0,
    static_cast<uint16_t>((fl & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(fl & 0x0000FFFF),
    static_cast<uint16_t>((acceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(acceleration_ & 0x0000FFFF),
    static_cast<uint16_t>((deceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(deceleration_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(torque_),
    static_cast<uint16_t>((fl != 0) ? ddd_lifetime_ms_ : 0),
    BLV_R_VALUE_DDD_TRIG
  };
  std::vector<uint16_t> fr_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_VEL,
    0,
    0,
    static_cast<uint16_t>((fr & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(fr & 0x0000FFFF),
    static_cast<uint16_t>((acceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(acceleration_ & 0x0000FFFF),
    static_cast<uint16_t>((deceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(deceleration_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(torque_),
    static_cast<uint16_t>((fr != 0) ? ddd_lifetime_ms_ : 0),
    BLV_R_VALUE_DDD_TRIG
  };
  std::vector<uint16_t> rl_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_VEL,
    0,
    0,
    static_cast<uint16_t>((rl & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(rl & 0x0000FFFF),
    static_cast<uint16_t>((acceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(acceleration_ & 0x0000FFFF),
    static_cast<uint16_t>((deceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(deceleration_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(torque_),
    static_cast<uint16_t>((rl != 0) ? ddd_lifetime_ms_ : 0),
    BLV_R_VALUE_DDD_TRIG
  };
  std::vector<uint16_t> rr_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_VEL,
    0,
    0,
    static_cast<uint16_t>((rr & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(rr & 0x0000FFFF),
    static_cast<uint16_t>((acceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(acceleration_ & 0x0000FFFF),
    static_cast<uint16_t>((deceleration_ & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(deceleration_ & 0x0000FFFF),
    0,
    static_cast<uint16_t>(torque_),
    static_cast<uint16_t>((rr != 0) ? ddd_lifetime_ms_ : 0),
    BLV_R_VALUE_DDD_TRIG
  };

  ret = dynamic_cast<Blvr4wdModbusIdShare*>(modbus_)->idShareSetRegisters(global_id_,
                           BLV_R_SHARE_WRITE_DDD_METHOD, BLV_R_VALUE_DDD_REGI_NUM, fl_list, fr_list, rl_list, rr_list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareSetRegisters (direct data drive) failed(%d)", ret);
  }
  return ret;
}

int32_t Blvr4wdController::setZeroVel()
{
  ROS_DEBUG("call Blvr4wdController::setZeroVel()");

  return (changeVel(0, 0, 0, 0));
}

int32_t Blvr4wdController::getMotorStatus()
{
  int32_t ret;
  std::vector<uint16_t> fl_res;
  std::vector<uint16_t> fr_res;
  std::vector<uint16_t> rl_res;
  std::vector<uint16_t> rr_res;
  ret = dynamic_cast<Blvr4wdModbusIdShare*>(modbus_)->idShareGetRegisters(global_id_,
                                   BLV_R_SHARE_READ_ALM, BLV_R_VALUE_STATUS_REGI_NUM, fl_res, fr_res, rl_res, rr_res);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareGetRegisters (get motor status) failed");
    return ret;
  }

  left_alarm_code_ = fl_res[BLV_R_SHARE_READ_ALM + 1];
  right_alarm_code_ = fr_res[BLV_R_SHARE_READ_ALM + 1];
  rear_left_alarm_code_ = rl_res[BLV_R_SHARE_READ_ALM + 1];
  rear_right_alarm_code_ = rr_res[BLV_R_SHARE_READ_ALM + 1];

  left_comm_err_code_ = fl_res[BLV_R_SHARE_READ_COMM_ERR + 1];
  right_comm_err_code_ = fr_res[BLV_R_SHARE_READ_COMM_ERR + 1];
  rear_left_comm_err_code_ = rl_res[BLV_R_SHARE_READ_COMM_ERR + 1];
  rear_right_comm_err_code_ = rr_res[BLV_R_SHARE_READ_COMM_ERR + 1];

  left_stop_status_ = ((fl_res[BLV_R_SHARE_READ_IO_STATUS_1 + 1] & BLV_R_MASK_EMG_STOP_BITS) != 0);
  right_stop_status_ = ((fr_res[BLV_R_SHARE_READ_IO_STATUS_1 + 1] & BLV_R_MASK_EMG_STOP_BITS) != 0);
  rear_left_stop_status_ = ((rl_res[BLV_R_SHARE_READ_IO_STATUS_1 + 1] & BLV_R_MASK_EMG_STOP_BITS) != 0);
  rear_right_stop_status_ = ((rr_res[BLV_R_SHARE_READ_IO_STATUS_1 + 1] & BLV_R_MASK_EMG_STOP_BITS) != 0);

  left_move_status_ = ((fl_res[BLV_R_SHARE_READ_IO_STATUS_5 + 1] & BLV_R_MASK_MOVE_BIT) != 0);
  right_move_status_ = ((fr_res[BLV_R_SHARE_READ_IO_STATUS_5 + 1] & BLV_R_MASK_MOVE_BIT) != 0);
  rear_left_move_status_ = ((rl_res[BLV_R_SHARE_READ_IO_STATUS_5 + 1] & BLV_R_MASK_MOVE_BIT) != 0);
  rear_right_move_status_ = ((rr_res[BLV_R_SHARE_READ_IO_STATUS_5 + 1] & BLV_R_MASK_MOVE_BIT) != 0);

  return RobotDriverErr::NOERR;
}

int32_t Blvr4wdController::getEncodeInfo()
{
  int32_t ret;
  std::vector<uint16_t> fl_res;
  std::vector<uint16_t> fr_res;
  std::vector<uint16_t> rl_res;
  std::vector<uint16_t> rr_res;
  int32_t addr = BLV_R_SHARE_READ_DETECT_POS;
  ret = dynamic_cast<Blvr4wdModbusIdShare*>(modbus_)->idShareGetRegisters(global_id_, addr,
                                                         BLV_R_VALUE_ENCODE_REGI_NUM, fl_res, fr_res, rl_res, rr_res);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareGetRegisters (get encode info) failed");
    return ret;
  }

  left_pos_ = fl_res[BLV_R_SHARE_READ_DETECT_POS - addr] << UINT16_WIDTH;
  left_pos_ += fl_res[BLV_R_SHARE_READ_DETECT_POS - addr + 1];
  right_pos_ = fr_res[BLV_R_SHARE_READ_DETECT_POS - addr] << UINT16_WIDTH;
  right_pos_ += fr_res[BLV_R_SHARE_READ_DETECT_POS - addr + 1];
  rear_left_pos_ = rl_res[BLV_R_SHARE_READ_DETECT_POS - addr] << UINT16_WIDTH;
  rear_left_pos_ += rl_res[BLV_R_SHARE_READ_DETECT_POS - addr + 1];
  rear_right_pos_ = rr_res[BLV_R_SHARE_READ_DETECT_POS - addr] << UINT16_WIDTH;
  rear_right_pos_ += rr_res[BLV_R_SHARE_READ_DETECT_POS - addr + 1];

  left_vel_ = fl_res[BLV_R_SHARE_READ_DETECT_VEL - addr] << UINT16_WIDTH;
  left_vel_ += fl_res[BLV_R_SHARE_READ_DETECT_VEL - addr + 1];
  right_vel_ = fr_res[BLV_R_SHARE_READ_DETECT_VEL - addr] << UINT16_WIDTH;
  right_vel_ += fr_res[BLV_R_SHARE_READ_DETECT_VEL - addr + 1];
  rear_left_vel_ = rl_res[BLV_R_SHARE_READ_DETECT_VEL - addr] << UINT16_WIDTH;
  rear_left_vel_ += rl_res[BLV_R_SHARE_READ_DETECT_VEL - addr + 1];
  rear_right_vel_ = rr_res[BLV_R_SHARE_READ_DETECT_VEL - addr] << UINT16_WIDTH;
  rear_right_vel_ += rr_res[BLV_R_SHARE_READ_DETECT_VEL - addr + 1];

  return RobotDriverErr::NOERR;
}

bool Blvr4wdController::checkAlarmStatus(uint16_t &fl, uint16_t &fr, uint16_t &rl, uint16_t &rr)
{
  fl = left_alarm_code_;
  fr = right_alarm_code_;
  rl = rear_left_alarm_code_;
  rr = rear_right_alarm_code_;
  return ((left_alarm_code_ != 0) || (right_alarm_code_ != 0) ||
          (rear_left_alarm_code_ != 0) || (rear_right_alarm_code_ != 0));
}

bool Blvr4wdController::checkEmgStopStatus()
{
  return (left_stop_status_ || right_stop_status_ || rear_left_stop_status_ || rear_right_stop_status_);
}

bool Blvr4wdController::checkMoveStatus(bool &fl, bool &fr, bool &rl, bool &rr)
{
  fl = left_move_status_;
  fr = right_move_status_;
  rl = rear_left_move_status_;
  rr = rear_right_move_status_;
  return (left_move_status_ || right_move_status_ || rear_left_move_status_ || rear_right_move_status_);
}

bool Blvr4wdController::checkMoveStatus()
{
  return (left_move_status_ || right_move_status_ || rear_left_move_status_ || rear_right_move_status_);
}

void Blvr4wdController::getActualVelocity(int32_t &fl, int32_t &fr, int32_t &rl, int32_t &rr)
{
  fl = left_vel_;
  fr = right_vel_;
  rl = rear_left_vel_;
  rr = rear_right_vel_;

  return;
}

int32_t Blvr4wdController::setPosition(int32_t fl, int32_t fr, int32_t rl, int32_t rr)
{
  ROS_DEBUG("call setPosition (FL:%d, FR:%d, RL:%d, RR:%d)", fl, fr, rl, rr);

  int32_t ret;
  std::vector<uint16_t> fl_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_POS,
    static_cast<uint16_t>((fl & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(fl & 0x0000FFFF),
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
  std::vector<uint16_t> fr_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_POS,
    static_cast<uint16_t>((fr & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(fr & 0x0000FFFF),
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
  std::vector<uint16_t> rl_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_POS,
    static_cast<uint16_t>((rl & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(rl & 0x0000FFFF),
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
  std::vector<uint16_t> rr_list
  {
    0,
    BLV_R_VALUE_DDD_METHOD_POS,
    static_cast<uint16_t>((rr & 0xFFFF0000) >> UINT16_WIDTH),
    static_cast<uint16_t>(rr & 0x0000FFFF),
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

  ret = dynamic_cast<Blvr4wdModbusIdShare*>(modbus_)->idShareSetRegisters(global_id_,
                          BLV_R_SHARE_WRITE_DDD_METHOD, BLV_R_VALUE_DDD_REGI_NUM, fl_list, fr_list, rl_list, rr_list);
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareSetRegisters (direct data drive) failed(%d)", ret);
  }
  return ret;
}

int32_t Blvr4wdController::setMotorRpm(double fl, double fr, double rl, double rr)
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
  ROS_DEBUG("call setMotorRpm (FL:%f[rpm], FR:%f[rmp], RL:%f[rmp], RR:%f[rmp])", fl, fr, rl, rr);

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
  ret = changeVel(correctVelocityValue(static_cast<int32_t>(fl * BLV_R_VALUE_VEL_UNIT_CONVERSION) * l_sign),
                  correctVelocityValue(static_cast<int32_t>(fr * BLV_R_VALUE_VEL_UNIT_CONVERSION) * r_sign),
                  correctVelocityValue(static_cast<int32_t>(rl * BLV_R_VALUE_VEL_UNIT_CONVERSION) * l_sign),
                  correctVelocityValue(static_cast<int32_t>(rr * BLV_R_VALUE_VEL_UNIT_CONVERSION) * r_sign));
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

int32_t Blvr4wdController::setMotorPos(double fl, double fr, double rl, double rr)
{
  int32_t ret;

  if (((status_ != RobotDriverStatus::STANDBY) && (status_ != RobotDriverStatus::POSITIONING)) ||
      ((status_ == RobotDriverStatus::POSITIONING) && ((fl != 0) || (fr != 0) || (rl != 0) || (rr != 0))))
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
  ROS_DEBUG("call setMotorPos (FL:%f, FR:%f, RL:%f, RR:%f)", fl, fr, rl, rr);

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
  ret = setPosition(static_cast<int32_t>(fl * BLV_R_VALUE_PULSE_PER_ROTATION * l_sign),
                    static_cast<int32_t>(fr * BLV_R_VALUE_PULSE_PER_ROTATION * r_sign),
                    static_cast<int32_t>(rl * BLV_R_VALUE_PULSE_PER_ROTATION * l_sign),
                    static_cast<int32_t>(rr * BLV_R_VALUE_PULSE_PER_ROTATION * r_sign));
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("set moving position failed(%d)", ret);
    return ret;
  }

  status_ = RobotDriverStatus::POSITIONING;

  return RobotDriverErr::NOERR;
}

int32_t Blvr4wdController::encoderPolling(double &fl, double &fr, double &rl, double &rr)
{
  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_WARN_THROTTLE(BLV_R_LOG_THROTTLE_SEC, "Controller is not active.");
    return RobotDriverErr::BAD_STATE;
  }

  int32_t ret;
  int32_t fl_vel;
  int32_t fr_vel;
  int32_t rl_vel;
  int32_t rr_vel;

  // read encode info
  ret = getEncodeInfo();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_WARN("getEncodeInfo failed(%d)", ret);
    return ret;
  }

  getActualVelocity(fl_vel, fr_vel, rl_vel, rr_vel);
  ROS_DEBUG("actual velocity (FL:%d, FR:%d, RL:%d, RR:%d)", fl_vel, fr_vel, rl_vel, rr_vel);

  // result
  int32_t l_sign = left_forward_rotation_is_positive_ ? 1 : -1;
  int32_t r_sign = right_forward_rotation_is_positive_ ? 1 : -1;
  fl = static_cast<double>(fl_vel * l_sign) / BLV_R_VALUE_VEL_UNIT_CONVERSION;
  fr = static_cast<double>(fr_vel * r_sign) / BLV_R_VALUE_VEL_UNIT_CONVERSION;
  rl = static_cast<double>(rl_vel * l_sign) / BLV_R_VALUE_VEL_UNIT_CONVERSION;
  rr = static_cast<double>(rr_vel * r_sign) / BLV_R_VALUE_VEL_UNIT_CONVERSION;

  return RobotDriverErr::NOERR;
}

int32_t Blvr4wdController::statusPolling(std::vector<robot_driver_msgs::MotorStatus> &list)
{
  if (status_ == RobotDriverStatus::NONE)
  {
    ROS_WARN_THROTTLE(BLV_R_LOG_THROTTLE_SEC, "Controller is not active.");
    return RobotDriverErr::BAD_STATE;
  }

  int32_t ret;
  uint16_t fl_errcode = 0;
  uint16_t fr_errcode = 0;
  uint16_t rl_errcode = 0;
  uint16_t rr_errcode = 0;

  ret = getMotorStatus();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("getMotorStatus falied(%d)", ret);
    return ret;
  }

  if (checkAlarmStatus(fl_errcode, fr_errcode, rl_errcode, rr_errcode))
  {
    if ((status_ != RobotDriverStatus::ERRSTOP) && (status_ != RobotDriverStatus::ERRSTOPPING))
    {
      ROS_ERROR("alarm occurred (FL:%2xh, FR:%2xh, RL:%2xh, RR:%2xh)", fl_errcode, fr_errcode, rl_errcode, rr_errcode);
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

  bool fl_move;
  bool fr_move;
  bool rl_move;
  bool rr_move;
  bool is_moving = checkMoveStatus(fl_move, fr_move, rl_move, rr_move);
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
  createMotorStatusMsg(status, BLV_R_LEFT_MOTOR_NAME, fl_errcode, fl_move);
  list.emplace_back(status);
  createMotorStatusMsg(status, BLV_R_RIGHT_MOTOR_NAME, fr_errcode, fr_move);
  list.emplace_back(status);
  createMotorStatusMsg(status, BLV_R_REAR_LEFT_MOTOR_NAME, rl_errcode, rl_move);
  list.emplace_back(status);
  createMotorStatusMsg(status, BLV_R_REAR_RIGHT_MOTOR_NAME, rr_errcode, rr_move);
  list.emplace_back(status);

  return RobotDriverErr::NOERR;
}

};  // namespace robot_driver_fw
