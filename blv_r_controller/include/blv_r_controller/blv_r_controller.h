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
 * @file blv_r_controller.h
 * @date 2024/02/06
 * @brief   OM BLV_R motor driver 制御クラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef BLV_R_CONTROLLER_BLV_R_CONTROLLER_H
#define BLV_R_CONTROLLER_BLV_R_CONTROLLER_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include "blv_r_controller/blv_r_modbus_id_share.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include "robot_driver_msgs/MotorStatus.h"

#define BLV_R_VALUE_VELOCITY_MIN     (-4000000)
#define BLV_R_VALUE_VELOCITY_MAX     (4000000)
#define BLV_R_VALUE_ACCELERATION_MIN (1)
#define BLV_R_VALUE_ACCELERATION_MAX (1000000000)
#define BLV_R_VALUE_TORQUE_MIN       (0)
#define BLV_R_VALUE_TORQUE_MAX       (10000)
#define BLV_R_VALUE_TORQUE_DEFAULT   (1000)

#define BLV_R_VALUE_PULSE_PER_ROTATION (36000.0)
#define BLV_R_VALUE_VEL_UNIT_CONVERSION (100.0)

#define BLV_R_VALUE_DREGI_NUM        (2)
#define BLV_R_VALUE_DDD_REGI_NUM     (14)
#define BLV_R_VALUE_STATUS_REGI_NUM  (8)
#define BLV_R_VALUE_ENCODE_REGI_NUM  (4)
#define BLV_R_VALUE_IDSHARE_REGI_NUM (6)

#define BLV_R_VALUE_DDD_METHOD_VEL   (51)
#define BLV_R_VALUE_DDD_METHOD_POS   (3)
#define BLV_R_VALUE_DDD_TRIG         (1)

#define BLV_R_VALUE_MENTE_OFF        (0)
#define BLV_R_VALUE_MENTE_ON         (1)
#define BLV_R_VALUE_MENTE_AUTO       (2)
#define BLV_R_VALUE_IMMEDIATE_STOP   (1)
#define BLV_R_VALUE_DEC_STOP         (2)
#define BLV_R_VALUE_FOLLOW_QSTOP     (3)

#define BLV_R_MASK_S_ON_BIT          (0x0001)
#define BLV_R_MASK_CLR_BIT           (0x0008)
#define BLV_R_MASK_STOP_BIT          (0x0020)
#define BLV_R_MASK_QSTOP_BIT         (0x0010)
#define BLV_R_MASK_MOVE_BIT          (0x0040)

#define BLV_R_MASK_EMG_STOP_BITS     (BLV_R_MASK_STOP_BIT | BLV_R_MASK_QSTOP_BIT)

#define BLV_R_SHARE_READ_ALM         (0x0000)  // NET-ID 0x0040
#define BLV_R_SHARE_READ_COMM_ERR    (0x0002)  // NET-ID 0x0056
#define BLV_R_SHARE_READ_IO_STATUS_1 (0x0004)  // NET-ID 0x00B8
#define BLV_R_SHARE_READ_IO_STATUS_5 (0x0006)  // NET-ID 0x00BC
#define BLV_R_SHARE_READ_DETECT_POS  (0x0008)  // NET-ID 0x004D
#define BLV_R_SHARE_READ_DETECT_VEL  (0x000A)  // NET-ID 0x0050

#define BLV_R_SHARE_WRITE_MOTOR_STOP (0x0000)  // NET-ID 0x00DF
#define BLV_R_SHARE_WRITE_ALM_RST    (0x0002)  // NET-ID 0x00C0
#define BLV_R_SHARE_WRITE_COMM_RST   (0x0004)  // NET-ID 0x00DE
#define BLV_R_SHARE_WRITE_DDD_METHOD (0x0006)  // NET-ID 0x002D
#define BLV_R_SHARE_WRITE_DDD_POS    (0x0008)  // NET-ID 0x002E
#define BLV_R_SHARE_WRITE_DDD_VEL    (0x000A)  // NET-ID 0x002F
#define BLV_R_SHARE_WRITE_DDD_ACC    (0x000C)  // NET-ID 0x0030
#define BLV_R_SHARE_WRITE_DDD_DEC    (0x000E)  // NET-ID 0x0031
#define BLV_R_SHARE_WRITE_DDD_TORQUE (0x0010)  // NET-ID 0x0032
#define BLV_R_SHARE_WRITE_DDD_TRIG   (0x0012)  // NET-ID 0x0033
#define BLV_R_SHARE_WRITE_DRIVER_IN  (0x0014)  // NET-ID 0x003E

#define BLV_R_REGI_ADDR_GLOBAL_ID    (0x0980)  // Share control global ID

#define BLV_R_MODBUS_DEVICE_DEFAULT  "/dev/ttyUSB0"
#define BLV_R_MODBUS_BAUDRATE_DEFAULT MODBUS_WRAPPER_BAUDRATE_115200
#define BLV_R_MODBUS_PARITY_DEFAULT  MODBUS_WRAPPER_PARITY_EVEN

#define BLV_R_LEFT_MOTOR_NAME        "left"
#define BLV_R_RIGHT_MOTOR_NAME       "right"

#define BLV_R_MODBUS_SLAVE_ADDR_MIN  (1)
#define BLV_R_MODBUS_SLAVE_ADDR_MAX  (31)
#define BLV_R_LEFT_MOTOR_DEFAULT_ID  (1)
#define BLV_R_RIGHT_MOTOR_DEFAULT_ID (2)
#define BLV_R_GLOBAL_DEFAULT_ID      (15)
#define BLV_R_LEFT_LOCAL_DEFAULT_ID  (1)
#define BLV_R_RIGHT_LOCAL_DEFAULT_ID (2)

#define BLV_R_LEFT_DIRECTION_POSITIVE (true)
#define BLV_R_RIGHT_DIRECTION_POSITIVE (false)

#define BLV_R_AUTO_EXCITATION_OFF_DEFAULT (false)
#define BLV_R_EXCITATION_OFF_DEFAULT_SEC (0.5)

#define BLV_R_MODBUS_DEVPATH_PARAM_STR "device_path"
#define BLV_R_LEFT_MOTOR_ID_PARAM_STR "left_id"
#define BLV_R_RIGHT_MOTOR_ID_PARAM_STR "right_id"
#define BLV_R_GLOBAL_ID_PARAM_STR "global_id"
#define BLV_R_LEFT_MOTOR_FORWARD_POSITIVE_STR "left_forward_rotation_is_positive"
#define BLV_R_RIGHT_MOTOR_FORWARD_POSITIVE_STR "right_forward_rotation_is_positive"
#define BLV_R_BAUDRATE_STR "baudrate"
#define BLV_R_PARITY_STR "parity"
#define BLV_R_STOP_BIT_STR "stop_bit"
#define BLV_R_TORQUE_IN_VEL_STR "torque_in_vel_controlling"
#define BLV_R_TORQUE_IN_POS_STR "torque_in_positioning"
#define BLV_R_SET_RPM_TIMEOUT_STR "set_rpm_timeout"
#define BLV_R_AUTO_EXCITATION_OFF_STR "auto_excitation_off"
#define BLV_R_EXCITATION_OFF_SEC_STR "excitation_off_sec"

#define BLV_R_SET_RPM_TIMEOUT_SEC_DEFAULT  (0.22)
#define BLV_R_DDD_LIFETIME_BY_TIMEOUT_RATE (1.2)

#define BLV_R_DETAIL_ERROR_MESSAGE_LENGTH  (20)

#define BLV_R_WAIT_MOTOR_S_ON_US           (50000)
#define BLV_R_WAIT_COMM_RESET_US           (1000000)

#define BLV_R_WAIT_MOTOR_STOP_US           (50000)

#define BLV_R_MODBUS_WAIT usleep(1000)

#define BLV_R_LOG_THROTTLE_SEC (60)

namespace robot_driver_fw
{
class BlvrController
{
public:
  /**
   * @brief コンストラクタ
   * @param nh [in]ROSノードハンドラ
   * @return BlvrControllerインスタンス
   * @details
   * - rosparamからパラメータを読み込み、ROS資源の生成を行う
   */
  explicit BlvrController(ros::NodeHandle &nh);

  /**
   * @brief デストラクタ
   */
  virtual ~BlvrController();

  /**
   * @brief 速度制御
   * @param left [in]左モーター回転速度[rpm]
   * @param right [in]右モーター回転速度[rpm]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BUSY モーター処理中
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getMotorStatus, BlvrController::checkMoveStatus, BlvrController::changeVel 
   * @details
   * - モーターが動作可能な状態であれば、指定回転速度をモーターに設定する
   * - 位置指定制御中は、ブロック動作状態を確認し、BUSYでなければ速度制御を行う
   * - モーターの速度制御を行った際は、制御途絶に備え、停止タイマーをセットする
   */
  int32_t setMotorRpm(double left, double right);

  /**
   * @brief 位置指定制御
   * @param left [in]左モーター回転数[r]
   * @param right [in]右モーター回転数[r]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BUSY モーター処理中
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getMotorStatus, BlvrController::checkMoveStatus, BlvrController::setPosition
   * @details
   * - モーターが動作可能な状態であれば、指定回転数をモーターに指示する
   * - 位置指定制御中は、ブロック動作状態を確認し、BUSYでなければ位置指定制御を行う
   * - 位置指定制御実行中に、回転数 0 が指定された場合は、モーターを停止する
   */
  int32_t setMotorPos(double left, double right);

  /**
   * @brief エンコーダポーリングコールバック関数
   * @param left [out]モーター回転数[rpm]
   * @param right [out]モーター回転数[rpm]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getEncodeInfo, BlvrController::getActualVelocity
   * @details
   * - モーターの実回転速度を取得する
   */
  int32_t encoderPolling(double &left, double &right);

  /**
   * @brief ステータスポーリングコールバック関数
   * @param list [out]モーター状態リスト
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getMotorStatus, BlvrController::checkAlarmStatus, BlvrController::checkEmgStopStatus,
   *     BlvrController::stopMotor, BlvrController::emgStop
   * @details
   * - モーターのエラー発生状態、及び、作動状態を取得し、結果に応じて内部状態を変更する
   * - 内部状態とエラーコードを返す
   * - 非常停止信号がONの際は、緊急停止処理を呼び出す
   */
  virtual int32_t statusPolling(std::vector<robot_driver_msgs::MotorStatus> &list);

  /**
   * @brief 初期化
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrModbus::init, BlvrModbusIdShare::configIdShareMode, BlvrController::resetAlarm,
   *     BlvrController::clearDeviation, BlvrController::startMotor
   * @details
   * - modbus通信確立後、モーターを初期化する
   */
  int32_t init();

  /**
   * @brief 終了
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::stopMotor, BlvrController::resetComm, BlvrModbus::fini
   * @details
   * - モーターを停止し、modbus通信解除する
   */
  int32_t fini();

  /**
   * @brief 再起動
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::fini, BlvrController::init
   * @details
   * - モーターを再起動(終了処理＋初期化処理)する
   */
  int32_t reset();

  /**
   * @brief 緊急停止
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::stopMotor
   * @details
   * - モーター動作中の場合はモータを停止する
   */
  int32_t emgStop();

  /**
   * @brief 位置指定制御速度コンフィグレーション
   * @param vel [in]速度[rpm]
   * @param acc [in]加速度[rpm/s]
   * @param dec [in]加速度[rpm/s]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::correctVelocityValue, BlvrController::correctAccelerationValue
   * @details
   * - 指定された速度パラメータを内部変数に設定する
   */
  int32_t configSetPosition(float vel, float acc, float dec);

  /**
   * @brief 状態取得
   * @param list [out]モーター状態リスト
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getMotorStatus, BlvrController::checkAlarmStatus, BlvrController::checkMoveStatus
   * @details
   * - エラー発生状態確認処理、作動状態確認処理を呼び出し、モーターの状態をlistで返す
   */
  virtual int32_t getStatus(std::vector<robot_driver_msgs::MotorStatus> &list);

  /**
   * @brief モーター励磁操作
   * @param status [in]励磁ON/OFF(ON:true/OFF:false)
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::NOT_SUPPORT 設定不可
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa なし
   * @details
   * - モーターの励磁状態を操作する
   */
  int32_t controlMotorExcitation(bool state);

  /**
   * @brief 加速度設定
   * @param acc [in]加速度[rpm/s]
   * @param dec [in]加速度[rpm/s]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::correctAccelerationValue
   * @details
   * - 指定された加速度パラメータを内部変数に設定する
   */
  void setAcceleration(float acc, float dec);

protected:
  ros::NodeHandle nh_;

  std::string tty_dev_path_;
  int32_t baudrate_;
  std::string parity_;
  int32_t stopbit_;
  int32_t left_target_id_;
  int32_t right_target_id_;
  int32_t global_id_;
  int32_t left_local_id_;
  int32_t right_local_id_;
  int32_t acceleration_;
  int32_t deceleration_;
  int32_t torque_;
  int32_t pos_vel_;
  int32_t pos_acc_;
  int32_t pos_dec_;
  int32_t pos_torque_;
  bool left_forward_rotation_is_positive_;
  bool right_forward_rotation_is_positive_;
  double set_rpm_timeout_sec_;
  uint16_t ddd_lifetime_ms_;
  bool auto_excitation_off_;
  double excitation_off_sec_;

  BlvrModbusIdShare *modbus_;

  bool direction_;
  int32_t status_;

  ros::Timer motor_stop_timer_;
  ros::Timer excitation_off_timer_;

  int16_t left_alarm_code_;
  int16_t right_alarm_code_;
  int16_t left_comm_err_code_;
  int16_t right_comm_err_code_;
  bool left_stop_status_;
  bool right_stop_status_;
  bool left_move_status_;
  bool right_move_status_;

  int32_t left_vel_;
  int32_t right_vel_;
  int32_t left_pos_;
  int32_t right_pos_;

  bool excitation_status_;
  bool manual_excitation_off_;

  int32_t correctVelocityValue(int32_t in);
  int32_t correctAccelerationValue(int32_t in);
  void createMotorStatusMsg(robot_driver_msgs::MotorStatus &status, const std::string &name,
                            int32_t err_code, bool move_status);
  int32_t startMotor();
  int32_t stopMotor();

private:
  double velocity_updating_interval_sec_;
  double status_polling_interval_sec_;

  void checkParam();
  void getConfig(ros::NodeHandle &nh);
  virtual int32_t modbusInit();
  int32_t initializeIdShareMode();
  int32_t turnOffMotorExcitation();
  int32_t resetAlarm();
  int32_t resetComm();
  int32_t clearDeviation();
  int32_t changeVel(int32_t left, int32_t right);
  virtual int32_t setZeroVel();
  int32_t setPosition(int32_t left, int32_t right);
  int32_t getMotorStatus();
  int32_t getEncodeInfo();
  bool checkAlarmStatus(uint16_t &left, uint16_t &right);
  bool checkEmgStopStatus();
  bool checkMoveStatus(bool &left, bool &right);
  bool checkMoveStatus();
  void getActualVelocity(int32_t &left, int32_t &right);
  void motorStopTimerCallback(const ros::TimerEvent&);
  void excitationOffTimerCallback(const ros::TimerEvent&);
};
};  // namespace robot_driver_fw
#endif  // BLV_R_CONTROLLER_BLV_R_CONTROLLER_H
