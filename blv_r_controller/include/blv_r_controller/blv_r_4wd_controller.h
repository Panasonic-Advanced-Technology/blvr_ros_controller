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
 * @file blv_r_4wd_controller.h
 * @date 2024/10/02
 * @brief   OM BLV_R motor driver 制御クラス for 4 wheel drive
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef BLV_R_CONTROLLER_BLV_R_4WD_CONTROLLER_H
#define BLV_R_CONTROLLER_BLV_R_4WD_CONTROLLER_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include "robot_driver_fw/mecanum_motor_driver_if.h"
#include "blv_r_controller/blv_r_4wd_modbus_id_share.h"
#include "blv_r_controller/blv_r_controller.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include "robot_driver_msgs/MotorStatus.h"

#define BLV_R_REAR_LEFT_MOTOR_NAME        "rear_left"
#define BLV_R_REAR_RIGHT_MOTOR_NAME       "rear_right"

#define BLV_R_REAR_LEFT_MOTOR_DEFAULT_ID  (3)
#define BLV_R_REAR_RIGHT_MOTOR_DEFAULT_ID (4)
#define BLV_R_REAR_LEFT_LOCAL_DEFAULT_ID  (3)
#define BLV_R_REAR_RIGHT_LOCAL_DEFAULT_ID (4)

#define BLV_R_REAR_LEFT_MOTOR_ID_PARAM_STR "rear_left_id"
#define BLV_R_REAR_RIGHT_MOTOR_ID_PARAM_STR "rear_right_id"

namespace robot_driver_fw
{
class Blvr4wdController : public BlvrController
{
public:
  /**
   * @brief コンストラクタ
   * @param nh [in]ROSノードハンドラ
   * @return Blvr4wdControllerインスタンス
   * @details
   * - rosparamからパラメータを読み込み、ROS資源の生成を行う
   */
  explicit Blvr4wdController(ros::NodeHandle &nh);

  /**
   * @brief デストラクタ
   */
  virtual ~Blvr4wdController();

  /**
   * @brief 速度制御
   * @param fl [in]左前モーター回転速度[rpm]
   * @param fr [in]右前モーター回転速度[rpm]
   * @param rl [in]左後モーター回転速度[rpm]
   * @param rr [in]右後モーター回転速度[rpm]
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
  int32_t setMotorRpm(double fl, double fr, double rl, double rr);

  /**
   * @brief 位置指定制御
   * @param fl [in]左前モーター回転数[r]
   * @param fr [in]右前モーター回転数[r]
   * @param rl [in]左後モーター回転数[r]
   * @param rr [in]右後モーター回転数[r]
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
  int32_t setMotorPos(double fl, double fr, double rl, double rr);

  /**
   * @brief エンコーダポーリングコールバック関数
   * @param fl [out]左前モーター回転数[rpm]
   * @param fr [out]右前モーター回転数[rpm]
   * @param rl [out]左後モーター回転数[rpm]
   * @param rr [out]右後モーター回転数[rpm]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getEncodeInfo, BlvrController::getActualVelocity
   * @details
   * - モーターの実回転速度を取得する
   */
  int32_t encoderPolling(double &fl, double &fr, double &rl, double &rr);

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
  int32_t statusPolling(std::vector<robot_driver_msgs::MotorStatus> &list) override;

  /**
   * @brief 状態取得
   * @param list [out]モーター状態リスト
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getMotorStatus, BlvrController::checkAlarmStatus, BlvrController::checkMoveStatus
   * @details
   * - エラー発生状態確認処理、作動状態確認処理を呼び出し、モーターの状態をlistで返す
   */
  int32_t getStatus(std::vector<robot_driver_msgs::MotorStatus> &list) override;

private:
  int32_t rear_left_target_id_;
  int32_t rear_right_target_id_;
  int32_t rear_left_local_id_;
  int32_t rear_right_local_id_;

  int16_t rear_left_alarm_code_;
  int16_t rear_right_alarm_code_;
  int16_t rear_left_comm_err_code_;
  int16_t rear_right_comm_err_code_;
  bool rear_left_stop_status_;
  bool rear_right_stop_status_;
  bool rear_left_move_status_;
  bool rear_right_move_status_;

  int32_t rear_left_vel_;
  int32_t rear_right_vel_;
  int32_t rear_left_pos_;
  int32_t rear_right_pos_;

  void checkParam();
  void getConfig(ros::NodeHandle &nh);
  int32_t modbusInit() override;
  int32_t initializeIdShareMode();
  int32_t changeVel(int32_t fl, int32_t fr, int32_t rl, int32_t rr);
  int32_t setZeroVel() override;
  int32_t setPosition(int32_t fl, int32_t fr, int32_t rl, int32_t rr);
  int32_t getMotorStatus();
  int32_t getEncodeInfo();
  bool checkAlarmStatus(uint16_t &fl, uint16_t &fr, uint16_t &rl, uint16_t &rr);
  bool checkEmgStopStatus();
  bool checkMoveStatus(bool &fl, bool &fr, bool &rl, bool &rr);
  bool checkMoveStatus();
  void getActualVelocity(int32_t &fl, int32_t &fr, int32_t &rl, int32_t &rr);
};
};  // namespace robot_driver_fw
#endif  // BLV_R_CONTROLLER_BLV_R_4WD_CONTROLLER_H
