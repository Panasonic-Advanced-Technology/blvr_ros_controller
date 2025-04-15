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
 * @file blv_r_manager.h
 * @date 2024/02/06
 * @brief   OM BLV_R motor driver 管理クラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef BLV_R_CONTROLLER_BLV_R_MANAGER_H
#define BLV_R_CONTROLLER_BLV_R_MANAGER_H

#include <vector>
#include <string>
#include "ros/ros.h"
#include "robot_driver_fw/motor_driver_if.h"
#include "blv_r_controller/blv_r_controller.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

namespace robot_driver_fw
{
class BlvrManager : public virtual MotorDriverIf
{
public:
  /**
   * @brief コンストラクタ
   * @param nh [in]ROSノードハンドラ
   * @return BlvrManagerインスタンス
   * @details
   * - rosparamからパラメータを読み込み、ROS資源の生成を行う
   */
  explicit BlvrManager(ros::NodeHandle &);

  /**
   * @brief デストラクタ
   */
  virtual ~BlvrManager();

  /**
   * @brief 初期化
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::init
   * @details
   * - モーターコントローラの初期化を呼び出す
   */
  virtual int32_t init(MotorDriverIf::InitParam &param);

  /**
   * @brief 終了
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::fini
   * @details
   * - モーターコントローラの終了を呼び出す
   */
  virtual int32_t fini();

  /**
   * @brief 再起動
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::reset
   * @details
   * - モーターコントローラの再起動を呼び出す
   */
  virtual int32_t reset();

  /**
   * @brief 緊急停止
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::emgStop
   * @details
   * - モーターコントローラの緊急停止を呼び出す
   */
  virtual int32_t emgStop();

  /**
   * @brief 速度制御
   * @param left [in]左モーター回転速度[rpm]
   * @param right [in]右モーター回転速度[rpm]
   * @return なし
   * @sa BlvrController::setMotorRpm
   * @details
   * - モーターコントローラに対して指定された回転速度を設定する
   */
  virtual void setMotorRPM(double left, double right);

  /**
   * @brief 位置指定制御
   * @param left [in]左モーター回転数[r]
   * @param right [in]右モーター回転数[r]
   * @return なし
   * @sa BlvrController::setMotorPos
   * @details
   * - モーターコントローラに対して指定された回転数を設定する
   */
  virtual void setMotorPOS(double left, double right);

  /**
   * @brief 位置指定制御速度コンフィグレーション
   * @param vel [in]モーター回転速度[rpm]
   * @param acc [in]モーター回転加速度[rpms]
   * @param dec [in]モーター回転減速度[rpms]
   * @retval RobotDriverErr::NOERR 正常終了
   * @sa BlvrController::configSetPosition
   * @details
   * - モーターコントローラに対して指定された速度パラメータを設定する
   */
  virtual int32_t configSetPos(float vel, float acc, float dec);

  /**
   * @brief モーター状態取得
   * @param status [out]モーター状態
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::getStatus
   * @details
   * - モーターコントローラからモータ状態を取得する
   */
  virtual int32_t getMotorStatus(std::vector<robot_driver_msgs::MotorStatus> &status);

  /**
   * @brief モーター励磁操作
   * @param state [in]モーター励磁ON/OFF(ON:true/OFF:false)
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::NOT_SUPPORT 設定不可
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrController::controlMotorExcitation
   * @details
   * - モーターコントローラを通して励磁状態を操作する
   */
  virtual int32_t controlMotorExcitation(bool state);

  /**
   * @brief エンコーダポーリングタイマーコールバック
   * @param なし
   * @return なし
   * @sa BlvrController::encoderPolling, MotorDriverIf::encoder_callback_
   * @details
   * - モーターコントローラからエンコーダ情報を取得し、コールバックを呼び出す
   */
  virtual void encoderPollingTimerCallback();

  /**
   * @brief エンコーダポーリングタイマーコールバック
   * @param なし
   * @return なし
   * @sa BlvrController::statusPolling, MotorDriverIf::status_callback_
   * @details
   * - モーターコントローラからモータ状態を取得し、コールバックを呼び出す
   */
  virtual void statusPollingTimerCallback();

protected:
  ros::NodeHandle nh_;
  BlvrController *controller_;

private:
  void checkParam();
  void getConfig(ros::NodeHandle &nh);
};
};  // namespace robot_driver_fw
#endif  // BLV_R_CONTROLLER_BLV_R_MANAGER_H
