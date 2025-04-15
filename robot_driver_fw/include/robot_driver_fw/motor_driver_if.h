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
 * @file motor_driver_if.h
 * @date 2023/09/12
 * @brief   motor driver interfaceクラス
 * @details モータードライバインタフェース
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_MOTOR_DRIVER_IF_H
#define ROBOT_DRIVER_FW_MOTOR_DRIVER_IF_H

#include <vector>
#include <string>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "ros/ros.h"
#include "robot_driver_msgs/MotorStatus.h"

namespace robot_driver_fw
{
class MotorDriverIf
{
public:
  /**
   * @brief コンストラクタ
   */
  MotorDriverIf()
  {}

  /**
   * @brief デストラクタ
   */
  virtual ~MotorDriverIf()
  {}

  /**
   * 初期化パラメータ構造体
   */
  struct InitParam
  {
    float vel_acc;          //!< モータ回転加速度[rpms]
    float vel_dec;          //!< モータ回転減速度[rpms]
  };

  /**
   * @brief 初期化インタフェース
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - モータードライバの初期化を行うインタフェース
   */
  virtual int32_t init(InitParam &param) = 0;

  /**
   * @brief 終了インタフェース
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - モータードライバの終了処理を行うインタフェース
   */
  virtual int32_t fini() = 0;

  /**
   * @brief 再起動インタフェース
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - モータードライバの再起動を行うインタフェース
   */
  virtual int32_t reset() = 0;

  /**
   * @brief 緊急停止インタフェース
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - モータードライバの緊急停止を行うインタフェース
   */
  virtual int32_t emgStop() = 0;

  /**
   * @brief 速度制御インタフェース
   * @param left [in]左モータ回転速度[rpm]
   * @param right [in]右モータ回転速度[rpm]
   * @return なし
   * @details
   * - モータードライバの速度制御を行うインタフェース
   */
  virtual void setMotorRPM(double left, double right) = 0;

  /**
   * @brief 位置指定制御インタフェース
   * @param left [in]左モータ回転数[r]
   * @param right [in]右モータ回転数[r]
   * @return なし
   * @details
   * - モータードライバの位置指定制御を行うインタフェース
   */
  virtual void setMotorPOS(double left, double right) = 0;

  /**
   * @brief 位置指定制御速度コンフィグレーションインタフェース
   * @param vel [in]モータ回転速度[rpm]
   * @param acc [in]モータ回転加速度[rpms]
   * @param dec [in]モータ回転減速度[rpms]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - モータードライバの位置指定制御での速度パラメータをコンフィグレーションするインタフェース
   */
  virtual int32_t configSetPos(float vel, float acc, float dec) = 0;

  /**
   * @brief モータ状態取得インタフェース
   * @param status [out]モータ状態
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - モータードライバの状態を取得するインタフェース
   */
  virtual int32_t getMotorStatus(std::vector<robot_driver_msgs::MotorStatus> &status) = 0;

  /**
   * @brief モーター励磁操作インタフェース
   * @param state [in]モーター励磁ON/OFF(ON:true/OFF:false)
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::NOT_SUPPORT 設定不可
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - モーター励磁状態を操作するインタフェース
   */
  virtual int32_t controlMotorExcitation(bool state) = 0;

  /**
   * @brief エンコーダポーリングタイマーコールバックインタフェース
   * @param なし
   * @return なし
   * @details
   * - ポーリングタイマーからモータードライバのエンコーダ情報取得を行うコールバックインタフェース
   */
  virtual void encoderPollingTimerCallback() = 0;

  /**
   * @brief ステータスポーリングタイマーコールバックインタフェース
   * @param なし
   * @return なし
   * @details
   * - ポーリングタイマーからモータードライバの状態取得を行うコールバックインタフェース
   */
  virtual void statusPollingTimerCallback() = 0;

  /**
   * @brief エンコーダコールバック登録インタフェース
   * @param func [in]コールバック関数
   * @param obj [in]コールバック関数のクラスオブジェクト
   * @return なし
   * @details
   * - エンコーダ情報取得完了時に呼び出されるコールバック関数の登録インタフェース
   */
  template<class T>
  void setEncoderCallback(void(T::*func)(double, double), T *obj)
  {
    encoder_callback_ = boost::bind(func, obj, _1, _2);
  }

  /**
   * @brief ステータスコールバック登録インタフェース
   * @param func [in]コールバック関数
   * @param obj [in]コールバック関数のクラスオブジェクト
   * @return なし
   * @details
   * - 状態取得完了時に呼び出されるコールバック関数の登録インタフェース
   */
  template<class T>
  void setStatusCallback(void (T::*func)(std::vector<robot_driver_msgs::MotorStatus>), T *obj)
  {
    status_callback_ = boost::bind(func, obj, _1);
  }

protected:
  boost::function<void(double, double)> encoder_callback_;
  boost::function<void(std::vector<robot_driver_msgs::MotorStatus>)> status_callback_;
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_MOTOR_DRIVER_IF_H
