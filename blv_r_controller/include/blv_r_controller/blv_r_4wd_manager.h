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
 * @file blv_r_4wd_manager.h
 * @date 2024/10/01
 * @brief   OM BLV_R motor driver 管理クラス for 4 wheel drive
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef BLV_R_CONTROLLER_BLV_R_4WD_MANAGER_H
#define BLV_R_CONTROLLER_BLV_R_4WD_MANAGER_H

#include "blv_r_controller/blv_r_manager.h"
#include "robot_driver_fw/mecanum_motor_driver_if.h"

namespace robot_driver_fw
{
class Blvr4wdManager : public MecanumMotorDriverIf, public BlvrManager
{
public:
  /**
   * @brief コンストラクタ
   * @param nh [in]ROSノードハンドラ
   * @return BlvrMecanumManagerインスタンス
   * @details
   * - rosparamからパラメータを読み込み、ROS資源の生成を行う
   */
  explicit Blvr4wdManager(ros::NodeHandle &nh);

  /**
   * @brief デストラクタ
   */
  virtual ~Blvr4wdManager();

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
  int32_t init(MotorDriverIf::InitParam &param) override;

  /**
   * @brief 速度制御
   * @param fl [in]左前モーター回転速度[rpm]
   * @param fr [in]右前モーター回転速度[rpm]
   * @param rl [in]左後モーター回転速度[rpm]
   * @param rr [in]右後モーター回転速度[rpm]
   * @return なし
   * @sa BlvrController::setMotorRpm
   * @details
   * - モーターコントローラに対して指定された回転速度を設定する
   */
  virtual void setMotorRPM(double fl, double fr, double rl, double rr);

  /**
   * @brief 位置指定制御
   * @param fl [in]左前モーター回転数[r]
   * @param fr [in]右前モーター回転数[r]
   * @param rl [in]左後モーター回転数[r]
   * @param rr [in]右後モーター回転数[r]
   * @return なし
   * @sa BlvrController::setMotorPos
   * @details
   * - モーターコントローラに対して指定された回転数を設定する
   */
  virtual void setMotorPOS(double fl, double fr, double rl, double rr);

  /**
   * @brief エンコーダポーリングタイマーコールバック
   * @param なし
   * @return なし
   * @sa BlvrController::encoderPolling, MotorDriverIf::encoder_callback_
   * @details
   * - モーターコントローラからエンコーダ情報を取得し、コールバックを呼び出す
   */
  void encoderPollingTimerCallback() override;

private:
  void checkParam();
  void getConfig(ros::NodeHandle &nh);
};
};  // namespace robot_driver_fw
#endif  // BLV_R_CONTROLLER_BLV_R_4WD_MANAGER_H
