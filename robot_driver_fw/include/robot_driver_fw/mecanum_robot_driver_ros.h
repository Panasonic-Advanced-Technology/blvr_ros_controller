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
 * @file mecanum_robot_driver_ros.h
 * @date 2024/04/02
 * @brief   robot driver ROS クラス メカナムホイール拡張
 * @details robot driver(共通部) ROS wrapper メカナムホイール拡張
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_MECANUM_ROBOT_DRIVER_ROS_H
#define ROBOT_DRIVER_FW_MECANUM_ROBOT_DRIVER_ROS_H

#include "robot_driver_fw/robot_driver_ros.h"
#include "robot_driver_fw/mecanum_robot_driver.h"

namespace robot_driver_fw
{

class MecanumRobotDriverRos : public RobotDriverRos
{
public:
  /**
   * @brief 継承コンストラクタ
   */
  using RobotDriverRos::RobotDriverRos;

  /**
   * @brief デストラクタ
   */
  ~MecanumRobotDriverRos();

  /**
   * @brief 初期化
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 内部通信異常
   * @sa RobotDriver::init
   * @details
   * - ロボットドライバの初期化を行う
   * - 初期化済みで呼び出された場合は、状態異常を返す
   */
  virtual int32_t init();

  /**
   * @brief エンコーダコールバック
   * @param fl [in]左前モーター回転数[rpm]
   * @param fr [in]右前モーター回転数[rpm]
   * @param rl [in]左後モーター回転数[rpm]
   * @param rr [in]右後モーター回転数[rpm]
   * @return なし
   * @sa RobotDriver::updateOdometry
   * @details
   * - 左右モーターの回転数から、オドメトリを更新する
   * - オドメトリ(~/odom)をpublishする
   */
  void encoderCallback(double fl, double fr, double rl, double rr);

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void cmdPosCallback(const geometry_msgs::Twist::ConstPtr& msg);
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_MECANUM_ROBOT_DRIVER_ROS_H
