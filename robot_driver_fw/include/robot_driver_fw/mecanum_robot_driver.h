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
 * @file mecanum_robot_driver.h
 * @date 2024/04/02
 * @brief   robot driver frameworkクラス メカナムホイール拡張
 * @details robot driver(共通部) メカナムホイール拡張
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_MECANUM_ROBOT_DRIVER_H
#define ROBOT_DRIVER_FW_MECANUM_ROBOT_DRIVER_H

#include "robot_driver_fw/robot_driver.h"
#include "robot_driver_fw/mecanum_odometry.h"
#include "robot_driver_fw/mecanum_motor_driver_if.h"

namespace robot_driver_fw
{
class MecanumRobotDriver : public RobotDriver
{
public:
  /**
   * @brief 継承コンストラクタ
   */
  using RobotDriver::RobotDriver;

  /**
   * @brief デストラクタ
   */
  ~MecanumRobotDriver();

  /**
   * @brief 初期化
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa Odometry::setWheelParams, Odometry::init, MotorDriverIf::init
   * @details
   * - オドメトリのパラメータ設定と初期化を行う
   * - モータードライバインタフェースのinitを呼び出し、モータードライバの初期化を行う
   */
  int32_t init();

  /**
   * @brief 速度指定
   * @param vx [in]移動速度[m/s]
   * @param vy [in]移動速度[m/s]
   * @param omega [in]旋回角速度[rad/s]
   * @return なし
   * @sa MotorDriverIf::setMotorRPM
   * @details
   * - 指定された移動速度、及び、旋回角速度を４つのモータ回転速度[rpm]に変換し、モータードライバへ指令を行う
   */
  void setTwist(double vx, double vy, double omega);

  /**
   * @brief 位置指定
   * @param x [in]移動位置[m]
   * @param y [in]移動位置[m]
   * @param yaw [in]旋回角度[rad]
   * @return なし
   * @sa MotorDriverIf::configSetPos, MotorDriverIf::setMotorPos
   * @details
   * - x または y が指定された場合、姿勢維持移動として、直進用の速度パラメータをモータードライバに設定する
   * - x または y が指定されず、yaw が指定された場合、旋回用の速度パラメータをモータードライバに設定する
   * - 指定された移動位置、又は、旋回角度を左右それぞれのモータ回転数[r]に変換し、モータードライバへ指令を行う
   * - x, y, yaw のどれも指定されなかった場合(x = 0, y = 0, yaw = 0)、モータードライバへ回転数 0 の指令を行う(位置指定のキャンセル)
   */
  void setPos(double x, double y, double yaw);

  /**
   * @brief オドメトリの更新
   * @param fl [in]左前モーター回転速度[rpm]
   * @param fr [in]右前モーター回転速度[rpm]
   * @param rl [in]左後モーター回転速度[rpm]
   * @param rr [in]右後モーター回転速度[rpm]
   * @param odom [out]オドメトリ
   * @retval true 更新成功
   * @retval false 更新失敗
   * @sa Odometry::updateByRPM, Odometry::init, Odometry::getX, Odometry::getY, Odometry::getHeading,
   *     Odometry::getLinear, Odometry::getAngular
   * @details
   * - ４つのモーター回転速度を基に、オドメトリの更新を行う
   * - オドメトリの更新に失敗した場合は、オドメトリの初期化を行う
   */
  bool updateOdometry(double fl, double fr, double rl, double rr, nav_msgs::Odometry &odom);

private:
  MecanumOdometry mecanum_odom_;

  double wheel_base_;   //!< ホイールベース
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_MECANUM_ROBOT_DRIVER_H
