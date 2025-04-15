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
 * @file mecanum_odometry.h
 * @date 2024/03/04
 * @brief   メカナムホイール用オドメトリ
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_MECANUM_ODOMETRY_H
#define ROBOT_DRIVER_FW_MECANUM_ODOMETRY_H

#include "ros/ros.h"

namespace robot_driver_fw
{

class MecanumOdometry
{
public:
  /**
   * @brief コンストラクタ
   */
  MecanumOdometry();

  /**
   * @brief デストラクタ
   */
  ~MecanumOdometry();

  /**
   * @brief 初期化
   * @param なし
   * @return なし
   * @details
   * - オドメトリの初期化を行う
   */
  void init();

  /**
   * @brief ホイールパラメータ設定
   * @param tread_width [in]トレッド幅[m]
   * @param wheel_base [in]ホイールベース[m]
   * @param wheel_radius [in]ホイール半径[m]
   * @return なし
   * @details
   * - ホイールパラメータを設定する
   *   update呼び出しまでに設定しておく必要がある
   */
  void setWheelParams(double tread_width, double wheel_base, double wheel_radius);

  /**
   * @brief オドメトリ更新
   * @param fl [in]左前ホイール回転速度[rad/s]
   * @param fr [in]右前ホイール回転速度[rad/s]
   * @param rl [in]左後ホイール回転速度[rad/s]
   * @param rr [in]右後ホイール回転速度[rad/s]
   * @retval true 更新成功
   * @retval false 更新失敗
   * @details
   * - オドメトリを更新する
   */
  bool update(double fl, double fr, double rl, double rr, ros::Time &time);

  /**
   * @brief x位置取得
   * @param なし
   * @return x位置[m]
   * @details
   * - x位置を取得する
   */
  double getX();

  /**
   * @brief y位置取得
   * @param なし
   * @return y位置[m]
   * @details
   * - y位置を取得する
   */
  double getY();

  /**
   * @brief 方向取得
   * @param なし
   * @return 方向[rad]
   * @details
   * - 方向を取得する
   */
  double getYaw();

  /**
   * @brief x方向速度取得
   * @param なし
   * @return x方向速度[m/s]
   * @details
   * - x方向速度を取得する
   */
  double getVX();

  /**
   * @brief y方向速度取得
   * @param なし
   * @return y方向速度[m/s]
   * @details
   * - y方向速度を取得する
   */
  double getVY();

  /**
   * @brief 角速度取得
   * @param なし
   * @return 角速度[rad/s]
   * @details
   * - 角速度を取得する
   */
  double getW();

private:
  double tread_width_;
  double wheel_base_;
  double wheel_radius_;
  double x_;
  double y_;
  double yaw_;
  ros::Time timestamp_;
  double vx_;
  double vy_;
  double w_;
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_MECANUM_ODOMETRY_H
