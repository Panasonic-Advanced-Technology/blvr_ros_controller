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
 * @file robot_driver.h
 * @date 2023/09/20
 * @brief   robot driver frameworkクラス
 * @details robot driver(共通部)
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_ROBOT_DRIVER_H
#define ROBOT_DRIVER_FW_ROBOT_DRIVER_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "robot_driver_fw/motor_driver_if.h"
#include "robot_driver_fw/odometry.h"
#include "robot_driver_fw/mecanum_odometry.h"
#include "robot_driver_fw/robot_status.h"
#include "robot_driver_msgs/MotorStatus.h"

#define ROBOT_DRIVER_DEFAULT_GEARRATIO (5.0)
#define ROBOT_DRIVER_DEFAULT_WHEEL     (0.12)
#define ROBOT_DRIVER_DEFAULT_TREAD     (0.50)
#define ROBOT_DRIVER_DEFAULT_WHEELBASE (0.50)
#define ROBOT_DRIVER_DEFAULT_ACC_IN_VEL (1.0)
#define ROBOT_DRIVER_DEFAULT_DEC_IN_VEL (1.0)
#define ROBOT_DRIVER_DEFAULT_LINEAR_VEL  (0.5)
#define ROBOT_DRIVER_DEFAULT_LINEAR_ACC  (1.0)
#define ROBOT_DRIVER_DEFAULT_LINEAR_DEC  (1.0)
#define ROBOT_DRIVER_DEFAULT_ANGULAR_VEL (1.0)
#define ROBOT_DRIVER_DEFAULT_ANGULAR_ACC (2.0)
#define ROBOT_DRIVER_DEFAULT_ANGULAR_DEC (2.0)
#define ROBOT_DRIVER_VELOCITY_UPDATING_INTERVAL_SEC (0.1)
#define ROBOT_DRIVER_ENCODER_POLLING_INTERVAL_SEC   (0.05)
#define ROBOT_DRIVER_STATUS_POLLING_INTERVAL_SEC    (0.5)

#define ROBOT_DRIVER_KEY_STR_VELOCITY_UPDATING_INTERVAL "velocity_updating_interval"
#define ROBOT_DRIVER_KEY_STR_ENCODER_POLLING_INTERVAL   "encoder_polling_interval"
#define ROBOT_DRIVER_KEY_STR_STATUS_POLLING_INTERVAL    "status_polling_interval"
#define ROBOT_DRIVER_KEY_STR_GEAR_RATIO  "gear_ratio"
#define ROBOT_DRIVER_KEY_STR_WHEEL_SIZE  "wheel_size"
#define ROBOT_DRIVER_KEY_STR_TREAD_WIDTH "tread_width"
#define ROBOT_DRIVER_KEY_STR_WHEEL_BASE  "wheel_base"
#define ROBOT_DRIVER_KEY_STR_ACC_IN_VEL  "acc_in_vel_controlling"
#define ROBOT_DRIVER_KEY_STR_DEC_IN_VEL  "dec_in_vel_controlling"
#define ROBOT_DRIVER_KEY_STR_LIN_VEL_POS "linear_vel_in_set_position"
#define ROBOT_DRIVER_KEY_STR_LIN_ACC_POS "linear_acc_in_set_position"
#define ROBOT_DRIVER_KEY_STR_LIN_DEC_POS "linear_dec_in_set_position"
#define ROBOT_DRIVER_KEY_STR_ANG_VEL_POS "angular_vel_in_set_position"
#define ROBOT_DRIVER_KEY_STR_ANG_ACC_POS "angular_acc_in_set_position"
#define ROBOT_DRIVER_KEY_STR_ANG_DEC_POS "angular_dec_in_set_position"

#define convMpsToRpm(v) ((v) / M_PI / wheel_size_ * gear_ratio_ * 60.0)
#define convTurnRadpsToRpm(w) (tread_width_ / wheel_size_ / 2.0 / M_PI * (w) * gear_ratio_ * 60.0)
#define convMToR(m) ((m) / M_PI / wheel_size_ * gear_ratio_)
#define convRadToR(q) (tread_width_ / wheel_size_ / 2.0 / M_PI * (q) * gear_ratio_)
#define convRpmToRadps(r) ((r) * 2.0 * M_PI / gear_ratio_ / 60.0)

namespace robot_driver_fw
{
enum RobotDriverErr
{
  NOERR           = 0,   //!< 成功
  NOT_FOUND       = -1,  //!< 指定リソースが見つからない
  NOT_PERM        = -2,  //!< アクセス権がない
  NOT_SUPPORT     = -3,  //!< 非サポート
  BAD_PARAM       = -4,  //!< パラメータエラー
  BUSY            = -5,  //!< 処理中
  BAD_STATE       = -6,  //!< 状態エラー。現状態でサポートされていない呼び出し。
  TIMEOUT         = -7,  //!< タイムアウト
  INTERNAL_ERR    = -8   //!< 内部エラー
};

std::string convRobotDriverErrToStr(RobotDriverErr code);

class RobotDriver
{
public:
  /**
   * @brief コンストラクタ
   * @param nh [in]ROSノードハンドラ
   * @param mdif [in]モータードライバインタフェースのインスタンス
   * @return RobotDriverインスタンス
   * @details
   * - rosparamからパラメータを読み込む
   */
  RobotDriver(ros::NodeHandle &nh, MotorDriverIf *mdif);

  /**
   * @brief デストラクタ
   */
  virtual ~RobotDriver();

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
  virtual int32_t init();

  /**
   * @brief 終了処理
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa MotorDriverIf::fini
   * @details
   * - モータードライバインタフェースのfiniを呼び出し、モータードライバの終了処理を行う
   */
  int32_t fini();

  /**
   * @brief 再起動処理
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa MotorDriverIf::reset
   * @details
   * - モータードライバインタフェースのresetを呼び出し、モータードライバの再起動処理を行う
   */
  int32_t reset();

  /**
   * @brief 速度指定
   * @param v [in]移動速度[m/s]
   * @param omega [in]旋回角速度[rad/s]
   * @return なし
   * @sa MotorDriverIf::setMotorRPM
   * @details
   * - 指定された移動速度、及び、旋回角速度を左右それぞれのモータ回転速度[rpm]に変換し、モータードライバへ指令を行う
   */
  void setTwist(double v, double omega);

  /**
   * @brief 位置指定
   * @param x [in]移動位置[m]
   * @param yaw [in]旋回角度[rad]
   * @return なし
   * @sa MotorDriverIf::configSetPos, MotorDriverIf::setMotorPos
   * @details
   * - x が指定された場合、直進(又は後退)として、直進用の速度パラメータをモータードライバに設定する
   * - x が指定されず、yaw が指定された場合、旋回用の速度パラメータをモータードライバに設定する
   * - 指定された移動位置、又は、旋回角度を左右それぞれのモータ回転数[r]に変換し、モータードライバへ指令を行う
   * - x, yaw のどちらも指定されなかった場合(x = 0, yaw = 0)、モータードライバへ回転数 0 の指令を行う(位置指定のキャンセル)
   */
  void setPos(double x, double yaw);

  /**
   * @brief 速度パラメータ設定
   * @param vel [in]移動速度[m/s]
   * @param acc [in]移動加速度[m/s^2]
   * @param dec [in]移動減速度[m/s^2]
   * @param a_vel [in]旋回角速度[rad/s]
   * @param a_acc [in]旋回角加速度[rad/s^2]
   * @param a_dec [in]旋回角減速度[rad/s^2]
   * @return なし
   * @details
   * - 移動速度/加速度/減速度、及び、旋回角速度/加速度/減速度を、モーター回転速度[rpm]に変換し、内部変数に保持する
   */
  void configSetPos(float vel, float acc, float dec, float a_vel, float a_acc, float a_dec);

  /**
   * @brief モーター状態取得
   * @param status [out]モーター状態リスト
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa MotorDriverIf::getMotorStatus
   * @details
   * - モータードライバインタフェースのモーター状態取得関数を呼び出し、モーター状態を取得する
   */
  int32_t getMotorStatus(std::vector<robot_driver_msgs::MotorStatus> &status);

  /**
   * @brief モーター励磁操作
   * @param state [in]励磁ON/OFF(ON:true/OFF:false)
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::NOT_SUPPORT 設定不可
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa MotorDriverIf::controlMotorExcitation
   * @details
   * - モータードライバインタフェースのモーター励磁操作関数を呼び出し、モーター励磁状態を操作する
   */
  int32_t controlMotorExcitation(bool state);

  /**
   * @brief エンコーダ情報取得ポーリングタイマーコールバック関数
   * @param 未使用
   * @return なし
   * @sa MotorDriverIf::encoderPollingTimerCallback
   * @details
   * - モータードライバインタフェースのエンコーダ情報取得ポーリングタイマーコールバック関数を呼び出す
   */
  void encoderPollingTimerCallback(const ros::TimerEvent&);

  /**
   * @brief ステータス情報取得ポーリングタイマーコールバック関数
   * @param 未使用
   * @return なし
   * @sa MotorDriverIf::statusPollingTimerCallback
   * @details
   * - モータードライバインタフェースのステータス情報取得ポーリングタイマーコールバック関数を呼び出す
   */
  void statusPollingTimerCallback(const ros::TimerEvent&);

  /**
   * @brief オドメトリの更新
   * @param left [in]左モーター回転速度[rpm]
   * @param right [in]右モーター回転速度[rpm]
   * @param odom [out]オドメトリ
   * @retval true 更新成功
   * @retval false 更新失敗
   * @sa Odometry::updateByRPM, Odometry::init, Odometry::getX, Odometry::getY, Odometry::getHeading,
   *     Odometry::getLinear, Odometry::getAngular
   * @details
   * - 左右モーター回転速度を基に、オドメトリの更新を行う
   * - オドメトリの更新に失敗した場合は、オドメトリの初期化を行う
   */
  bool updateOdometry(double left, double right, nav_msgs::Odometry &odom);

  /**
   * @brief モーター状態確認
   * @param status [in]モーター状態
   * @param emg_status [out]非常停止(ボタン)状態
   * @param e_stop [out]エラー停止状態
   * @param detail [out]エラー詳細情報
   * @return なし
   * @sa RobotDriver::emergencyStop
   * @details
   * - 左右モーター状態とエラー詳細情報を基に、非常停止状態、エラー停止状態、エラー詳細情報を返す
   * - 片方のモーターのみエラー停止状態だった場合は、緊急停止を実行する
   */
  void confirmStatus(std::vector<robot_driver_msgs::MotorStatus> status,
                     bool &emg_status, bool &e_stop, std::string &detail);

  /**
   * @brief 緊急停止
   * @param なし
   * @return なし
   * @sa MotorDriverIf::emgStop
   * @details
   * - モータードライバインタフェースのemgStopを呼び出し、モータードライバを緊急停止させる
   */
  void emergencyStop();

protected:
  ros::NodeHandle nh_;
  MotorDriverIf *mdif_;

  double gear_ratio_;   //!< 総減速比
  double wheel_size_;   //!< 車輪径
  double tread_width_;  //!< トレッド

  float vel_acc_;  //!< 速度制御時の加速度
  float vel_dec_;  //!< 速度制御時の減速度

  float pos_linear_vel_;   //!< 位置指定制御時の速度
  float pos_linear_acc_;   //!< 位置指定制御時の加速度
  float pos_linear_dec_;   //!< 位置指定制御時の減速度
  float pos_angular_vel_;  //!< 位置指定制御時の角速度
  float pos_angular_acc_;  //!< 位置指定制御時の角加速度
  float pos_angular_dec_;  //!< 位置指定制御時の角減速度

  bool isPositiveNumber(const char *str, double val);
  bool isPositiveNumber(const char *str, float val);
  int32_t checkParam();

private:
  Odometry odom_;

  void getConfig(ros::NodeHandle &nh);
  bool isMatchStatus(std::vector<robot_driver_msgs::MotorStatus> list, RobotDriverStatus status, int &number);
  bool isMatchStatus(std::vector<robot_driver_msgs::MotorStatus> list, RobotDriverStatus status);
  bool isMatchErrorStatus(std::vector<robot_driver_msgs::MotorStatus> list, int &number);
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_ROBOT_DRIVER_H
