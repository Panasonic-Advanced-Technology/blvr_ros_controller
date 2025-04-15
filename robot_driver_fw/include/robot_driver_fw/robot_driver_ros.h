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
 * @file robot_driver_ros.h
 * @date 2023/09/20
 * @brief   robot driver ROS クラス
 * @details robot driver(共通部) ROS wrapper
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_ROBOT_DRIVER_ROS_H
#define ROBOT_DRIVER_FW_ROBOT_DRIVER_ROS_H

#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "robot_driver_fw/robot_driver.h"
#include "robot_driver_fw/motor_driver_if.h"
#include "robot_driver_fw/robot_status.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyResponse.h"
#include "std_srvs/SetBool.h"
#include "robot_driver_msgs/ConfigSetPos.h"
#include "robot_driver_msgs/GetMotorStatus.h"
#include "robot_driver_msgs/MotorStatus.h"

#define ROBOT_DRIVER_LEFT_MOTOR_NAME "left"
#define ROBOT_DRIVER_RIGHT_MOTOR_NAME "right"

#define ROBOT_DRIVER_ODOM_FRAME_ID_DEFAULT       "odom"
#define ROBOT_DRIVER_ODOM_CHILD_FRAME_ID_DEFAULT "base_link"
#define ROBOT_DRIVER_KEY_STR_FRAME_ID            "frame_id"
#define ROBOT_DRIVER_KEY_STR_CHILD_FRAME_ID      "child_frame_id"

namespace robot_driver_fw
{

class RobotDriverRos
{
public:
  /**
   * @brief コンストラクタ
   * @param nh [in]ROSノードハンドラ
   * @param mdif [in]モータードライバインタフェースのインスタンス
   * @return RobotDriverRosインスタンス
   * @details
   * - rosparamからパラメータを読み込む
   * - subscriber/publisher/service serverを生成する
   * - モータードライバインターフェースへ各種コールバックを登録する
   */
  RobotDriverRos(ros::NodeHandle &nh, MotorDriverIf *mdif);

  /**
   * @brief デストラクタ
   */
  virtual ~RobotDriverRos();

  /**
   * @brief メインループ
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @sa RobotDriver::encoderPollingTimerCallback, RobotDriver::statusPollingTimerCallback
   * @details
   * - 各種タイマーを生成した後、ros::spin()を読み出し定常状態となる
   * - 未初期化で呼び出された場合は、状態異常エラーを返す
   */
  int32_t mainLoop();

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
   * @brief 終了処理
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 内部通信異常
   * @sa RobotDriver::fini
   * @details
   * - ロボットドライバの終了処理を行う
   * - 未初期化で呼び出された場合は、状態異常エラーを返す
   */
  int32_t fini();

  /**
   * @brief 再起動
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 内部通信異常
   * @sa RobotDriver::reset
   * @details
   * - ロボットドライバの再起動を行う
   * - 未初期化で呼び出された場合は、状態異常エラーを返す
   */
  int32_t reset();

  /**
   * @brief エンコーダコールバック
   * @param left [in]左モーター回転数[rpm]
   * @param right [in]右モーター回転数[rpm]
   * @return なし
   * @sa RobotDriver::updateOdometry
   * @details
   * - 左右モーターの回転数から、オドメトリを更新する
   * - オドメトリ(~/odom)をpublishする
   */
  void encoderCallback(double left, double right);

  /**
   * @brief ステータスコールバック
   * @param state [in]モーターの状態
   * @return なし
   * @sa RobotDriver::confirmStatus
   * @details
   * - 左右モーターのどちらかにエラーが発生していた場合は、もう片方のモーターを停止させる
   * - ステータス(~/status)をpublishする
   * - 停止した場合は(~/estop)をpublishする
   */
  void statusCallback(std::vector<robot_driver_msgs::MotorStatus> status);

protected:
  enum class status : int
  {
    NONE = -1,
    OFF = 0,
    ON = 1
  };

  ros::NodeHandle nh_;
  MotorDriverIf *mdif_;
  RobotDriver *rb_driver_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber cmd_pos_sub_;
  ros::Publisher odometry_pub_;
  ros::Publisher alive_error_pub_;

  bool initialized_;
  bool velocity_update_;
  std::string frame_id_;
  std::string child_frame_id_;

private:
  ros::Publisher emg_status_pub_;
  ros::Publisher e_stop_pub_;
  ros::Publisher status_pub_;
  RobotStatus rb_status_;
  double velocity_updating_interval_sec_;
  double encoder_polling_interval_sec_;
  double status_polling_interval_sec_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer config_set_pos_srv_;
  ros::ServiceServer get_status_srv_;
  ros::ServiceServer control_excitation_srv_;
  ros::Timer velocity_updating_timer_;
  ros::Timer encoder_polling_timer_;
  ros::Timer status_polling_timer_;

  status estop_status_;
  status emg_status_;

  void getConfig();
  void velocityUpdatingTimerCallback(const ros::TimerEvent&);
  bool resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  bool configSetPositionCallback(robot_driver_msgs::ConfigSetPos::Request &req, robot_driver_msgs::ConfigSetPos::Response &res);
  bool getStatusCallback(robot_driver_msgs::GetMotorStatus::Request &, robot_driver_msgs::GetMotorStatus::Response &res);
  bool controlExcitationCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  virtual void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  virtual void cmdPosCallback(const geometry_msgs::Twist::ConstPtr& msg);

  #define CONV2STATUS(st) ((st) ? status::ON : status::OFF)
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_ROBOT_DRIVER_ROS_H
