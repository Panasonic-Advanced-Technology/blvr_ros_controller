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
 * @file blv_r_manager_main.cpp
 * @date 2024/10/03
 * @brief   OM BLV_R motor driver 管理クラス メイン関数
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include "ros/ros.h"
#include "robot_driver_fw/robot_driver_ros.h"
#include "blv_r_controller/blv_r_manager.h"

/*!
 * @brief   blv_r_controller nodeメイン関数
 * @param   argc 引数の数(未使用)
 * @param   argv 引数の中身(未使用)
 * @return  リターンコード
 * @details
 *  - ros::init()を呼び出し、ノードを初期化する
 *  - BlvrManagerクラスのインスタンスを作成する
 *  - RobotDriverRosクラスのインスタンスを作成する
 *  - mainLoop()を呼び出す
 */
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "blv_r_controller");
  ros::NodeHandle private_nh("~");
  robot_driver_fw::BlvrManager blv_r = robot_driver_fw::BlvrManager(private_nh);
  robot_driver_fw::RobotDriverRos rb_ros = robot_driver_fw::RobotDriverRos(private_nh, &blv_r);

  int32_t ret;
  ret = rb_ros.init();
  if (ret != robot_driver_fw::RobotDriverErr::NOERR)
  {
    ROS_ERROR("init failed(%d)", ret);

    // wait ros shutdown
    ros::waitForShutdown();

    return ret;
  }

  rb_ros.mainLoop();

  rb_ros.fini();

  return(0);
}
