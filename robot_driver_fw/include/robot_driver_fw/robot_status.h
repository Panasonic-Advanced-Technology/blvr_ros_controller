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
 * @file robot_status.h
 * @date 2023/09/20
 * @brief   robot status クラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_ROBOT_STATUS_H
#define ROBOT_DRIVER_FW_ROBOT_STATUS_H

#include <string>

namespace robot_driver_fw
{
enum RobotDriverStatus
{
  NONE = 0,       //!< 未設定
  INITIALIZING,   //!< 初期化中
  STANDBY,        //!< 待機中
  RUNNING,        //!< 稼働中
  POSITIONING,    //!< 位置指定動作中
  FINALIZING,     //!< 終了処理中
  ERRSTOP,        //!< エラー停止中
  EMGSTOP,        //!< 非常停止中
  ERREMG,         //!< エラー緊急停止中
  ERRSTOPPING,    //!< エラー停止移行中
  EMGSTOPPING     //!< 非常停止移行中
};

class RobotStatus
{
public:
  RobotStatus();
  ~RobotStatus();

private:
  bool estop_;
  bool brake_;
  int32_t left_count_;
  int32_t right_count_;
  bool left_cmd_;
  bool right_cmd_;
  uint8_t battery_;
  RobotDriverStatus status_;
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_ROBOT_STATUS_H
