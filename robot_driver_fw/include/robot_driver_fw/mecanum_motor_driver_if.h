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
 * @file mecanum_motor_driver_if.h
 * @date 2024/04/02
 * @brief   motor driver interfaceクラス メカナムホイール拡張
 * @details モータードライバインタフェース メカナムホイール拡張
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_MECANUM_MOTOR_DRIVER_IF_H
#define ROBOT_DRIVER_FW_MECANUM_MOTOR_DRIVER_IF_H

#include "robot_driver_fw/motor_driver_if.h"

namespace robot_driver_fw
{
class MecanumMotorDriverIf : public virtual MotorDriverIf
{
public:
  /**
   * @brief コンストラクタ
   */
  MecanumMotorDriverIf()
  {}

  /**
   * @brief デストラクタ
   */
  virtual ~MecanumMotorDriverIf()
  {}

  /**
   * @brief 速度制御インタフェース
   * @param fl [in]左前モータ回転速度[rpm]
   * @param rl [in]左後モータ回転速度[rpm]
   * @param fr [in]右前モータ回転速度[rpm]
   * @param rr [in]右後モータ回転速度[rpm]
   * @return なし
   * @details
   * - モータードライバの速度制御を行うインタフェース
   */
  virtual void setMotorRPM(double fl, double fr, double rl, double rr) = 0;

  /**
   * @brief 位置指定制御インタフェース
   * @param fl [in]左前モータ回転数[r]
   * @param rl [in]左後モータ回転数[r]
   * @param fr [in]右前モータ回転数[r]
   * @param rr [in]右後モータ回転数[r]
   * @return なし
   * @details
   * - モータードライバの位置指定制御を行うインタフェース
   */
  virtual void setMotorPOS(double fl, double fr, double rl, double rr) = 0;

  /**
   * @brief エンコーダコールバック登録インタフェース
   * @param func [in]コールバック関数
   * @param obj [in]コールバック関数のクラスオブジェクト
   * @return なし
   * @details
   * - エンコーダ情報取得完了時に呼び出されるコールバック関数の登録インタフェース
   */
  template<class T>
  void setMecanumEncoderCallback(void(T::*func)(double, double, double, double), T *obj)
  {
    mecanum_encoder_callback_ = boost::bind(func, obj, _1, _2, _3, _4);
  }

protected:
  boost::function<void(double, double, double, double)> mecanum_encoder_callback_;
};

};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_MECANUM_MOTOR_DRIVER_IF_H
