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
 * @file modbus_wrapper.h
 * @date 2024/02/07
 * @brief   Modbus通信ラッパークラス
 * @details libmodbusを用いたModbus RTU通信を行うためのラッパークラス
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef ROBOT_DRIVER_FW_MODBUS_WRAPPER_H
#define ROBOT_DRIVER_FW_MODBUS_WRAPPER_H

#include <string>
#include <modbus/modbus.h>

#define MODBUS_WRAPPER_BAUDRATE_2400   (2400)
#define MODBUS_WRAPPER_BAUDRATE_4800   (4800)
#define MODBUS_WRAPPER_BAUDRATE_9600   (9600)
#define MODBUS_WRAPPER_BAUDRATE_19200  (19200)
#define MODBUS_WRAPPER_BAUDRATE_38400  (38400)
#define MODBUS_WRAPPER_BAUDRATE_57600  (57600)
#define MODBUS_WRAPPER_BAUDRATE_115200 (115200)
#define MODBUS_WRAPPER_BAUDRATE_230400 (230400)
#define MODBUS_WRAPPER_PARITY_NONE     "none"
#define MODBUS_WRAPPER_PARITY_EVEN     "even"
#define MODBUS_WRAPPER_PARITY_ODD      "odd"
#define MODBUS_WRAPPER_STOPBIT_DEFAULT (1)
#define MODBUS_WRAPPER_STOPBIT_EXT     (2)
#define MODBUS_WRAPPER_DATA_BIT_LEN    (8)

#define MODBUS_WRAPPER_CRC_INIT        (0xFFFF)
#define MODBUS_WRAPPER_CRC_XOR         (0xA001)

#define MODBUS_WRAPPER_BROADCAST_TIMEOUT (10000)
#define MODBUS_WRAPPER_RESPONSE_TIMEOUT (500000)

namespace robot_driver_fw
{
class ModbusWrapper
{
public:
  /**
   * @brief コンストラクタ
   */
  ModbusWrapper();

  /**
   * @brief デストラクタ
   */
  ~ModbusWrapper();

  /**
   * @brief 初期化
   * @param dev_path [in]シリアルデバイスパス
   * @param baudrate [in]ボーレート
   * @param parity [in]パリティ(none|even|odd)
   * @param stopbit [in]ストップビット(1|2)
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa ModbusWrapper::checkParam
   * @details
   * - modbus通信確立する
   */
  int32_t init(std::string dev_path, int32_t baudrate, std::string parity, int32_t stopbit);

  /**
   * @brief 終了
   * @param なし
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_STATE 状態異常
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - modbus通信解除する
   */
  int32_t fini();

  /**
   * @brief コイル書き込み
   * @param target_id [in]対象スレーブアドレス
   * @param address [in]対象コイルアドレス
   * @param state [in]書き込む状態(true|false)
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM スレーブアドレス不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - 対象スレーブの対象コイルに状態を書き込む
   */
  int32_t setCoil(int32_t target_id, int32_t address, bool state);

  /**
   * @brief コイル読み出し
   * @param target_id [in]対象スレーブアドレス
   * @param address [in]対象コイルアドレス
   * @param state [out]コイルの状態(true|false)
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM スレーブアドレス不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - 対象スレーブの対象コイルから状態を読み出す
   */
  int32_t getCoil(int32_t target_id, int32_t address, bool *state);

  /**
   * @brief レジスタ書き込み
   * @param target_id [in]対象スレーブアドレス
   * @param address [in]対象レジスタアドレス
   * @param value [in]書き込む値
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM スレーブアドレス不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - 対象スレーブの対象レジスタに値を書き込む
   */
  int32_t setRegister(int32_t target_id, int32_t address, uint16_t value);

  /**
   * @brief 複数レジスタ書き込み
   * @param target_id [in]対象スレーブアドレス
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]書き込むレジスタ数
   * @param list [in]書き込む値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM スレーブアドレス不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - 対象スレーブの連続した複数のレジスタに値を書き込む
   */
  int32_t setMultiRegisters(int32_t target_id, int32_t address, uint16_t num, uint16_t *list);

  /**
   * @brief レジスタ読み出し
   * @param target_id [in]対象スレーブアドレス
   * @param address [in]対象レジスタアドレス
   * @param value [out]レジスタの値
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM スレーブアドレス不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - 対象スレーブの対象レジスタから値を読み出す
   */
  int32_t getRegister(int32_t target_id, int32_t address, uint16_t *value);

  /**
   * @brief 複数レジスタ読み出し
   * @param target_id [in]対象スレーブアドレス
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]読み出すレジスタ数
   * @param list [out]レジスタの値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM スレーブアドレス不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @details
   * - 対象スレーブの連続した複数のレジスタから値を読み出す
   */
  int32_t getMultiRegisters(int32_t target_id, int32_t address, uint16_t num, uint16_t *list);

private:
  std::string tty_dev_path_;
  int32_t baudrate_;
  std::string parity_;
  int32_t stopbit_;

  modbus_t *modbus_ctx_;

  int32_t checkParam();
};
};  // namespace robot_driver_fw
#endif  // ROBOT_DRIVER_FW_MODBUS_WRAPPER_H
