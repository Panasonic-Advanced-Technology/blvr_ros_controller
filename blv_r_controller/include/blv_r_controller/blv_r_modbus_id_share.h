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
 * @file blv_r_modbus_id_share.h
 * @date 2024/02/07
 * @brief   Modbus通信拡張(IDシェアモード)クラス
 * @details Orientalmotor独自方式のIDシェアモードでModbus通信を行うためのクラス
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef BLV_R_CONTROLLER_BLV_R_MODBUS_ID_SHARE_H
#define BLV_R_CONTROLLER_BLV_R_MODBUS_ID_SHARE_H

#include <vector>
#include "robot_driver_fw/modbus_wrapper.h"

#define BLV_R_MODBUS_GLOBAL_ID_MIN   (1)
#define BLV_R_MODBUS_GLOBAL_ID_MAX   (31)
#define BLV_R_MODBUS_LOCAL_ID_MIN    (1)
#define BLV_R_MODBUS_LOCAL_ID_2WD_MAX (2)
#define BLV_R_SHARE_CTRL_NUM_2WD      (2)

#define BLV_R_MODBUS_FUNC_READ_REGI  (0x03)

namespace robot_driver_fw
{
class BlvrModbusIdShare : public ModbusWrapper
{
public:
  /**
   * @brief コンストラクタ
   */
  BlvrModbusIdShare();

  /**
   * @brief デストラクタ
   */
  virtual ~BlvrModbusIdShare();

  /**
   * @brief IDシェアモードコンフィグ
   * @param global_id [in]GlobalID
   * @param left_local_id [in]左モータのLocalID
   * @param right_local_id [in]右モータのLocalID
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM パラメータ異常
   * @sa BlvrModbusIdShare::checkParam
   * @details
   * - IDシェアモードの対象となるGlobalID,LocalID(左右モータ)を登録する
   */
  int32_t configIdShareMode(int32_t global_id, int32_t left_local_id, int32_t right_local_id);

  /**
   * @brief 複数レジスタ書き込み(IDシェアモード)
   * @param target_id [in]対象GlobalID
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]書き込むレジスタ数
   * @param l_list [in]左モータのレジスタへ書き込む値のリスト[num]
   * @param r_list [in]右モータのレジスタへ書き込む値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM GlobalID不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa ModbusWrapper::setMultiRegisters
   * @details
   * - 対象GlobalIDのシェアグループ(左右モータ)の連続した複数のレジスタに値を書き込む
   */
  int32_t idShareSetRegisters(int32_t target_id, int32_t address, uint16_t num,
                              const std::vector<uint16_t> &l_list, const std::vector<uint16_t> &r_list);

  /**
   * @brief 複数レジスタ書き込み(IDシェアモード:同一データ書き込み)
   * @param target_id [in]対象GlobalID
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]書き込むレジスタ数
   * @param list [in]レジスタへ書き込む値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM GlobalID不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa BlvrModbusIdShare::idShareSetRegisters
   * @details
   * - 対象GlobalIDのシェアグループ(左右モータ)の連続した複数のレジスタに値を書き込む
   */
  virtual int32_t idShareSetRegistersSameData(int32_t target_id, int32_t address, uint16_t num,
                                              const std::vector<uint16_t> &list);

  /**
   * @brief 複数レジスタ読み出し(IDシェアモード)
   * @param target_id [in]対象GlobalID
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]読み出すレジスタ数
   * @param l_list [in]左モータのレジスタから読み出す値のリスト[num]
   * @param r_list [in]右モータのレジスタから読み出す値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM GlobalID不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa ModbusWrapper::getMultiRegisters, BlvrModbusIdShare::calcCrc
   * @details
   * - 対象GlobalIDのシェアグループ(左右モータ)の連続した複数のレジスタから値を読み出す
   */
  int32_t idShareGetRegisters(int32_t target_id, int32_t address, uint16_t num,
                              std::vector<uint16_t> &l_list, std::vector<uint16_t> &r_list);

protected:
  int32_t target_id_;
  int32_t left_local_id_;
  int32_t right_local_id_;
  int32_t share_ctrl_num_;
  uint16_t calcCrc(uint16_t init_crc, const std::vector<uint8_t> &data);

private:
  int32_t checkParam();
};
};  // namespace robot_driver_fw
#endif  // BLV_R_CONTROLLER_BLV_R_MODBUS_ID_SHARE_H
