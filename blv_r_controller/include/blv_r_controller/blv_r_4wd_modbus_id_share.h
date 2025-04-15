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
 * @file blv_r_4wd_modbus_id_share.h
 * @date 2024/10/02
 * @brief   Modbus通信拡張(IDシェアモード)クラス for 4 wheel drive
 * @details Orientalmotor独自方式のIDシェアモードでModbus通信を行うためのクラス
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#ifndef BLV_R_CONTROLLER_BLV_R_4WD_MODBUS_ID_SHARE_H
#define BLV_R_CONTROLLER_BLV_R_4WD_MODBUS_ID_SHARE_H

#include "blv_r_controller/blv_r_modbus_id_share.h"

#define BLV_R_MODBUS_LOCAL_ID_4WD_MAX (4)
#define BLV_R_SHARE_CTRL_NUM_4WD      (4)

namespace robot_driver_fw
{
class Blvr4wdModbusIdShare : public BlvrModbusIdShare
{
public:
  /**
   * @brief コンストラクタ
   */
  Blvr4wdModbusIdShare();

  /**
   * @brief デストラクタ
   */
  virtual ~Blvr4wdModbusIdShare();

  /**
   * @brief IDシェアモードコンフィグ
   * @param global_id [in]GlobalID
   * @param fl_local_id [in]左前モータのLocalID
   * @param fr_local_id [in]右前モータのLocalID
   * @param rl_local_id [in]左後モータのLocalID
   * @param rr_local_id [in]右後モータのLocalID
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM パラメータ異常
   * @sa BlvrModbusIdShare::checkParam
   * @details
   * - IDシェアモードの対象となるGlobalID,LocalID(4モータ)を登録する
   */
  int32_t configIdShareMode(int32_t global_id, int32_t fl_local_id, int32_t fr_local_id,
                            int32_t rl_local_id, int32_t rr_local_id);

  /**
   * @brief 複数レジスタ書き込み(IDシェアモード)
   * @param target_id [in]対象GlobalID
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]書き込むレジスタ数
   * @param fl_list [in]左前モータのレジスタへ書き込む値のリスト[num]
   * @param fr_list [in]右前モータのレジスタへ書き込む値のリスト[num]
   * @param rl_list [in]左後モータのレジスタへ書き込む値のリスト[num]
   * @param rr_list [in]右後モータのレジスタへ書き込む値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM GlobalID不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa ModbusWrapper::setMultiRegisters
   * @details
   * - 対象GlobalIDのシェアグループ(4モータ)の連続した複数のレジスタに値を書き込む
   */
  int32_t idShareSetRegisters(int32_t target_id, int32_t address, uint16_t num,
                              const std::vector<uint16_t> &fl_list, const std::vector<uint16_t> &fr_list,
                              const std::vector<uint16_t> &rl_list, const std::vector<uint16_t> &rr_list);

  /**
   * @brief 複数レジスタ書き込み(IDシェアモード:同一データ書き込み)
   * @param target_id [in]対象GlobalID
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]書き込むレジスタ数
   * @param list [in]レジスタへ書き込む値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM GlobalID不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa Blvr4wdModbusIdShare::idShareSetRegisters
   * @details
   * - 対象GlobalIDのシェアグループ(4モータ)の連続した複数のレジスタに同一データを書き込む
   */
  int32_t idShareSetRegistersSameData(int32_t target_id, int32_t address, uint16_t num,
                                      const std::vector<uint16_t> &list) override;

  /**
   * @brief 複数レジスタ読み出し(IDシェアモード)
   * @param target_id [in]対象GlobalID
   * @param address [in]対象レジスタの先頭アドレス
   * @param num [in]読み出すレジスタ数
   * @param fl_list [in]左前モータのレジスタから読み出す値のリスト[num]
   * @param fr_list [in]右前モータのレジスタから読み出す値のリスト[num]
   * @param rl_list [in]左後モータのレジスタから読み出す値のリスト[num]
   * @param rr_list [in]右後モータのレジスタから読み出す値のリスト[num]
   * @retval RobotDriverErr::NOERR 正常終了
   * @retval RobotDriverErr::BAD_PARAM GlobalID不正
   * @retval RobotDriverErr::INTERNAL_ERR 通信異常
   * @sa ModbusWrapper::getMultiRegisters, BlvrModbusIdShare::calcCrc
   * @details
   * - 対象GlobalIDのシェアグループ(4モータ)の連続した複数のレジスタから値を読み出す
   */
  int32_t idShareGetRegisters(int32_t target_id, int32_t address, uint16_t num,
                              std::vector<uint16_t> &fl_list, std::vector<uint16_t> &fr_list,
                              std::vector<uint16_t> &rl_list, std::vector<uint16_t> &rr_list);

private:
  int32_t target_id_;
  int32_t left_local_id_;
  int32_t right_local_id_;
  int32_t rear_left_local_id_;
  int32_t rear_right_local_id_;
  int32_t share_ctrl_num_;

  int32_t checkParam();
};
};  // namespace robot_driver_fw
#endif  // BLV_R_CONTROLLER_BLV_R_4WD_MODBUS_ID_SHARE_H
