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
 * @file blv_r_4wd_modbus_id_share.cpp
 * @date 2024/10/02
 * @brief   Modbus通信拡張(IDシェアモード)クラス for 4 wheel drive
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>
#include "robot_driver_fw/modbus_wrapper.h"
#include "blv_r_controller/blv_r_4wd_modbus_id_share.h"
#include "robot_driver_fw/robot_driver.h"

namespace robot_driver_fw
{
Blvr4wdModbusIdShare::Blvr4wdModbusIdShare() :
target_id_(0),
left_local_id_(0),
right_local_id_(0),
rear_left_local_id_(0),
rear_right_local_id_(0),
share_ctrl_num_(0)
{
}

Blvr4wdModbusIdShare::~Blvr4wdModbusIdShare()
{
}

int32_t Blvr4wdModbusIdShare::checkParam()
{
  if ((target_id_ < BLV_R_MODBUS_GLOBAL_ID_MIN) || (target_id_ > BLV_R_MODBUS_GLOBAL_ID_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((left_local_id_ < BLV_R_MODBUS_LOCAL_ID_MIN) || (left_local_id_ > BLV_R_MODBUS_LOCAL_ID_4WD_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((right_local_id_ < BLV_R_MODBUS_LOCAL_ID_MIN) || (right_local_id_ > BLV_R_MODBUS_LOCAL_ID_4WD_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((rear_left_local_id_ < BLV_R_MODBUS_LOCAL_ID_MIN) || (rear_left_local_id_ > BLV_R_MODBUS_LOCAL_ID_4WD_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((rear_right_local_id_ < BLV_R_MODBUS_LOCAL_ID_MIN) || (rear_right_local_id_ > BLV_R_MODBUS_LOCAL_ID_4WD_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((left_local_id_ == right_local_id_) ||
      (left_local_id_ == rear_left_local_id_) || (left_local_id_ == rear_right_local_id_) ||
      (right_local_id_ == rear_left_local_id_) || (right_local_id_ == rear_right_local_id_) ||
      (rear_left_local_id_ == rear_right_local_id_))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  return RobotDriverErr::NOERR;
}

int32_t Blvr4wdModbusIdShare::configIdShareMode(int32_t global_id,
                                                int32_t fl_local_id, int32_t fr_local_id,
                                                int32_t rl_local_id, int32_t rr_local_id)
{
  target_id_ = global_id;
  left_local_id_ = fl_local_id;
  right_local_id_ = fr_local_id;
  rear_left_local_id_ = rl_local_id;
  rear_right_local_id_ = rr_local_id;
  share_ctrl_num_ = BLV_R_SHARE_CTRL_NUM_4WD;
  return checkParam();
}

int32_t Blvr4wdModbusIdShare::idShareSetRegisters(int32_t target_id, int32_t address, uint16_t num,
                                                  const std::vector<uint16_t> &fl_list,
                                                  const std::vector<uint16_t> &fr_list,
                                                  const std::vector<uint16_t> &rl_list,
                                                  const std::vector<uint16_t> &rr_list)
{
  if (target_id != target_id_)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  int32_t ret;
  int32_t reg_num = num * BLV_R_SHARE_CTRL_NUM_4WD;
  std::vector<uint16_t> data;
  data.reserve(reg_num);

  for (int i = BLV_R_MODBUS_LOCAL_ID_MIN; i <= BLV_R_MODBUS_LOCAL_ID_4WD_MAX; i++)
  {
    if (i == left_local_id_)
    {
      std::copy(fl_list.begin(), fl_list.end(), std::back_inserter(data));
    }
    else if (i == right_local_id_)
    {
      std::copy(fr_list.begin(), fr_list.end(), std::back_inserter(data));
    }
    else if (i == rear_left_local_id_)
    {
      std::copy(rl_list.begin(), rl_list.end(), std::back_inserter(data));
    }
    else if (i == rear_right_local_id_)
    {
      std::copy(rr_list.begin(), rr_list.end(), std::back_inserter(data));
    }
    else
    {
      // nop
    }
  }

  ret = setMultiRegisters(target_id, address, reg_num, data.data());
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareSetRegisters failed(%d)", ret);
  }
  return ret;
}

int32_t Blvr4wdModbusIdShare::idShareSetRegistersSameData(int32_t target_id, int32_t address, uint16_t num,
                                                          const std::vector<uint16_t> &list)
{
  ROS_DEBUG("call Blvr4wdModbusIdShare::idShareSetRegistersSameData(id=%d, address=0x%x)", target_id, address);

  return (idShareSetRegisters(target_id, address, num, list, list, list, list));
}

int32_t Blvr4wdModbusIdShare::idShareGetRegisters(int32_t target_id, int32_t address, uint16_t num,
                                                  std::vector<uint16_t> &fl_list, std::vector<uint16_t> &fr_list,
                                                  std::vector<uint16_t> &rl_list, std::vector<uint16_t> &rr_list)
{
  if (target_id != target_id_)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  int32_t ret;
  int32_t reg_num = (num + 1) * BLV_R_SHARE_CTRL_NUM_4WD;
  std::vector<uint16_t> data(reg_num);
  ret = getMultiRegisters(target_id, address, reg_num, data.data());
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareGetRegisters failed(%d)", ret);
    return ret;
  }
  fl_list.clear();
  fr_list.clear();
  rl_list.clear();
  rr_list.clear();
  fl_list.reserve(num);
  fr_list.reserve(num);
  rl_list.reserve(num);
  rr_list.reserve(num);
  std::vector<uint8_t> temp_list =
                  {static_cast<uint8_t>(target_id), BLV_R_MODBUS_FUNC_READ_REGI, static_cast<uint8_t>(reg_num * 2)};
  temp_list.reserve(reg_num + 3);

  // エラーチェック
  uint16_t crc = 0;
  int32_t proc_size = 0;
  int32_t comp_size = 0;

  for (int j = BLV_R_MODBUS_LOCAL_ID_MIN; j <= BLV_R_MODBUS_LOCAL_ID_4WD_MAX; j++)
  {
    // CRC対象データサイズ
    if (j != BLV_R_MODBUS_LOCAL_ID_MIN)
    {
      // スレーブ間用エラーチェック対象データはフレームの先頭からのため、前のCRCと次のスレーブのデータサイズ
      proc_size = num + 1;
    }
    else
    {
      // 先頭スレーブは前のCRCがないため、スレーブのデータサイズ
      proc_size = num;
    }

    // temp_listにデータを追加
    for (int i = 0; i < proc_size; i++)
    {
      temp_list.emplace_back((data[comp_size + i] >> UINT8_WIDTH) & 0xFF);
      temp_list.emplace_back(data[comp_size + i] & 0xFF);
    }
    crc = calcCrc(MODBUS_WRAPPER_CRC_INIT, temp_list);
    crc = ((crc >> UINT8_WIDTH) & 0x00FF) | ((crc << UINT8_WIDTH) & 0xFF00);
    if (data[((num + 1) * j) - 1] != crc)
    {
      ROS_ERROR("idShareGetRegisters first slave data CRC error (0x%x != 0x%x)", crc, data[((num + 1) * j) - 1]);
      ret = RobotDriverErr::INTERNAL_ERR;
    }
    else
    {
      if (j == left_local_id_)
      {
        std::copy(data.begin() + ((num + 1) * (j - 1)),
                  data.begin() + ((num + 1) * j) - 1, std::back_inserter(fl_list));
      }
      else if (j == right_local_id_)
      {
        std::copy(data.begin() + ((num + 1) * (j - 1)),
                  data.begin() + ((num + 1) * j) - 1, std::back_inserter(fr_list));
      }
      else if (j == rear_left_local_id_)
      {
        std::copy(data.begin() + ((num + 1) * (j - 1)),
                  data.begin() + ((num + 1) * j) - 1, std::back_inserter(rl_list));
      }
      else if (j == rear_right_local_id_)
      {
        std::copy(data.begin() + ((num + 1) * (j - 1)),
                  data.begin() + ((num + 1) * j) - 1, std::back_inserter(rr_list));
      }
      else
      {
        // nop
      }
    }
    comp_size += proc_size;
  }

  return ret;
}

};  // namespace robot_driver_fw
