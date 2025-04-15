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
 * @file blv_r_modbus_id_share.cpp
 * @date 2024/02/07
 * @brief   Modbus通信拡張(IDシェアモード)クラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>
#include "robot_driver_fw/modbus_wrapper.h"
#include "blv_r_controller/blv_r_modbus_id_share.h"
#include "robot_driver_fw/robot_driver.h"

namespace robot_driver_fw
{
BlvrModbusIdShare::BlvrModbusIdShare() :
target_id_(0),
left_local_id_(0),
right_local_id_(0),
share_ctrl_num_(0)
{
}

BlvrModbusIdShare::~BlvrModbusIdShare()
{
}

int32_t BlvrModbusIdShare::checkParam()
{
  if ((target_id_ < BLV_R_MODBUS_GLOBAL_ID_MIN) || (target_id_ > BLV_R_MODBUS_GLOBAL_ID_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((left_local_id_ < BLV_R_MODBUS_LOCAL_ID_MIN) || (left_local_id_ > BLV_R_MODBUS_LOCAL_ID_2WD_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((right_local_id_ < BLV_R_MODBUS_LOCAL_ID_MIN) || (right_local_id_ > BLV_R_MODBUS_LOCAL_ID_2WD_MAX))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if (left_local_id_ == right_local_id_)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  return RobotDriverErr::NOERR;
}

int32_t BlvrModbusIdShare::configIdShareMode(int32_t global_id, int32_t left_local_id, int32_t right_local_id)
{
  target_id_ = global_id;
  left_local_id_ = left_local_id;
  right_local_id_ = right_local_id;
  share_ctrl_num_ = BLV_R_SHARE_CTRL_NUM_2WD;
  return checkParam();
}

int32_t BlvrModbusIdShare::idShareSetRegisters(int32_t target_id, int32_t address, uint16_t num,
                                               const std::vector<uint16_t> &l_list,
                                               const std::vector<uint16_t> &r_list)
{
  if (target_id != target_id_)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  int32_t ret;
  int32_t reg_num = num * BLV_R_SHARE_CTRL_NUM_2WD;
  std::vector<uint16_t> data;
  if (left_local_id_ < right_local_id_)
  {
    data = l_list;
    data.insert(data.end(), r_list.begin(), r_list.end());
  }
  else
  {
    data = r_list;
    data.insert(data.end(), l_list.begin(), l_list.end());
  }

  ret = setMultiRegisters(target_id, address, reg_num, data.data());
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareSetRegisters failed(%d)", ret);
  }
  return ret;
}

int32_t BlvrModbusIdShare::idShareSetRegistersSameData(int32_t target_id, int32_t address, uint16_t num,
                                                       const std::vector<uint16_t> &list)
{
  ROS_DEBUG("call BlvrModbusIdShare::idShareSetRegistersSameData(id=%d, address=0x%x)", target_id, address);

  return (idShareSetRegisters(target_id, address, num, list, list));
}

uint16_t BlvrModbusIdShare::calcCrc(uint16_t init_crc, const std::vector<uint8_t> &data)
{
  uint16_t crc = init_crc;
  size_t num = data.size();
  for (int i = 0; i < static_cast<int>(num); i++)
  {
    crc = crc ^ data[i];
    for (int j = 0; j < 8; j++)
    {
      if (crc & 0x0001)
      {
        crc = (crc >> 1) ^ MODBUS_WRAPPER_CRC_XOR;
      }
      else
      {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}

int32_t BlvrModbusIdShare::idShareGetRegisters(int32_t target_id, int32_t address, uint16_t num,
                                               std::vector<uint16_t> &l_list, std::vector<uint16_t> &r_list)
{
  if (target_id != target_id_)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  int32_t ret;
  int32_t reg_num = (num + 1) * BLV_R_SHARE_CTRL_NUM_2WD;
  std::vector<uint16_t> data(reg_num);
  ret = getMultiRegisters(target_id, address, reg_num, data.data());
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("idShareGetRegisters failed(%d)", ret);
    return ret;
  }
  l_list.resize(0);
  r_list.resize(0);
  std::vector<uint8_t> temp_list =
                  {static_cast<uint8_t>(target_id), BLV_R_MODBUS_FUNC_READ_REGI, static_cast<uint8_t>(reg_num * 2)};

  // エラーチェック
  uint16_t crc;
  for (int i = 0; i < num; i++)
  {
    temp_list.push_back((data[i] >> UINT8_WIDTH) & 0xFF);
    temp_list.push_back(data[i] & 0xFF);
  }
  crc = calcCrc(MODBUS_WRAPPER_CRC_INIT, temp_list);
  crc = ((crc >> UINT8_WIDTH) & 0x00FF) | ((crc << UINT8_WIDTH) & 0xFF00);
  if (data[num] != crc)
  {
    ROS_ERROR("idShareGetRegisters first slave data CRC error (0x%x != 0x%x)", crc, data[num]);
    ret = RobotDriverErr::INTERNAL_ERR;
  }
  else
  {
    if (left_local_id_ < right_local_id_)
    {
      std::copy(data.begin(), data.begin() + num, std::back_inserter(l_list));
    }
    else
    {
      std::copy(data.begin(), data.begin() + num, std::back_inserter(r_list));
    }
  }
  // スレーブ間用エラーチェック対象データはフレームの先頭からのため、temp_listに前のCRCと次のスレーブのデータを追加
  for (int i = 0; i < num + 1; i++)
  {
    temp_list.push_back((data[num + i] >> UINT8_WIDTH) & 0xFF);
    temp_list.push_back(data[num + i] & 0xFF);
  }
  crc = calcCrc(MODBUS_WRAPPER_CRC_INIT, temp_list);
  crc = ((crc >> UINT8_WIDTH) & 0x00FF) | ((crc << UINT8_WIDTH) & 0xFF00);
  if (data[(num + 1) + num] != crc)
  {
    ROS_ERROR("idShareGetRegisters second slave data CRC error (0x%x != 0x%x)", crc, data[(num + 1) + num]);
    ret = RobotDriverErr::INTERNAL_ERR;
  }
  else
  {
    if (left_local_id_ < right_local_id_)
    {
      std::copy(data.begin() + num + 1, data.end() - 1, std::back_inserter(r_list));
    }
    else
    {
      std::copy(data.begin() + num + 1, data.end() - 1, std::back_inserter(l_list));
    }
  }

  return ret;
}


};  // namespace robot_driver_fw
