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
 * @file modbus_wrapper.cpp
 * @date 2024/02/07
 * @brief   Modbus通信ラッパークラス
 * @details 
 * @author Panasonic Advanced Technology Development Co.,Ltd.
 */
#include <string>
#include <modbus/modbus.h>
#include "robot_driver_fw/modbus_wrapper.h"
#include "robot_driver_fw/robot_driver.h"

#define ENABLE_MODBUS_DEBUG  (0)

namespace robot_driver_fw
{
ModbusWrapper::ModbusWrapper() :
modbus_ctx_(nullptr)
{
}

ModbusWrapper::~ModbusWrapper()
{
  if (modbus_ctx_ != nullptr)
  {
    (void)fini();
  }
}

int32_t ModbusWrapper::checkParam()
{
  if (tty_dev_path_.empty())
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((baudrate_ != MODBUS_WRAPPER_BAUDRATE_2400) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_4800) &&
      (baudrate_ != MODBUS_WRAPPER_BAUDRATE_9600) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_19200) &&
      (baudrate_ != MODBUS_WRAPPER_BAUDRATE_38400) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_57600) &&
      (baudrate_ != MODBUS_WRAPPER_BAUDRATE_115200) && (baudrate_ != MODBUS_WRAPPER_BAUDRATE_230400))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((parity_ != MODBUS_WRAPPER_PARITY_NONE) && (parity_ != MODBUS_WRAPPER_PARITY_EVEN) &&
      (parity_ != MODBUS_WRAPPER_PARITY_ODD))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  if ((stopbit_ != MODBUS_WRAPPER_STOPBIT_DEFAULT) && (stopbit_ != MODBUS_WRAPPER_STOPBIT_EXT))
  {
    return RobotDriverErr::BAD_PARAM;
  }
  return RobotDriverErr::NOERR;
}

int32_t ModbusWrapper::init(std::string dev_path, int32_t baudrate, std::string parity, int32_t stopbit)
{
  int32_t ret;

  tty_dev_path_ = dev_path;
  baudrate_ = baudrate;
  parity_ = parity;
  stopbit_ = stopbit;
  ret = checkParam();
  if (ret != RobotDriverErr::NOERR)
  {
    ROS_ERROR("invalid serial parameter");
    return RobotDriverErr::BAD_PARAM;
  }

  if (modbus_ctx_ != nullptr)
  {
    ROS_ERROR("already initialized");
    return RobotDriverErr::BAD_STATE;
  }

  int8_t parity_char = std::toupper(parity_[0]);
  modbus_ctx_ = modbus_new_rtu(tty_dev_path_.c_str(),
                               baudrate_,
                               parity_char,
                               MODBUS_WRAPPER_DATA_BIT_LEN,
                               stopbit_);
  if (modbus_ctx_ == nullptr)
  {
    int err = errno;
    if (err == EINVAL)
    {
      ret = RobotDriverErr::BAD_PARAM;
    }
    else
    {
      ret = RobotDriverErr::INTERNAL_ERR;
    }
    ROS_ERROR("modbus_new_rtu failed(%d)", ret);
    return ret;
  }
  else
  {
#if ENABLE_MODBUS_DEBUG
    modbus_set_debug(modbus_ctx_, true);
#endif
    int mret;
    mret = modbus_connect(modbus_ctx_);
    if (mret < 0)
    {
      ret = RobotDriverErr::BUSY;
      ROS_ERROR("modbus_connect failed(%d)", ret);
      fini();
      return ret;
    }
  }
  ROS_INFO("Connect %s complete", tty_dev_path_.c_str());

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = MODBUS_WRAPPER_RESPONSE_TIMEOUT;
#ifdef ROS_NOETIC
  modbus_set_response_timeout(modbus_ctx_, timeout.tv_sec, timeout.tv_usec);
#else
  modbus_set_response_timeout(modbus_ctx_, &timeout);
#endif

  return ret;
}

int32_t ModbusWrapper::fini()
{
  if (modbus_ctx_ != nullptr)
  {
    modbus_close(modbus_ctx_);
    modbus_free(modbus_ctx_);
    modbus_ctx_ = nullptr;
  }

  return RobotDriverErr::NOERR;
}

int32_t ModbusWrapper::setCoil(int32_t target_id, int32_t address, bool state)
{
  int32_t ret;
  ret = modbus_set_slave(modbus_ctx_, target_id);
  if (ret < 0)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  ret = modbus_write_bit(modbus_ctx_, address, state);
  if (ret < 0)
  {
    int err = errno;
    if ((target_id == MODBUS_BROADCAST_ADDRESS) && (err == ETIMEDOUT))
    {
      return RobotDriverErr::NOERR;
    }
    ROS_ERROR("modbus_write_bit failed(%d:%s)", err, modbus_strerror(err));
    return RobotDriverErr::INTERNAL_ERR;
  }
  return RobotDriverErr::NOERR;
}

int32_t ModbusWrapper::getCoil(int32_t target_id, int32_t address, bool *state)
{
  int32_t ret;
  ret = modbus_set_slave(modbus_ctx_, target_id);
  if (ret < 0)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  uint8_t value;
  ret = modbus_read_bits(modbus_ctx_, address, 1, &value);
  if (ret < 0)
  {
    int err = errno;
    if ((target_id == MODBUS_BROADCAST_ADDRESS) && (err == ETIMEDOUT))
    {
      return RobotDriverErr::NOERR;
    }
    ROS_ERROR("modbus_read_bits failed(%d:%s)", err, modbus_strerror(err));
    return RobotDriverErr::INTERNAL_ERR;
  }
  *state = (value == TRUE) ? true : false;
  return RobotDriverErr::NOERR;
}

int32_t ModbusWrapper::setRegister(int32_t target_id, int32_t address, uint16_t value)
{
  int32_t ret;
  ret = modbus_set_slave(modbus_ctx_, target_id);
  if (ret < 0)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  ret = modbus_write_register(modbus_ctx_, address, value);
  if (ret < 0)
  {
    int err = errno;
    if ((target_id == MODBUS_BROADCAST_ADDRESS) && (err == ETIMEDOUT))
    {
      return RobotDriverErr::NOERR;
    }
    ROS_ERROR("modbus_write_register failed(%d:%s)", err, modbus_strerror(err));
    return RobotDriverErr::INTERNAL_ERR;
  }
  return RobotDriverErr::NOERR;
}

int32_t ModbusWrapper::setMultiRegisters(int32_t target_id, int32_t address, uint16_t num, uint16_t *list)
{
  int32_t ret;
  ret = modbus_set_slave(modbus_ctx_, target_id);
  if (ret < 0)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  ret = modbus_write_registers(modbus_ctx_, address, num, list);
  if (ret < 0)
  {
    int err = errno;
    if ((target_id == MODBUS_BROADCAST_ADDRESS) && (err == ETIMEDOUT))
    {
      return RobotDriverErr::NOERR;
    }
    ROS_ERROR("modbus_write_registers failed(%d:%s)", err, modbus_strerror(err));
    return RobotDriverErr::INTERNAL_ERR;
  }
  return RobotDriverErr::NOERR;
}

int32_t ModbusWrapper::getRegister(int32_t target_id, int32_t address, uint16_t *value)
{
  int32_t ret;
  ret = modbus_set_slave(modbus_ctx_, target_id);
  if (ret < 0)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  ret = modbus_read_registers(modbus_ctx_, address, 1, value);
  if (ret != 1)
  {
    int err = errno;
    if ((target_id == MODBUS_BROADCAST_ADDRESS) && (err == ETIMEDOUT))
    {
      return RobotDriverErr::NOERR;
    }
    ROS_ERROR("modbus_read_registers failed(%d:%s)", err, modbus_strerror(err));
    return RobotDriverErr::INTERNAL_ERR;
  }
  return RobotDriverErr::NOERR;
}

int32_t ModbusWrapper::getMultiRegisters(int32_t target_id, int32_t address, uint16_t num, uint16_t *list)
{
  int32_t ret;
  ret = modbus_set_slave(modbus_ctx_, target_id);
  if (ret < 0)
  {
    return RobotDriverErr::BAD_PARAM;
  }
  ret = modbus_read_registers(modbus_ctx_, address, num, list);
  if (ret != num)
  {
    int err = errno;
    if ((target_id == MODBUS_BROADCAST_ADDRESS) && (err == ETIMEDOUT))
    {
      return RobotDriverErr::NOERR;
    }
    ROS_ERROR("modbus_read_registers failed(%d:%s)", err, modbus_strerror(err));
    return RobotDriverErr::INTERNAL_ERR;
  }
  return RobotDriverErr::NOERR;
}

};  // namespace robot_driver_fw
