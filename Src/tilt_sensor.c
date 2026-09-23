#include "tilt_sensor.h"
#include "usart.h"
#include "main.h"
#include "modbus.h"
#include "us_sensor.h"    /* 引用 SENSOR_RX_BUF_SIZE 常量 */

/* ======================================================================
 * 文件：tilt_sensor.c
 * 用途：倾角传感器（SMSA11-v15，RS232/Modbus RTU）驱动
 * 硬件：单轴 Roll，Modbus 地址 05，波特率 115200
 * 说明：倾角传感器不存在"盲区"问题，查询失败时保持上一帧值
 * ====================================================================== */

/* 当前倾角值（单位：度）
 * tilt_roll：Roll 角度，范围约 ±15°（由传感器量程决定）
 * tilt_pitch / tilt_yaw：预留，当前未使用 */
float tilt_roll  = 0;
float tilt_pitch = 0;
float tilt_yaw   = 0;

/**
  * @brief  清除 USART3 错误标志并清空接收寄存器
  * @note   每次查询前调用，防止旧数据干扰
  */
void Tilt_ClearErrors(void)
{
  __HAL_UART_CLEAR_PEFLAG(&huart3);
  __HAL_UART_CLEAR_FEFLAG(&huart3);
  __HAL_UART_CLEAR_NEFLAG(&huart3);
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
    (void)(huart3.Instance->DR);
  }
}

/**
  * @brief  向倾角传感器发送 Modbus 查询帧，并轮询接收应答
  * @param  buf  接收数据缓冲区（外部提供）
  * @param  len  输出参数，返回实际接收到的字节数
  * @retval 1=成功收到数据；0=超时未收到
  *
  * @note   发送帧格式（8字节）：[05][0x04][0x00][0x00][0x00][0x03][CRC_L][CRC_H]
  *         功能码 0x04，读取 3 个寄存器（Roll/Pitch/Yaw，但当前只用 Roll）
  *
  * @note   超时参数：
  *         - 总超时：25ms
  *         - 空闲超时：2ms（115200bps 下帧间隔很短）
  *         正常应答 11 字节，115200bps 下耗时约 0.96ms，25ms 绰绰有余。
  */
uint8_t Tilt_QueryRaw(uint8_t *buf, uint8_t *len)
{
  uint8_t frame[8];
  uint16_t crc;
  uint32_t tickstart, last_rx_tick;
  uint8_t idx = 0;

  /* 组装 Modbus 查询帧 */
  frame[0] = TILT_ADDR;   /* 地址 05 */
  frame[1] = 0x04;        /* 功能码：读输入寄存器 */
  frame[2] = 0x00;
  frame[3] = 0x00;
  frame[4] = 0x00;
  frame[5] = 0x03;        /* 读 3 个寄存器 */
  crc = Modbus_CRC16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  /* 发送（115200bps 很快，超时 30ms 足够） */
  HAL_UART_Transmit(&huart3, frame, 8, 30);

  /* 轮询接收 */
  tickstart = HAL_GetTick();
  last_rx_tick = tickstart;

  while ((HAL_GetTick() - tickstart) < 25) {
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
      if (idx < SENSOR_RX_BUF_SIZE) {
        buf[idx++] = (uint8_t)(huart3.Instance->DR & 0xFF);
        last_rx_tick = HAL_GetTick();
      } else {
        /* 缓冲区溢出保护 */
        (void)(huart3.Instance->DR);
      }
    }
    /* 115200bps 帧间隔短，2ms 空闲即认为帧结束 */
    if (idx > 0 && (HAL_GetTick() - last_rx_tick) > 2) break;
  }

  *len = idx;
  return (idx > 0);
}

/**
  * @brief  解析倾角传感器应答帧，提取 Roll 角度
  * @param  buf  接收到的数据缓冲区
  * @param  len  接收到的数据长度
  * @retval 1=解析成功；0=帧格式/CRC错误
  *
  * @note   应答帧格式（11字节）：
  *         [05][0x04][0x06][Roll_H][Roll_L][Pitch_H][Pitch_L][Yaw_H][Yaw_L][CRC_L][CRC_H]
  *         Roll = (Roll_H<<8 | Roll_L) / 100.0  （单位：度，有符号）
  */
uint8_t Tilt_ParseResponse(uint8_t *buf, uint8_t len)
{
  /* 检查帧长度 */
  if (len < 11) return 0;
  /* 检查地址、功能码、数据字节数 */
  if (buf[0] != TILT_ADDR) return 0;
  if (buf[1] != 0x04) return 0;
  if (buf[2] != 0x06) return 0;   /* 3个寄存器 = 6字节数据 */

  /* CRC 校验（前 9 字节参与 CRC 计算） */
  uint16_t calc_crc = Modbus_CRC16(buf, 9);
  if (buf[9] != (calc_crc & 0xFF)) return 0;
  if (buf[10] != ((calc_crc >> 8) & 0xFF)) return 0;

  /* 提取 Roll（有符号 16 位，除以 100 得到角度） */
  int16_t roll_raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[4]);
  tilt_roll = roll_raw / 100.0f;

  /* Pitch 和 Yaw 当前未使用 */
  tilt_pitch = 0;
  tilt_yaw = 0;

  return 1;
}
