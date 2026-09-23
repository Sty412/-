#include "modbus.h"

/* ======================================================================
 * 文件：modbus.c
 * 用途：Modbus RTU 协议 CRC16 校验计算
 * 说明：超声波和倾角传感器均使用 Modbus RTU 通信。
 *       每帧数据（除 CRC 本身外）需附加 2 字节 CRC16 校验码。
 *       CRC 计算方式：初值 0xFFFF，多项式 0xA001（Modbus 标准）。
 * ====================================================================== */

/**
  * @brief  计算 Modbus CRC16 校验值
  * @param  data  数据缓冲区指针（不含 CRC 本身）
  * @param  len   数据长度（字节数）
  * @retval 16 位 CRC 值
  *
  * @note   算法说明（Modbus RTU 标准）：
  *         1. CRC 初值 = 0xFFFF
  *         2. 对每个字节：CRC ^= data[i]
  *         3. 对 CRC 的最低位判断 8 次：
  *            - 如果最低位为 1：CRC = (CRC >> 1) ^ 0xA001
  *            - 如果最低位为 0：CRC = CRC >> 1
  *         4. 最终 CRC 值：低字节在前、高字节在后填入帧尾
  *
  * @note   示例：查询帧 [01][04][00][00][00][01]
  *         CRC = Modbus_CRC16(frame, 6) -> 0x31CA
  *         帧尾附加 [0xCA][0x31]
  */
uint16_t Modbus_CRC16(uint8_t *data, uint8_t len)
{
  uint16_t crc = 0xFFFF;   /* CRC 初值 */

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];        /* 当前字节与 CRC 异或 */

    for (uint8_t j = 0; j < 8; j++) {
      /* 判断最低位 */
      if (crc & 1) {
        /* 最低位为 1：右移 1 位，再异或 0xA001 */
        crc = (crc >> 1) ^ 0xA001;
      } else {
        /* 最低位为 0：直接右移 1 位 */
        crc = crc >> 1;
      }
    }
  }

  return crc;
}
