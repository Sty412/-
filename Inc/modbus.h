#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdint.h>

/* ======================================================================
 * 文件：modbus.h
 * 用途：Modbus RTU 协议 CRC16 校验计算
 * 说明：超声波和倾角传感器均使用 Modbus RTU 通信，
 *       每帧数据末尾需要附加 2 字节 CRC16 校验码
 * ====================================================================== */

/* 计算 Modbus CRC16 校验值
 * 参数：data - 数据缓冲区；len - 数据长度（不含 CRC 本身）
 * 返回：16位 CRC 值，低字节在前、高字节在后填入帧尾 */
uint16_t Modbus_CRC16(uint8_t *data, uint8_t len);

#endif
