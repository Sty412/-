#ifndef __CAN_TX_H
#define __CAN_TX_H

#include <stdint.h>

/* ======================================================================
 * 文件：can_tx.h
 * 用途：CAN 总线状态帧发送
 * 说明：通过 CAN 向上位机汇报报警状态和各传感器数据
 * 波特率：250kbps（在 CubeMX 中配置）
 * 帧 ID：0x100（标准帧）
 * ====================================================================== */

/* CAN 启动 + 过滤器配置（必须在 main() 中 MX_CAN_Init() 之后调用） */
void CAN_Start(void);

/* 打包并发送一帧状态数据（8字节）
 * 说明：每 100ms 由 Sensor_Task() 调用一次 */
void CAN_SendStatus(void);

/* 获取 CAN 发送失败计数（调试用） */
uint32_t CAN_GetTxFailCount(void);

#endif
