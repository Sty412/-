#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdint.h>

/* ======================================================================
 * 文件：debug.h
 * 用途：串口调试输出（USART1，USB-TTL，波特率 115200）
 * 说明：用于现场调试观察传感器数据和报警状态
 * ====================================================================== */

/* 十六进制打印（当前已禁用，空函数） */
void Debug_Hex(const char *prefix, uint8_t *data, uint8_t len);

/* 打印当前报警状态一行（格式如下）
 * 输出示例：ALARM=NORMAL | US1=150 US2=120 US3=9999 US4=200 | R=1.50 | D=*0 T= 0 | BLIND=[....] | PC13=OFF BUZ=OFF
 * 说明：仅在报警等级变化时打印，或每 5 秒强制打印一次 */
void Debug_Print(void);

#endif
