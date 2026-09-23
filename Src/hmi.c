/* ======================================================================
 * 文件：hmi.c
 * 用途：串口屏（HMI）显示驱动
 * 硬件：大彩 DC48270DN043 四寸串口屏，接 USART1（PA9/PA10，115200bps）
 *
 * 说明：本文件包含两种输出模式的实现：
 *       1. ASCII 文本模式（默认）：格式化字符串输出，兼容串口助手调试
 *       2. DGUS 二进制模式：发送大彩标准协议帧（需配置 VP 地址）
 *
 * 【重要】USART1 同时用于 debug.c 调试打印和本 HMI 输出。
 *         外接串口屏时务必拔掉 USB-TTL，避免两设备同时驱动 TX 冲突。
 * ====================================================================== */

#include "hmi.h"
#include "us_sensor.h"
#include "tilt_sensor.h"
#include "alarm.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 报警等级文本（对应 alarm_level 0~4）
 * 说明：使用拼音/英文，避免 Keil ARMCC V5 对 UTF-8 中文编码支持问题 */
static const char *alarm_text[] = {
  "ZHENGCHANG",   /* 正常 */
  "YIJI",         /* 一级报警 */
  "ERJI",         /* 二级报警 */
  "SANJI",        /* 三级报警 */
  "JINJI"         /* 紧急报警 */
};

/* 上次刷新时间戳，控制刷新间隔 */
static uint32_t hmi_last_tick = 0;

#ifdef HMI_MODE_DGUS
/* ======================================================================
 * DGUS 二进制模式：发送 0x82 写变量存储器指令
 * 帧格式：[0x5A][0xA5][LEN_H][LEN_L][0x82][VP_H][VP_L][DATA...]
 * ====================================================================== */

/**
  * @brief  发送 DGUS 写 VP 指令（16位无符号整数）
  * @param  vp    变量存储器地址
  * @param  value 要写入的数值
  */
static void HMI_DGUS_WriteVP_U16(uint16_t vp, uint16_t value)
{
  uint8_t frame[9];
  frame[0] = 0x5A;          /* 帧头 */
  frame[1] = 0xA5;
  frame[2] = 0x00;          /* 数据长度高字节 */
  frame[3] = 0x05;          /* 数据长度低字节（指令1 + VP2 + 数据2 = 5） */
  frame[4] = 0x82;          /* 写变量存储器指令 */
  frame[5] = (vp >> 8) & 0xFF;   /* VP 高字节 */
  frame[6] = vp & 0xFF;          /* VP 低字节 */
  frame[7] = (value >> 8) & 0xFF;
  frame[8] = value & 0xFF;
  HAL_UART_Transmit(&huart1, frame, 9, 50);
}

/**
  * @brief  发送 DGUS 写 VP 指令（32位有符号整数，用于倾角 x100）
  * @param  vp    变量存储器地址
  * @param  value 要写入的数值
  */
static void HMI_DGUS_WriteVP_S32(uint16_t vp, int32_t value)
{
  uint8_t frame[11];
  frame[0] = 0x5A;
  frame[1] = 0xA5;
  frame[2] = 0x00;
  frame[3] = 0x07;          /* 指令1 + VP2 + 数据4 = 7 */
  frame[4] = 0x82;
  frame[5] = (vp >> 8) & 0xFF;
  frame[6] = vp & 0xFF;
  frame[7] = (value >> 24) & 0xFF;
  frame[8] = (value >> 16) & 0xFF;
  frame[9] = (value >> 8) & 0xFF;
  frame[10] = value & 0xFF;
  HAL_UART_Transmit(&huart1, frame, 11, 50);
}

#endif /* HMI_MODE_DGUS */

/* ======================================================================
 * ASCII 文本模式：格式化字符串输出
 * 输出格式：US1=150,US2=150,US3=150,US4=150,TILT=0.1,ALARM=正常\r\n
 * ====================================================================== */

/**
  * @brief  格式化距离值为字符串（盲区/故障时显示"--"）
  * @param  dist  距离值（mm）
  * @param  buf   输出缓冲区（至少 8 字节）
  */
static void HMI_FormatDist(float dist, char *buf)
{
  if (dist > 9000.0f) {
    strcpy(buf, "--");      /* 未在线或无效 */
  } else if (dist < 1.0f) {
    strcpy(buf, "0");       /* 盲区期间强制为 0 */
  } else {
    /* 四舍五入到整数毫米 */
    int16_t mm = (int16_t)(dist + 0.5f);
    sprintf(buf, "%d", mm);
  }
}

/**
  * @brief  刷新串口屏显示
  * @note   每 HMI_UPDATE_MS(300ms) 执行一次，避免刷屏过快
  *         在 Sensor_Task 末尾调用即可
  */
void HMI_Update(void)
{
  uint32_t tick_now = HAL_GetTick();

  /* 控制刷新间隔，避免串口拥堵 */
  if ((tick_now - hmi_last_tick) < HMI_UPDATE_MS) return;
  hmi_last_tick = tick_now;

#ifdef HMI_MODE_ASCII

  /* ---------- ASCII 文本模式 ----------
   * 输出示例：US1=150,US2=150,US3=150,US4=150,TILT=0.1,ALARM=正常\r\n
   * 说明：字符串以 \r\n 结尾，方便串口助手或屏幕文本控件解析 */
  char txbuf[128];
  char s1[8], s2[8], s3[8], s4[8];

  HMI_FormatDist(us_dist_mm[0], s1);
  HMI_FormatDist(us_dist_mm[1], s2);
  HMI_FormatDist(us_dist_mm[2], s3);
  HMI_FormatDist(us_dist_mm[3], s4);

  /* 格式化输出：US1=,US2=,US3=,US4=,TILT=,ALARM= */
  sprintf(txbuf, "US1=%s,US2=%s,US3=%s,US4=%s,TILT=%.1f,ALARM=%s\r\n",
          s1, s2, s3, s4,
          tilt_roll,
          alarm_text[alarm_level]);

  HAL_UART_Transmit(&huart1, (uint8_t *)txbuf, strlen(txbuf), 100);

#elif defined(HMI_MODE_DGUS)

  /* ---------- DGUS 二进制模式 ----------
   * 说明：将数据写入屏幕变量存储器（VP），屏幕控件自动刷新显示。
   *       距离单位为 mm，以无符号 16 位整数发送（最大 65535）。
   *       倾角单位为度的 x100（例如 1.23° 发送 123），以 32 位有符号发送。
   *       报警等级发送 0~4。
   *
   * 【注意】VP 地址必须在 DGUS Tool 中与实际控件地址一致！ */

  /* US1~US4：距离（mm），盲区/故障时发送 0xFFFF 表示无效 */
  uint16_t u1 = (us_online[0] && us_dist_mm[0] < 9000.0f) ? (uint16_t)(us_dist_mm[0] + 0.5f) : 0xFFFF;
  uint16_t u2 = (us_online[1] && us_dist_mm[1] < 9000.0f) ? (uint16_t)(us_dist_mm[1] + 0.5f) : 0xFFFF;
  uint16_t u3 = (us_online[2] && us_dist_mm[2] < 9000.0f) ? (uint16_t)(us_dist_mm[2] + 0.5f) : 0xFFFF;
  uint16_t u4 = (us_online[3] && us_dist_mm[3] < 9000.0f) ? (uint16_t)(us_dist_mm[3] + 0.5f) : 0xFFFF;

  HMI_DGUS_WriteVP_U16(HMI_VP_US1, u1);
  HMI_DGUS_WriteVP_U16(HMI_VP_US2, u2);
  HMI_DGUS_WriteVP_U16(HMI_VP_US3, u3);
  HMI_DGUS_WriteVP_U16(HMI_VP_US4, u4);

  /* 倾角：度的 x100，例如 1.23° 发送 123 */
  int32_t tilt_x100 = (int32_t)(tilt_roll * 100.0f);
  HMI_DGUS_WriteVP_S32(HMI_VP_TILT, tilt_x100);

  /* 报警等级：0~4 */
  HMI_DGUS_WriteVP_U16(HMI_VP_ALARM, (uint16_t)alarm_level);

#endif
}
