#include "debug.h"
#include "usart.h"
#include "alarm.h"
#include "us_sensor.h"
#include "tilt_sensor.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ======================================================================
 * 文件：debug.c
 * 用途：串口调试输出（USART1，PA9/PA10，USB-TTL，波特率 115200）
 * 说明：用于现场观察传感器数据和报警状态，默认不刷屏。
 *       输出格式：ALARM=等级 | US1~US4距离 | 倾角 | 来源标记 | 盲区标记 | LED蜂鸣器状态
 * ====================================================================== */

/**
  * @brief  十六进制打印（当前已禁用）
  * @note   如果需要调试 Modbus 原始数据，可取消函数体内的 (void) 注释，
  *         改为实际打印逻辑。
  */
void Debug_Hex(const char *prefix, uint8_t *data, uint8_t len)
{
  (void)prefix;
  (void)data;
  (void)len;
  /* 如需调试 Modbus 帧，在这里添加 printf 打印逻辑 */
}

/**
  * @brief  打印当前系统状态（一行）
  * @note   打印策略：
  *         - 报警等级变化时立即打印（方便观察触发和解除时刻）
  *         - 等级不变时，每 50 帧（约 5 秒）强制打印一次（防止误以为死机）
  *
  * @note   输出示例：
  *   ALARM=NORMAL | US1=150 US2=120 US3=9999 US4=200 | R=1.50 | D=*0 T= 0 | BLIND=[....] | PC13=OFF BUZ=OFF
  *
  *   字段说明：
  *   - ALARM：整体报警等级（NORMAL/LEVEL1/LEVEL2/LEVEL3/EMERGENCY）
  *   - US1~US4：4路超声波距离（mm），9999=未知/无效
  *   - R：倾角 Roll（度），带符号
  *   - D=*0：超声波独立等级，* 表示当前整体报警由超声波主导
  *   - T= 0：倾角独立等级，* 表示当前整体报警由倾角主导
  *   - BLIND=[....]：盲区标记，B=处于盲区锁定，.=正常
  *   - PC13：板载灯状态（ON/OFF/BLINK）
  *   - BUZ：蜂鸣器状态（OFF/1Hz/5Hz/ON）
  */
void Debug_Print(void)
{
  char buf[240];           /* 打印缓冲区 */
  int n;
  static uint8_t last_level = 0xFF;   /* 上一次的报警等级（0xFF 表示初始） */
  static uint8_t force_cnt = 0;       /* 强制打印计数器 */

  /* 判断是否需要打印：
   * 条件1：报警等级发生变化 -> 立即打印
   * 条件2：等级未变，但已经累计 50 帧（约 5 秒）-> 强制打印一次 */
  if (alarm_level == last_level && force_cnt < 50) {
    force_cnt++;
    return;   /* 不需要打印，直接返回 */
  }

  /* 更新状态 */
  last_level = alarm_level;
  force_cnt = 0;

  /* 提取距离值（转为整数显示） */
  int d1 = (int)(us_dist_mm[0]);
  int d2 = (int)(us_dist_mm[1]);
  int d3 = (int)(us_dist_mm[2]);
  int d4 = (int)(us_dist_mm[3]);

  /* 提取倾角（放大 100 倍后转为整数，方便格式化） */
  int r = (int)(tilt_roll * 100.0f);

  /* 报警等级字符串 */
  const char *level_str;
  switch (alarm_level) {
    case ALARM_NONE:      level_str = "NORMAL";    break;
    case ALARM_LEVEL1:    level_str = "LEVEL1";    break;
    case ALARM_LEVEL2:    level_str = "LEVEL2";    break;
    case ALARM_LEVEL3:    level_str = "LEVEL3";    break;
    case ALARM_EMERGENCY: level_str = "EMERGENCY"; break;
    default:              level_str = "UNKNOWN";   break;
  }

  /* 来源标记：* 表示该来源是当前主导报警源 */
  const char *d_src = (dist_alarm_level >= tilt_alarm_level) ? "*" : " ";
  const char *t_src = (tilt_alarm_level >= dist_alarm_level) ? "*" : " ";

  /* 盲区标记字符串：B=盲区锁定，.=正常 */
  char blind_mark[5] = {0};
  for (uint8_t i = 0; i < 4; i++) {
    blind_mark[i] = (us_blind_cnt[i] > 0) ? 'B' : '.';
  }

  /* 蜂鸣器状态字符串 */
  const char *buz_state;
  switch (alarm_level) {
    case ALARM_NONE:      buz_state = "OFF";    break;
    case ALARM_LEVEL1:    buz_state = "1Hz";    break;
    case ALARM_LEVEL2:    buz_state = "5Hz";    break;
    case ALARM_LEVEL3:    buz_state = "ON";     break;
    case ALARM_EMERGENCY: buz_state = "ON";     break;
    default:              buz_state = "???";    break;
  }

  /* 板载灯状态字符串 */
  const char *led_state_pc13 = (alarm_level == ALARM_EMERGENCY) ? "ON" :
                               (alarm_level == ALARM_NONE) ? "OFF" : "BLINK";

  /* 格式化输出字符串 */
  n = snprintf(buf, sizeof(buf),
    "ALARM=%s | US1=%d US2=%d US3=%d US4=%d | R=%s%d.%02d | D=%s%d T=%s%d | BLIND=[%s] | PC13=%s BUZ=%s\r\n",
    level_str, d1, d2, d3, d4,
    (r<0?"-":""), (r>=0?r:-r)/100, (r>=0?r:-r)%100,
    d_src, dist_alarm_level, t_src, tilt_alarm_level,
    blind_mark,
    led_state_pc13, buz_state);

  /* 通过 USART1 发送（调试串口，USB-TTL） */
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 500);
}
