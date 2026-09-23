#include "alarm.h"
#include "us_sensor.h"
#include "tilt_sensor.h"
#include <math.h>

/* ======================================================================
 * 文件：alarm.c
 * 用途：报警等级计算与状态机消抖
 * 说明：根据 4 路超声波最小距离和倾角 Roll 绝对值，计算当前报警等级。
 *       为防止报警在边界值附近频繁抖动，引入升级/降级状态机。
 * ====================================================================== */

/* 消抖后的最终报警等级（LED、蜂鸣器、CAN 都使用这个值） */
uint8_t alarm_level = ALARM_NONE;

/* 消抖前的原始等级（仅用于调试参考） */
uint8_t alarm_level_raw = ALARM_NONE;

/* 降级消抖计数器（升级立即生效，不需要此计数器） */
uint8_t debounce_cnt = 0;

/* 超声波独立报警等级（0~4），用于区分报警来源 */
uint8_t dist_alarm_level = ALARM_NONE;

/* 倾角独立报警等级（0~4），用于区分报警来源 */
uint8_t tilt_alarm_level = ALARM_NONE;

/**
  * @brief  根据传感器数据计算当前报警等级
  * @note   计算流程：
  *         1. 取 4 路超声波有效距离的最小值（盲区期间视为 0）
  *         2. 根据最小距离计算 dist_level（0~4）
  *         3. 根据 |tilt_roll| 计算 tilt_level（0~4）
  *         4. 整体等级 = max(dist_level, tilt_level)
  *         5. 额外检测：距离跳变 >200mm 或传感器无回波 -> 强制紧急
  *
  * @note   报警阈值（距离）：
  *         >=120mm -> 正常(0)
  *         100~120mm -> 一级(1)
  *         80~100mm -> 二级(2)
  *         <80mm -> 紧急(4)
  *
  * @note   报警阈值（倾角）：
  *         <2° -> 正常(0)
  *         2~3° -> 一级(1)
  *         3~4° -> 二级(2)
  *         >=4° -> 紧急(4)
  *
  * 【可微调】如果现场测试发现阈值太敏感或太迟钝，
  *          修改 alarm.h 中的 DIST_NORMAL、DIST_LEVEL1、TILT_NORMAL、TILT_LEVEL1、TILT_LEVEL2
  */
void Calculate_Alarm_Level(void)
{
  uint8_t i;
  float min_dist = 99999.0f;   /* 4路中的最小有效距离 */
  uint8_t dist_level = ALARM_NONE;
  uint8_t tilt_level = ALARM_NONE;
  uint8_t level;

  /* ---- 步骤1：找出 4 路超声波的最小有效距离 ----
   * 盲区期间（us_blind_cnt>0）effective_dist = 0，强制参与最小值比较 */
  for (i = 0; i < 4; i++) {
    float effective_dist = (us_blind_cnt[i] > 0) ? 0.0f : us_dist_mm[i];
    if (effective_dist < min_dist) min_dist = effective_dist;
  }

  /* ---- 步骤2：根据最小距离计算超声波报警等级 ---- */
  if (min_dist >= DIST_NORMAL)       dist_level = ALARM_NONE;       /* >=120mm */
  else if (min_dist >= DIST_LEVEL1)  dist_level = ALARM_LEVEL1;    /* 100~120mm */
  else if (min_dist >= 80.0f)        dist_level = ALARM_LEVEL2;    /* 80~100mm */
  else                               dist_level = ALARM_EMERGENCY; /* <80mm 或盲区 */

  /* ---- 步骤3：根据倾角绝对值计算倾角报警等级 ---- */
  float abs_roll = fabsf(tilt_roll);
  if (abs_roll < TILT_NORMAL)       tilt_level = ALARM_NONE;       /* <2° */
  else if (abs_roll < TILT_LEVEL1)  tilt_level = ALARM_LEVEL1;    /* 2~3° */
  else if (abs_roll < TILT_LEVEL2)  tilt_level = ALARM_LEVEL2;    /* 3~4° */
  else                              tilt_level = ALARM_EMERGENCY; /* >=4° */

  /* ---- 步骤4：整体等级取更严重的那个 ---- */
  level = (dist_level > tilt_level) ? dist_level : tilt_level;

  /* ---- 步骤5：跳变检测（异常剧烈变化）----
   * 说明：如果某路传感器相邻两帧距离变化 >200mm，认为数据异常，
   *       强制进入紧急报警。
   *       跳过初始值 9999（上电第一帧）。 */
  for (i = 0; i < 4; i++) {
    if (us_dist_last[i] > 9000.0f) continue;   /* 初始值，跳过 */
    if (fabsf(us_dist_mm[i] - us_dist_last[i]) > DIST_JUMP_MM) {
      level = ALARM_EMERGENCY;
    }
  }

  /* ---- 步骤6：无回波故障检测 ----
   * 说明：某路传感器在线后（us_online=1），连续 NO_ECHO_MAX(3) 次查询失败，
   *       认为传感器断线/故障，强制紧急报警（安全第一）。
   *       传感器未在线时（上电未通电/未接线），不触发此报警，避免误报。 */
  for (i = 0; i < 4; i++) {
    if (us_online[i] && us_no_echo_cnt[i] >= NO_ECHO_MAX) {
      level = ALARM_EMERGENCY;
    }
  }

  /* 保存独立等级（用于 CAN 发送和调试） */
  dist_alarm_level = dist_level;
  tilt_alarm_level = tilt_level;

  /* 如果跳变/无回波导致紧急，但超声波本身没到紧急，
   * 把 dist_alarm_level 也设为紧急（方便排查问题来源） */
  if (level == ALARM_EMERGENCY && dist_level < ALARM_EMERGENCY) {
    dist_alarm_level = ALARM_EMERGENCY;
  }

  /* 保存原始等级（消抖前） */
  alarm_level_raw = level;
}

/**
  * @brief  报警状态机：对 new_level 做升级/降级消抖
  * @param  new_level  由 Calculate_Alarm_Level() 计算出的当前原始等级
  * @note   消抖规则：
  *         - 升级（new_level > alarm_level）：立即生效，不消抖
  *           原因：危险情况要第一时间报警，不能拖。
  *         - 降级（new_level < alarm_level）：需连续 DEBOUNCE_DOWN 次确认
  才生效。
  *           原因：防止在边界值附近频繁跳变，造成 LED/蜂鸣器闪烁骚扰。
  *         - 不变：消抖计数器清零。
  *
  * 【可微调】如果现场报警仍然频繁跳变，可增大 DEBOUNCE_DOWN（改为 2 或 3）
  */
void Alarm_StateMachine(uint8_t new_level)
{
  if (new_level > alarm_level) {
    /* 升级：立即生效 */
    alarm_level = new_level;
    debounce_cnt = 0;
  } else if (new_level < alarm_level) {
    /* 降级：计数器累加，达到阈值才生效 */
    debounce_cnt++;
    if (debounce_cnt >= DEBOUNCE_DOWN) {
      alarm_level = new_level;
      debounce_cnt = 0;
    }
  } else {
    /* 等级不变，消抖计数器清零 */
    debounce_cnt = 0;
  }
}
