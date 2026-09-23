#ifndef __ALARM_H
#define __ALARM_H

#include <stdint.h>

/* ======================================================================
 * 文件：alarm.h
 * 用途：报警等级计算与状态机消抖
 * 说明：根据 4 路超声波最小距离 + 倾角 Roll 值，计算当前报警等级
 *       并做升级/降级消抖，防止报警抖动
 * ====================================================================== */

/* 报警等级枚举 */
#define ALARM_NONE      0   /* 正常，无报警 */
#define ALARM_LEVEL1    1   /* 一级：100~120mm 或 2~3° */
#define ALARM_LEVEL2    2   /* 二级：80~100mm 或 3~4° */
#define ALARM_LEVEL3    3   /* 三级：预留，当前与 EMERGENCY 同效果 */
#define ALARM_EMERGENCY 4   /* 紧急：<80mm 或 >=4° 或传感器故障 */

/* ======================================================================
 * 倾角报警阈值（单位：度）
 * 【可微调】如果现场发现倾角太敏感/太迟钝，可调整以下阈值
 * ====================================================================== */
#define TILT_NORMAL     2.0f   /* <2°：正常 */
#define TILT_LEVEL1     3.0f   /* 2~3°：一级 */
#define TILT_LEVEL2     4.0f   /* 3~4°：二级 */
/* >=4°：紧急 */

/* 消抖参数 */
#define DEBOUNCE_UP     1   /* 升级立即生效（无需消抖） */
#define DEBOUNCE_DOWN   1   /* 降级立即生效（无需消抖）
                             * 【可微调】如果现场报警频繁跳变，可改为 2 或 3 */

/* ======================================================================
 * 函数声明
 * ====================================================================== */

/* 根据传感器数据计算当前报警等级
 * 说明：取 4 路超声波最小距离算 dist_level，取 |tilt_roll| 算 tilt_level，
 *       整体取 max(dist, tilt)。同时检测跳变和无回波故障。
 *       结果写入 dist_alarm_level、tilt_alarm_level、alarm_level_raw */
void Calculate_Alarm_Level(void);

/* 报警状态机：对 new_level 做升级/降级消抖
 * 说明：升级立即生效；降级需连续 DEBOUNCE_DOWN 次确认才生效
 *       最终输出到全局变量 alarm_level */
void Alarm_StateMachine(uint8_t new_level);

/* ======================================================================
 * 全局变量
 * ====================================================================== */

extern uint8_t alarm_level;       /* 消抖后的最终报警等级（LED/蜂鸣器/CAN 使用） */
extern uint8_t alarm_level_raw;   /* 消抖前的原始等级（调试参考） */
extern uint8_t debounce_cnt;      /* 降级消抖计数器 */
extern uint8_t dist_alarm_level;  /* 超声波独立报警等级（0~4） */
extern uint8_t tilt_alarm_level;  /* 倾角独立报警等级（0~4） */

#endif
