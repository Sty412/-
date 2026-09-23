#include "indicator.h"
#include "main.h"
#include "alarm.h"

/* ======================================================================
 * 文件：indicator.c
 * 用途：声光报警指示器驱动
 * 硬件连接（STM32F103C8T6）：
 *   - PC13：板载绿色 LED（低电平亮，高电平灭）
 *           用途：显示整体报警严重程度
 *   - PB0：  蜂鸣器（高电平触发=响，低电平=静音）
 *           用途：声音报警
 *   - PB1：  外接红色 LED（高电平亮，低电平灭）
 *           用途：与蜂鸣器同步闪烁/常亮
 *
 * 【重要】如果更换了蜂鸣器类型或 LED 接线方式，
 *        请修改 indicator.h 中的 BUZZER_HIGH_TRIGGER 宏定义。
 * ====================================================================== */

/* LED 闪烁计时器：记录上次翻转时刻 */
static uint32_t tick_pc13 = 0;   /* PC13 绿灯 */
static uint32_t tick_pb1  = 0;   /* PB1 红灯 */

/**
  * @brief  根据报警等级返回闪烁间隔（单位：毫秒）
  * @param  level 报警等级（0~4）
  * @retval 闪烁间隔，0 表示常亮或常灭
  *
  * @note   闪烁节奏定义：
  *         NONE(0)     -> 0ms      常灭
  *         LEVEL1(1)   -> 1000ms   1Hz 慢闪（周期 2 秒）
  *         LEVEL2(2)   -> 200ms    5Hz 快闪（周期 400ms）
  *         LEVEL3(3)   -> 200ms    同 LEVEL2
  *         EMERGENCY(4)-> 0ms     常亮
  *
  * 【可微调】如果现场觉得闪烁太快/太慢，修改下面的 return 值即可
  */
static uint32_t Level_Interval(uint8_t level)
{
  switch (level) {
    case ALARM_NONE:      return 0;      /* 常灭 */
    case ALARM_LEVEL1:    return 1000;   /* 1Hz 慢闪 */
    case ALARM_LEVEL2:    return 200;    /* 5Hz 快闪 */
    case ALARM_LEVEL3:    return 200;    /* 同快闪 */
    case ALARM_EMERGENCY: return 0;      /* 常亮 */
    default:              return 0;
  }
}

/**
  * @brief  初始化 GPIO 并做上电自检
  * @note   1. 使能 GPIOC 和 GPIOB 时钟
  *         2. 配置 PC13、PB0、PB1 为推挽输出
  *         3. 初始状态：全部熄灭/静音
  *         4. 上电自检：绿灯闪 1 次 -> 蜂鸣器嘀 2 声 -> 红灯闪 2 次
  *            （用于确认硬件连接正常）
  */
void Indicator_Init(void)
{
  /* 使能 GPIO 时钟 */
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN;

  /* ----- PC13 配置：推挽输出，50MHz -----
   * CRH 寄存器控制 PC8~PC15，每 4 位控制一个引脚。
   * PC13 对应 bit[23:20]，配置为 0x3（推挽输出，50MHz） */
  GPIOC->CRH &= ~(0xF << 20);
  GPIOC->CRH |=  (0x3 << 20);
  GPIOC->BSRR = (1U << 13);   /* 高电平 = 灭（PC13 板载灯低电平亮） */

  /* ----- PB0 配置：推挽输出，50MHz（蜂鸣器）----- */
  GPIOB->CRL &= ~(0xF << 0);
  GPIOB->CRL |=  (0x3 << 0);
#ifdef BUZZER_HIGH_TRIGGER
  /* 高电平触发：初始低电平 = 静音 */
  GPIOB->BSRR = (1U << 16);   /* bit16 = PB0 复位（拉低） */
#else
  /* 低电平触发：初始高电平 = 静音 */
  GPIOB->BSRR = (1U << 0);    /* bit0 = PB0 置位（拉高） */
#endif

  /* ----- PB1 配置：推挽输出，50MHz（红色 LED）----- */
  GPIOB->CRL &= ~(0xF << 4);
  GPIOB->CRL |=  (0x3 << 4);
  GPIOB->BSRR = (1U << 17);   /* bit17 = PB1 复位（拉低）= 灭 */

  /* ==================== 上电自检 ====================
   * 目的：确认 LED 和蜂鸣器硬件连接正常
   * 现象：绿灯闪一下 -> 蜂鸣器响两下 -> 红灯闪两下 */

  /* PC13 绿灯闪 1 次 */
  for (uint8_t i = 0; i < 1; i++) {
    GPIOC->BSRR = (1U << 29); HAL_Delay(150);  /* 低电平 = 亮 */
    GPIOC->BSRR = (1U << 13); HAL_Delay(150);  /* 高电平 = 灭 */
  }

  /* PB0 蜂鸣器嘀 2 声 */
  for (uint8_t i = 0; i < 2; i++) {
#ifdef BUZZER_HIGH_TRIGGER
    GPIOB->BSRR = (1U << 0);  HAL_Delay(100);  /* 高电平 = 响 */
    GPIOB->BSRR = (1U << 16); HAL_Delay(100);  /* 低电平 = 停 */
#else
    GPIOB->BSRR = (1U << 16); HAL_Delay(100);  /* 低电平 = 响 */
    GPIOB->BSRR = (1U << 0);  HAL_Delay(100);  /* 高电平 = 停 */
#endif
  }

  /* PB1 红灯闪 2 次 */
  for (uint8_t i = 0; i < 2; i++) {
    GPIOB->BSRR = (1U << 1);  HAL_Delay(150);  /* 高电平 = 亮 */
    GPIOB->BSRR = (1U << 17); HAL_Delay(150);  /* 低电平 = 灭 */
  }
}

/**
  * @brief  更新 PC13 板载绿灯状态
  * @param  level 整体报警等级（0~4）
  * @note   调用周期：50ms（由 main.c 主循环控制）
  *         NONE -> 灭；EMERGENCY -> 常亮；其他 -> 按等级频率闪烁
  */
void LED_PC13_Update(uint8_t level)
{
  uint32_t now = HAL_GetTick();
  static uint8_t on = 0;            /* 当前亮灭状态 */
  uint32_t interval = Level_Interval(level);

  if (level == ALARM_NONE) {
    /* 正常状态：灯灭 */
    GPIOC->BSRR = (1U << 13);   /* 高电平 = 灭 */
    on = 0;
  }
  else if (level == ALARM_EMERGENCY) {
    /* 紧急状态：灯常亮 */
    GPIOC->BSRR = (1U << 29);   /* 低电平 = 亮（BSRR 高 16 位为复位） */
    on = 1;
  }
  else {
    /* 一级/二级：按 interval 周期闪烁 */
    if (now - tick_pc13 >= interval) {
      tick_pc13 = now;
      if (on) {
        GPIOC->BSRR = (1U << 13); on = 0;   /* 灭 */
      } else {
        GPIOC->BSRR = (1U << 29); on = 1;   /* 亮 */
      }
    }
  }
}

/**
  * @brief  更新 PB1 外接红灯状态
  * @param  level 整体报警等级（0~4），与蜂鸣器同步
  * @note   调用周期：50ms
  *         逻辑与 LED_PC13 完全相同，只是控制的 GPIO 不同
  */
void LED_PB1_Update(uint8_t level)
{
  uint32_t now = HAL_GetTick();
  static uint8_t on = 0;
  uint32_t interval = Level_Interval(level);

  if (level == ALARM_NONE) {
    GPIOB->BSRR = (1U << 17);   /* 低电平 = 灭 */
    on = 0;
  }
  else if (level == ALARM_EMERGENCY) {
    GPIOB->BSRR = (1U << 1);    /* 高电平 = 亮 */
    on = 1;
  }
  else {
    if (now - tick_pb1 >= interval) {
      tick_pb1 = now;
      if (on) {
        GPIOB->BSRR = (1U << 17); on = 0;   /* 灭 */
      } else {
        GPIOB->BSRR = (1U << 1);  on = 1;   /* 亮 */
      }
    }
  }
}

/**
  * @brief  更新 PB0 蜂鸣器状态
  * @param  level 整体报警等级（0~4）
  * @note   调用周期：50ms
  *
  * @note   蜂鸣器声音模式：
  *         NONE(0)     -> 静音
  *         LEVEL1(1)   -> 1Hz 间歇鸣（响 500ms，停 500ms）
  *         LEVEL2(2)   -> 5Hz 间歇鸣（响 100ms，停 100ms）
  *         LEVEL3(3)   -> 长鸣
  *         EMERGENCY(4)-> 长鸣
  *
  * 【可微调】如果现场觉得蜂鸣器声音太吵或不够明显，
  *          修改 switch-case 中的 interval 值即可。
  */
void Buzzer_Update(uint8_t level)
{
  uint32_t now = HAL_GetTick();
  static uint32_t buzz_tick = 0;    /* 上次翻转时刻 */
  static uint8_t buzz_on = 0;       /* 当前响/停状态 */
  uint32_t interval = 0;

  /* 根据等级确定间歇鸣的间隔 */
  switch (level) {
    case ALARM_NONE:      interval = 0;    break;   /* 静音 */
    case ALARM_LEVEL1:    interval = 500;  break;   /* 1Hz */
    case ALARM_LEVEL2:    interval = 100;  break;   /* 5Hz */
    case ALARM_LEVEL3:    interval = 0;    break;   /* 长鸣 */
    case ALARM_EMERGENCY: interval = 0;    break;   /* 长鸣 */
    default:              interval = 0;    break;
  }

  if (level == ALARM_NONE) {
    /* 静音 */
#ifdef BUZZER_HIGH_TRIGGER
    GPIOB->BSRR = (1U << 16);   /* 低电平 = 静音 */
#else
    GPIOB->BSRR = (1U << 0);    /* 高电平 = 静音 */
#endif
    buzz_on = 0;
  }
  else if (level == ALARM_EMERGENCY || level == ALARM_LEVEL3) {
    /* 三级/紧急：长鸣 */
#ifdef BUZZER_HIGH_TRIGGER
    GPIOB->BSRR = (1U << 0);    /* 高电平 = 响 */
#else
    GPIOB->BSRR = (1U << 16);   /* 低电平 = 响 */
#endif
    buzz_on = 1;
  }
  else {
    /* 一级/二级：间歇鸣 */
    if (now - buzz_tick >= interval) {
      buzz_tick = now;
      if (buzz_on) {
        /* 正在响 -> 停 */
#ifdef BUZZER_HIGH_TRIGGER
        GPIOB->BSRR = (1U << 16);  /* 低电平 = 停 */
#else
        GPIOB->BSRR = (1U << 0);   /* 高电平 = 停 */
#endif
        buzz_on = 0;
      } else {
        /* 正在停 -> 响 */
#ifdef BUZZER_HIGH_TRIGGER
        GPIOB->BSRR = (1U << 0);   /* 高电平 = 响 */
#else
        GPIOB->BSRR = (1U << 16);  /* 低电平 = 响 */
#endif
        buzz_on = 1;
      }
    }
  }
}
