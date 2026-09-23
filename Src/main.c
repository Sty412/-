/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 定日镜清洗车防撞报警系统 - 主程序
  *
  * 系统概述：
  *   - 采集 4 路超声波（RS485/Modbus，地址 01~04，9600bps）
  *   - 采集 1 路倾角传感器（RS232/Modbus，地址 05，115200bps）
  *   - 根据距离和倾角计算三级报警，驱动 LED + 蜂鸣器
  *   - 通过 CAN 总线（250kbps，帧 ID 0x100）向上位机发送状态
  *
  * 主循环架构：
  *   - TIM2 定时中断每 100ms 置位 g_sensor_ready 标志
  *   - 主循环检测到标志后执行 Sensor_Task()，完成 5 个传感器查询
  *   - LED + 蜂鸣器每 50ms 刷新一次，保证闪烁流畅
  *
  * 【重要】如需修改报警阈值、盲区边界、消抖次数等参数，
  *         请去对应头文件中找 "【可微调】" 标注的宏定义。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "us_sensor.h"    /* 超声波驱动 */
#include "tilt_sensor.h"  /* 倾角驱动 */
#include "alarm.h"        /* 报警计算 */
#include "indicator.h"    /* LED + 蜂鸣器 */
#include "can_tx.h"       /* CAN 发送 */
#include "debug.h"        /* 串口调试打印 */
#include "hmi.h"          /* 串口屏显示 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* g_sensor_ready：TIM2 中断置 1，主循环检测到后清 0 并执行采集
 * volatile 修饰：防止编译器优化，确保主循环能从中断中正确读取 */
volatile uint8_t g_sensor_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Sensor_Task(void);      /* 传感器采集任务（100ms 执行一次） */
void Force_USART_Baudrates(void);   /* 覆盖 CubeMX 默认波特率 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  TIM2 周期中断回调函数
  * @note   CubeMX 配置的 TIM2 周期为 100ms（PSC=35999, ARR=199）
  *         每次中断将 g_sensor_ready 置 1，通知主循环执行 Sensor_Task()
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2) {
    g_sensor_ready = 1;
  }
}

/* USER CODE END 0 */

/**
  * @brief  程序入口点
  * @retval int
  */
int main(void)
{
  /* HAL 库初始化（时钟、滴答定时器等） */
  HAL_Init();

  /* 系统时钟配置：72MHz（HSE 8MHz -> PLL x9） */
  SystemClock_Config();

  /* CubeMX 生成的外设初始化 */
  MX_GPIO_Init();
  MX_USART1_UART_Init();   /* 调试串口，PA9/PA10，115200 */
  MX_USART2_UART_Init();   /* RS485 -> 4路超声波，PA2/PA3，CubeMX默认9600 */
  MX_USART3_UART_Init();   /* RS232 -> 倾角传感器，PB10/PB11，CubeMX默认115200 */
  MX_CAN_Init();           /* CAN 外设初始化（PB8/PB9，250kbps） */
  MX_TIM2_Init();          /* TIM2 定时器初始化（100ms 周期） */

  /* USER CODE BEGIN 2 */

  /* 【可微调】覆盖 USART 波特率
   * 说明：CubeMX 生成的是默认配置，这里强制改成实际需要的值
   *       USART2=9600（超声波），USART3=115200（倾角） */
  Force_USART_Baudrates();

  /* 启动 CAN 控制器 + 配置过滤器（必须调用，否则发不出去） */
  CAN_Start();

  /* 【上电延时】等待超声波传感器完成初始化
   * 说明：超声波模块上电后需要几百毫秒才能稳定工作，
   *       如果立即开始查询，可能收到 0 或错误值导致误报警。
   *       这里延时 500ms，让传感器彻底就绪。 */
  HAL_Delay(500);

  /* 初始化 LED 和蜂鸣器 GPIO，并做上电自检闪烁 */
  Indicator_Init();

  /* 启动 TIM2 定时中断（100ms 周期） */
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* ======================================================================
   * 主循环（无限循环，永不退出）
   * ====================================================================== */
  while (1)
  {
    uint32_t tick_now = HAL_GetTick();   /* 获取系统运行毫秒数 */
    static uint32_t tick_led = 0;        /* LED 上次刷新时间 */

    /* ---------- 50ms 刷新 LED + 蜂鸣器 ----------
     * 说明：独立于传感器采集，保证闪烁节奏稳定流畅 */
    if (tick_now - tick_led >= 50) {
      tick_led = tick_now;
      LED_PC13_Update(alarm_level);   /* 板载绿灯：整体报警 */
      LED_PB1_Update(alarm_level);    /* 外接红灯：与蜂鸣器同步 */
      Buzzer_Update(alarm_level);     /* PB0 蜂鸣器：声音报警 */
    }

    /* ---------- 100ms 传感器采集（TIM2 中断标志驱动） ----------
     * 说明：g_sensor_ready 由 TIM2 中断每 100ms 置 1。
     *       如果 Sensor_Task() 执行时间超过 100ms（比如某个传感器超时），
     *       下一帧会延迟到本轮主循环中执行，不会丢失。 */
    if (g_sensor_ready) {
      g_sensor_ready = 0;
      Sensor_Task();
    }

    /* 低功耗轮询：休眠 5ms，避免 CPU 空转 100% */
    HAL_Delay(5);
  }
}

/**
  * @brief  传感器采集与报警计算任务（每 100ms 执行一次）
  * @note   本函数按顺序查询 US1~US4 + 倾角，然后计算报警等级。
  *         正常情况下耗时约 90ms，刚好在 100ms 周期内完成。
  *         最坏情况（全部超时）约 125ms，偶尔超周期。
  */
static void Sensor_Task(void)
{
  /* 接收缓冲区：大小 32 字节，防止异常数据溢出写坏内存 */
  uint8_t rx[SENSOR_RX_BUF_SIZE];
  uint8_t rxlen;    /* 实际接收到的字节数 */
  uint8_t i;

  /* ---- 保存上一帧距离（用于跳变检测）---- */
  for (i = 0; i < 4; i++) {
    us_dist_last[i] = us_dist_mm[i];
  }

  /* ======================================================================
   * US1 查询（地址 01）
   * 逻辑：发送查询 -> 接收应答 -> 解析 -> 失败时补盲区或标记未知
   * ====================================================================== */
  US_ClearErrors();
  rxlen = 0;
  if (US_QueryRaw(US_ADDR1, rx, &rxlen)) {
    /* 收到数据，尝试解析 */
    if (!US_ParseResponse(rx, rxlen, US_ADDR1)) {
      /* 解析失败（CRC 错/格式错） */
      us_no_echo_cnt[0]++;
      if (us_blind_cnt[0] == 0) {
        /* 不在盲区时：如果上一帧 <80mm，补盲区（障碍物可能还在）；否则标记未知 */
        if (us_dist_mm[0] < US_BLIND_MM) us_blind_cnt[0] = BLIND_HOLD_CNT;
        else us_dist_mm[0] = 9999.0f;   /* 9999 = 未知/无效 */
      }
    } else {
      /* 解析成功 */
      us_no_echo_cnt[0] = 0;
    }
  } else {
    /* 超时未收到任何数据 */
    us_no_echo_cnt[0]++;
    if (us_blind_cnt[0] == 0) {
      if (us_dist_mm[0] < US_BLIND_MM) us_blind_cnt[0] = BLIND_HOLD_CNT;
      else us_dist_mm[0] = 9999.0f;
    }
  }

  /* ======================================================================
   * US2 查询（地址 02）- 逻辑与 US1 完全相同
   * ====================================================================== */
  US_ClearErrors();
  rxlen = 0;
  if (US_QueryRaw(US_ADDR2, rx, &rxlen)) {
    if (!US_ParseResponse(rx, rxlen, US_ADDR2)) {
      us_no_echo_cnt[1]++;
      if (us_blind_cnt[1] == 0) {
        if (us_dist_mm[1] < US_BLIND_MM) us_blind_cnt[1] = BLIND_HOLD_CNT;
        else us_dist_mm[1] = 9999.0f;
      }
    } else {
      us_no_echo_cnt[1] = 0;
    }
  } else {
    us_no_echo_cnt[1]++;
    if (us_blind_cnt[1] == 0) {
      if (us_dist_mm[1] < US_BLIND_MM) us_blind_cnt[1] = BLIND_HOLD_CNT;
      else us_dist_mm[1] = 9999.0f;
    }
  }

  /* ======================================================================
   * US3 查询（地址 03）- 逻辑与 US1 完全相同
   * ====================================================================== */
  US_ClearErrors();
  rxlen = 0;
  if (US_QueryRaw(US_ADDR3, rx, &rxlen)) {
    if (!US_ParseResponse(rx, rxlen, US_ADDR3)) {
      us_no_echo_cnt[2]++;
      if (us_blind_cnt[2] == 0) {
        if (us_dist_mm[2] < US_BLIND_MM) us_blind_cnt[2] = BLIND_HOLD_CNT;
        else us_dist_mm[2] = 9999.0f;
      }
    } else {
      us_no_echo_cnt[2] = 0;
    }
  } else {
    us_no_echo_cnt[2]++;
    if (us_blind_cnt[2] == 0) {
      if (us_dist_mm[2] < US_BLIND_MM) us_blind_cnt[2] = BLIND_HOLD_CNT;
      else us_dist_mm[2] = 9999.0f;
    }
  }

  /* ======================================================================
   * US4 查询（地址 04）- 逻辑与 US1 完全相同
   * ====================================================================== */
  US_ClearErrors();
  rxlen = 0;
  if (US_QueryRaw(US_ADDR4, rx, &rxlen)) {
    if (!US_ParseResponse(rx, rxlen, US_ADDR4)) {
      us_no_echo_cnt[3]++;
      if (us_blind_cnt[3] == 0) {
        if (us_dist_mm[3] < US_BLIND_MM) us_blind_cnt[3] = BLIND_HOLD_CNT;
        else us_dist_mm[3] = 9999.0f;
      }
    } else {
      us_no_echo_cnt[3] = 0;
    }
  } else {
    us_no_echo_cnt[3]++;
    if (us_blind_cnt[3] == 0) {
      if (us_dist_mm[3] < US_BLIND_MM) us_blind_cnt[3] = BLIND_HOLD_CNT;
      else us_dist_mm[3] = 9999.0f;
    }
  }

  /* ======================================================================
   * 倾角传感器查询（地址 05）
   * 说明：倾角没有盲区逻辑，查询失败时保持上一帧值（由调用方判断 no_echo_cnt）
   * ====================================================================== */
  Tilt_ClearErrors();
  rxlen = 0;
  if (Tilt_QueryRaw(rx, &rxlen)) {
    Tilt_ParseResponse(rx, rxlen);
  }

  /* ======================================================================
   * 报警计算与状态更新
   * ====================================================================== */
  Calculate_Alarm_Level();   /* 根据传感器数据计算各独立等级 */

  /* 整体报警等级 = max(超声波等级, 倾角等级) */
  uint8_t new_level = (dist_alarm_level > tilt_alarm_level) ? dist_alarm_level : tilt_alarm_level;
  Alarm_StateMachine(new_level);  /* 经过消抖后更新 alarm_level */

  /* ======================================================================
   * CAN 总线发送状态帧
   * ====================================================================== */
  CAN_SendStatus();

  /* ======================================================================
   * 串口调试打印（USART1，USB-TTL）
   * 说明：仅在报警等级变化时打印，或每 5 秒强制打印一次，避免刷屏
   * ====================================================================== */
  Debug_Print();

  /* ======================================================================
   * 串口屏刷新（USART1，大彩 DC48270DN043）
   * 说明：每 300ms 刷新一次屏幕显示。
   *       【重要】外接串口屏时，请拔掉 USB-TTL，避免 USART1 总线冲突。
   * ====================================================================== */
  HMI_Update();
}

/**
  * @brief  系统时钟配置
  * @note   72MHz：HSE 8MHz -> 经 PLL 倍频 9 倍 = 72MHz
  *         APB1 = 36MHz（但 TIM2 时钟会翻倍到 72MHz）
  *         APB2 = 72MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */

/**
  * @brief  强制覆盖 USART 波特率
  * @note   CubeMX 生成的初始化代码可能使用默认波特率，
  *         这里在 MX_USARTx_UART_Init() 之后重新初始化，
  *         确保 USART2=9600（超声波），USART3=115200（倾角）
  */
void Force_USART_Baudrates(void)
{
  huart2.Init.BaudRate = 9600;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

  huart3.Init.BaudRate = 115200;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

/* USER CODE END 4 */

/**
  * @brief  错误处理函数
  * @note   一旦发生致命错误（如外设初始化失败），关闭所有中断并死循环
  *         此时 LED 和蜂鸣器停止刷新，表现为"卡死"
  */
void Error_Handler(void)
{
  __disable_irq();   /* 关闭全局中断 */
  while (1) {}       /* 死循环，等待人工复位 */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
