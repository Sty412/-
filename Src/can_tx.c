#include "can_tx.h"
#include "can.h"
#include "alarm.h"
#include "us_sensor.h"
#include "tilt_sensor.h"

/* ======================================================================
 * 文件：can_tx.c
 * 用途：CAN 总线状态帧发送
 * 硬件：STM32F103 PB8/PB9 -> TJA1050 收发器 -> CAN 总线
 * 波特率：250kbps（在 CubeMX 中配置）
 * 帧格式：标准帧，ID = 0x100，数据长度 8 字节
 *
 * 【重要】MX_CAN_Init() 只初始化外设，不启动控制器。
 *         必须在 main() 中调用 CAN_Start() 后才能正常发送。
 * ====================================================================== */

#define CAN_TX_ID_STATUS  0x100   /* 状态帧的标准帧 ID */

#define CAN_DIST_INVALID  0xFF    /* 距离无效标记（Byte3~6 用） */
#define CAN_DIST_MAX_CM   254     /* 最大有效距离 = 2540mm，超过饱和到此值 */

/* CAN 外设句柄（由 CubeMX 生成，在 can.c 中定义） */
extern CAN_HandleTypeDef hcan;

/* CAN 发送失败计数器（调试用，可通过串口打印排查总线问题） */
static uint32_t can_tx_fail_cnt = 0;

/**
  * @brief  启动 CAN 控制器并配置过滤器
  * @note   【必须调用】MX_CAN_Init() 之后、CAN_SendStatus() 之前调用。
  *         本系统只发送不接收，过滤器配置为"全部通过"模式。
  *         如果忘记调用，HAL_CAN_AddTxMessage 会返回错误，外部主机收不到数据。
  */
void CAN_Start(void)
{
  CAN_FilterTypeDef sFilterConfig;

  /* 配置过滤器：32 位掩码模式，全部通过（本系统只发不收） */
  sFilterConfig.FilterBank = 0;                    /* 过滤器组 0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;/* 掩码模式 */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;             /* ID = 0 */
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;         /* 掩码 = 0，全部通过 */
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;         /* 启用过滤器 */
  sFilterConfig.SlaveStartFilterBank = 14;         /* 单 CAN 外设时固定 14 */

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();   /* 过滤器配置失败，进入死循环 */
  }

  /* 启动 CAN 控制器（进入 Normal 模式，可以收发） */
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();   /* 启动失败，进入死循环 */
  }
}

/**
  * @brief  将距离值打包为 CAN 字节（含饱和和无效标记）
  * @param  dist_mm      距离值（单位：mm）
  * @param  no_echo_cnt  无回波计数器
  * @retval CAN 字节值：0~254=距离(cm)，0xFF=无效/故障
  *
  * @note   无效条件（满足任一即返回 0xFF）：
  *         - dist_mm > 9000（初始值/未知）
  *         - dist_mm < 0（异常）
  *         - no_echo_cnt >= NO_ECHO_MAX（连续 3 次查询失败）
  *
  * @note   正常距离 > 2540mm 时，饱和到 254（0xFE），不会溢出。
  */
static uint8_t Pack_Dist_Cm(float dist_mm, uint8_t no_echo_cnt)
{
  /* 检查无效条件 */
  if (dist_mm > 9000.0f || dist_mm < 0.0f || no_echo_cnt >= NO_ECHO_MAX) {
    return CAN_DIST_INVALID;   /* 0xFF */
  }

  /* 距离转厘米 */
  uint16_t cm = (uint16_t)(dist_mm / 10.0f);

  /* 饱和处理：超过 254cm 时截断到 254，避免溢出 */
  if (cm > CAN_DIST_MAX_CM) {
    cm = CAN_DIST_MAX_CM;
  }

  return (uint8_t)cm;
}

/**
  * @brief  打包并发送一帧 CAN 状态数据
  * @note   调用周期：100ms（与传感器采集同步）
  *
  * @note   CAN 数据帧格式（8字节）：
  *         Byte0: [bit7:4] US1~US4 故障标志 | [bit3:0] 整体报警等级
  *                bit7=US1故障, bit6=US2, bit5=US3, bit4=US4
  *         Byte1: 超声波独立报警等级（0~4）
  *         Byte2: 倾角独立报警等级（0~4）
  *         Byte3: US1 距离（0~254=cm, 0xFF=无效）
  *         Byte4: US2 距离（同上）
  *         Byte5: US3 距离（同上）
  *         Byte6: US4 距离（同上）
  *         Byte7: 倾角 Roll + 128（-128~+127° 映射到 0~255）
  *
  * 【可微调】如果上位机需要其他数据格式，修改 TxData[x] 的赋值即可。
  */
void CAN_SendStatus(void)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;
  HAL_StatusTypeDef status;

  /* ---- 计算故障标志位 ----
   * bit7=US1故障, bit6=US2, bit5=US3, bit4=US4 */
  uint8_t fault_flags = 0;
  if (us_dist_mm[0] > 9000.0f || us_no_echo_cnt[0] >= NO_ECHO_MAX) fault_flags |= 0x80;
  if (us_dist_mm[1] > 9000.0f || us_no_echo_cnt[1] >= NO_ECHO_MAX) fault_flags |= 0x40;
  if (us_dist_mm[2] > 9000.0f || us_no_echo_cnt[2] >= NO_ECHO_MAX) fault_flags |= 0x20;
  if (us_dist_mm[3] > 9000.0f || us_no_echo_cnt[3] >= NO_ECHO_MAX) fault_flags |= 0x10;

  /* ---- 打包 8 字节数据 ---- */
  TxData[0] = fault_flags | (alarm_level & 0x0F);   /* 高4bit故障标志 + 低4bit等级 */
  TxData[1] = dist_alarm_level;                     /* 超声波独立等级 */
  TxData[2] = tilt_alarm_level;                     /* 倾角独立等级 */
  TxData[3] = Pack_Dist_Cm(us_dist_mm[0], us_no_echo_cnt[0]);  /* US1 距离 */
  TxData[4] = Pack_Dist_Cm(us_dist_mm[1], us_no_echo_cnt[1]);  /* US2 距离 */
  TxData[5] = Pack_Dist_Cm(us_dist_mm[2], us_no_echo_cnt[2]);  /* US3 距离 */
  TxData[6] = Pack_Dist_Cm(us_dist_mm[3], us_no_echo_cnt[3]);  /* US4 距离 */
  TxData[7] = (uint8_t)(tilt_roll + 128.0f);        /* 倾角 Roll 偏移映射 */

  /* ---- 配置 CAN 帧头 ---- */
  TxHeader.StdId = CAN_TX_ID_STATUS;     /* 标准帧 ID = 0x100 */
  TxHeader.ExtId = 0;                    /* 不使用扩展 ID */
  TxHeader.IDE = CAN_ID_STD;             /* 标准帧 */
  TxHeader.RTR = CAN_RTR_DATA;           /* 数据帧（不是远程帧） */
  TxHeader.DLC = 8;                      /* 数据长度 8 字节 */
  TxHeader.TransmitGlobalTime = DISABLE; /* 不发送时间戳 */

  /* ---- 发送 ---- */
  status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
  if (status != HAL_OK) {
    /* 发送失败（如邮箱已满、控制器未启动等），计数器累加 */
    can_tx_fail_cnt++;
  }

//	uint32_t err = HAL_CAN_GetError(&hcan);
//	  if (err != HAL_CAN_ERROR_NONE) {
//		/* 只在错误变化时打印，避免刷屏（需要定义静态变量保存上次错误码） */
//		static uint32_t last_err = 0;
//		if (err != last_err) {
//		  last_err = err;
//		  printf("CAN Error: 0x%08lX\r\n", err);
//		}
//}
}
/**
  * @brief  获取 CAN 发送失败计数
  * @retval 累计发送失败次数
  * @note   调试用，可通过串口打印排查 CAN 总线问题。
  *         如果持续增加，检查：1.CAN 线是否接反；2.终端电阻；3.波特率是否匹配
  */
uint32_t CAN_GetTxFailCount(void)
{
  return can_tx_fail_cnt;
}
