#include "us_sensor.h"
#include "usart.h"
#include "main.h"
#include "modbus.h"

/* ======================================================================
 * 文件：us_sensor.c
 * 用途：4路超声波传感器（RS485/Modbus RTU）驱动
 * 说明：每个超声波模块通过 USART2 发送 Modbus 查询帧，轮询接收应答。
 *       近距离（<80mm）时传感器数据乱跳，引入"盲区锁定"机制保证报警稳定。
 * ====================================================================== */

/* 4路超声波当前距离（单位：mm）
 * 初始值 9999 表示"尚未读到有效数据"
 * 盲区期间强制为 0，查询失败/超时时也设为 9999 */
float us_dist_mm[4] = {9999, 9999, 9999, 9999};

/* 上一帧距离，用于跳变检测（相邻两帧变化 >200mm 视为异常） */
float us_dist_last[4] = {9999, 9999, 9999, 9999};

/* 无回波计数器：连续查询失败次数
 * >= NO_ECHO_MAX(3) 时认为传感器故障，触发紧急报警 */
uint8_t us_no_echo_cnt[4] = {0, 0, 0, 0};

/* 盲区倒计时：>0 表示该路传感器处于盲区锁定状态
 * 每帧自动递减（除非被刷新为 BLIND_HOLD_CNT），倒计时期间强制紧急报警 */
uint8_t us_blind_cnt[4] = {0, 0, 0, 0};

/* 盲区解除安全确认计数器
 * 盲区期间读到 80~120mm 时，需连续 BLIND_SAFE_EXIT_CNT 帧确认才解除 */
uint8_t us_blind_safe_cnt[4] = {0, 0, 0, 0};

/* 传感器在线标志：1=该路至少成功收到过一帧有效数据
 * 说明：只有在线后，无回波检测才会触发紧急报警。
 *       上电时传感器未通电/未接线，不会误报警。
 *       收到第一帧有效数据后自动置位，软件复位前不会清零。 */
uint8_t us_online[4] = {0, 0, 0, 0};

/* ======================================================================
 * 【上电启动稳定期】
 * 说明：超声波传感器上电后，第一帧可能返回 0 或错误的小值（尚未初始化完成），
 *       如果直接按正常逻辑处理，会误判为"有障碍物"进入盲区，导致上电就报警。
 *       本计数器在上电后前 20 次采集（约 2 秒）内生效：
 *       - 即使读到 <80mm，也不进入盲区，距离强制设为 9999（无效）
 *       - 2 秒后计数器归零，恢复正常报警逻辑
 * 【可微调】如果现场传感器上电后需要更久稳定，可增大到 30 或 50
 * ====================================================================== */
static uint8_t g_us_startup_cnt = 20;

/**
  * @brief  清除 USART2 错误标志并清空接收寄存器
  * @note   每次查询前调用，防止上一次的错误标志/残留数据干扰本次接收
  *         清除的错误类型：奇偶校验错(PE)、帧格式错(FE)、噪声错(NE)、溢出错(ORE)
  */
void US_ClearErrors(void)
{
  __HAL_UART_CLEAR_PEFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  /* 把 DR 寄存器中残留的数据读空 */
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
    (void)(huart2.Instance->DR);
  }
}

/**
  * @brief  向指定超声波发送 Modbus 查询帧，并轮询接收应答
  * @param  addr  传感器地址（1~4）
  * @param  buf   接收数据缓冲区（外部提供，大小至少 SENSOR_RX_BUF_SIZE）
  * @param  len   输出参数，返回实际接收到的字节数
  * @retval 1=成功收到数据；0=超时未收到
  *
  * @note   发送帧格式（8字节）：[Addr][0x04][0x00][0x00][0x00][0x01][CRC_L][CRC_H]
  *         功能码 0x04 = 读输入寄存器，读取 1 个寄存器（距离值）
  *
  * @note   超时参数：
  *         - 总超时：25ms（如果 25ms 内没收到任何数据，认为超时）
  *         - 空闲超时：4ms（收到数据后，如果 4ms 内没新字节，认为帧结束）
  *         正常应答约 7 字节，9600bps 下耗时约 7.3ms，25ms 足够。
  *
  * 【可微调】如果现场电磁干扰严重、RS485 总线质量差，可适当增大总超时到 50ms
  */
uint8_t US_QueryRaw(uint8_t addr, uint8_t *buf, uint8_t *len)
{
  uint8_t frame[8];
  uint16_t crc;
  uint32_t tickstart, last_rx_tick;
  uint8_t idx = 0;

  /* 组装 Modbus 查询帧 */
  frame[0] = addr;        /* 设备地址 */
  frame[1] = 0x04;        /* 功能码：读输入寄存器 */
  frame[2] = 0x00;        /* 寄存器起始地址高字节 */
  frame[3] = 0x00;        /* 寄存器起始地址低字节 */
  frame[4] = 0x00;        /* 读取数量高字节 */
  frame[5] = 0x01;        /* 读取数量低字节（读 1 个寄存器） */
  crc = Modbus_CRC16(frame, 6);
  frame[6] = crc & 0xFF;           /* CRC 低字节在前 */
  frame[7] = (crc >> 8) & 0xFF;    /* CRC 高字节在后 */

  /* 通过 USART2 发送 8 字节，超时 30ms */
  HAL_UART_Transmit(&huart2, frame, 8, 30);

  /* 等待发送完成（TC=Transmission Complete），确保 RS485 方向切换前数据已发出 */
  while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC));

  /* RS485 方向切换延时 3ms：等总线稳定后才开始接收 */
  HAL_Delay(3);

  /* 开始轮询接收 */
  tickstart = HAL_GetTick();
  last_rx_tick = tickstart;

  while ((HAL_GetTick() - tickstart) < 25) {   /* 总超时 25ms */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
      /* RXNE=Receive Data Register Not Empty，表示收到新字节 */
      if (idx < SENSOR_RX_BUF_SIZE) {
        /* 缓冲区未满，存入 buf */
        buf[idx++] = (uint8_t)(huart2.Instance->DR & 0xFF);
        last_rx_tick = HAL_GetTick();   /* 更新最后收到字节的时间 */
      } else {
        /* 缓冲区已满（异常数据过多），丢弃后续字节，防止越界写坏内存 */
        (void)(huart2.Instance->DR);
      }
    }
    /* 如果已经收到过数据，且 4ms 内没有新字节，认为一帧结束，退出 */
    if (idx > 0 && (HAL_GetTick() - last_rx_tick) > 4) break;
  }

  *len = idx;               /* 返回实际接收字节数 */
  return (idx > 0);         /* 只要收到至少 1 字节就认为"有应答" */
}

/**
  * @brief  解析超声波应答帧，更新距离值和盲区状态
  * @param  buf           接收到的数据缓冲区
  * @param  len           接收到的数据长度
  * @param  expected_addr 期望的传感器地址（1~4）
  * @retval 1=解析成功（数据有效或盲区状态正确维护）；0=帧格式/CRC错误
  *
  * @note   应答帧格式（7字节）：[Addr][0x04][0x02][Data_H][Data_L][CRC_L][CRC_H]
  *         距离值 = (Data_H<<8 | Data_L) / 10.0  （单位：mm）
  *
  * @note   盲区逻辑（核心稳定性机制）：
  *         1. 非盲区时读到 <80mm -> 进入盲区，us_dist_mm=0，倒计时拉满
  *         2. 盲区期间读到 <80mm -> 刷新倒计时，保持盲区
  *         3. 盲区期间读到 >=80mm -> 安全确认计数+1，连续3帧才解除
  *            （原因：50mm 时传感器可能跳出 100mm/200mm 假数据，单帧解除会导致报警闪断）
  *         4. 盲区期间坏帧/超时 -> 保持盲区，倒计时不减
  */
uint8_t US_ParseResponse(uint8_t *buf, uint8_t len, uint8_t expected_addr)
{
  /* idx：数组下标（0~3），对应 US1~US4 */
  uint8_t idx = expected_addr - 1;

  /* ==================== 盲区状态处理 ==================== */
  if (us_blind_cnt[idx] > 0) {
    /* 当前处于盲区锁定状态，尝试解析当前帧 */
    if (len >= 7 && buf[0] == expected_addr && buf[1] == 0x04 && buf[2] == 0x02) {
      uint16_t calc_crc = Modbus_CRC16(buf, 5);
      if (buf[5] == (calc_crc & 0xFF) && buf[6] == ((calc_crc >> 8) & 0xFF)) {
        /* CRC 通过，说明传感器确实在线并正确应答 */
        us_online[idx] = 1;

        uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
        float mm = raw / 10.0f;

        if (mm >= US_BLIND_MM) {
          /* 读到 >=80mm：可能是传感器假数据（近距离乱跳），需连续确认才解除
           * 原因：50mm 时传感器可能偶发跳出 100mm、200mm、500mm 等假数据，
           *       如果单帧或 2 帧就解除，蜂鸣器/LED 会闪断一下。
           *       连续 BLIND_SAFE_EXIT_CNT(3) 帧确认后才解除，确保真正安全了。 */
          us_blind_safe_cnt[idx]++;
          if (us_blind_safe_cnt[idx] >= BLIND_SAFE_EXIT_CNT) {
            /* 连续多帧确认安全，真正解除盲区 */
            us_blind_cnt[idx] = 0;
            us_blind_safe_cnt[idx] = 0;
            us_dist_mm[idx] = mm;
            return 1;
          }
          /* 尚未确认够次数，继续保持盲区，距离显示 0 */
          us_dist_mm[idx] = 0.0f;
          return 1;
        } else {
          /* 读到 <80mm：障碍物确实还在近距离，清零安全计数，刷新盲区倒计时 */
          us_blind_safe_cnt[idx] = 0;
          us_blind_cnt[idx] = BLIND_HOLD_CNT;   /* 重新拉满倒计时 */
          us_dist_mm[idx] = 0.0f;
          return 1;
        }
      }
    }
    /* 盲区期间帧格式错/CRC错/超时：保持盲区状态，倒计时不减 */
    us_dist_mm[idx] = 0.0f;
    return 1;
  }

  /* ==================== 非盲区：正常解析 ==================== */
  /* 检查帧长度（至少 7 字节） */
  if (len < 7) return 0;
  /* 检查地址、功能码、数据长度是否匹配 */
  if (buf[0] != expected_addr) return 0;
  if (buf[1] != 0x04) return 0;
  if (buf[2] != 0x02) return 0;

  /* CRC 校验 */
  uint16_t calc_crc = Modbus_CRC16(buf, 5);
  if (buf[5] != (calc_crc & 0xFF)) return 0;
  if (buf[6] != ((calc_crc >> 8) & 0xFF)) return 0;

  /* CRC 通过，传感器在线 */
  us_online[idx] = 1;

  /* 提取距离值（寄存器值 / 10 = 毫米） */
  uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
  float mm = raw / 10.0f;

  /* 【启动稳定期】上电后前 20 次采集（约 2 秒），忽略近距离读数
   * 原因：传感器上电初始化期间可能返回 0 或错误值，导致误报警。
   *       稳定期内将距离设为 9999（无效），不参与报警计算。
   *       每成功解析一帧，计数器减 1，减到 0 后恢复正常逻辑。 */
  if (g_us_startup_cnt > 0) {
    g_us_startup_cnt--;
    us_dist_mm[idx] = 9999.0f;   /* 9999 = 无效/未知，报警计算会忽略 */
    return 1;
  }

  if (mm < US_BLIND_MM) {
    /* 进入盲区：距离强制为 0，启动倒计时 */
    us_dist_mm[idx] = 0.0f;
    us_blind_cnt[idx] = BLIND_HOLD_CNT;
  } else {
    /* 正常距离，直接保存 */
    us_dist_mm[idx] = mm;
  }

  return 1;
}
