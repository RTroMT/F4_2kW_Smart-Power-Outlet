/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hlw8032.h"
#include <stdio.h>      /* snprintf, for the debug line on USART6 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HLW_UART         huart1
#define HLW_RX_SIZE    128u                     /* ~293 ms of data at 4800 8E1        */
#define HLW_FRAME_LEN    HLW8032_FRAME_LENGTH

/* Byte positions inside a frame (datasheet REV 1.5) */
#define HLW_CHECK_BYTE     0x5Au                  /* value of Check REG                 */
#define HLW_IDX_CHECK      1u                     /* Check REG                          */
#define HLW_IDX_SUM_FIRST  2u                     /* checksum covers bytes 2 ...        */
#define HLW_IDX_SUM_LAST   22u                    /* ... to 22                          */
#define HLW_IDX_CHECKSUM   23u                    /* CheckSum REG                       */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/** Set the coefficient to be matched with hardware configuration
 *  Schematic default is 1.88 (V_COEF) and 1.0 (I_COEF)
 */
static const HLW8032_Config_t hlw_cfg = {
  .voltage_coef = 1.92f,
  .current_coef = 0.35
};
static HLW8032_Data_t hlw_data;                   /* latest measurements (read this)    */

/* DMA ring buffer: hardware writes here, we read behind it */
static uint8_t  hlw_ring[HLW_RX_SIZE];
static uint16_t hlw_read_idx;                     /* next byte WE will read             */

/* Sliding window holding the last (up to) 24 bytes */
static uint8_t  hlw_window[HLW_FRAME_LEN];
static uint8_t  hlw_window_len;                   /* how many bytes are in the window   */

static volatile bool hlw_uart_error;              /* set in the ISR, handled in the loop */

/* Debug counters - watch them in the debugger */
static uint32_t hlw_frames_ok;                    /* frames accepted by the parser      */
static uint32_t hlw_frames_rejected;              /* frames the chip flagged as bad     */
static uint32_t hlw_bytes_dropped;                /* bytes skipped while re-aligning    */
static uint32_t hlw_dma_restarts;                 /* DMA restarted after a UART error   */
static uint32_t hlw_bytes_rx;                     /* total bytes received from the HLW8032 */
static volatile uint32_t hlw_uart_errors;         /* number of UART error callbacks     */
static volatile uint32_t hlw_last_error_code;     /* PE=0x01 NE=0x02 FE=0x04 ORE=0x08 DMA=0x10 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_BUILTIN_Init() {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(150);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(200);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(150);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(200);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/* ---- Step 1: where is the DMA right now? -------------------------------- */

/**@brief
 *  Index in hlw_ring where the DMA will write its NEXT byte.
 */
static uint16_t hlw_dma_write_index(void)
{
  /* NDTR = bytes the DMA still has to write before it wraps; counts down from RING_SIZE */
  uint32_t remaining = __HAL_DMA_GET_COUNTER(HLW_UART.hdmarx);

  if (remaining == 0u || remaining >= HLW_RX_SIZE) {
    return 0u;                                    /* just started or just wrapped */
  }
  return (uint16_t)(HLW_RX_SIZE - remaining);
}


/* ---- Step 2: window helpers --------------------------------------------- */

/**@brief
 *  Add one byte to the window. If the window is full, the oldest byte is dropped (slide).
 */
static void hlw_window_push(uint8_t byte)
{
  if (hlw_window_len >= HLW_FRAME_LEN) {
    memmove(hlw_window, hlw_window + 1, HLW_FRAME_LEN - 1u);
    hlw_window_len = (uint8_t)(HLW_FRAME_LEN - 1u);
    hlw_bytes_dropped++;
  }
  hlw_window[hlw_window_len] = byte;
  hlw_window_len++;
}

/* Does the (full) window look like a real frame?  Only header + checksum are tested here;
 * the State REG is judged later by HLW8032_Parse_Data(). */
static bool hlw_window_is_frame(void)
{
  /* 1) Check REG must be 0x5A */
  if (hlw_window[HLW_IDX_CHECK] != HLW_CHECK_BYTE) {
    return false;
  }

  /* 2) Low 8 bits of the sum of bytes 2..22 must equal byte 23 */
  uint8_t sum = 0u;
  for (uint32_t i = HLW_IDX_SUM_FIRST; i <= HLW_IDX_SUM_LAST; i++) {
    sum = (uint8_t)(sum + hlw_window[i]);
  }
  return (sum == hlw_window[HLW_IDX_CHECKSUM]);
}


/* ---- Step 3: one byte in, maybe one frame out --------------------------- */
/**@brief
 * Feed ONE received byte into the frame finder.            [important]
 *
 * Idea: after every byte, ask "do the last 24 bytes form a frame?"
 *   - yes -> parse it and EMPTY the window. The next frame then starts exactly at the
 *            next byte, so we stay aligned without doing anything else.
 *   - no  -> do nothing; the next byte will push the oldest one out (slide by one) and we
 *            ask again. This is how we re-align after power-up in the middle of a frame,
 *            a lost byte or a corrupted byte (we lose at most that one frame).
 *
 * Returns true only if a frame was found AND accepted by HLW8032_Parse_Data().
 */
static bool hlw_feed_byte(uint8_t byte)
{
  /* 1) store the byte */
  hlw_window_push(byte);

  /* 2) window not full yet -> nothing to check */
  if (hlw_window_len < HLW_FRAME_LEN) {
    return false;
  }

  /* 3) full window but not a frame -> keep sliding on the next byte */
  if (!hlw_window_is_frame()) {
    return false;
  }

  /* 4) it is a frame: decode it into hlw_data */
  HLW8032_Status_t status = HLW8032_Parse_Data(hlw_window, HLW_FRAME_LEN, &hlw_data, &hlw_cfg);

  /* 5) these 24 bytes are used up, whatever the parser said */
  hlw_window_len = 0u;

  if (status == HLW8032_OK) {
    hlw_frames_ok++;
    return true;
  }
  hlw_frames_rejected++;                          /* e.g. State REG 0xAA: chip reports an error */
  return false;
}


/* ---- Step 4: drain the ring buffer -------------------------------------- */
/**@brief
 * Read every byte the DMA has added to the ring since the last call.   [important]
 *
 * Two indices walk around the ring:
 *     hlw_read_idx   - where we stopped reading last time (ours)
 *     write index    - where the DMA will write next (hardware)
 * Everything between them is new data. We read it in order and wrap at the end of the array.
 * Only bytes BEFORE the write index are touched, so the DMA writing at the same time
 * cannot give us a half-written byte.
 *
 * Returns true if at least one valid frame was decoded.
 */
static bool hlw_process_new_bytes(void)
{
  bool new_frame = false;

  /* 1) take a snapshot of the DMA position */
  uint16_t write_idx = hlw_dma_write_index();

  /* 2) feed all bytes from our position up to the snapshot */
  while (hlw_read_idx != write_idx) {
    hlw_bytes_rx++;
    if (hlw_feed_byte(hlw_ring[hlw_read_idx])) {
      new_frame = true;
    }

    /* 3) advance, wrapping at the end of the ring */
    hlw_read_idx++;
    if (hlw_read_idx >= HLW_RX_SIZE) {
      hlw_read_idx = 0u;
    }
  }
  return new_frame;
}


/* ---- Step 5: error recovery --------------------------------------------- */
/**@brief
 * Restart the DMA after a UART error.
 *
 * On STM32F4 HAL, a parity / framing / noise / overrun error during DMA reception makes HAL
 * STOP the DMA and call HAL_UART_ErrorCallback(). Without a restart we would never receive
 * another byte, so we re-arm it here.
 */
static void hlw_restart_dma(void)
{
  /* 1) clear the flag first, so a new error during the restart is not lost */
  hlw_uart_error = false;

  /* 2) make sure the DMA is really stopped */
  (void)HAL_UART_DMAStop(&HLW_UART);

  /* 3) the DMA will write from index 0 again, and the partial frame is useless */
  hlw_read_idx   = 0u;
  hlw_window_len = 0u;

  /* 4) start again; if HAL is busy, try again on the next loop pass */
  if (HAL_UART_Receive_DMA(&HLW_UART, hlw_ring, HLW_RX_SIZE) == HAL_OK) {
    hlw_dma_restarts++;
  } else {
    hlw_uart_error = true;
  }
}

/**@brief
 * Start receiving HLW8032 frames. Call once, after MX_USARTx_Init().
 * Returns HAL_OK if the circular DMA reception was started.
 */
static HAL_StatusTypeDef hlw_start(void)
{
  HLW8032_Init(&hlw_data);
  hlw_read_idx   = 0u;
  hlw_window_len = 0u;
  hlw_uart_error = false;

  return HAL_UART_Receive_DMA(&HLW_UART, hlw_ring, HLW_RX_SIZE);
}

/**@brief
 * Call on EVERY pass of the main loop.                      [important]
 *
 *   1) decode the bytes received since the last call   (also the ones that arrived just
 *      before an error, so nothing already received is thrown away)
 *   2) if the UART reported an error, restart the DMA
 *
 * Returns true if at least one new valid frame was decoded; the new values are then in
 * hlw_data (voltage, current, power_active, power_factor, ...).
 *
 * It must be called more often than the ring fills up (~290 ms), so avoid long blocking
 * code in the main loop (e.g. one huge polled SPI transfer to the ILI9341 or HAL_Delay).
 */
static bool hlw_poll(void)
{
  bool new_frame = hlw_process_new_bytes();

  if (hlw_uart_error) {
    hlw_restart_dma();
  }
  return new_frame;
}

/**@brief
 * HAL calls this when the UART detects an error (parity, framing, noise, overrun).
 * Only a flag is set here; the real work is done by hlw_restart_dma() from the main loop,
 * so the interrupt stays short.
 * If another module already defines HAL_UART_ErrorCallback, merge this into it.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == HLW_UART.Instance) {
    hlw_uart_errors++;
    hlw_last_error_code = HAL_UART_GetError(huart);
    hlw_uart_error = true;
  }
}








/* HLW8032 DEBUG BEGIN ------------------------------------------------------ */
/**
 * Debug output: one text line per update, sent on USART6 with interrupt-driven TX.
 *
 * Example line:
 *   V=220.15V I=0.452A P=95.30W S=99.50VA PF=0.96 | upd=VIP ovf=--- | rx=9876 ok=123 rej=0 drop=13 rst=0 err=0(0x00)
 *
 *   upd  = values refreshed by the last frame (V / I / P, '-' = not refreshed)
 *   ovf  = values the chip reported as overflow, i.e. ~0 (V / I / P, '-' = normal)
 *   rx   = bytes received from the HLW8032 (must keep growing, ~436 per second)
 *   ok / rej / drop / rst = frame counters. Healthy link: rej=0, drop small and constant, rst=0.
 *   err  = UART errors seen so far (last error code in hex: PE=01 NE=02 FE=04 ORE=08 DMA=10)
 *
 * The line is printed every HLW_DBG_PERIOD_MS even when NO frame has arrived (heartbeat),
 * so a missing HLW8032 shows up as rx=0 / ok=0 instead of a silent terminal.
 *
 * CubeMX: USART6 115200 8N1 and NVIC "USART6 global interrupt" enabled.
 * Floats are printed as fixed-point integers on purpose: printf("%f") prints nothing on
 * newlib-nano unless "-u _printf_float" is added to the linker flags.
 */
#define DBG_UART           huart6
#define HLW_DBG_PERIOD_MS  500u     /* minimum time between lines; 0 = print every frame */

static char     hlw_dbg_line[200];  /* must stay untouched until the interrupt TX has finished */
static uint32_t hlw_dbg_last_ms;    /* time of the last line that was started */

/* value * scale, rounded, as an integer (the HLW8032 values are never negative) */
static uint32_t hlw_scaled(float value, uint32_t scale)
{
  if (value < 0.0f) {
    value = 0.0f;
  }
  return (uint32_t)(value * (float)scale + 0.5f);
}

/* 'on' -> the letter, otherwise '-' */
static char hlw_flag(bool on, char letter)
{
  return on ? letter : '-';
}

/**
 * One-time test message, sent at start-up through the SAME interrupt-driven path.
 * If it never shows up in the terminal the problem is USART6 itself (pin, baud, wiring,
 * NVIC), not the HLW8032.
 */
static void hlw_debug_hello(void)
{
  static const char msg[] = "\r\nHLW8032 debug on USART6 ready\r\n";
  (void)HAL_UART_Transmit_IT(&DBG_UART, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1u));
}

/**
 * Print the latest HLW8032 data on DBG_UART without blocking.
 * It silently does nothing when it is too early (throttle) or when the previous line
 * is still being sent.
 */
static void hlw_debug_print(const HLW8032_Data_t *m)
{
  /* 1) throttle: not more than one line per HLW_DBG_PERIOD_MS */
  uint32_t now = HAL_GetTick();
  if (HLW_DBG_PERIOD_MS != 0u && (now - hlw_dbg_last_ms) < HLW_DBG_PERIOD_MS) {
    return;
  }

  /* 2) previous line still being sent by the interrupt? Do NOT touch the buffer then */
  if (DBG_UART.gState != HAL_UART_STATE_READY) {
    return;
  }

  /* 3) floats -> fixed-point integers */
  uint32_t v  = hlw_scaled(m->voltage,         100u);   /* V,  2 decimals */
  uint32_t i  = hlw_scaled(m->current,        1000u);   /* A,  3 decimals */
  uint32_t p  = hlw_scaled(m->power_active,    100u);   /* W,  2 decimals */
  uint32_t s  = hlw_scaled(m->power_apparent,  100u);   /* VA, 2 decimals */
  uint32_t pf = hlw_scaled(m->power_factor,    100u);   /*     2 decimals */

  /* 4) build the text line */
  int len = snprintf(hlw_dbg_line, sizeof(hlw_dbg_line),
                     "V=%lu.%02luV I=%lu.%03luA P=%lu.%02luW S=%lu.%02luVA PF=%lu.%02lu"
                     " | upd=%c%c%c ovf=%c%c%c"
                     " | rx=%lu ok=%lu rej=%lu drop=%lu rst=%lu err=%lu(0x%02lX)\r\n",
                     (unsigned long)(v / 100u),   (unsigned long)(v % 100u),
                     (unsigned long)(i / 1000u),  (unsigned long)(i % 1000u),
                     (unsigned long)(p / 100u),   (unsigned long)(p % 100u),
                     (unsigned long)(s / 100u),   (unsigned long)(s % 100u),
                     (unsigned long)(pf / 100u),  (unsigned long)(pf % 100u),
                     hlw_flag(m->voltage_updated, 'V'),
                     hlw_flag(m->current_updated, 'I'),
                     hlw_flag(m->power_updated,   'P'),
                     hlw_flag(m->voltage_overflow, 'V'),
                     hlw_flag(m->current_overflow, 'I'),
                     hlw_flag(m->power_overflow,   'P'),
                     (unsigned long)hlw_bytes_rx,
                     (unsigned long)hlw_frames_ok,
                     (unsigned long)hlw_frames_rejected,
                     (unsigned long)hlw_bytes_dropped,
                     (unsigned long)hlw_dma_restarts,
                     (unsigned long)hlw_uart_errors,
                     (unsigned long)hlw_last_error_code);
  if (len <= 0) {
    return;
  }
  if ((size_t)len >= sizeof(hlw_dbg_line)) {
    len = (int)sizeof(hlw_dbg_line) - 1;                 /* line was truncated */
  }

  /* 5) start the interrupt transmission; it returns immediately */
  if (HAL_UART_Transmit_IT(&DBG_UART, (uint8_t *)hlw_dbg_line, (uint16_t)len) == HAL_OK) {
    hlw_dbg_last_ms = now;
  }
}
/* HLW8032 DEBUG END -------------------------------------------------------- */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /**@brief
   *  LED INIT BY TOGGLE
   *  500ms DELAY
  **/
  LED_BUILTIN_Init();

  /* Start receiving HLW8032 frames (UART + circular DMA) */
  if (hlw_start() != HAL_OK) {
    Error_Handler();
  }

  /* Test line on USART6: proves the debug path works before any HLW8032 data arrives */
  hlw_debug_hello();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1) decode whatever the HLW8032 sent since the last pass */
    if (hlw_poll()) {
      /* A new valid frame was decoded: hlw_data holds the latest measurements.
         Later: update the display, relay logic, ... */
    }

    /* 2) debug heartbeat: prints every HLW_DBG_PERIOD_MS even if no frame has arrived,
          so a silent HLW8032 is visible as rx=0 / ok=0 instead of a silent terminal */
    hlw_debug_print(&hlw_data);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */