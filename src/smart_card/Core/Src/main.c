/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
/* BSP LCD driver */
#include "stm32_adafruit_lcd.h"
/* BSP TS driver */
#include "stm32_adafruit_ts.h"

#include "contact.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void Init();
struct contact_t get_data(uint8_t req);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=============================================================================
/* Setting section (please set the necessary things in this section) */

/* Touchscreen calibrate at starting
   - 0: off (for the touchscreen, use the TS_CINDEX values in stm32_adafruit_ts.h)
   - 1: on  (the touchscreen must be calibrated at startup)
   - 2: on and printf (the touchscreen must be calibrated at startup and printf the cindex values)
   - 3: on and displays the TS_CINDEX values on the screen */
#define TS_CALIBRATE 0

/* If TS_CALIBRATE == 3 -> Text line size */
#define TS_CALIBTEXTSIZE 12

//=============================================================================
#ifdef osCMSIS
#define Delay(t) osDelay(t)
#define GetTime() osKernelSysTick()
#else
#define Delay(t) HAL_Delay(t)
#define GetTime() HAL_GetTick()
#endif

#if TS_CALIBRATE == 0
#define ts_calib()
#elif TS_CALIBRATE > 0

#include "ts.h"

#define CALIBDELAY 500
#define CALIBBOXSIZE 6
#define CALIBBOXPOS 15
#define TOUCHDELAY 50

extern TS_DrvTypeDef* ts_drv;

//-----------------------------------------------------------------------------
void touchcalib_drawBox(int32_t x, int32_t y, uint16_t cl) {
    BSP_LCD_SetTextColor(cl);
    BSP_LCD_DrawRect(x - CALIBBOXSIZE / 2, y - CALIBBOXSIZE / 2, CALIBBOXSIZE, CALIBBOXSIZE);
}

//-----------------------------------------------------------------------------
/* Touchscreen calibration function */
void ts_calib(void) {
    uint16_t tx, ty;
    ts_three_points tc, dc; /* touchscreen and display corrdinates */
#if TS_CALIBRATE == 2
    ts_cindex ci;
#elif TS_CALIBRATE == 3
    ts_cindex ci;
    static char s[16];
#endif

    dc.x0 = 20;
    dc.y0 = 20;
    dc.x1 = BSP_LCD_GetXSize() >> 1;
    dc.x2 = BSP_LCD_GetXSize() - 1 - 20;
    dc.y1 = BSP_LCD_GetYSize() - 1 - 20;
    dc.y2 = BSP_LCD_GetYSize() >> 1;

    touchcalib_drawBox(dc.x0, dc.y0, LCD_COLOR_YELLOW);
    Delay(CALIBDELAY);
    while (!ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);
    ts_drv->GetXY(0, &tx, &ty);
    tc.x0 = tx;
    tc.y0 = ty;

    while (ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);

    touchcalib_drawBox(dc.x0, dc.y0, LCD_COLOR_GRAY);
    touchcalib_drawBox(dc.x1, dc.y1, LCD_COLOR_YELLOW);
    Delay(CALIBDELAY);
    while (!ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);
    ts_drv->GetXY(0, &tx, &ty);
    tc.x1 = tx;
    tc.y1 = ty;
    while (ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);

    touchcalib_drawBox(dc.x1, dc.y1, LCD_COLOR_GRAY);
    touchcalib_drawBox(dc.x2, dc.y2, LCD_COLOR_YELLOW);
    Delay(CALIBDELAY);
    while (!ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);
    ts_drv->GetXY(0, &tx, &ty);
    tc.x2 = tx;
    tc.y2 = ty;
    while (ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);

#if TS_CALIBRATE == 1
    BSP_TS_CalibCalc(&tc, &dc, NULL);
#elif TS_CALIBRATE == 2
    BSP_TS_CalibCalc(&tc, &dc, &ci);
    BSP_TS_SetCindex(&ci);
    printf("\r\n#define  TS_CINDEX            {%d, %d, %d, %d, %d, %d, %d}\r\n", (int)ci[0], (int)ci[1], (int)ci[2],
           (int)ci[3], (int)ci[4], (int)ci[5], (int)ci[6]);
#elif TS_CALIBRATE == 3
    BSP_TS_CalibCalc(&tc, &dc, &ci);
    BSP_TS_SetCindex(&ci);
    BSP_LCD_DisplayStringAt(10, 0, (uint8_t*)"#define TS_CINDEX", LEFT_MODE);
    for (uint32_t i = 0; i < 7; i++) {
        sprintf(s, "%d", (int)ci[i]);
        BSP_LCD_DisplayStringAt(10, (i + 1) * TS_CALIBTEXTSIZE, (uint8_t*)s, LEFT_MODE);
    }
    Delay(CALIBDELAY);
    while (!ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);
    while (ts_drv->DetectTouch(0))
        Delay(TOUCHDELAY);
#endif
    Delay(CALIBDELAY);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
}

#endif


// typedef struct {
//     char* name;
//     char* email;
//     char* phone;
//     char* notes;
// } contact;
//
// typedef enum { my_card, next_card, previous_card, back, reset } request_type;
//
struct contact_t current_contact;

void draw_main_page(uint16_t x_boxsize, uint16_t y_boxsize, uint16_t color) {
    BSP_LCD_SetTextColor(color);
    BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, 0, x_boxsize, y_boxsize);
    BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2, (uint8_t*)"MY INFO",
                            CENTER_MODE);
    BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, y_boxsize, x_boxsize, y_boxsize);
    BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize + y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2,
                            (uint8_t*)"NEXT", CENTER_MODE);
    BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, y_boxsize * 2, y_boxsize, y_boxsize);
    BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize * 2 + y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2,
                            (uint8_t*)"PREVIOUS", CENTER_MODE);
}

void draw_my_page(uint16_t x_boxsize, uint16_t y_boxsize, uint16_t color) {
    BSP_LCD_SetTextColor(color);
    BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, 0, x_boxsize, y_boxsize);
    BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2, (uint8_t*)"BACK",
                            CENTER_MODE);
}

void draw_contact(struct contact_t c, int erase) {
    if (erase == 0) {
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height, (uint8_t*)c.name, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 2, (uint8_t*)c.email, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 3, (uint8_t*)c.phone, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 4, (uint8_t*)c.notes, CENTER_MODE);
    }
    else {
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height, (uint8_t*)c.name, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 2, (uint8_t*)c.email, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 3, (uint8_t*)c.phone, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 4, (uint8_t*)c.notes, CENTER_MODE);
        free(c.name);
        free(c.email);
        free(c.phone);
        free(c.notes);
    }
}

// contact get_data(request_type request) {
//     // Allocate memory for strings (if needed)
//     current_contact.name = (char*)malloc(50 * sizeof(char));
//     current_contact.email = (char*)malloc(50 * sizeof(char));
//     current_contact.phone = (char*)malloc(15 * sizeof(char));
//     current_contact.notes = (char*)malloc(100 * sizeof(char));
//
//     switch (request) {
//     case my_card:
//         snprintf(current_contact.name, 50, "my card");
//         snprintf(current_contact.email, 50, "john.doe@example.com");
//         snprintf(current_contact.phone, 15, "+123456789");
//         snprintf(current_contact.notes, 100, "This is my card.");
//         break;
//
//     case next_card:
//         snprintf(current_contact.name, 50, "next card");
//         snprintf(current_contact.email, 50, "jane.smith@example.com");
//         snprintf(current_contact.phone, 15, "+987654321");
//         snprintf(current_contact.notes, 100, "This is the next card.");
//         break;
//
//     case previous_card:
//         snprintf(current_contact.name, 50, "previous card");
//         snprintf(current_contact.email, 50, "bob.johnson@example.com");
//         snprintf(current_contact.phone, 15, "+1122334455");
//         snprintf(current_contact.notes, 100, "This is the previous card.");
//         break;
//
//     case back:
//         snprintf(current_contact.name, 50, "back card");
//         snprintf(current_contact.email, 50, "john.doe@example.com");
//         snprintf(current_contact.phone, 15, "+123456789");
//         snprintf(current_contact.notes, 100, "This is my card.");
//         break;
//
//     case reset:
//         snprintf(current_contact.name, 50, "reset card");
//         snprintf(current_contact.email, 50, "john.doe@example.com");
//         snprintf(current_contact.phone, 15, "+123456789");
//         snprintf(current_contact.notes, 100, "This is my card.");
//         break;
//
//     default:
//         snprintf(current_contact.name, 50, "Unknown");
//         snprintf(current_contact.email, 50, "unknown@example.com");
//         snprintf(current_contact.phone, 15, "N/A");
//         snprintf(current_contact.notes, 100, "Unknown request type.");
//         break;
//     }
//
//     printf("Contact Set: %s, %s, %s, %s\n", current_contact.name, current_contact.email, current_contact.phone,
//            current_contact.notes);
//     return current_contact;
// }

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
    MX_LPUART1_UART_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */
    Init();

    TS_StateTypeDef ts;
    uint16_t x_boxsize, y_boxsize;
    uint16_t oldcolor, currentcolor;

    BSP_LCD_Init();
    BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
    BSP_LCD_Clear(LCD_COLOR_BLACK);

    ts_calib();

    x_boxsize = BSP_LCD_GetYSize() / 3;
    y_boxsize = BSP_LCD_GetYSize() / 3;
    draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);

    currentcolor = LCD_COLOR_RED;

    enum page { my_page, main_page };
    enum page current_page;
    current_page = main_page;
    int touched = 0;

    while (1) {
        BSP_TS_GetState(&ts);
        if (ts.TouchDetected && touched == 0) {
            switch (current_page) {
            case main_page:
                y_boxsize = BSP_LCD_GetYSize() / 3;
                if (ts.X > BSP_LCD_GetXSize() - x_boxsize) {
                    // touching a button
                    if (ts.Y >= 0 && ts.Y < y_boxsize) {
                        // my page
                        BSP_LCD_DrawPixel(ts.X, ts.Y, LCD_COLOR_BLUE);
                        draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_BLACK);
                        current_page = my_page;
                        draw_my_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);
                        draw_contact(current_contact, 1);
                        draw_contact(get_data(MY_CARD), 0);
                    }
                    else if (ts.Y >= y_boxsize && ts.Y < y_boxsize * 2) {
                        // next
                        BSP_LCD_DrawPixel(ts.X, ts.Y, LCD_COLOR_WHITE);
                        draw_contact(current_contact, 1);
                        draw_contact(get_data(NEXT_CARD), 0);
                    }
                    else if (ts.Y >= y_boxsize * 2 && ts.Y < y_boxsize * 3) {
                        // previous
                        BSP_LCD_DrawPixel(ts.X, ts.Y, LCD_COLOR_GREEN);
                        draw_contact(current_contact, 1);
                        draw_contact(get_data(PREV_CARD), 0);
                    }
                }
                else {
                    BSP_LCD_DrawPixel(ts.X, ts.Y, currentcolor);
                }
                break;
            case my_page:
                if (ts.X > BSP_LCD_GetXSize() - x_boxsize) {
                    if (ts.Y >= 0 && ts.Y < y_boxsize) {
                        // back
                        BSP_LCD_DrawPixel(ts.X, ts.Y, LCD_COLOR_BLUE);
                        draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_BLACK);
                        draw_contact(current_contact, 1);
                        draw_contact(get_data(BACK), 0);
                        current_page = main_page;
                        draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);
                    }
                }
                else {
                    BSP_LCD_DrawPixel(ts.X, ts.Y, currentcolor);
                }
                break;
            }
            touched = 1;
        }
        if (!ts.TouchDetected) {
            touched = 0;
        }
        Delay(1);
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        HAL_Delay(1000);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 16;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00B07CB4;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {

    /* USER CODE BEGIN LPUART1_Init 0 */

    /* USER CODE END LPUART1_Init 0 */

    /* USER CODE BEGIN LPUART1_Init 1 */

    /* USER CODE END LPUART1_Init 1 */
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN LPUART1_Init 2 */

    /* USER CODE END LPUART1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1000;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 31;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 999;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, RFID_SS_Pin | LCD_BL_Pin | RFID_RST_Pin | LED_ERR_Pin | LED_PASS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LCD_RST_Pin | TS_CS_Pin | LCD_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, STM_LED_Pin | LCD_RS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : RFID_SS_Pin LCD_BL_Pin RFID_RST_Pin LED_ERR_Pin
                             LED_PASS_Pin */
    GPIO_InitStruct.Pin = RFID_SS_Pin | LCD_BL_Pin | RFID_RST_Pin | LED_ERR_Pin | LED_PASS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LCD_RST_Pin STM_LED_Pin */
    GPIO_InitStruct.Pin = LCD_RST_Pin | STM_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : IMU_INT_Pin */
    GPIO_InitStruct.Pin = IMU_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : TS_CS_Pin LCD_RS_Pin LCD_CS_Pin */
    GPIO_InitStruct.Pin = TS_CS_Pin | LCD_RS_Pin | LCD_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
