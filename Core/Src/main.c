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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L1X_api.h"
#include "button.h"
#include "eeprom.h"
#include "math.h"
#include "miniPrintf.h"
#include "movingAvg.h"
#include "parkingConfig.h"
#include "smartLED.h"
#include "timer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATUS_RESET = 0,
    STATUS_RUNNING = 1,
    STATUS_STOP = 2,
} parkingSensorStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Objects */
button_t userBtn;
userTimer_t timerButton, timerBrightness, timerMeasure;
smartLED_t LEDstrip;
VL53L1_Dev_t VL53L1X_dev;
movingAvg_t _distMovAvg, _appSpeedMovAvg;
/* Sensor status */
parkingSensorStatus_t status = STATUS_STOP;
/* Distance measurement */
uint16_t measStopCnt = 0, measStartCnt = 0;
uint16_t actualDistance = 0, targetDistance = 1000, previousDistance = 0;
uint8_t blink = 0;
/* LED brightness control */
int8_t brightnessDir = 2;
uint16_t brightnessSaveCnt = 0, brightnessWaitCnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

    /* Button initialization */
    buttonInit(&userBtn, BUTTON_TYPE_NORMAL, configBUTTON_DEBOUNCING_MS, configBUTTON_RESET_MS, configBUTTON_LONGPRESS_MS, configBUTTON_VERYLONGPRESS_MS);

    /* Timers initialization */
    timerInit(&timerButton, configTIMER_BUTTON_MS);
    timerInit(&timerBrightness, configTIMER_BRIGHTNESS_MS);
    timerInit(&timerMeasure, configTIMER_MEASURE_MS);

    /* Moving average initialization */
    movingAvgInit(&_distMovAvg, configMOVAVG_LIDAR_SAMPLES);
    movingAvgInit(&_appSpeedMovAvg, configMOVAVG_APPSPEED_SAMPLES);

    /* LED Strip initialization */
    LEDstrip.chip = WS2812B;
    LEDstrip.type = SMARTLED_RGB;
    LEDstrip.size = configLED_NUMBER;
    LEDstrip.htim = &htim1;
    LEDstrip.timType = SMARTLED_TIMER_EXTENDED;
    LEDstrip.timChannel = TIM_CHANNEL_3;
    LEDstrip.LEDperIRQ = configLED_PER_IRQ;
    if (smartLED_init(&LEDstrip) != SMARTLED_SUCCESS) {
        for (;;) {}
    }

    /* VL53L1X initialization */
    VL53L1X_dev.I2cHandle = &hi2c1;
    VL53L1X_dev.I2cDevAddr = 0x52;
    uint8_t buf = 0;
    while (buf == 0) {
        VL53L1X_BootState(VL53L1X_dev, &buf);
        HAL_Delay(2);
    }
    VL53L1X_SensorInit(VL53L1X_dev);
    VL53L1X_StartTemperatureUpdate(VL53L1X_dev);                            /* Perform temperature calibration */
    VL53L1X_SetDistanceMode(VL53L1X_dev, 2);                                /* 1=short, 2=long */
    VL53L1X_SetTimingBudgetInMs(VL53L1X_dev, configLIDAR_TIMING_BUDGET_MS); /* in ms possible values [15, 20, 33, 50, 100(default), 200, 500] */
    VL53L1X_SetInterMeasurementInMs(VL53L1X_dev, configLIDAR_IM_TIME_MS);   /* in ms, IM must be >= TB+ 5ms, otherwise IM*2 */
    //VL53L1X_SetOffset(VL53L1X_dev,20); /* offset compensation in mm */
    //VL53L1X_SetXtalk(VL53L1X_dev, &xtalk);
    VL53L1X_SetROI(VL53L1X_dev, 16, 16); /* minimum ROI 4,4 */
    VL53L1X_SetROICenter(VL53L1X_dev, 199);
    //VL53L1X_CalibrateOffset(VL53L1X_dev, 140, &offset); /* may take few second to perform the offset cal*/
    //VL53L1X_CalibrateXtalk(VL53L1X_dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
    VL53L1X_StartRanging(VL53L1X_dev); /* This function has to be called to enable the ranging */
    uint8_t drdy = 0;
    VL53L1X_ClearInterrupt(VL53L1X_dev);

    /* Wait for first data to be ready so to synchronize with reading loop */
    while (!drdy) {
        VL53L1X_CheckForDataReady(VL53L1X_dev, &drdy);
        HAL_Delay(1);
    }
    VL53L1X_ClearInterrupt(VL53L1X_dev); /* clear interrupt has to be called to enable next interrupt*/

    /* Start timers */
    timerStart(&timerButton, HAL_GetTick());
    timerStart(&timerMeasure, HAL_GetTick());

    /* Initialize EEPROM emulation*/
    HAL_FLASH_Unlock();
    if (EEPROM_Init() != EEPROM_SUCCESS) {
        Error_Handler();
    }

    /* Read target distance and brightness values stored in flash memory */
    if (EEPROM_ReadVariable(0, (uint16_t*)&(LEDstrip._brightness))) {
        smartLED_setBrightness(&LEDstrip, 255);
    }
    if (EEPROM_ReadVariable(1, &targetDistance)) {
        targetDistance = 800;
    }

    /* Turn off LEDs */
    HAL_Delay(200);
    smartLED_updateAllRGBColors(&LEDstrip, 0, 0, 0);
    while (smartLED_startTransfer(&LEDstrip) != SMARTLED_SUCCESS) {
        HAL_Delay(10);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        /* Button reading loop */
        if (timerEventExists(&timerButton)) {
            timerClear(&timerButton);
            buttonPressType_t buttonPress = buttonGetPress(&userBtn, HAL_GetTick());
            if (buttonPress == BUTTON_VERYLONG_PRESS) {
                /* Start brightness changing loop */
                if ((brightnessDir % 2) == 0) {
                    /* Stop LIDAR loop */
                    timerStop(&timerMeasure);
                    brightnessSaveCnt = configLED_BRIGHTNESS_SAVE_DELAY_MS / configTIMER_BRIGHTNESS_MS;
                    brightnessWaitCnt = configLED_BRIGHTNESS_WAIT_MS / configTIMER_BRIGHTNESS_MS;
                    brightnessDir >>= 1;
                    smartLED_updateAllRGBColors(&LEDstrip, 0, 255, 0);
                    timerStart(&timerBrightness, HAL_GetTick());
                }
            } else if (buttonPress == BUTTON_RELEASE_PRESS) {
                /* Invert brightness increase / decrease direction */
                brightnessDir = (brightnessDir == 0) ? 2 : (-brightnessDir * 2);
            } else if (buttonPress == BUTTON_DOUBLE_PRESS) {
                /* Save target distance */
                targetDistance = actualDistance;
                EEPROM_WriteVariable(1, targetDistance);
            }
        }

        /* Brightness changing loop */
        if (timerEventExists(&timerBrightness)) {
            timerClear(&timerBrightness);
            if ((brightnessDir % 2) != 0) {
                if (brightnessDir == 1) {
                    smartLED_increaseBrightness(&LEDstrip);
                } else if (brightnessDir == -1) {
                    smartLED_decreaseBrightness(&LEDstrip);
                }
                uint16_t LEDnr = (configLED_NUMBER * LEDstrip._brightness) / 255;
                for (uint16_t ii = 0; ii < configLED_NUMBER; ii++) {
                    smartLED_updateColor(&LEDstrip, ii, SMARTLED_GREEN, (ii <= LEDnr) ? 0xFF : 0);
                    smartLED_updateColor(&LEDstrip, ii, SMARTLED_RED, 0);
                    smartLED_updateColor(&LEDstrip, ii, SMARTLED_BLUE, 0);
                }

                if (((LEDstrip._brightness == 0xFF) || (LEDstrip._brightness == 0))) {
                    /* If brightnessDir is 1 or -1 and brightness is either 0xFF or 0, wait before reversing direction */
                    brightnessDir = 3;
                    if (!brightnessWaitCnt--) {
                        brightnessDir = (LEDstrip._brightness == 0xFF) ? -1 : 1;
                        brightnessWaitCnt = configLED_BRIGHTNESS_WAIT_MS / configTIMER_BRIGHTNESS_MS;
                    }
                }
            } else if ((brightnessDir % 2) == 0) {
                /* If brightnessDir is 2 or -2 run countdown then stop and save value */
                if (!brightnessSaveCnt--) {
                    /* Reset brightness direction to 2, stop timer, save value and turn off LEDs */
                    brightnessDir = 2;
                    timerStop(&timerBrightness);
                    EEPROM_WriteVariable(0, LEDstrip._brightness);
                    smartLED_updateAllRGBColors(&LEDstrip, 0, 0, 0);
                    /* Re-start measurement timer */
                    /* Wait for first data to be ready so to synchronize with reading loop */
                    uint8_t drdy = 0;
                    while (!drdy) {
                        VL53L1X_CheckForDataReady(VL53L1X_dev, &drdy);
                        HAL_Delay(1);
                    }
                    VL53L1X_ClearInterrupt(VL53L1X_dev); /* clear interrupt has to be called to enable next interrupt*/
                    timerStart(&timerMeasure, HAL_GetTick());
                }
            }
#ifdef DEBUG
            miniPrintf("Brightness:%d\n", LEDstrip._brightness);
#endif /* DEBUG */
            while (smartLED_startTransfer(&LEDstrip) != SMARTLED_SUCCESS) {
                HAL_Delay(10);
            }
        }

        /* Measurement loop */
        if (timerEventExists(&timerMeasure)) {
            timerClear(&timerMeasure);
            uint8_t drdy = 0;
            VL53L1X_CheckForDataReady(VL53L1X_dev, &drdy);
            if (drdy) {
                uint8_t rangeStatus;
                uint16_t distance;
                VL53L1X_GetRangeStatus(VL53L1X_dev, &rangeStatus);
                VL53L1X_GetDistance(VL53L1X_dev, &distance);
                //	VL53L1X_GetSignalRate(VL53L1X_dev, &signalRate);
                //	VL53L1X_GetAmbientRate(VL53L1X_dev, &ambientRate);
                VL53L1X_ClearInterrupt(VL53L1X_dev); /* clear interrupt has to be called to enable next interrupt*/

                if (rangeStatus == 0) {
                    previousDistance = actualDistance;
                    actualDistance = (uint16_t)roundf(movingAvgCalc(&_distMovAvg, distance));
                    float appSpeed = movingAvgCalc(&_appSpeedMovAvg, (actualDistance - previousDistance) * 1e3 / configTIMER_MEASURE_MS);
#ifdef DEBUG
                    miniPrintf(">TDist:%d\n>CDist:%d\n", targetDistance, actualDistance);
#endif /* DEBUG */
                    /* Finite state machine */
                    switch (status) {
                        case STATUS_RUNNING:
                            if (actualDistance >= (targetDistance + configMEASURING_STOP_MM)) {
                                status = STATUS_RESET;
                            } else if (appSpeed > configMIN_APPROACH_SPEED_MM_S) {
                                if (!measStopCnt--) {
                                    status = STATUS_STOP;
                                }
                            } else {
                                measStopCnt = configMEASURING_STOP_DELAY_MS / configTIMER_MEASURE_MS;
                            }
                            break;
                        case STATUS_STOP:
                            if (actualDistance >= (targetDistance + configMEASURING_STOP_MM)) {
                                status = STATUS_RESET;
                            } else if (actualDistance <= (targetDistance + configMEASURING_RANGE_MM)) {
                                if (appSpeed <= configMIN_APPROACH_SPEED_MM_S) {
                                    if (!measStartCnt--) {
                                        status = STATUS_RUNNING;
                                    }
                                } else {
                                    measStartCnt = configMEASURING_START_DELAY_MS / configTIMER_MEASURE_MS;
                                }
                            }
                            break;
                        case STATUS_RESET:
                            if (actualDistance <= (targetDistance + configMEASURING_RANGE_MM)) {
                                status = STATUS_RUNNING;
                            }
                            break;
                        default: break;
                    }

                    /* Control LED color */
                    if (status == STATUS_RUNNING) {
                        if (actualDistance <= (targetDistance + configMIN_DISTANCE_MM)) {
                            /* Show solid red light */
                            blink = ~blink;
                            smartLED_updateAllRGBColors(&LEDstrip, blink, 0, 0);
                            smartLED_startTransfer(&LEDstrip);
                        } else {
                            /* Show decreasing green bar */
                            uint16_t LEDnr =
                                (uint16_t)(roundf(configLED_NUMBER * fminf((float)(actualDistance - targetDistance - configMIN_DISTANCE_MM) / (configMEASURING_RANGE_MM - configMIN_DISTANCE_MM), 1)));
                            for (uint16_t ii = 0; ii < configLED_NUMBER; ii++) {
                                smartLED_updateColor(&LEDstrip, ii, SMARTLED_GREEN, (ii <= LEDnr) ? 0xFF : 0);
                                smartLED_updateColor(&LEDstrip, ii, SMARTLED_RED, 0);
                                smartLED_updateColor(&LEDstrip, ii, SMARTLED_BLUE, 0);
                            }
                            while (smartLED_startTransfer(&LEDstrip) != SMARTLED_SUCCESS) {
                                HAL_Delay(10);
                            }
                        }
                    } else {
                        /* Turn off LEDs */
                        smartLED_updateAllRGBColors(&LEDstrip, 0, 0, 0);
                        while (smartLED_startTransfer(&LEDstrip) != SMARTLED_SUCCESS) {
                            HAL_Delay(10);
                        }
                    }
                } else {
#ifdef DEBUG
                    miniPrintf("LIDARstatus:%d\n", rangeStatus);
#endif /* DEBUG */
                    /* Turn off LEDs */
                    smartLED_updateAllRGBColors(&LEDstrip, 0, 0, 0);
                    while (smartLED_startTransfer(&LEDstrip) != SMARTLED_SUCCESS) {
                        HAL_Delay(10);
                    }
                }
            }
        }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void) {
    /* Process user timer(s) */
    timerProcess(&timerButton, HAL_GetTick());
    timerProcess(&timerBrightness, HAL_GetTick());
    timerProcess(&timerMeasure, HAL_GetTick());
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BUTTON_Pin) {
        /* Process buttons */
        buttonEvent(&userBtn, !HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin), HAL_GetTick());
    } else {
        __NOP();
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef* htim) {
    /* Process LED strip */
    if (htim->Instance == htim1.Instance) {
        smartLED_updateTransfer(&LEDstrip, SMARTLED_IRQ_HALFCPLT);
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    /* Process LED strip */
    if (htim->Instance == htim1.Instance) {
        smartLED_updateTransfer(&LEDstrip, SMARTLED_IRQ_FINISHED);
    }
}

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
    while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
