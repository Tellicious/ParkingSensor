/* BEGIN Header */
/**
 ******************************************************************************
 * \file            parkingConfig.h
 * \author          Andrea Vivani
 * \brief           Configuration parameters for parking sensor
 ******************************************************************************
 * \copyright
 *
 * Copyright 2024 Andrea Vivani
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 ******************************************************************************
 */
/* END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARKINGCONFIG_H__
#define __PARKINGCONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Timers configuration ------------------------------------------------------*/
/* Button reading timer interval in ms */
#define configTIMER_BUTTON_MS              100
/* LED control timer interval in ms */
#define configTIMER_LED_MS                 1000
/* LED brightness control timer interval in ms */
#define configTIMER_BRIGHTNESS_MS          50
/* LIDAR reading timer interval in ms */
#define configTIMER_MEASURE_MS             210

/* Measure configuration -----------------------------------------------------*/
/* Distance from targetDistance in mm where LEDs start blinking red */
#define configMIN_DISTANCE_MM              50
/* Distance from targetDistance in mm where LEDs light up in green */
#define configMEASURING_RANGE_MM           1200
/* Distance from targetDistance in mm where LEDs turn off */
#define configMEASURING_STOP_MM            (configMEASURING_RANGE_MM + 100)
/* Minimum approach speed to sensor in mm/s to detect if something is approaching or not */
#define configMIN_APPROACH_SPEED_MM_S      -15
/* Delay in ms to turn on measuring when still */
#define configMEASURING_START_DELAY_MS     500u
/* Delay in ms to turn off measuring when still */
#define configMEASURING_STOP_DELAY_MS      5000u

/* LED strip configuration ---------------------------------------------------*/
/* Number of LEDs in the strip */
#define configLED_NUMBER                   40
/* Number of LEDs processed per IRQ */
#define configLED_PER_IRQ                  30
/* Lenght in ms of waiting time when max or min brightness is reached, before reversing direction */
#define configLED_BRIGHTNESS_WAIT_MS       3000u
/* Delay in ms to save brightness */
#define configLED_BRIGHTNESS_SAVE_DELAY_MS 5000u

/* LIDAR configuration -------------------------------------------------------*/
/* Timing budget in ms. Possible values [15, 20, 33, 50, 100(default), 200, 500] */
#define configLIDAR_TIMING_BUDGET_MS       200
/* Inter-measurement time in ms. IM must be >= TB + 5ms, otherwise TB*2 */
#define configLIDAR_IM_TIME_MS             205

/* Button configuration ------------------------------------------------------*/
/* Button debouncing time in ms */
#define configBUTTON_DEBOUNCING_MS         50
/* Button press reset time in ms */
#define configBUTTON_RESET_MS              400
/* Button long-press time in ms */
#define configBUTTON_LONGPRESS_MS          1000
/* Button very-long-press time in ms */
#define configBUTTON_VERYLONGPRESS_MS      2000

/* MovingAverage configuration -----------------------------------------------*/
/* Number of samples for LIDAR moving average */
#define configMOVAVG_LIDAR_SAMPLES         2
/* Number of samples for approach speed moving average */
#define configMOVAVG_APPSPEED_SAMPLES      3

#ifdef __cplusplus
}
#endif

#endif /* __PARKINGCONFIG_H__ */