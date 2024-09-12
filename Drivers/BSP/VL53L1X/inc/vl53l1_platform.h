/**
 * @file  vl53l1_platform.h
 * @brief Those platform functions are platform dependent and have to be implemented by the user
 */

#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t I2cDevAddr;
    uint8_t drdy;
    uint8_t rangeStatus;
    uint16_t distance;
    uint16_t signalRate;
    uint16_t ambientRate;
    I2C_HandleTypeDef* I2cHandle;

} VL53L1_Dev_t;

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(VL53L1_Dev_t* dev, uint16_t index, uint8_t* pdata, uint32_t count);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(VL53L1_Dev_t* dev, uint16_t index, uint8_t* pdata, uint32_t count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrByte(VL53L1_Dev_t* dev, uint16_t index, uint8_t data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrWord(VL53L1_Dev_t* dev, uint16_t index, uint16_t data);
/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrDWord(VL53L1_Dev_t* dev, uint16_t index, uint32_t data);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdByte(VL53L1_Dev_t* dev, uint16_t index, uint8_t* pdata);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdWord(VL53L1_Dev_t* dev, uint16_t index, uint16_t* pdata);
/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(VL53L1_Dev_t* dev, uint16_t index, uint32_t* pdata);
/** @brief VL53L1_WaitMs() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WaitMs(VL53L1_Dev_t* pdev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif
