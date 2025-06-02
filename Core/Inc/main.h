/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mini-cli.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#define WRITE_BIT_HIGH 1
#define WRITE_BIT_LOW 0
bool bIs_cell_balanced(uint8_t cell);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef union
{
    struct
    {
        uint16_t udOverDischargeVoltageProtectionValue;    // 0x01
        uint16_t udOverCurrentProtectionValue;             // 0x02
        uint16_t udBatteryCapacity;                        // 0x03
        uint16_t udNumberOfBatteryStrings;                 // 0x04
        uint16_t udOverChargeVoltageProtectionValue;       // 0x05
        uint16_t udOverTemperatureProtectionValue;         // 0x06
        uint16_t udControlChannelState;                    // 0x07
        uint16_t udAutoBalanceState;                       // 0x08
        uint16_t udDischargeCapacityResetFlag;             // 0x09
        uint16_t udOverChargeRecoveryVoltageValue;         // 0x0A
        uint16_t udDischargeProtectionRecoveryVoltageValue;// 0x0B
        uint16_t udChannelDefaultStateAfterPowerOn;        // 0x0C
        uint16_t udHostDisplaySwitch;                      // 0x0D
        uint16_t udHostVoltagePowerOffVoltageValue;        // 0x0E
        uint16_t udHostVoltagePowerOffDelay;               // 0x0F
        uint16_t udCommunicationConnectedFlag;             // 0x10
        uint16_t udBalanceStartVoltageDuringCharging;      // 0x11
        uint16_t udClearTotalDischargeCapacityFlag;        // 0x12
        uint16_t udClearHistoricalLogsFlag;                // 0x13
        uint16_t udManualUsageCapacityAh;                  // 0x14
        uint16_t udAutoResetUseCapacitySwitch;             // 0x15
        uint16_t udPreChargeDelayTime;                     // 0x16
        uint16_t udDifferentialPressureSettingValue;       // 0x17
        uint16_t udLowTempProtectionSettingValue;          // 0x18
        uint16_t udHallSensorType;                         // 0x19
        uint16_t udFanStartTemperature;                    // 0x1A
        uint16_t udPTCHeatingStartTemperature;             // 0x1B
        uint16_t udCANCommunicationStatus;                 // 0x1C
    };
    uint16_t audCommandDataArray[28];  // 0x01 to 0x1C (28 total)
} BMS_COMMAND_RECEPTION;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
