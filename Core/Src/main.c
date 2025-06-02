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
#include "lwip.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/tcp.h"
void handle_extended(uint8_t header, uint16_t value, char *resp, int *len);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NUM_MODULES 16
#define NUM_CELLS   192
#define TRACE(...) trace_printf(__VA_ARGS__)
#define MODULES_NO 16
#define GROUPS_PER_MODULE      4
#define NUMBER_OF_BYTES_PER_GROUP 8
#define NUMBER_OF_CELLS_PER_GROUP 3
#define CELLS_PER_MODULE 12
#define NUMBER_OF_CELLS 192
#define NUMBER_OF_GROUPS 85
#define BITMASK_WORDS   ((NUM_CELLS + 63) / 64)  // = 3
static uint64_t ulBalancedCellsMask[BITMASK_WORDS] = {0};
#define WORD_IDX(cell)   ((cell) / 64)
#define BIT_POS(cell)    ((cell) % 64)
#define LIMIT_TRIGGERED 1
#define LIMIT_NOT_TRIGGERED 0
#define INITAIL_DISCHARGE_PROTECTION 2.70f
#define INITAIL_CHARGE_PROTECTION 3.66f
#define TX_BUFFER_SIZE 1500
uint32_t sensor_value = 0;
static struct tcp_pcb *active_tpcb = NULL;
////////////////////////////////////////////////////////////////
#define FLASH_STORAGE_ADDR  0x081E0000  // Bank 2, Sector 7 start address
#define SECTOR_SIZE         131072      // 128KB
#define PAGE_SIZE           64          // 64 bytes per page
#define NUM_PAGES           (SECTOR_SIZE / PAGE_SIZE)  // 2048 pages
#define MAGIC_NUMBER        0xDEADBEEF
//////////////////////////////////////////////////////////////////

#define taskENTER_CRITICAL() __disable_irq()
#define taskEXIT_CRITICAL() __enable_irq()


//defines for flash memory access

uint8_t unNumberofModules = 16;
uint8_t unCell_1 = 0;
uint8_t unNumberOfConnectedCells = 0;
uint64_t ulBalanced_cells_mask = 0;
uint8_t unDataRecieved;
float fCellsUndervoltageBalancingLimit = 2.8500;
float fCellsMinVoltageForBalancing = 0;
float fCellMinvoltage;
float fCellMaxvoltage;
float fChargeprotectionLimit = 3.66;
float fDisChargeprotectionLimit = 2.70;

FDCAN_TxHeaderTypeDef   TxHeader;
FDCAN_RxHeaderTypeDef   RxHeader;
uint8_t                 TxData[8];
uint8_t dataToSend[8] = {0x11, 0x22, 0x33};
uint8_t               RxData[8];
uint8_t indx = 0;
uint8_t unDataSendingGroupNumber = 0;
uint8_t unDataSendingCurrentGroup = 0;
bool bBmsSendData = 0;
uint8_t unMajorVersion = 0;
uint8_t unMinorVersion = 0;
uint8_t unBuild = 0;

uint8_t unDate = 0;
uint8_t unMonth = 0;
uint8_t unTime_hours;
uint8_t untime_minutes;
//freertos

#define CELLS_PER_FRAME  3
#define MAX_CELLS       (sizeof(bmsdata.udRawCellVoltages)/sizeof(bmsdata.udRawCellVoltages[0]))

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			unDataRecieved = 1;
		}
		if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		{
			Error_Handler();
		}
	}
}
typedef struct
{
	float fCellVoltage [192];
	uint16_t udRawCellVoltages[192];

    uint8_t unCellChargeProtetionLimit: 1;
    uint8_t unCellDisChargeProtetionLimit: 1;
    uint8_t unCellProtectionLimit;
    uint8_t unMaxSOCLimit : 1;
    uint8_t unMinSOCLimit: 1;
    uint8_t unMaxNumberOfcellsLimit : 1;
    uint8_t unDisChargeCurrentLimit : 1;
    uint8_t unOverCurrentLimit : 1;
    bool bProtection;
    float fTotalVoltage;
}
BMSDATA;
BMSDATA bmsdata = {0};

typedef struct
{
	uint8_t wkup; // DUMMY WAKE UP BYTE*
    uint8_t stcvad[2];  // STCVAD command
    uint8_t rdcva[2];   // RDCVA command   *READ CELL VOLTAGE GROUP A*
    uint8_t rdcvb[2];   // RDCVB command   *READ CELL VOLTAGE GROUP B*
    uint8_t rdcvc[2];   // RDCVC command   *READ CELL VOLTAGE GROUP C*
    uint8_t rdcvd[2];   // RDCVD command   *READ CELL VOLTAGE GROUP D*
    uint8_t wrcfg[2];   // WRCFG command   *WRITE CONFIGURATIOM*
    uint8_t rdcfg[2];   // WRCFG command   *READ CONFIGURATION*
    uint8_t adcv[2];    // ADCV command    *START ADC VOLTAGE CONVERSION*
    uint8_t adcv2[2];    // ADCV command   *START ADC VOLTAGE CONVERSION(2ND COMMAND)*
    uint8_t clrcell[2]; // CLRCELL command *CLEAR CELL STATUS*
    uint8_t polladc[2]; // POLLADC command *START AND POLL ADC STATUS*

} LTC6804_Commands;

const LTC6804_Commands ltc6804_commands =
{
	.wkup = 0x00,
    .stcvad  = {0x02, 0x60}, // STCVAD
    .rdcva   = {0x00, 0x04}, // RDCVA
    .rdcvb   = {0x00, 0x06}, // RDCVB
    .rdcvc   = {0x00, 0x08}, // RDCVC
    .rdcvd   = {0x00, 0x0A}, // RDCVD
    .wrcfg   = {0x00, 0x01}, // WRCFG
	.rdcfg   = {0x00, 0x02}, // RDCFG
    .adcv    = {0x03, 0x70}, // ADCV
	.adcv2   = {0x05, 0x11}, // ADCV2(confirm this command from the bms capture)
    .clrcell = {0x07, 0x11}, // CLRCELL
    .polladc = {0x9F, 0x14}, // POLLADC
};

typedef union
{
    uint8_t byte;
    struct
	{
        uint8_t unAdcopt : 1; // Bit 0
        uint8_t unSwtrd  : 1; // Bit 1
        uint8_t unRefon  : 1; // Bit 2
        uint8_t unGpio1  : 1; // Bit 3
        uint8_t unGpio2  : 1; // Bit 4
        uint8_t unGpio3  : 1; // Bit 5
        uint8_t unGpio4  : 1; // Bit 6
        uint8_t unGpio5  : 1; // Bit 7
    };
} CFGR0;

typedef union
{
    uint8_t byte;
    struct
	{
        uint8_t unVuv_bit0 : 1; // Bit 0
        uint8_t unVuv_bit1 : 1; // Bit 1
        uint8_t unVuv_bit2 : 1; // Bit 2
        uint8_t unVuv_bit3 : 1; // Bit 3
        uint8_t unVuv_bit4 : 1; // Bit 4
        uint8_t unVuv_bit5 : 1; // Bit 5
        uint8_t unVuv_bit6 : 1; // Bit 6
        uint8_t unVuv_bit7 : 1; // Bit 7
    };
} CFGR1;

typedef union
{
    uint8_t byte;
    struct
	{
        uint8_t unVuv_bit8  : 1; // Bit 0
        uint8_t unVuv_bit9  : 1; // Bit 1
        uint8_t unVuv_bit10 : 1; // Bit 2
        uint8_t unVuv_bit11 : 1; // Bit 3
        uint8_t unVov_bit0  : 1; // Bit 4
        uint8_t unVov_bit1  : 1; // Bit 5
        uint8_t unVov_bit2  : 1; // Bit 6
        uint8_t unVov_bit3  : 1; // Bit 7
    };
} CFGR2;

typedef union
{
    uint8_t byte;
    struct
	{
        uint8_t unVov_bit4  : 1; // Bit 0
        uint8_t unVov_bit5  : 1; // Bit 1
        uint8_t unVov_bit6  : 1; // Bit 2
        uint8_t unVov_bit7  : 1; // Bit 3
        uint8_t unVnVov_bit8 : 1; // Bit 4
        uint8_t unVov_bit9  : 1; // Bit 5
        uint8_t unVov_bit10 : 1; // Bit 6
        uint8_t unVov_bit11 : 1; // Bit 7
    };
} CFGR3;

typedef union
{
    uint8_t byte;
    struct
	{
        uint8_t unDcc1 : 1; // Bit 0: Cell 1 balancing
        uint8_t unDcc2 : 1; // Bit 1: Cell 2 balancing
        uint8_t unDcc3 : 1; // Bit 2: Cell 3 balancing
        uint8_t unDcc4 : 1; // Bit 3: Cell 4 balancing
        uint8_t unDcc5 : 1; // Bit 4: Cell 5 balancing
        uint8_t unDcc6 : 1; // Bit 5: Cell 6 balancing
        uint8_t unDcc7 : 1; // Bit 6: Cell 7 balancing
        uint8_t unDcc8 : 1; // Bit 7: Cell 8 balancing
    };
} CFGR4;

typedef union
{
    uint8_t byte;
    struct
	{
        uint8_t unDcc9   : 1; // Bit 0: Cell 9 balancing
        uint8_t unDcc10  : 1; // Bit 1: Cell 10 balancing
        uint8_t unDcc11  : 1; // Bit 2: Cell 11 balancing
        uint8_t unDcc12  : 1; // Bit 3: Cell 12 balancing
        uint8_t unDcto0  : 1; // Bit 4: Discharge timeout bit 0
        uint8_t unDcto1  : 1; // Bit 5: Discharge timeout bit 1
        uint8_t unDcto2  : 1; // Bit 6: Discharge timeout bit 2
        uint8_t unDcto3  : 1; // Bit 7: Discharge timeout bit 3
    };
} CFGR5;

typedef struct
{
	uint16_t udCellVoltage1;
	uint16_t udCellVoltage2;
	uint16_t udCellVoltage3;
	uint16_t udPECcellvoltages;
}
Group;

typedef struct
{
    CFGR0 cfgr0;
    CFGR1 cfgr1;
    CFGR2 cfgr2;
    CFGR3 cfgr3;
    CFGR4 cfgr4;
    CFGR5 cfgr5;
} Config;

    CFGR0 cfgr0 = {0};
    CFGR1 cfgr1 = {0};
    CFGR2 cfgr2 = {0};
    CFGR3 cfgr3 = {0};
    CFGR4 cfgr4 = {0};
    CFGR5 cfgr5 = {0};

typedef struct
{
	Group group[4];
	Config configure;
	/* the groups are defined as
	*  group[0] = Group A
	*  group[1] = Group B
	*  group[2] = Group C
	*  group[3] = Group D */

}
Module;

typedef struct
{
	Module module[MODULES_NO];
}
BMS;

typedef struct
{
	uint8_t unData[NUMBER_OF_BYTES_PER_GROUP];
}GROUP;

typedef struct
{
	GROUP group[NUMBER_OF_GROUPS];
}BMS_SEND_CAN_DATA;

BMS_SEND_CAN_DATA candata = {0};


BMS_COMMAND_RECEPTION canparam = {0};

typedef struct
{
	    uint16_t udTotalVoltage;                    // Group 3
	    uint16_t udTotalCurrent;                    // Group 3
	    uint16_t udTotalPower;                      // Group 3
	    uint16_t udBatteryUsageCapacity;            // Group 4
	    uint16_t udBatteryCapacityPercentage;       // Group 4
	    uint16_t udChargingCapacity;                // Group 4
	    uint16_t udRemainingCapacity;               // Group 5
	    uint16_t udHostTemperature;                 // Group 6
	    uint16_t udStatusAccounting;                // Group 6, bitfield per Table 1
	    uint16_t udLowVoltagePowerOutageProtection; // Group 80
	    uint16_t udLowVoltagePowerOutageDelayed;    // Group 80
	    uint16_t udNumOfTriggeringProtectionCells;  // Group 80
	    uint16_t udBalancedReferenceVoltage;        // Group 81
	    uint16_t udMinimumVoltage;                  // Group 81
	    uint16_t udMaximumVoltage;                  // Group 81
	    uint8_t  unChargingAndDischargingMOSStatus; // Group 81
	    uint32_t udAccumulatedTotalCapacity;        // Group 82, 32-bit value
	    uint8_t  unLCDStatus;                       // Group 82
	    uint8_t  unProtectingHistoricalLogs;        // Group 83, per Table 2
}BMS_DATA_CALCULATIONS;

BMS_DATA_CALCULATIONS bms_calc = {0};

typedef struct
{
	uint32_t magic;
	uint32_t sequence;
	BMS_COMMAND_RECEPTION data;

}FlashPage;

typedef enum
{
	STATE_BALANCING_IDLE,
	STATE_CHECK_BALANCING_COMMAND,
	STATE_CHECK_UNDERVOLTAGE,
	STATE_GET_MIN_CELL_VOLTAGE,
	STATE_START_BALANCING,
	STATE_CHECK_FOR_BALANCED_CELLS,
	STATE_STOP_BALANCING_FOR_BALANCED_CELLS,
	STATE_BALANCING_COMPLETE,
	STATE_BALANCING_FAULT

}STATE_CELL_BALANCING;
STATE_CELL_BALANCING current_balancing_state = STATE_BALANCING_IDLE;
const char* CELL_BALANCING_STATES [] =
{
	"STATE_BALANCING IDLE",
	"STATE_CHECK_BALANCING_COMMAND",
	"STATE_CHECK_UNDERVOLTAGE",
	"STATE_GET_MIN_CELL_VOLTAGE",
	"STATE_START_BALANCING",
	"STATE_CHECK_FOR_BALANCED_CELLS",
	"STATE_STOP_BALANCING_FOR_BALANCED_CELLS",
	"STATE_BALANCING_COMPLETE",
	"STATE_BALANCING_FAULT"
};
BMS bms1 = {0};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vPopulateArray(uint8_t* unTarget, const uint8_t* unSource, uint8_t unSize);
void vSetconfigToZero();
void vsetInitialWriteConfig();
void vSetbms1configToZero(void);
void vSetbms1Module1Config(void);
void vSetbms2Module1Config(void);
void vSetbms3Module1Config(void);
void vSetbms4Module1Config(void);
void vSetbms5Module1Config(void);
void vSetbms6Module1Config(void);
void vSerializeModuleConfig(const Module* module, uint8_t* cfg6);
void vSetAllModulesConfig();
float fGetminimunVoltage();
void vParseCellVoltages();
void vPrintCellVoltages(void);
void vCellBalancingStateMachine();
void vSetDischargeBitsofAllCells();
void vSet_cell_balanced(uint8_t cell);
void vClear_cell_balanced(uint8_t cell);
void vToggle_cell_balanced(uint8_t cell);
void vPrint_balanced_cells(void);
uint8_t unGetModulenumber(uint8_t cellNumber);
uint8_t unGetCellNumber(uint8_t cellNumber);
uint8_t unGetCellGroupNumber(uint8_t cellNumber);
uint8_t unGetCfgrRegisterNumber(uint8_t cellNumber);
void vPrint_struct_sizes(void);
void vPrintCanRxData();
void vCAN_SendMessage(uint32_t stdId);
void SendCellVoltages(void);
void vPopulateCandataSendingStruct(void);
void vSendAllCANGroups(void);
void vPrintCandataGroups(void);
void vSetGroupNumber(void);
void vPopulateCellsDataGroups(void);
void vPopulateGroup1(void);
void vPopulateGroup2(void);
void vPopulateGroup3(void);
void init_dummy_bms_data(BMS_DATA_CALCULATIONS *bms);
void vPopulateGroup4(void);
void vPopulateGroup5(void);
void vPopulateGroup6(void);
void vPopulateGroup80(void);
void vPopulateGroup81(void);
void vPopulateGroup82(void);
void vPopulateGroup83(void);
void vPopulateGroup84(void);
void vPrintCandataParams(void);
void vHandleCommand_Reception(uint8_t *data);
void vPrintBmsCalcData(void);
uint32_t FindLatestPage(void);
void LoadFromFlash(void);
HAL_StatusTypeDef SaveToFlash(uint32_t currentIdx);
void UpdateParameter(uint16_t index, uint16_t value);
float vCalculateTotalVoltage(void);
float fChargeProtection(void);
float fDisChargeProtection(void);
void vProtections();
void vTurnONRelay();
void vTurnOFFRelay();
void vPrintFirmwareInfo(uint8_t MajorVersion,uint8_t MinorVersion,uint8_t Build,uint8_t Day,uint8_t Month,uint8_t Hour,uint8_t Minute);
void vPrintFirmwareVersion(uint8_t MajorVersion,uint8_t MinorVersion,uint8_t Build);
void vPrintFirmwareDate(uint8_t Day,uint8_t Month);
void vPrintFirmwareTime(uint8_t Hour,uint8_t Minute);
void send_cell_data(struct tcp_pcb *tpcb);
void trace_eth(const char *format, ...);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


typedef struct {
    uint16_t cells[NUM_CELLS];  // Cell voltage data (max 64 cells)
} CellData_t;

typedef struct {
    uint16_t pecs[NUM_MODULES][4];  // 4 groups (A–D) × 6 modules
} PECData_t;

CellData_t cell_data;
PECData_t pec_data;

#define NUM_DEVICES  16
#define CS_GPIO_PORT    GPIOA
#define CS_GPIO_PIN     GPIO_PIN_4
//uint8_t spi1TxData[10] = {0xaa, 0xbb, 0xcc ,0xdd ,0xee ,0xff ,0x00 ,0x11};
uint8_t spi1TxData[10] = {0x0B, 0xA1, 0xD0 ,0xBA ,0xFD ,0xe4 ,0x65 ,0xD1, 0xe1,0xd4};

//uint8_t spiTxData[10] = {0xab, 0xa1, 0xcd ,0x0ff ,0x63 ,0xaa ,0x76 ,0x43, 0x87,0xdd};
uint8_t spiTxData[20] = {0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00, 0x0a,0x00,0xbb,0xac,0xaa,0x12};
uint8_t spiRxData[10] = {0};
 // Example data
//uint8_t spi1RxData[10] = {0}; // Buffer to store received data
uint8_t spi2RxData[10] = {0};  // Adjust size as needed
uint8_t spi1RxData[30] = {0};         // Enough for cmd + dummy read
uint8_t unBalancing = 0;
uint8_t unNumber_of_Cells = 192;
uint16_t udDelay_for_1_Module = 30;
uint8_t unNumber_of_Modules = 16;

uint8_t pec8(const uint8_t *buf, uint8_t len);
void LTC6804_ReadCells(void);
uint16_t pec15(uint8_t *data, uint8_t len) ;
void LTC6804_WritebalancingConfig();


#define MAX_CELLS 11

static const unsigned int crc15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
void Set_PWM_DutyCycle(uint32_t duty);
int read_pilot_voltage();
float calculate_voltage(uint32_t adc_value);
uint8_t unGetNumberofModules();
void vStartBalancing();

uint8_t rxByte;
uint8_t uartReady = 0;
uint32_t adc_value;
void SPI_SendData(void);
void SPI_StartReceiving(void);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void SPI_SendData_Blocking(void);
void SPI_TransmitReceive_LTC6804(void);
void SPI_Communicate_LTC6804(void);
static void LTC6804_ReadGroup(uint8_t msb, uint8_t lsb);
static void LTC6804_StartADC2(void);
void SPI_Communicate_LTC6804_Stack();
void tcp_server_init(void);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static void handle_led_command(char *data, char *response, int *len);
static void handle_custom_value(char *data, char *response, int *len);
static void tcp_server_error(void *arg, err_t err);
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
static void handle_balance_command(char *data, char *response, int *len);
void send_charge_discharge_limits(void);


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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_FDCAN1_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_ADC_Start(&hadc2);
  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  HAL_SPI_Receive_IT(&hspi2, spi2RxData, sizeof(spi2RxData));
  tcp_server_init();

	uint32_t lastSendTick = HAL_GetTick();
	vSetbms1configToZero();
	vSetAllModulesConfig();
	TRACE("ALL MODULES CONFIGURED TO 0xFC \r\n");
	char msg[128];

	if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
	{
		Error_Handler();
		TRACE("CAN NOT STARTED \r\n");
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		/* Notification Error */
		TRACE("NOTIFICATION NOT STARTED\r\n");
		Error_Handler();
	}

	  TxHeader.Identifier = 0x11;
	  TxHeader.IdType = FDCAN_STANDARD_ID;
	  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	  TxHeader.MessageMarker = 0;

	  init_dummy_bms_data(&bms_calc);

  //char buffer[10];  // Buffer to store incoming UART data
  //uint8_t buffer_index = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
// for (uint8_t i=0; i<8; i++)
// {
//	TxData[i] = i;
// }
	  TxData[0] = 0x56;
	  TxData[1] = 0x25;
	  TxData[2] = 0x23;
	  TxData[3] = 0xab;
	  TxData[4] = 0xcd;
	  TxData[5] = 0xef;
	  TxData[6] = 0x22;
	  TxData[7] = 0x33;
	  LoadFromFlash();
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  vTurnONRelay();
	  fDisChargeprotectionLimit = canparam.udOverDischargeVoltageProtectionValue/100.0f;
	  fChargeprotectionLimit = canparam.udOverChargeVoltageProtectionValue/100.0f;
	  if(fDisChargeprotectionLimit == 0.0f)
	  {
		  fDisChargeprotectionLimit = INITAIL_DISCHARGE_PROTECTION ;
	  }
	  if(fChargeprotectionLimit == 0.0f)
	  {
		 fChargeprotectionLimit = INITAIL_CHARGE_PROTECTION;
	  }
	  char buf [50];
	  vPrintFirmwareInfo(0, 0, 1, 27, 05, 18, 35);


 /*IMPROVEMENTS TO BE DONE*/
 //TOOD CAN BMS Handle the commands from the user to control the bms ( DONE )
 //TDOD CAN Handling for reception of data (make fucntions (avoid hardcoding the RX BYTES)(DONE)
 //TODO Populate the other GROUPS (BMS CAN SEND DATA )(DONE)
 //TODO Delay for number of modules acccording to the datasheet(PENDING)
 //TODO Number of cells and modules to be handled correctly (DONE (tested -> 80 percent check with saelee and chenging number of modueles))
 //TODO Too many variables for NUMBER OF CELLS/MODULES etc used, make them global and same var for all the fucntionality (PENDING)
 //TODO Remove the debug condition for balancing and let it run freely (will take one second -> PENDING)
 //TODO Move the bms and can to seperate files(PENDING)
 //TODO Move the parsing of the cells to some kind o f loop to make it adapt differnet number of cells and modules(DONE)
 //TODO Make a fucntion for GPIO set/reset
  while (1)
  {
	  MX_LWIP_Process();
	if ((HAL_GetTick() - lastSendTick) >= 500)
	{

		if (bmsdata.bProtection == false)
		{
			lastSendTick = HAL_GetTick();
			sensor_value++;
//
			snprintf(buf, sizeof(buf), "OVER CHARGE LIMIT: %.3f \r\n", fChargeprotectionLimit);
			TRACE("%s", buf);
			snprintf(buf, sizeof(buf), "DISCHARGE LIMIT: %.3f \r\n", fDisChargeprotectionLimit);
			TRACE("%s", buf);
//
//            if (active_tpcb != NULL)
//            {
//                send_cell_data(active_tpcb);
//            }
			vProtections();
			SPI_Communicate_LTC6804_Stack();
			vParseCellVoltages();
			vPrintCanRxData();

			vPopulateCandataSendingStruct();
			if(bBmsSendData == true)
			{
				vSendAllCANGroups();
			}
			if(unBalancing == 1 )
			{
//				vPrintCellVoltages();
				vCellBalancingStateMachine();
			}
			else
			{
				vSetAllModulesConfig();

			}
			lastSendTick = HAL_GetTick();
		}
		else
		{
			TRACE("PROTECTIONS TRIGGERED \r\n");
			vTurnOFFRelay();
		}
//
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 6;
  hfdcan1.Init.NominalTimeSeg1 = 33;
  hfdcan1.Init.NominalTimeSeg2 = 6;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 20;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x0;
  sFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Function for sending the SPI DATA (on interrupt )
 */

void SPI_SendData(void)
{
    HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&hspi1, spi1TxData, sizeof(spi1TxData));
    if(status == HAL_OK)
    {
    	  char msg[] = "SPI DATA SEND :\r\n";
    	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    if (status != HAL_OK)
    {
  	  char msg[] = "SPI SEND DATA ERROR :\r\n";
  	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

/**
 * @brief  Transmit and receive SPI Data at the same time (crucial for reading the voltage groups from the ltc 6804)
 */

void SPI_TransmitReceive(void)
{

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(&hspi1, spiTxData, spiRxData, sizeof(spiTxData));

    if (status != HAL_OK)
    {
        char msg[] = "SPI TX/RX START ERROR\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    // NSS will be set high in the callback
}

/**
 * @brief  Calculate the 15-bit PEC (CRC) for LTC6804.
 *         Implements the datasheet bit-by-bit algorithm:
 *         seed=0x0010, poly=x15+x14+x10+x8+x7+x4+x3+1 (0x4599),
 *         append a 0 LSB at the end.
 */

uint16_t calculate_pec15(uint8_t len, const uint8_t *data)
{
    const uint16_t POLY15 = 0x4599;
    uint16_t pec = 0x0010;  // 15-bit register (bit14:bit0), seed = 0b000000000010000

    // Process each byte, MSB first
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int bit = 7; bit >= 0; bit--) {
            // DIN = next data bit
            uint16_t din = (byte >> bit) & 1;
            // topbit = PEC[14]
            uint16_t topbit = (pec >> 14) & 1;
            // IN0 = DIN XOR PEC[14]
            uint16_t in0 = topbit ^ din;

            // shift the PEC register left by 1 (drop bit14, move everything up)
            pec = (pec << 1) & 0x7FFF;

            // if IN0==1, XOR in the polynomial taps
            if (in0) {
                pec ^= POLY15;
            }
        }
    }

    // At end, PEC is a 15-bit value in bits [14:0].
    // Append a zero LSB to make it a 16-bit word, per datasheet.
    return (uint16_t)(pec << 1);
}

/**
 * @brief   complete spi transactions (fucntions ) to read data from ltc6804-1
 * for 6 nodules attahced
 */

static void spi_cs_transfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, HAL_MAX_DELAY);
}

/**
 * @brief   use this function to wakeup the LTC6804 from sleep mode
 * it sends dummy bytes just to pull cs low for some time to induce activity
 * on the cs line
 */

static void LTC6804_Wakeup(void)
{
	//send a dummy byte to wake up the ltc6804 from sleep
    uint8_t dummy = ltc6804_commands.wkup;
    HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);
}

/**
 * @brief   writing the initial config (0xfc) to the ltc6804
 * this function will send the configuration bytes for each module
 * (6 configuration bytes followed by 2 PEC bytes ) in a daisy chain
 * the configuration of the first module is sent at the last (for daisy chain)
 * because in a daisy chain the first byte sent is pushed to the last
 */

static void LTC6804_WriteConfig(void)
{
    // 1) Prepare the command header (WRCFG command + PEC)
    uint8_t cmd[2];
    vPopulateArray(cmd, ltc6804_commands.wrcfg, 2);
    uint16_t pec_cmd = calculate_pec15(2, cmd);

    // 2) Build the ( 4 (write config + pec ) + 8 x num_modules (16)->(configurations ) )52-byte transmit buffer
    uint8_t tx[(8 * NUM_MODULES) + 4 ];
    // 2a) Header: cmd + PEC (4 bytes)
    tx[0] = cmd[0];
    tx[1] = cmd[1];
    tx[2] = (uint8_t)(pec_cmd >> 8);
    tx[3] = (uint8_t)(pec_cmd & 0xFF);
    // 2b) Add each module's configuration in reverse order (6 modules, 8 bytes each: 6-byte config + 2-byte PEC)
    for (int i = 0; i < MODULES_NO; i++)
    {
        // Serialize the module's configuration
        uint8_t cfg6[6];
        vSerializeModuleConfig(&bms1.module[i], cfg6);
        // Calculate PEC for this module's configuration
        uint16_t pec_cfg = calculate_pec15(6, cfg6);

        // Copy into the buffer: 6 bytes config + 2 bytes PEC, in reverse module order
        int off = 4 + (MODULES_NO - 1 - i) * 8;
        memcpy(&tx[off], cfg6, 6);
        tx[off + 6] = (uint8_t)(pec_cfg >> 8);
        tx[off + 7] = (uint8_t)(pec_cfg & 0xFF);
    }

    // 3) Transmit the entire 52-byte buffer
    HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), HAL_MAX_DELAY);
}


//---------------------------------------------------------------------------
// 3) Broadcast-Start ADC Conversion (ADCV MD=10, DCP=1 → 0x03 0x70 + PEC)
static void LTC6804_StartADC(void)
{
//    uint8_t cmd[2] = { 0x03, 0x70 };
    uint8_t cmd[2];
    vPopulateArray(cmd,ltc6804_commands.adcv,2);
    uint16_t pec = calculate_pec15(2, cmd);
    uint8_t pkt[4] = { cmd[0], cmd[1], (uint8_t)(pec>>8), (uint8_t)pec };
    HAL_SPI_Transmit(&hspi1, pkt, 4, HAL_MAX_DELAY);
}
// 4) Broadcast-Start ADC Conversion 2nd command as per the top bms sniffing
static void LTC6804_StartADC2(void)
{
    uint8_t cmd[2];
    vPopulateArray(cmd,ltc6804_commands.adcv2,2);
    uint16_t pec = calculate_pec15(2, cmd);
    uint8_t pkt[4] = { cmd[0], cmd[1], (uint8_t)(pec>>8), (uint8_t)pec };
    HAL_SPI_Transmit(&hspi1, pkt, 4, HAL_MAX_DELAY);
}
//---------------------------------------------------------------------------
// 5) Poll ADC status (PLADC = 0x9F 0x14 + PEC) → returns SDO low while busy
static void LTC6804_PollADC(void)
{
	/*not recommended to use this function/command*/
    uint8_t cmd[2];
    vPopulateArray(cmd,ltc6804_commands.polladc,2);
    uint16_t pec = calculate_pec15(2, cmd);
    uint8_t pkt[4] = { cmd[0], cmd[1], (uint8_t)(pec>>8), (uint8_t)pec };
    uint8_t dummy = ltc6804_commands.wkup;
    HAL_SPI_Transmit(&hspi1, pkt, 4, HAL_MAX_DELAY);

//    do
//    {
//      HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);
//    }
//    while (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9) == GPIO_PIN_RESET);
}

//---------------------------------------------------------------------------
// 6) Broadcast-Read Cell Voltage Group A  (repeat for B, C, D with 0x04 0x06,0x08,0x0A)
void LTC6804_ReadGroup(uint8_t msb, uint8_t lsb)
{
    static uint8_t group_index = 0;  // 0 to 3 for Groups A–D

    uint8_t cmd[2] = { msb, lsb };
    uint16_t pec = calculate_pec15(2, cmd);

    uint8_t tx[ 4 + (8*NUM_MODULES)], rx[4 + (8*NUM_MODULES)];
    tx[0] = cmd[0];
    tx[1] = cmd[1];
    tx[2] = (uint8_t)(pec >> 8);
    tx[3] = (uint8_t)(pec & 0xFF);
    memset(tx + 4, 0xFF, 8 * NUM_MODULES);

    HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx,  4 + (8*NUM_MODULES), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET);

    uint8_t *group_data = &rx[4];

    for (int module = 0; module < NUM_MODULES; module++)
    {
        int base = module * 8;

        // Read and store 3 cell voltages (6 bytes)
        bms1.module[module].group[group_index].udCellVoltage1 = group_data[base] | (group_data[base + 1] << 8);
        bms1.module[module].group[group_index].udCellVoltage2 = group_data[base + 2] | (group_data[base + 3] << 8);
        bms1.module[module].group[group_index].udCellVoltage3 = group_data[base + 4] | (group_data[base + 5] << 8);

        // Read and store PEC (2 bytes after 6 data bytes)
        uint8_t pec_msb = group_data[base + 6];
        uint8_t pec_lsb = group_data[base + 7];
        bms1.module[module].group[group_index].udPECcellvoltages = (pec_msb << 8) | pec_lsb;
    }

    group_index = (group_index + 1) % 4;
}

/**
 * @brief   Print raw cells data obtained from the LTC 6804 as well as the cells data
 * converted to float
 */

void vPrintCellVoltages(void)
{
    char msg[64];
    vCalculateTotalVoltage();
    uint8_t unNumberofconnectedCells = 192;
    TRACE("TOTAL PACK VOLTAGE : %.4f V \r\n ", bmsdata.fTotalVoltage);
    trace_eth("TOTAL PACK VOLTAGE_eth: %.4f", bmsdata.fTotalVoltage);
    TRACE("Number Of Connected Cells: %u \r\n ", unNumberOfConnectedCells);
    for(uint8_t i=0; i < unNumberofconnectedCells ; i ++)
    {
        snprintf(msg, sizeof(msg), "Cell %u Voltage: %.4f V\r\n",(i+1),bmsdata.fCellVoltage[i] );
        TRACE("%s", msg);
    }
    for(uint8_t i=0; i < unNumberofconnectedCells ; i ++)
    {
        snprintf(msg, sizeof(msg), "Cell %u Voltage RAW : 0x%X \r\n",(i+1),bmsdata.udRawCellVoltages[i] );
        TRACE("%s", msg);
    }
}

//---------------------------------------------------------------------------
// 6) Broadcast-Clear Cell Voltage Registers (CLRCELL = 0x07 0x11 + PEC)
static void LTC6804_ClearCell(void)
{

    uint8_t cmd[2];
    vPopulateArray(cmd,ltc6804_commands.clrcell,2);
    uint16_t pec = calculate_pec15(2, cmd);
    uint8_t pkt[4] = { cmd[0], cmd[1], (uint8_t)(pec>>8), (uint8_t)pec };
    HAL_SPI_Transmit(&hspi1, pkt, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

}

//---------------------------------------------------------------------------
/**
 * @brief   This is the Top level function for LTC6804 handling,
 * it does all the transactions required to read the data Only
 * this function needs to be called in the main loop
 */

void SPI_Communicate_LTC6804_Stack(void)
{
    unGetNumberofModules();

	LTC6804_Wakeup();
	LTC6804_WriteConfig();
	LTC6804_StartADC();
	HAL_Delay(unNumber_of_Modules * udDelay_for_1_Module);
	LTC6804_Wakeup();
	LTC6804_ReadGroup(0x00, 0x04);
	LTC6804_Wakeup();
	LTC6804_ReadGroup(0x00, 0x06);
	LTC6804_Wakeup();
	LTC6804_ReadGroup(0x00, 0x08);
	LTC6804_Wakeup();
	LTC6804_ReadGroup(0x00, 0x0A);
}

/*
*	@brief	  wrapper for hal_uart_transmit to make
*	prints a bit easier
*/

void trace_printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Send to UART
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Send to Ethernet using trace_eth logic
    if (active_tpcb != NULL && active_tpcb->state == ESTABLISHED)
    {
        char log_buffer[sizeof(buffer) + 5];
        snprintf(log_buffer, sizeof(log_buffer), "LOG:%s\n", buffer);

        if (tcp_sndbuf(active_tpcb) > strlen(log_buffer))
        {
            err_t err = tcp_write(active_tpcb, log_buffer, strlen(log_buffer), TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK)
            {
                tcp_output(active_tpcb);
            }
        }
    }
}

/*
*    @brief		calculating the number of modules from the number of cells
*    will add a command un console to get the number of cells from the user.
*/

uint8_t unGetNumberofModules()
{
    if (unNumber_of_Cells == 0 || unNumber_of_Cells > 192)
    {
        TRACE("ERROR: Invalid number of cells (%d). Must be between 1 and 192.\r\n", unNumber_of_Cells);
        return 0;
    }

    unNumber_of_Modules = (unNumber_of_Cells + 11) / 12;
    return unNumber_of_Modules;
}

/*
 * @brief   This fucntion will populate an array with another arrays' values
*/

void vPopulateArray(uint8_t* unTarget, const uint8_t* unSource, uint8_t unSize)
{
	// Copy 2 bytes from src to dest
    memcpy(unTarget, unSource, unSize);
}

/*
 * @brief   This fucntion will set all the configure bytes for all the modules to zero initially
*/

void vSetbms1configToZero(void)
{
    for (int i = 0; i < MODULES_NO; i++)
    {
        // Set all bits in the Config struct of each module to zero
        bms1.module[i].configure.cfgr0 = (CFGR0){0};
        bms1.module[i].configure.cfgr1 = (CFGR1){0};
        bms1.module[i].configure.cfgr2 = (CFGR2){0};
        bms1.module[i].configure.cfgr3 = (CFGR3){0};
        bms1.module[i].configure.cfgr4 = (CFGR4){0};
        bms1.module[i].configure.cfgr5 = (CFGR5){0};
    }
}

/*
 * 	  @brief   This fucntion will set initial configuration of the fisrt module in the daisy chain
 * 	  we have set the configuration to 0xFC as per the data sheet to make the modules ready for starting adc conversions
 * 	  as well as reading the cell voltages
*/

void vSetbms1Module1Config(void)
{
	//initial configuration 0xFC ->first module -> module [0]
	bms1.module[0].configure.cfgr0.unGpio5 = 1;
	bms1.module[0].configure.cfgr0.unGpio4 = 1;
	bms1.module[0].configure.cfgr0.unGpio3 = 1;
	bms1.module[0].configure.cfgr0.unGpio2 = 1;
	bms1.module[0].configure.cfgr0.unGpio1 = 1;
	bms1.module[0].configure.cfgr0.unRefon = 1;
	bms1.module[0].configure.cfgr0.unSwtrd = 0;
	bms1.module[0].configure.cfgr0.unAdcopt = 0;
	bms1.module[0].configure.cfgr1 = (CFGR1){0};
	bms1.module[0].configure.cfgr2 = (CFGR2){0};
	bms1.module[0].configure.cfgr3 = (CFGR3){0};
	bms1.module[0].configure.cfgr4 = (CFGR4){0};
	bms1.module[0].configure.cfgr5 = (CFGR5){0};
}

/*
 * 	@brief   This fucntion will set initial configuration of the second module in the daisy chain
 * 	we have set the configuration to 0xFC as per the data sheet to make the modules ready for starting adc conversions
 * 	as well as reading the cell voltages
*/

void vSetbms2Module1Config(void)
{
	//initial configuration 0xFC ->second module -> module [1]
	bms1.module[1].configure.cfgr0.unGpio5 = 1;
	bms1.module[1].configure.cfgr0.unGpio4 = 1;
	bms1.module[1].configure.cfgr0.unGpio3 = 1;
	bms1.module[1].configure.cfgr0.unGpio2 = 1;
	bms1.module[1].configure.cfgr0.unGpio1 = 1;
	bms1.module[1].configure.cfgr0.unRefon = 1;
	bms1.module[1].configure.cfgr0.unSwtrd = 0;
	bms1.module[1].configure.cfgr0.unAdcopt = 0;
	bms1.module[1].configure.cfgr1 = (CFGR1){0};
	bms1.module[1].configure.cfgr2 = (CFGR2){0};
	bms1.module[1].configure.cfgr3 = (CFGR3){0};
	bms1.module[1].configure.cfgr4 = (CFGR4){0};
	bms1.module[1].configure.cfgr5 = (CFGR5){0};
}

/*
 * 	  @brief   This fucntion will set initial configuration of the third module in the daisy chain
 * 	  we have set the configuration to 0xFC as per the data sheet to make the modules ready for starting adc conversions
 * 	  as well as reading the cell voltages
*/

void vSetbms3Module1Config(void)
{
	//initial configuration 0xFC ->third module -> module [2]
	bms1.module[2].configure.cfgr0.unGpio5 = 1;
	bms1.module[2].configure.cfgr0.unGpio4 = 1;
	bms1.module[2].configure.cfgr0.unGpio3 = 1;
	bms1.module[2].configure.cfgr0.unGpio2 = 1;
	bms1.module[2].configure.cfgr0.unGpio1 = 1;
	bms1.module[2].configure.cfgr0.unRefon = 1;
	bms1.module[2].configure.cfgr0.unSwtrd = 0;
	bms1.module[2].configure.cfgr0.unAdcopt = 0;
	bms1.module[2].configure.cfgr1 = (CFGR1){0};
	bms1.module[2].configure.cfgr2 = (CFGR2){0};
	bms1.module[2].configure.cfgr3 = (CFGR3){0};
	bms1.module[2].configure.cfgr4 = (CFGR4){0};
	bms1.module[2].configure.cfgr5 = (CFGR5){0};
}

/*
 * @brief   This fucntion will set initial configuration of the fourth module in the daisy chain
 * 	  we have set the configuration to 0xFC as per the data sheet to make the modules ready for starting adc conversions
 * 	  as well as reading the cell voltages
 */

void vSetbms4Module1Config(void)
{
	//initial configuration 0xFC ->fourth module -> module [2]
	bms1.module[3].configure.cfgr0.unGpio5 = 1;
	bms1.module[3].configure.cfgr0.unGpio4 = 1;
	bms1.module[3].configure.cfgr0.unGpio3 = 1;
	bms1.module[3].configure.cfgr0.unGpio2 = 1;
	bms1.module[3].configure.cfgr0.unGpio1 = 1;
	bms1.module[3].configure.cfgr0.unRefon = 1;
	bms1.module[3].configure.cfgr0.unSwtrd = 0;
	bms1.module[3].configure.cfgr0.unAdcopt = 0;
	bms1.module[3].configure.cfgr1 = (CFGR1){0};
	bms1.module[3].configure.cfgr2 = (CFGR2){0};
	bms1.module[3].configure.cfgr3 = (CFGR3){0};
	bms1.module[3].configure.cfgr4 = (CFGR4){0};
	bms1.module[3].configure.cfgr5 = (CFGR5){0};
}

/*
 *    @brief   This fucntion will set initial configuration of the fifth module in the daisy chain
 * 	  we have set the configuration to 0xFC as per the data sheet to make the modules ready for starting adc conversions
 * 	  as well as reading the cell voltages
 */

void vSetbms5Module1Config(void)
{
	//initial configuration 0xFC ->fifth module -> module [2]
	bms1.module[4].configure.cfgr0.unGpio5 = 1;
	bms1.module[4].configure.cfgr0.unGpio4 = 1;
	bms1.module[4].configure.cfgr0.unGpio3 = 1;
	bms1.module[4].configure.cfgr0.unGpio2 = 1;
	bms1.module[4].configure.cfgr0.unGpio1 = 1;
	bms1.module[4].configure.cfgr0.unRefon = 1;
	bms1.module[4].configure.cfgr0.unSwtrd = 0;
	bms1.module[4].configure.cfgr0.unAdcopt = 0;
	bms1.module[4].configure.cfgr1 = (CFGR1){0};
	bms1.module[4].configure.cfgr2 = (CFGR2){0};
	bms1.module[4].configure.cfgr3 = (CFGR3){0};
	bms1.module[4].configure.cfgr4 = (CFGR4){0};
	bms1.module[4].configure.cfgr5 = (CFGR5){0};
}

/*
 *    @brief   This fucntion will set initial configuration of the sixth module in the daisy chain
 * 	  we have set the configuration to 0xFC as per the data sheet to make the modules ready for starting adc conversions
 * 	  as well as reading the cell voltages
 */

void vSetbms6Module1Config(void)
{
	//initial configuration 0xFC ->sixth module -> module [2]
	bms1.module[5].configure.cfgr0.unGpio5 = 1;
	bms1.module[5].configure.cfgr0.unGpio4 = 1;
	bms1.module[5].configure.cfgr0.unGpio3 = 1;
	bms1.module[5].configure.cfgr0.unGpio2 = 1;
	bms1.module[5].configure.cfgr0.unGpio1 = 1;
	bms1.module[5].configure.cfgr0.unRefon = 1;
	bms1.module[5].configure.cfgr0.unSwtrd = 0;
	bms1.module[5].configure.cfgr0.unAdcopt = 0;
	bms1.module[5].configure.cfgr1 = (CFGR1){0};
	bms1.module[5].configure.cfgr2 = (CFGR2){0};
	bms1.module[5].configure.cfgr3 = (CFGR3){0};
	bms1.module[5].configure.cfgr4 = (CFGR4){0};
	bms1.module[5].configure.cfgr5 = (CFGR5){0};

	for (uint8_t i= 6 ; i< 16 ; i ++)
	{
		bms1.module[i].configure.cfgr0.unGpio5 = 1;
		bms1.module[i].configure.cfgr0.unGpio4 = 1;
		bms1.module[i].configure.cfgr0.unGpio3 = 1;
		bms1.module[i].configure.cfgr0.unGpio2 = 1;
		bms1.module[i].configure.cfgr0.unGpio1 = 1;
		bms1.module[i].configure.cfgr0.unRefon = 1;
		bms1.module[i].configure.cfgr0.unSwtrd = 0;
		bms1.module[i].configure.cfgr0.unAdcopt = 0;
		bms1.module[i].configure.cfgr1 = (CFGR1){0};
		bms1.module[i].configure.cfgr2 = (CFGR2){0};
		bms1.module[i].configure.cfgr3 = (CFGR3){0};
		bms1.module[i].configure.cfgr4 = (CFGR4){0};
		bms1.module[i].configure.cfgr5 = (CFGR5){0};
	}
}

/*
 * @brief   This fucntion will set serialize the 6 modules' configurations bytes
 */

void vSerializeModuleConfig(const Module* module, uint8_t* cfg6)
{
    cfg6[0] = *(uint8_t*)&module->configure.cfgr0;
    cfg6[1] = *(uint8_t*)&module->configure.cfgr1;
    cfg6[2] = *(uint8_t*)&module->configure.cfgr2;
    cfg6[3] = *(uint8_t*)&module->configure.cfgr3;
    cfg6[4] = *(uint8_t*)&module->configure.cfgr4;
    cfg6[5] = *(uint8_t*)&module->configure.cfgr5;
}

/*
 * @brief   This fucntion will set The configurations of all the modules to
 * whatever they are initialized or set.
 */

void vSetAllModulesConfig()
{
	vSetbms1Module1Config();
	vSetbms2Module1Config();
	vSetbms3Module1Config();
	vSetbms4Module1Config();
	vSetbms5Module1Config();
	vSetbms6Module1Config();
}

/*
 * @brief   Parse and populate the Struct containing the raw and float
 *  cell voltages
 */

void vParseCellVoltages(void)
{
    for (uint8_t m = 0; m < NUM_MODULES; m++)
    {
        for (uint8_t g = 0; g < GROUPS_PER_MODULE; g++)
        {
            // Pointer to the first of the 3 cell voltage fields in this group:
            uint16_t *pv = &bms1.module[m].group[g].udCellVoltage1;
            for (uint8_t c = 0; c < NUMBER_OF_CELLS_PER_GROUP; c++)
            {
                uint16_t raw = pv[c];  // udCellVoltage1 + c
                // Flat index: module‑major, then group, then cell-in-group
                uint16_t idx = (m * CELLS_PER_MODULE)
                             + (g * NUMBER_OF_CELLS_PER_GROUP)
                             + c;
                bmsdata.udRawCellVoltages[idx] = raw;
                bmsdata.fCellVoltage[idx]     = (raw == 0xFFFF)
                                              ? 0.0f
                                              : raw * 0.0001f;
            }
        }
    }
}

/*
 * @brief   Get the cell number with the minimum voltage , while ignoring
 * the cells with zero voltages (as they are not connected)
 */

float fGetminimunVoltage()
{
	uint8_t unNum_of_cells = 192;
	float fmin_voltage = 0;
	fmin_voltage = bmsdata.fCellVoltage[0];
	for(uint8_t i = 1 ; i < unNum_of_cells; i++)
	{
		if(bmsdata.fCellVoltage[i] != 0.00f)
		{
			if(bmsdata.fCellVoltage[i] < fmin_voltage)
			{
				fmin_voltage = bmsdata.fCellVoltage[i] ;
			}
		}
	}
	return fmin_voltage;
}

/*
 * @brief   State-machine for the cell balancing , in this SM we first get the
 * cell with minimum voltage anf then set the dischage bit of all the cells to
 * 1 (which will start discharging all the cells ). when any of the cells voltage
 * reaches the minimum cell voltage we stop discharging that cell by setting its
 * corresponding dcc bit to 0
 */

void vCellBalancingStateMachine()
{
	switch(current_balancing_state)
	{
	case STATE_BALANCING_IDLE:
		{
			current_balancing_state = STATE_CHECK_BALANCING_COMMAND;
			TRACE("IDLE STATE \r\n");
			break;
		}
	case STATE_CHECK_BALANCING_COMMAND:
		{
			if(unBalancing == 1)
			{
				TRACE("BALANCING TURNED ON \r\n");
				current_balancing_state = STATE_CHECK_UNDERVOLTAGE;
			}
			else
			{
				TRACE("WAITING FOR BALANCING COMMAND \r\n");
				current_balancing_state = STATE_BALANCING_IDLE;
			}
			break;
		}
	case STATE_CHECK_UNDERVOLTAGE:
		{
			uint8_t unNumberofConnectedCells = 192;
			for(uint8_t i = 0; i < unNumberofConnectedCells; i++)
			{
				if(bmsdata.fCellVoltage[i] > 0.5000f)
				{
					if(bmsdata.fCellVoltage[i] < fCellsUndervoltageBalancingLimit)
					{
						TRACE("SOME CELLS ARE BELOW 3.000 VOLTS \r\n");
						TRACE("PLEASE FULLY CHARGE THE PACK FOR LESSER ENERGY WASTE \r\n");
						current_balancing_state = STATE_BALANCING_FAULT; // in the fault set the unbalance to 0
					}
				}
			}
			if(current_balancing_state == STATE_BALANCING_FAULT)
			{
				TRACE("STATE_BALANCING_FAULT \r\n");
			}
			else
			{
				current_balancing_state = STATE_GET_MIN_CELL_VOLTAGE;
				TRACE("NEXT->STATE_GET_MIN_CELL_VOLTAGE \r\n");
			}
			TRACE("UNDER VOLTAGE CHECK SUCCESSFUL \r\n");
			break;
		}
	case STATE_GET_MIN_CELL_VOLTAGE:
		{
			fCellsMinVoltageForBalancing = fGetminimunVoltage();
//			fCellsMinVoltageForBalancing = 3.2000;
			current_balancing_state = STATE_START_BALANCING;
			TRACE("MIN CELL VOLTAGE %.4f: \r\n",fCellsMinVoltageForBalancing);
			break;
		}
	case STATE_START_BALANCING:
		{
			vSetDischargeBitsofAllCells();
			current_balancing_state = STATE_CHECK_FOR_BALANCED_CELLS;
			break;
		}
    case STATE_CHECK_FOR_BALANCED_CELLS:
    {
        // Scan every cell and mark those below threshold as “balanced”
        for (uint16_t i = 0; i < unNumber_of_Cells; i++)
        {
            float voltage = bmsdata.fCellVoltage[i];
            if (voltage > 0.00f && voltage <= fCellsMinVoltageForBalancing)
            {
                vSet_cell_balanced(i);
                uint8_t module         = unGetModulenumber(i);
                uint8_t cell_in_module = unGetCellNumber(i);
                uint8_t reg            = unGetCfgrRegisterNumber(cell_in_module);
                uint8_t bit_pos        = (reg == 4)
                                         ? cell_in_module
                                         : (cell_in_module - 8);

                if (reg == 4)
                {
                    bms1.module[module].configure.cfgr4.byte &= ~(1 << bit_pos);
                }
                else
                {
                    bms1.module[module].configure.cfgr5.byte &= ~(1 << bit_pos);
                }
                TRACE("Cell %u (M%u:C%u) balanced, DCC cleared\r\n",
                      i, module, cell_in_module);

            }
        }

        TRACE("CHECKING FOR CELLS TO BE STOPPED FROM DISCHARGING COMPLETE\r\n");
        current_balancing_state = STATE_STOP_BALANCING_FOR_BALANCED_CELLS;
        break;
    }
    case STATE_STOP_BALANCING_FOR_BALANCED_CELLS:
    {
        TRACE("STOPPING THE BALANCED CELLS\r\n");
        bool balancing_complete = true;

        for (uint16_t i = 0; i < unNumber_of_Cells; i++)
        {
            float voltage = bmsdata.fCellVoltage[i];
            bool  is_balanced = bIs_cell_balanced(i);

            uint8_t module         = unGetModulenumber(i);
            uint8_t cell_in_module = unGetCellNumber(i);
            uint8_t reg            = unGetCfgrRegisterNumber(cell_in_module);
            uint8_t bit_pos        = (reg == 4)
                                     ? cell_in_module
                                     : (cell_in_module - 8);

            if (voltage == 0.0f)
            {
                // Zero voltage → always mark balanced & clear DCC
                if (!is_balanced) vSet_cell_balanced(i);
                if (reg == 4)
                {
                    bms1.module[module].configure.cfgr4.byte &= ~(1 << bit_pos);
                }
                else
                {
                    bms1.module[module].configure.cfgr5.byte &= ~(1 << bit_pos);
                }
//                                TRACE("Cell %u (zero voltage) marked balanced, DCC cleared\r\n", i);
            }
            else
            {
                if (!is_balanced)
                {
                    if (reg == 4)
                    {
                        bms1.module[module].configure.cfgr4.byte |= (1 << bit_pos);
                    }
                    else
                    {
                        bms1.module[module].configure.cfgr5.byte |= (1 << bit_pos);
                    }
                    vClear_cell_balanced(i);
                    balancing_complete = false;
                    TRACE("Cell %u (v=%.3f V, not balanced) DCC set to 1\r\n", i, voltage);
                }
                else
                {
                    // Already balanced → clear DCC
                    if (reg == 4)
                    {
                        bms1.module[module].configure.cfgr4.byte &= ~(1 << bit_pos);
                    }
                    else
                    {
                        bms1.module[module].configure.cfgr5.byte &= ~(1 << bit_pos);
                    }
                    TRACE("Cell %u (balanced) DCC cleared\r\n", i);
                }
            }
        }

        if (balancing_complete)
        {
            TRACE("Balancing complete for all cells\r\n");
            current_balancing_state = STATE_BALANCING_COMPLETE;
        }
        else
        {
            TRACE("Balancing not complete, continuing discharge\r\n");
            current_balancing_state = STATE_CHECK_FOR_BALANCED_CELLS;
        }
        break;
    }
	case STATE_BALANCING_COMPLETE:
		{
			TRACE("BALANCING COMPLETE \r\n");
			current_balancing_state = STATE_BALANCING_IDLE ;
			break;
		}
	case STATE_BALANCING_FAULT:
		{
			unBalancing = 0;
			TRACE("BALANCING FAULT \r\n");
			current_balancing_state = STATE_BALANCING_IDLE;
			break;
		}
	}
}

/*
 * @brief   Set the balancing bit of a particular cell (setting the bit to 1)
 * this will indicate that the cell has been balanced
 */

void vSet_cell_balanced(uint8_t cell)
{
    if (cell < NUM_CELLS)
    {
        ulBalancedCellsMask[WORD_IDX(cell)] |= (1ULL << BIT_POS(cell));
    }
}

/*
 * @brief   Clear the balancing bit of a particular cell (setting the bit to 0)
 * this will indicate that the cell has not been balanced
 */

void vClear_cell_balanced(uint8_t cell)
{
    if (cell < NUM_CELLS)
    {
        ulBalancedCellsMask[WORD_IDX(cell)] &= ~(1ULL << BIT_POS(cell));
    }
}

/*
 * @brief   Toggle the cell balance indication bit of a specific cell
 */

void vToggle_cell_balanced(uint8_t cell)
{
    if (cell < NUM_CELLS)
    {
        ulBalancedCellsMask[WORD_IDX(cell)] ^= (1ULL << BIT_POS(cell));
    }
}

/*
 * @brief   Check if a particular cell is balanced or not.
 * we have a uint64_t variable with each bit representing
 * that the cell is balanced (1) or not balanced (0)
 */

bool bIs_cell_balanced(uint8_t cell)
{
    if (cell >= NUM_CELLS) return false;
    return (ulBalancedCellsMask[WORD_IDX(cell)] >> BIT_POS(cell)) & 1U;
}

/*
 * @brief  Print the cell balance indication status for all the cells
 */

void vPrint_balanced_cells(void)
{
    for (uint16_t i = 0; i < NUM_CELLS; i++)
    {
        if (bIs_cell_balanced(i))
        {
            TRACE("Cell %u is balanced\r\n", i);
        }
    }
}

/*
 * @brief this function return the number of modules , which is calculated from
 * the number of cells which are taken as an input from the user
*/

uint8_t unGetModulenumber(uint8_t cellNumber)
{
    if (cellNumber >= NUM_CELLS)
        return 0xFF;  // Invalid module
    return cellNumber / 12;  // Returns 0 to 15
}

/*
 * @brief to get the position of a particular cell inside a module
*/

uint8_t unGetCellNumber(uint8_t cellNumber)
{
    if (cellNumber >= NUM_CELLS)
        return 0xFF;  // Invalid cell number
    return cellNumber % 12;  // Returns 0 to 11
}

/*
 * @brief to get the group name (A,B,C,D) of a particular cell inside a module
*/

uint8_t unGetCellGroupNumber(uint8_t cellNumber)
{
    uint8_t cellPos = unGetCellNumber(cellNumber);
    if (cellPos == 0)  // invalid
        return 0xFF;
    return (cellPos - 1) / 3;
}

/*
 * @brief to get the number of configuration register of a module
 * i.e. for cells 1-8 dcc bits are in the cfgr4 and for the next 4 cells
 * of the module they are in the cfgr5
*/

uint8_t unGetCfgrRegisterNumber(uint8_t cellNumber)
{
    uint8_t cellPos = unGetCellNumber(cellNumber);
    if (cellPos == 0xFF)
        return 0;  // Invalid
    return (cellPos <= 7) ? 4 : 5;  // 0-7 → CFGR4, 8-11 → CFGR5
}

/*
 * @brief This fucntion will set the discharge bit of all the cells to 1, which
 * means all the cells will start discharging - this needs to be done at the start of the state,achine
 * for cell balancing
*/

void vSetDischargeBitsofAllCells()
{
	uint8_t unNumberofModules = 16;

	for(uint8_t i = 0; i < unNumberofModules ; i++)
	{
		bms1.module[i].configure.cfgr4.unDcc1 = 1;
		bms1.module[i].configure.cfgr4.unDcc2 = 1;
		bms1.module[i].configure.cfgr4.unDcc3 = 1;
		bms1.module[i].configure.cfgr4.unDcc4 = 1;
		bms1.module[i].configure.cfgr4.unDcc5 = 1;
		bms1.module[i].configure.cfgr4.unDcc6 = 1;
		bms1.module[i].configure.cfgr4.unDcc7 = 1;
		bms1.module[i].configure.cfgr4.unDcc8 = 1;
		bms1.module[i].configure.cfgr5.unDcc9 = 1;
		bms1.module[i].configure.cfgr5.unDcc10 = 1;
		bms1.module[i].configure.cfgr5.unDcc11 = 1;
		bms1.module[i].configure.cfgr5.unDcc12 = 1;
//		bms1.module[i].configure.cfgr5.unDcto0 = 0;
//		bms1.module[i].configure.cfgr5.unDcto1 = 1;
//		bms1.module[i].configure.cfgr5.unDcto2 = 1;
//		bms1.module[i].configure.cfgr5.unDcto3 = 1;

	}
}

/*
 * @brief To Print sizes of all the structs used in the code
*/

void vPrint_struct_sizes(void)
{
    TRACE("Size of CFGR0: %u bytes\r\n", sizeof(CFGR0));
    TRACE("Size of CFGR1: %u bytes\r\n", sizeof(CFGR1));
    TRACE("Size of CFGR2: %u bytes\r\n", sizeof(CFGR2));
    TRACE("Size of CFGR3: %u bytes\r\n", sizeof(CFGR3));
    TRACE("Size of CFGR4: %u bytes\r\n", sizeof(CFGR4));
    TRACE("Size of CFGR5: %u bytes\r\n", sizeof(CFGR5));
    TRACE("Size of Group: %u bytes\r\n", sizeof(Group));
    TRACE("Size of Config: %u bytes\r\n", sizeof(Config));
    TRACE("Size of Module: %u bytes\r\n", sizeof(Module));
    TRACE("Size of BMS: %u bytes\r\n", sizeof(BMS));
    TRACE("Size of GROUP: %u bytes\r\n", sizeof(GROUP));
    TRACE("Size of BMS_SEND_CAN_DATA: %u bytes\r\n", sizeof(BMS_SEND_CAN_DATA));
}

/*
 * @brief SPI Send&Receive complete callback
*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        char msg[128];
        snprintf(msg, sizeof(msg), "SPI1 RX done: ");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        for (int i = 0; i < sizeof(spi1RxData); i++) {
            snprintf(msg, sizeof(msg), "%02X ", spi1RxData[i]);
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
}

/*
 * @brief SPI Reception complete callback
*/

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        HAL_SPI_Receive_IT(&hspi2, spi2RxData, sizeof(spi2RxData));
    }
}

/*
 * @brief Print the received CAN data , this fucntion also handles
 * the reception of bms commands  (start/stop sending data)
*/

void vPrintCanRxData()
{
	if(unDataRecieved == 1)
	{
		unDataRecieved = 0;
		if(RxHeader.Identifier == 0xf4)
		{
			if((RxData[0] == 0x1c) && (RxData[1] == 0x00) && (RxData[2] == 0x02) )
			{
				TRACE("BMS SEND DATA COMMAND RECIEVED \r\n");
				bBmsSendData = true;
			}
			else if((RxData[0] == 0x1c) && (RxData[1] == 0x00) && (RxData[2] == 0x01) )
			{
				TRACE("BMS SEND DATA COMMAND RECIEVED \r\n");
				bBmsSendData = false;
			}
			else
			{
				vHandleCommand_Reception(RxData);
			}
		}
		TRACE(" CAN DATA RX DONE\r\n");
		TRACE("DLC:%lu   ID:0X%X  DATA: 0X%x 0X%x 0X%x 0X%x 0X%x 0X%x 0X%x 0X%x\r\n",RxHeader.DataLength ,RxHeader.Identifier,RxData[0],RxData[1],RxData[2],RxData[3],RxData[4],RxData[5],RxData[6],RxData[7]);
	}
}

/*
 * @brief This fucjtions sends a CAN message with a CAN TX IDENTIFIER passed
 * to the fucntion
*/

void vCAN_SendMessage( uint32_t stdId )
{
	 TxHeader.Identifier = stdId;
	 if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)!= HAL_OK)
	 {
		TRACE("CAN MESSAGE SENDING ERROR");
	    Error_Handler();
	 }
	 else
	 {
//				TRACE("CAN DATA TX DONE \r\n");
	 }
}

/*
 * @brief this fucntion populates the struct which has all the groups
 * of data to be sent on the CAN (84 groups initially ) , the first byte of
 * each group is the index of the data group
*/

void vPopulateCandataSendingStruct(void)
{
	vSetGroupNumber();
	vPopulateCellsDataGroups();
	vPopulateGroup1();
	vPopulateGroup2();
	vPopulateGroup3();
	vPopulateGroup4();
	vPopulateGroup5();
	vPopulateGroup6();
	vPopulateGroup80();
	vPopulateGroup81();
	vPopulateGroup82();
	vPopulateGroup83();
	vPopulateGroup84();
}

/*
 * @brief this fucntion will set the first byte of each group to the
 * group number (as CAN data is being sent in 84 groups)
*/

void vSetGroupNumber(void)
{
    for (uint8_t i = 0; i < NUMBER_OF_GROUPS; i++)
    {
        memset(candata.group[i].unData, 0x00, 8);
        candata.group[i].unData[0] = i;
    }
}

/*
 * @brief this fucntion will populate the cells data groups starting
 * from group number 7.
*/

void vPopulateCellsDataGroups(void)
{
    for (uint8_t i = 0; i < 64; i++)  // 64 active voltage groups //192 cells
    {
        uint8_t groupIndex = i + 7;
        uint8_t cellStartIndex = i * NUMBER_OF_CELLS_PER_GROUP;

        for (uint8_t j = 0; j < NUMBER_OF_CELLS_PER_GROUP; j++)
        {
            uint16_t voltage = bmsdata.udRawCellVoltages[cellStartIndex + j];
            candata.group[groupIndex].unData[1 + j * 2] = voltage >> 8;
            candata.group[groupIndex].unData[2 + j * 2] = voltage & 0xFF;
        }
    }
}

/*
 * @brief  these functions (vPopulateGroupX) will populate the groups for sending
 * via the CANBUS first byte of every group is the header / identifier
 * of the data and it has already been set for all the groups in in the
 * vSetGroupNumber() function
*/

void vPopulateGroup1(void)
{
	candata.group[1].unData[1] = canparam.udOverDischargeVoltageProtectionValue >> 8;
	candata.group[1].unData[2] = canparam.udOverDischargeVoltageProtectionValue  & 0xff;
	candata.group[1].unData[3] = canparam.udOverCurrentProtectionValue >> 8;
	candata.group[1].unData[4] = canparam.udOverCurrentProtectionValue & 0xff;
	candata.group[1].unData[5] = canparam.udBatteryCapacity  >> 8;
	candata.group[1].unData[6] = canparam.udBatteryCapacity & 0xff;
    candata.group[1].unData[7] = 0x00;
}

void vPopulateGroup2(void)
{
	candata.group[2].unData[1] = canparam.udNumberOfBatteryStrings >> 8;
	candata.group[2].unData[2] = canparam.udNumberOfBatteryStrings & 0xff;
	candata.group[2].unData[3] = canparam.udOverChargeVoltageProtectionValue >> 8;
	candata.group[2].unData[4] = canparam.udOverChargeVoltageProtectionValue & 0xff;
	candata.group[2].unData[5] = canparam.udOverTemperatureProtectionValue  >> 8;
	candata.group[2].unData[6] = canparam.udOverTemperatureProtectionValue & 0xff;
    candata.group[2].unData[7] = 0x00;
}

void vPopulateGroup3(void)
{
	candata.group[3].unData[1] = bms_calc.udTotalVoltage >> 8;
	candata.group[3].unData[2] = bms_calc.udTotalVoltage & 0xFF;
	candata.group[3].unData[3] = bms_calc.udTotalCurrent >> 8;
	candata.group[3].unData[4] = bms_calc.udTotalCurrent & 0xFF;
	candata.group[3].unData[5] = bms_calc.udTotalPower >> 8;
	candata.group[3].unData[6] = bms_calc.udTotalPower & 0xFF;
    candata.group[3].unData[7] = 0x00;
}

void vPopulateGroup4(void)
{

    candata.group[4].unData[1] = bms_calc.udBatteryUsageCapacity >> 8;
    candata.group[4].unData[2] = bms_calc.udBatteryUsageCapacity & 0xFF;
    candata.group[4].unData[3] = bms_calc.udBatteryCapacityPercentage >> 8;
    candata.group[4].unData[4] = bms_calc.udBatteryCapacityPercentage & 0xFF;
    candata.group[4].unData[5] = bms_calc.udChargingCapacity >> 8;
    candata.group[4].unData[6] = bms_calc.udChargingCapacity & 0xFF;
    candata.group[4].unData[7] = 0x00;
}

void vPopulateGroup5(void)
{
    candata.group[5].unData[1] = canparam.udOverChargeRecoveryVoltageValue >> 8;
    candata.group[5].unData[2] = canparam.udOverChargeRecoveryVoltageValue & 0xFF;
    candata.group[5].unData[3] = canparam.udDischargeProtectionRecoveryVoltageValue >> 8;
    candata.group[5].unData[4] = canparam.udDischargeProtectionRecoveryVoltageValue & 0xFF;
    candata.group[5].unData[5] = bms_calc.udRemainingCapacity >> 8;
    candata.group[5].unData[6] = bms_calc.udRemainingCapacity & 0xFF;
    candata.group[5].unData[7] = 0x00;
}

void vPopulateGroup6(void)
{
    candata.group[6].unData[1] = bms_calc.udHostTemperature >> 8;
    candata.group[6].unData[2] = bms_calc.udHostTemperature & 0xFF;
    candata.group[6].unData[3] = bms_calc.udStatusAccounting >> 8;
    candata.group[6].unData[4] = bms_calc.udStatusAccounting & 0xFF;
    candata.group[6].unData[5] = canparam.udBalanceStartVoltageDuringCharging >> 8;
    candata.group[6].unData[6] = canparam.udBalanceStartVoltageDuringCharging & 0xFF;
    candata.group[6].unData[7] = 0x00;
}

void vPopulateGroup80(void)
{
    candata.group[80].unData[1] = bms_calc.udLowVoltagePowerOutageProtection >> 8;
    candata.group[80].unData[2] = bms_calc.udLowVoltagePowerOutageProtection & 0xFF;
    candata.group[80].unData[3] = bms_calc.udLowVoltagePowerOutageDelayed >> 8;
    candata.group[80].unData[4] = bms_calc.udLowVoltagePowerOutageDelayed & 0xFF;
    candata.group[80].unData[5] = bms_calc.udNumOfTriggeringProtectionCells >> 8;
    candata.group[80].unData[6] = bms_calc.udNumOfTriggeringProtectionCells & 0xFF;
    candata.group[80].unData[7] = 0x00;
}

void vPopulateGroup81(void)
{
    candata.group[81].unData[1] = bms_calc.udBalancedReferenceVoltage >> 8;
    candata.group[81].unData[2] = bms_calc.udBalancedReferenceVoltage & 0xFF;
    candata.group[81].unData[3] = bms_calc.udMinimumVoltage >> 8;
    candata.group[81].unData[4] = bms_calc.udMinimumVoltage & 0xFF;
    candata.group[81].unData[5] = bms_calc.udMaximumVoltage >> 8;
    candata.group[81].unData[6] = bms_calc.udMaximumVoltage & 0xFF;
    candata.group[81].unData[7] = bms_calc.unChargingAndDischargingMOSStatus; // 8-bit
}

void vPopulateGroup82(void)
{
    candata.group[82].unData[1] = (bms_calc.udAccumulatedTotalCapacity >> 24) & 0xFF;
    candata.group[82].unData[2] = (bms_calc.udAccumulatedTotalCapacity >> 16) & 0xFF;
    candata.group[82].unData[3] = (bms_calc.udAccumulatedTotalCapacity >> 8) & 0xFF;
    candata.group[82].unData[4] = bms_calc.udAccumulatedTotalCapacity & 0xFF;
    candata.group[82].unData[5] = canparam.udPreChargeDelayTime >> 8;
    candata.group[82].unData[6] = canparam.udPreChargeDelayTime & 0xFF;
    candata.group[82].unData[7] = bms_calc.unLCDStatus; // 8-bit
}
void vPopulateGroup83(void)
{
    candata.group[83].unData[1] = canparam.udDifferentialPressureSettingValue >> 8;
    candata.group[83].unData[2] = canparam.udDifferentialPressureSettingValue & 0xFF;
    candata.group[83].unData[3] = canparam.udAutoResetUseCapacitySwitch >> 8;
    candata.group[83].unData[4] = canparam.udAutoResetUseCapacitySwitch & 0xFF;
    candata.group[83].unData[5] = canparam.udLowTempProtectionSettingValue >> 8;
    candata.group[83].unData[6] = canparam.udLowTempProtectionSettingValue & 0xFF;
    candata.group[83].unData[7] = bms_calc.unProtectingHistoricalLogs; // 8-bit
}
void vPopulateGroup84(void)
{
    candata.group[84].unData[1] = canparam.udHallSensorType >> 8;
    candata.group[84].unData[2] = canparam.udHallSensorType & 0xFF;
    candata.group[84].unData[3] = canparam.udFanStartTemperature >> 8;
    candata.group[84].unData[4] = canparam.udFanStartTemperature & 0xFF;
    candata.group[84].unData[5] = canparam.udPTCHeatingStartTemperature >> 8;
    candata.group[84].unData[6] = canparam.udPTCHeatingStartTemperature & 0xFF;
    candata.group[84].unData[7] = canparam.udChannelDefaultStateAfterPowerOn & 0xFF;
}

/*
 * @brief  Assign Dummy values to the bms data to see what the bms is sending
 * on the CANBUS
*/

void init_dummy_bms_data(BMS_DATA_CALCULATIONS *bms)
{
    if (bms == NULL)
    {
        return;
    }

    // Assigning dummy values
    bms->udTotalVoltage = 3600;                    // 3.6V (in millivolts)
    bms->udTotalCurrent = 1000;                    // 1A (in milliamperes)
    bms->udTotalPower = 3600;                      // 3.6W (in milliwatts)
    bms->udBatteryUsageCapacity = 500;             // 500 mAh used
    bms->udBatteryCapacityPercentage = 75;         // 75%
    bms->udChargingCapacity = 200;                 // 200 mAh charged
    bms->udRemainingCapacity = 1500;               // 1500 mAh remaining
    bms->udHostTemperature = 250;                  // 25.0°C (in tenths of a degree)
    bms->udStatusAccounting = 0x1234;              // Arbitrary status value
    bms->udLowVoltagePowerOutageProtection = 2800; // 2.8V (in millivolts)
    bms->udLowVoltagePowerOutageDelayed = 1000;    // 1 second (in milliseconds)
    bms->udNumOfTriggeringProtectionCells = 4;     // 4 cells
    bms->udBalancedReferenceVoltage = 3500;        // 3.5V (in millivolts)
    bms->udMinimumVoltage = 3000;                  // 3.0V (in millivolts)
    bms->udMaximumVoltage = 4200;                  // 4.2V (in millivolts)
    bms->unChargingAndDischargingMOSStatus = 0x01; // Charging on, discharging off
    bms->udAccumulatedTotalCapacity = 100000;      // 100 Ah (in milliampere-hours)
    bms->unLCDStatus = 0x01;                       // LCD on
    bms->unProtectingHistoricalLogs = 0x00;        // No protection logs
}

//TEMPLATE
//void vPopulateGroupX(void)
//{
//	candata.group[1].unData[1] = canparam. >> 8;
//	candata.group[1].unData[2] = canparam. & 0xff;
//	candata.group[1].unData[3] = canparam. >> 8;
//	candata.group[1].unData[4] = canparam. & 0xff;
//	candata.group[1].unData[5] = canparam.  >> 8;
//	candata.group[1].unData[6] = canparam. & 0xff;
//}

/*
 * @brief this fucntion will send all the populated groups of data on the CANBUS
*/

void vSendAllCANGroups(void)
{
    for (uint8_t i = 0; i < NUMBER_OF_GROUPS; i++)
    {
        memcpy(TxData, candata.group[i].unData, 8);
        vCAN_SendMessage(0xf5);
//        TRACE("CAN MESSAGE SENT %u  \r\n", i );
        HAL_Delay(1);
    }
}

/*
 * @brief To print all the populated can_data_sending_struct.
 * this will print all the 84 groups of data ,exactly as they will be
 * sent on the CANBUS
*/

void vPrintCandataGroups(void)
{
    char msg[128];

    for (uint8_t i = 0; i < NUMBER_OF_GROUPS; i++) {
        snprintf(msg, sizeof(msg), "Group %u: ", i);
        TRACE("%s", msg);

        for (uint8_t k = 0; k < 8; k++) {
            snprintf(msg, sizeof(msg), "%02X ", candata.group[i].unData[k]);
            TRACE("%s", msg);
        }
        TRACE("\r\n");
    }
}


/*
 * @brief Calculate the total battery Voltage
*/

float vCalculateTotalVoltage(void)
{
     static float total_voltage = 0.0f;

    for (uint8_t i = 0; i < NUM_CELLS; i++)
    {
        if (bmsdata.fCellVoltage[i] < 0.6f || bmsdata.fCellVoltage[i] > 5.0f)
        {

        }
        total_voltage += bmsdata.fCellVoltage[i];
    }
    bmsdata.fTotalVoltage = total_voltage;
    return total_voltage;

}


/*
 * @brief Get the maximum cell voltage and set overcharge protection flag
 * @details If max cell voltage exceeds fChargeprotectionLimit (initially 3.65V),
 *          set bmsdata.ov_fault to 1, otherwise 0
 * @return Maximum cell voltage, or -1.0f if invalid
 */
float fChargeProtection(void)
{
    float max_voltage = 0.0f;

    for (uint8_t i = 0; i < NUM_CELLS; i++)
    {
        if (bmsdata.fCellVoltage[i] < 0.0f || bmsdata.fCellVoltage[i] > 5.0f)
        {
        	bmsdata.unCellProtectionLimit = 1;
            return -1.0f;
        }
        if (bmsdata.fCellVoltage[i] > max_voltage)
        {
            max_voltage = bmsdata.fCellVoltage[i];
        }
    }

    if (max_voltage > fChargeprotectionLimit)
    {
        bmsdata.unCellChargeProtetionLimit = 1;
    }
    else
    {
        bmsdata.unCellChargeProtetionLimit = 0;
    }

    return max_voltage;
}

/*
 * @brief Get the minimum cell voltage and set discharge protection flag
 * @details If min cell voltage is below fDisChargeprotectionLimit (initially 2.70V),
 *          set bmsdata.uv_fault to 1, otherwise 0
 * @return Minimum cell voltage, or -1.0f if invalid
 */
float fDisChargeProtection(void)
{
    float min_voltage = fChargeprotectionLimit; // Initialize to a high value
    unNumberOfConnectedCells = 0;
    for (uint8_t i = 0; i < NUM_CELLS; i++)
    {
    	if (bmsdata.fCellVoltage[i] < 0.0f || bmsdata.fCellVoltage[i] > 5.0f)
        {
    		bmsdata.unCellProtectionLimit = 1;
            return -1.0f;
        }
        if (bmsdata.fCellVoltage[i] == 0.0f)
        {
        	unNumberOfConnectedCells++;
        }
        else
        {
        	min_voltage = bmsdata.fCellVoltage[i];
        }
    }
    unNumberOfConnectedCells = NUM_CELLS - unNumberOfConnectedCells;

    if (min_voltage > 0.6 && min_voltage < fDisChargeprotectionLimit)
    {
        bmsdata.unCellDisChargeProtetionLimit= 1;
    }
    else
    {
    	bmsdata.unCellDisChargeProtetionLimit= 0;
    }

    return min_voltage;
}

/*
 * @brief BMS protections wrapper, this fucntion handles
 * all the protection and trips the bms if any protection
 * is triggered
*/
void vProtections()
{
	fChargeProtection();
	fDisChargeProtection();

	bmsdata.bProtection = false;
	if(bmsdata.unCellProtectionLimit == LIMIT_TRIGGERED )
	{
		TRACE("Cell Protection Limit\r\n ");
		bmsdata.bProtection = true;
	}
	if(bmsdata.unCellChargeProtetionLimit== LIMIT_TRIGGERED )
	{
		TRACE("Cell Charge Protection Limit\r\n ");
		bmsdata.bProtection = true;
	}
	if(bmsdata.unCellDisChargeProtetionLimit == LIMIT_TRIGGERED )
	{
		TRACE("Cell DisCharge Protection Limit\r\n ");
		bmsdata.bProtection = true;
	}
}

/*
 * @brief Set the GPIO PIN for the REALY to HIGH
*/
void vTurnONRelay()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	TRACE("RELAY TURNED ON \r\n");
}

/*
 * @brief Set the GPIO PIN for the REALY to LOW
*/
void vTurnOFFRelay()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	TRACE("RELAY TURNED OFF \r\n");
}

/*
 * @brief Print Firmware Version ,Date and Time
*/
void vPrintFirmwareInfo(uint8_t MajorVersion,uint8_t MinorVersion,uint8_t Build,uint8_t Day,uint8_t Month,uint8_t Hour,uint8_t Minute)
{
	vPrintFirmwareVersion(MajorVersion, MinorVersion, Build);
	vPrintFirmwareDate(Day, Month);
	vPrintFirmwareTime(Hour, Minute);
}

/*
 * @brief Print Firmware Version
*/
void vPrintFirmwareVersion(uint8_t MajorVersion,uint8_t MinorVersion,uint8_t Build)
{
	TRACE("FIRMWARE VERSION %u.%u.%u \r\n" ,MajorVersion,MinorVersion,Build);
}

/*
 * @brief Print Date
*/

void vPrintFirmwareDate(uint8_t Day,uint8_t Month)
{
	TRACE("DATE %u/%u \r\n" ,Day,Month);
}

/*
 * @brief Print Time
*/

void vPrintFirmwareTime(uint8_t Hour,uint8_t Minute)
{
	TRACE("TIME %u:%u \r\n" ,Hour,Minute);
}

/*
 * @brief Print BMS parameters that will be calculate inside our
 * state-machine
*/

void vPrintBmsCalcData(void)
{
    TRACE("\r\n---- BMS Calculated Data ----\r\n");
    // Group 3
    TRACE("Total Voltage (V)                     = %.2f\r\n", bms_calc.udTotalVoltage / 100.0f);
    TRACE("Total Current (A)                     = %.2f\r\n", bms_calc.udTotalCurrent / 100.0f);
    TRACE("Total Power (W)                       = %.2f\r\n", bms_calc.udTotalPower / 10.0f);
    // Group 4
    TRACE("Battery Usage Capacity (Ah)          = %.2f\r\n", bms_calc.udBatteryUsageCapacity / 100.0f);
    TRACE("Battery Capacity Percentage (%%)      = %u%%\r\n", bms_calc.udBatteryCapacityPercentage);
    TRACE("Charging Capacity (Ah)               = %.2f\r\n", bms_calc.udChargingCapacity / 100.0f);
    // Group 5
    TRACE("Remaining Capacity (Ah)              = %.2f\r\n", bms_calc.udRemainingCapacity / 100.0f);
    // Group 6
    TRACE("Host Temperature (°C)                = %.1f\r\n", bms_calc.udHostTemperature / 10.0f);
    TRACE("Status Accounting (bitfield)         = 0x%04X\r\n", bms_calc.udStatusAccounting);
    // Group 80
    TRACE("Low Voltage Power Outage Prot. (V)   = %.2f\r\n", bms_calc.udLowVoltagePowerOutageProtection / 100.0f);
    TRACE("Low Voltage Power Outage Delay (s)   = %u\r\n", bms_calc.udLowVoltagePowerOutageDelayed);
    TRACE("Triggered Protection Cells Count     = %u\r\n", bms_calc.udNumOfTriggeringProtectionCells);
    // Group 81
    TRACE("Balanced Reference Voltage (mV)      = %u\r\n", bms_calc.udBalancedReferenceVoltage);
    TRACE("Minimum Cell Voltage (V)             = %.3f\r\n", bms_calc.udMinimumVoltage / 1000.0f);
    TRACE("Maximum Cell Voltage (V)             = %.3f\r\n", bms_calc.udMaximumVoltage / 1000.0f);
    TRACE("Charging/Discharging MOS Status      = 0x%02X\r\n", bms_calc.unChargingAndDischargingMOSStatus);
    // Group 82
    TRACE("Accumulated Total Capacity (Ah)      = %.2f\r\n", bms_calc.udAccumulatedTotalCapacity / 100.0f);
    TRACE("LCD Status                           = 0x%02X\r\n", bms_calc.unLCDStatus);
    // Group 83
    TRACE("Protecting Historical Logs Flag      = 0x%02X\r\n", bms_calc.unProtectingHistoricalLogs);
    TRACE("---- End of Calculated Data ----\r\n\r\n");
}

/*
 * @brief Print all the configuration parameters of the BMS
 * This will print the elements of the struct which has all
 * the parameters
*/

void vPrintCandataParams(void)
{
    TRACE("\r\n---- BMS Configuration Parameters ----\r\n");
    TRACE("0x01: OverDischargeVoltageProtectionValue       = 0x%04X\r\n", canparam.udOverDischargeVoltageProtectionValue);
    TRACE("0x02: OverCurrentProtectionValue                = 0x%04X\r\n", canparam.udOverCurrentProtectionValue);
    TRACE("0x03: BatteryCapacity                           = 0x%04X\r\n", canparam.udBatteryCapacity);
    TRACE("0x04: NumberOfBatteryStrings                    = 0x%04X\r\n", canparam.udNumberOfBatteryStrings);
    TRACE("0x05: OverChargeVoltageProtectionValue          = 0x%04X\r\n", canparam.udOverChargeVoltageProtectionValue);
    TRACE("0x06: OverTemperatureProtectionValue            = 0x%04X\r\n", canparam.udOverTemperatureProtectionValue);
    TRACE("0x07: ControlChannelState                       = 0x%04X\r\n", canparam.udControlChannelState);
    TRACE("0x08: AutoBalanceState                          = 0x%04X\r\n", canparam.udAutoBalanceState);
    TRACE("0x09: DischargeCapacityResetFlag                = 0x%04X\r\n", canparam.udDischargeCapacityResetFlag);
    TRACE("0x0A: OverChargeRecoveryVoltageValue            = 0x%04X\r\n", canparam.udOverChargeRecoveryVoltageValue);
    TRACE("0x0B: DischargeProtectionRecoveryVoltageValue   = 0x%04X\r\n", canparam.udDischargeProtectionRecoveryVoltageValue);
    TRACE("0x0C: ChannelDefaultStateAfterPowerOn           = 0x%04X\r\n", canparam.udChannelDefaultStateAfterPowerOn);
    TRACE("0x0D: HostDisplaySwitch                         = 0x%04X\r\n", canparam.udHostDisplaySwitch);
    TRACE("0x0E: HostVoltagePowerOffVoltageValue           = 0x%04X\r\n", canparam.udHostVoltagePowerOffVoltageValue);
    TRACE("0x0F: HostVoltagePowerOffDelay                  = 0x%04X\r\n", canparam.udHostVoltagePowerOffDelay);
    TRACE("0x10: CommunicationConnectedFlag                = 0x%04X\r\n", canparam.udCommunicationConnectedFlag);
    TRACE("0x11: BalanceStartVoltageDuringCharging         = 0x%04X\r\n", canparam.udBalanceStartVoltageDuringCharging);
    TRACE("0x12: ClearTotalDischargeCapacityFlag           = 0x%04X\r\n", canparam.udClearTotalDischargeCapacityFlag);
    TRACE("0x13: ClearHistoricalLogsFlag                   = 0x%04X\r\n", canparam.udClearHistoricalLogsFlag);
    TRACE("0x14: ManualUsageCapacityAh                     = 0x%04X\r\n", canparam.udManualUsageCapacityAh);
    TRACE("0x15: AutoResetUseCapacitySwitch                = 0x%04X\r\n", canparam.udAutoResetUseCapacitySwitch);
    TRACE("0x16: PreChargeDelayTime                        = 0x%04X\r\n", canparam.udPreChargeDelayTime);
    TRACE("0x17: DifferentialPressureSettingValue          = 0x%04X\r\n", canparam.udDifferentialPressureSettingValue);
    TRACE("0x18: LowTempProtectionSettingValue             = 0x%04X\r\n", canparam.udLowTempProtectionSettingValue);
    TRACE("0x19: HallSensorType                            = 0x%04X\r\n", canparam.udHallSensorType);
    TRACE("0x1A: FanStartTemperature                       = 0x%04X\r\n", canparam.udFanStartTemperature);
    TRACE("0x1B: PTCHeatingStartTemperature                = 0x%04X\r\n", canparam.udPTCHeatingStartTemperature);
    TRACE("0x1C: CANCommunicationStatus                    = 0x%04X\r\n", canparam.udCANCommunicationStatus);
    TRACE("---- End of Parameters ----\r\n\r\n");
}

/*
 * @brief This fucntion will handle the commands reception
 * and population , it will recieve a command from the use
 * via CANBUS adn update the appropriate parameter on the basis
 * of the index (first byte of the data)
*/

void vHandleCommand_Reception(uint8_t *data)
{
    if (data == NULL)
        return;

    uint8_t index = data[0];
    if (index == 0 || index > 28)
    {
        TRACE("Invalid command index: 0x%02X\r\n", index);
        return;
    }
    uint16_t value = ((uint16_t)data[1] << 8) | data[2];
    TRACE("Command index 0x%02X received with value: 0x%04X\r\n", index, value);
    UpdateParameter(index, value);
    TRACE(" Flash Memory Upated \r\n");
}

/*
 * @brief Find the page of the memory sector to write the
 * data of the params struct
*/

uint32_t FindLatestPage(void)
{
    FlashPage *pages = (FlashPage *)FLASH_STORAGE_ADDR;
    uint32_t latestSeq = 0;
    uint32_t latestIdx = 0;

    for (uint32_t i = 0; i < NUM_PAGES; i++)
    {
        if (pages[i].magic == 0xFFFFFFFF)
        {
            return (i == 0) ? 0 : i - 1;  // Return last valid page
        }
        if (pages[i].magic == MAGIC_NUMBER && pages[i].sequence > latestSeq)
        {
            latestSeq = pages[i].sequence;
            latestIdx = i;
        }
    }
    return latestIdx;  // Sector full, return last page
}

/*
 * @brief Load data from the flash memory
 * to populate the can param struct.
*/
void LoadFromFlash(void)
{
    uint32_t idx = FindLatestPage();
    FlashPage *page = (FlashPage *)(FLASH_STORAGE_ADDR + idx * PAGE_SIZE);

    if (page->magic == MAGIC_NUMBER)
    {
        memcpy(&canparam, &page->data, sizeof(BMS_COMMAND_RECEPTION));
    }
    else
    {
        // Initialize with defaults if no valid data
        memset(&canparam, 0, sizeof(BMS_COMMAND_RECEPTION));
        SaveToFlash(0);  // Start with page 0
    }
}

/*
 * @brief Save data to next page on the flash.
*/
HAL_StatusTypeDef SaveToFlash(uint32_t currentIdx)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0;
    static uint32_t sequence = 0;
    uint32_t nextIdx = (currentIdx + 1) % NUM_PAGES;

    // Check if sector is full (next page is used or wrapping to 0)
    FlashPage *nextPage = (FlashPage *)(FLASH_STORAGE_ADDR + nextIdx * PAGE_SIZE);
    if (nextIdx == 0 || nextPage->magic != 0xFFFFFFFF)
    {
        // Erase Sector 7 of Bank 2 when full
        status = HAL_FLASH_Unlock();
        if (status != HAL_OK) return status;

        eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseInit.Banks = FLASH_BANK_2;
        eraseInit.Sector = FLASH_SECTOR_7;  // Sector 7 in Bank 2
        eraseInit.NbSectors = 1;
        status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);  // Fixed: Use sectorError
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
        sequence = 0;  // Reset sequence after erase
        nextIdx = 0;   // Start from page 0
        HAL_FLASH_Lock();
    }

    // Write to the next page
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return status;

    FlashPage newPage;
    newPage.magic = MAGIC_NUMBER;
    newPage.sequence = ++sequence;  // Increment sequence for each write
    newPage.data = canparam;

    uint8_t *data = (uint8_t *)&newPage;
    uint32_t addr = FLASH_STORAGE_ADDR + nextIdx * PAGE_SIZE;
    for (uint32_t i = 0; i < sizeof(FlashPage); i += 32)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + i, (uint32_t)(data + i));
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

/*
 * @brief Update the parameter if changed and then save it
 * to the next page on the flash memory.
*/
void UpdateParameter(uint16_t index, uint16_t value)
{
    // Adjust index: Convert 1-based index to 0-based
    if (index >= 1 && index <= 28)
    {
        uint8_t internalIndex = index - 1;
        BMS_COMMAND_RECEPTION old = canparam;
        canparam.audCommandDataArray[internalIndex] = value;
        if (memcmp(&old, &canparam, sizeof(BMS_COMMAND_RECEPTION)) != 0)
        {
            uint32_t currentIdx = FindLatestPage();
            if (SaveToFlash(currentIdx) != HAL_OK)
            {
                TRACE("ERROR UPDATING THE FLASH MEMORY (UPDATE PARAM FAILED)\r\n");
            }
        }
    }
    else
    {
        TRACE("Invalid index in UpdateParameter(): %d\r\n", index);
    }
}
static void trim(char *str)
{
    char *start = str;
    // Skip leading whitespace
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    // Find end and trim trailing whitespace
    char *end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
    // Move trimmed string to start
    memmove(str, start, strlen(start) + 1);
}

// send_cell_data function (unchanged)
void send_cell_data(struct tcp_pcb *tpcb)
{
    if (tpcb == NULL || tpcb != active_tpcb)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"No active connection\n", 21, HAL_MAX_DELAY);
        return;
    }

    char buffer[TX_BUFFER_SIZE]; // Ensure TX_BUFFER_SIZE is at least 1000
    int len = 0;

    len += snprintf(buffer + len, sizeof(buffer) - len, "CELL_DATA:");
    if (len >= sizeof(buffer))
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Buffer overflow at header\n", 26, HAL_MAX_DELAY);
        return;
    }

    for (int i = 0; i < 93; i++)
    {
        char temp[16];
        int temp_len = snprintf(temp, sizeof(temp), "%.4f%s", bmsdata.fCellVoltage[i], (i < 93) ? "," : "");
        if (temp_len < 0 || temp_len >= sizeof(temp))
        {
            HAL_UART_Transmit(&huart3, (uint8_t*)"Temp buffer error\n", 18, HAL_MAX_DELAY);
            return;
        }
        if (len + temp_len < sizeof(buffer))
        {
            memcpy(buffer + len, temp, temp_len);
            len += temp_len;
        }
        else
        {
            HAL_UART_Transmit(&huart3, (uint8_t*)"Buffer overflow at data\n", 24, HAL_MAX_DELAY);
            return;
        }
    }

    if (len < sizeof(buffer) - 1)
    {
        buffer[len++] = '\n';
        buffer[len] = '\0';
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Buffer overflow at newline\n", 27, HAL_MAX_DELAY);
        return;
    }

    char len_msg[32];
//    snprintf(len_msg, sizeof(len_msg), "Data length: %d\n", len);
//    HAL_UART_Transmit(&huart3, (uint8_t*)len_msg, strlen(len_msg), HAL_MAX_DELAY);

    if (tcp_sndbuf(tpcb) <= len)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"TCP send buffer too small\n", 26, HAL_MAX_DELAY);
        return;
    }

    err_t err = tcp_write(tpcb, buffer, len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK)
    {
        char err_msg[32];
        snprintf(err_msg, sizeof(err_msg), "tcp_write failed: %d\n", err);
        HAL_UART_Transmit(&huart3, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
        return;
    }
    tcp_output(tpcb);
    send_charge_discharge_limits();
}

// handle_led_command function (unchanged)
static void handle_led_command(char *data, char *response, int *len)
{
    if (strcmp(data, "ON") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn LED on
        *len = snprintf(response, 32, "LED turned ON");
    }
    else if (strcmp(data, "OFF") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn LED off
        *len = snprintf(response, 32, "LED turned OFF");
    }
    else {
        *len = 0; // No match, let handle_custom_value process it
    }
}

// handle_custom_value function (unchanged)
static void handle_custom_value(char *data, char *response, int *len)
{
    // Check if the input is a valid digit string
    int i = 0;
    while (data[i] != '\0') {
        if (!isdigit((unsigned char)data[i])) {
            *len = snprintf(response, 32, "Invalid value: %s", data);
            return;
        }
        i++;
    }
    *len = snprintf(response, 32, "Received value: %s", data);
}

// handle_extended function (unchanged)
void handle_extended(uint8_t header, uint16_t value, char *resp, int *len)
{
    char uart_buf[64];
    int uart_len = snprintf(uart_buf, sizeof(uart_buf), "Received extended: header=0x%02X, value=0x%04X\r\n", header, value);
    HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, HAL_MAX_DELAY);

    switch (header) {
        case 0x01: *len = snprintf(resp, 32, "0x01=%04X", value); break;
        case 0x02: *len = snprintf(resp, 32, "0x02=%04X", value); break;
        case 0x03: *len = snprintf(resp, 32, "0x03=%04X", value); break;
        case 0x04: *len = snprintf(resp, 32, "0x04=%04X", value); break;
        case 0x05: *len = snprintf(resp, 32, "0x05=%04X", value); break;
        case 0x06: *len = snprintf(resp, 32, "0x06=%04X", value); break;
        case 0x07: *len = snprintf(resp, 32, "0x07=%04X", value); break;
        case 0x08: *len = snprintf(resp, 32, "0x08=%04X", value); break;
        case 0x09: *len = snprintf(resp, 32, "0x09=%04X", value); break;
        case 0x0A: *len = snprintf(resp, 32, "0x0A=%04X", value); break;
        case 0x0B: *len = snprintf(resp, 32, "0x0B=%04X", value); break;
        case 0x0C: *len = snprintf(resp, 32, "0x0C=%04X", value); break;
        case 0x0D: *len = snprintf(resp, 32, "0x0D=%04X", value); break;
        case 0x0E: *len = snprintf(resp, 32, "0x0E=%04X", value); break;
        case 0x0F: *len = snprintf(resp, 32, "0x0F=%04X", value); break;
        case 0x10: *len = snprintf(resp, 32, "0x10=%04X", value); break;
        case 0x11: *len = snprintf(resp, 32, "0x11=%04X", value); break;
        case 0x12: *len = snprintf(resp, 32, "0x12=%04X", value); break;
        case 0x13: *len = snprintf(resp, 32, "0x13=%04X", value); break;
        case 0x14: *len = snprintf(resp, 32, "0x14=%04X", value); break;
        case 0x15: *len = snprintf(resp, 32, "0x15=%04X", value); break;
        case 0x16: *len = snprintf(resp, 32, "0x16=%04X", value); break;
        case 0x17: *len = snprintf(resp, 32, "0x17=%04X", value); break;
        case 0x18: *len = snprintf(resp, 32, "0x18=%04X", value); break;
        case 0x19: *len = snprintf(resp, 32, "0x19=%04X", value); break;
        case 0x1A: *len = snprintf(resp, 32, "0x1A=%04X", value); break;
        case 0x1B: *len = snprintf(resp, 32, "0x1B=%04X", value); break;
        case 0x1C: *len = snprintf(resp, 32, "0x1C=%04X", value); break;
        default:
            *len = snprintf(resp, 32, "Unknown 0x%02X", header);
            break;
    }
}

// Modified tcp_server_accept to support persistent connection
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    if (err != ERR_OK || newpcb == NULL)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Accept failed\n", 14, HAL_MAX_DELAY);
        return ERR_VAL;
    }
    if (active_tpcb != NULL)
    {
        tcp_close(active_tpcb);
    }
    active_tpcb = newpcb;
    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, tcp_server_recv);
    tcp_err(newpcb, tcp_server_error);
    tcp_sent(newpcb, tcp_server_sent);
    tcp_poll(newpcb, tcp_server_poll, 4); // Poll every 4 * 250ms = 1 second
//    HAL_UART_Transmit(&huart3, (uint8_t*)"Client connected\n", 17, HAL_MAX_DELAY);
    return ERR_OK;
}
static void handle_balance_command(char *data, char *response, int *len)
{
    if (strcmp(data, "BALANCE_ON") == 0)
    {
        unBalancing = 1;
        *len = snprintf(response, 32, "Balance turned ON");
    }
    else if (strcmp(data, "BALANCE_OFF") == 0)
    {
        unBalancing = 0;
        *len = snprintf(response, 32, "Balance turned OFF");
    }
    else
    {
        *len = 0; // No match, let handle_custom_value process it
    }
}

void trace_eth(const char *format, ...)
{
    if (active_tpcb == NULL || active_tpcb->state != ESTABLISHED)
        return;

    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsprintf(buffer, format, args);
    va_end(args);

    if (len < 0 || len >= sizeof(buffer))
        return;

    // Add "LOG:" prefix and newline
    char log_buffer[sizeof(buffer) + 5];
    snprintf(log_buffer, sizeof(log_buffer), "LOG:%s\n", buffer);

    if (tcp_sndbuf(active_tpcb) <= strlen(log_buffer))
        return;

    err_t err = tcp_write(active_tpcb, log_buffer, strlen(log_buffer), TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK)
        return;

    tcp_output(active_tpcb);
}

void send_charge_discharge_limits(void)
{
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "LIMITS:%.2f;%.2f\n", fChargeprotectionLimit, fDisChargeprotectionLimit);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Send to Ethernet using the existing trace_eth logic
    if (active_tpcb != NULL && active_tpcb->state == ESTABLISHED)
    {
        if (tcp_sndbuf(active_tpcb) > strlen(buffer))
        {
            err_t err = tcp_write(active_tpcb, buffer, strlen(buffer), TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK)
            {
                tcp_output(active_tpcb);
            }
        }
    }
}

// Modified tcp_server_recv to support persistent connection
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Connection closed by client\n", 27, HAL_MAX_DELAY);
        if (tpcb == active_tpcb)
        {
            tcp_close(tpcb);
            active_tpcb = NULL;
        }
        return ERR_OK;
    }

    if (err != ERR_OK)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Receive error\n", 14, HAL_MAX_DELAY);
        pbuf_free(p);
        return err;
    }

    // Ensure data is null-terminated and within bounds
    char *data = (char*)p->payload;
    if (p->len >= 128)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Received data too long\n", 23, HAL_MAX_DELAY);
        pbuf_free(p);
        return ERR_OK;
    }
    data[p->len] = '\0';
    trim(data);

    HAL_UART_Transmit(&huart3, (uint8_t*)"Received: ", 10, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)"\n", 1, HAL_MAX_DELAY);

    char response[128];
    int len = 0;

    if (strcmp(data, "get_counter") == 0)
    {
        len = snprintf(response, sizeof(response), "%d", sensor_value);
    }
    else if (strcmp(data, "get_cell_data") == 0)
    {
        send_cell_data(tpcb);
        pbuf_free(p);
        return ERR_OK; // Data already sent, no further response needed
    }
    else if (strcmp(data, "ON") == 0 || strcmp(data, "OFF") == 0)
    {
        handle_led_command(data, response, &len);
    }
    else if (strcmp(data, "BALANCE_ON") == 0 || strcmp(data, "BALANCE_OFF") == 0)
    {
        handle_balance_command(data, response, &len);
    }
    else if (strlen(data) == 6 && isxdigit(data[0]) && isxdigit(data[1]) &&
             isxdigit(data[2]) && isxdigit(data[3]) && isxdigit(data[4]) && isxdigit(data[5]))
    {
        char header_str[3] = { data[0], data[1], '\0' };
        char value_str[5] = { data[2], data[3], data[4], data[5], '\0' };
        uint8_t header = (uint8_t)strtol(header_str, NULL, 16);
        uint16_t value = (uint16_t)strtol(value_str, NULL, 16);
        handle_extended(header, value, response, &len);
    }
    else
    {
        handle_custom_value(data, response, &len);
    }

    if (len > 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Sending: ", 9, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, (uint8_t*)response, len, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, (uint8_t*)"\n", 1, HAL_MAX_DELAY);

        err_t write_err = tcp_write(tpcb, response, len, TCP_WRITE_FLAG_COPY);
        if (write_err != ERR_OK)
        {
            HAL_UART_Transmit(&huart3, (uint8_t*)"tcp_write failed\n", 17, HAL_MAX_DELAY);
        }
        else
        {
            tcp_output(tpcb);
            HAL_Delay(50); // Added delay to give client time to read
        }
    }
    pbuf_free(p);
    return ERR_OK;
}

// Added tcp_server_error callback for persistent connection
static void tcp_server_error(void *arg, err_t err)
{
    if (active_tpcb != NULL)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Connection error\n", 17, HAL_MAX_DELAY);
        tcp_close(active_tpcb);
        active_tpcb = NULL;
    }
}

// Added tcp_server_sent callback for persistent connection
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
//    HAL_UART_Transmit(&huart3, (uint8_t*)"Data sent, waiting for next request\n", 35, HAL_MAX_DELAY);
    return ERR_OK;
}

// Added tcp_server_poll callback for persistent connection
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    if (tpcb != active_tpcb)
        return ERR_OK;

    if (tpcb->state == ESTABLISHED)
    {
//        HAL_UART_Transmit(&huart3, (uint8_t*)"Polling, connection alive\n", 26, HAL_MAX_DELAY);
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Connection timed out\n", 21, HAL_MAX_DELAY);
        tcp_close(tpcb);
        active_tpcb = NULL;
    }
    return ERR_OK;
}

// tcp_server_init function (unchanged)
void tcp_server_init(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (pcb == NULL) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"TCP PCB creation failed\n", 24, HAL_MAX_DELAY);
        return;
    }

    err_t err = tcp_bind(pcb, IP_ADDR_ANY, 8080);
    if (err != ERR_OK) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"TCP bind failed: ", 17, HAL_MAX_DELAY);
        char err_buf[16];
        snprintf(err_buf, sizeof(err_buf), "err=%d\n", err);
        HAL_UART_Transmit(&huart3, (uint8_t*)err_buf, strlen(err_buf), HAL_MAX_DELAY);
        tcp_close(pcb);
        return;
    }

    pcb = tcp_listen(pcb);
    if (pcb == NULL) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"TCP listen failed\n", 18, HAL_MAX_DELAY);
        return;
    }

    tcp_accept(pcb, tcp_server_accept);
    HAL_UART_Transmit(&huart3, (uint8_t*)"TCP server started on port 8080\n", 31, HAL_MAX_DELAY);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
