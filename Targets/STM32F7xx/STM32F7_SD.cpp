// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
// Copyright GHI Electronics, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "STM32F7.h"

#ifdef INCLUDE_SD
// sdio
#define SD_DATATIMEOUT                  ((uint32_t)0xFFFFFFFF)

/* Exported constants --------------------------------------------------------*/
#define SDIO_ClockEdge_Rising               ((uint32_t)0x00000000)
#define SDIO_ClockEdge_Falling              ((uint32_t)0x00002000)
#define IS_SDIO_CLOCK_EDGE(EDGE) (((EDGE) == SDIO_ClockEdge_Rising) || \
                                  ((EDGE) == SDIO_ClockEdge_Falling))

#define SDIO_ClockBypass_Disable             ((uint32_t)0x00000000)
#define SDIO_ClockBypass_Enable              ((uint32_t)0x00000400)
#define IS_SDIO_CLOCK_BYPASS(BYPASS) (((BYPASS) == SDIO_ClockBypass_Disable) || \
                                     ((BYPASS) == SDIO_ClockBypass_Enable))

#define SDIO_ClockPowerSave_Disable         ((uint32_t)0x00000000)
#define SDIO_ClockPowerSave_Enable          ((uint32_t)0x00000200)
#define IS_SDIO_CLOCK_POWER_SAVE(SAVE) (((SAVE) == SDIO_ClockPowerSave_Disable) || \
                                        ((SAVE) == SDIO_ClockPowerSave_Enable))

#define SDIO_BusWide_1b                     ((uint32_t)0x00000000)
#define SDIO_BusWide_4b                     ((uint32_t)0x00000800)
#define SDIO_BusWide_8b                     ((uint32_t)0x00001000)
#define IS_SDIO_BUS_WIDE(WIDE) (((WIDE) == SDIO_BusWide_1b) || ((WIDE) == SDIO_BusWide_4b) || \
                                ((WIDE) == SDIO_BusWide_8b))

#define SDIO_HardwareFlowControl_Disable    ((uint32_t)0x00000000)
#define SDIO_HardwareFlowControl_Enable     ((uint32_t)0x00004000)
#define IS_SDIO_HARDWARE_FLOW_CONTROL(CONTROL) (((CONTROL) == SDIO_HardwareFlowControl_Disable) || \
                                                ((CONTROL) == SDIO_HardwareFlowControl_Enable))

#define SDIO_PowerState_OFF                 ((uint32_t)0x00000000)
#define SDIO_PowerState_ON                  ((uint32_t)0x00000003)
#define IS_SDIO_POWER_STATE(STATE) (((STATE) == SDIO_PowerState_OFF) || ((STATE) == SDIO_PowerState_ON))

#define SDIO_IT_CCRCFAIL                    ((uint32_t)0x00000001)
#define SDIO_IT_DCRCFAIL                    ((uint32_t)0x00000002)
#define SDIO_IT_CTIMEOUT                    ((uint32_t)0x00000004)
#define SDIO_IT_DTIMEOUT                    ((uint32_t)0x00000008)
#define SDIO_IT_TXUNDERR                    ((uint32_t)0x00000010)
#define SDIO_IT_RXOVERR                     ((uint32_t)0x00000020)
#define SDIO_IT_CMDREND                     ((uint32_t)0x00000040)
#define SDIO_IT_CMDSENT                     ((uint32_t)0x00000080)
#define SDIO_IT_DATAEND                     ((uint32_t)0x00000100)
#define SDIO_IT_STBITERR                    ((uint32_t)0x00000200)
#define SDIO_IT_DBCKEND                     ((uint32_t)0x00000400)
#define SDIO_IT_CMDACT                      ((uint32_t)0x00000800)
#define SDIO_IT_TXACT                       ((uint32_t)0x00001000)
#define SDIO_IT_RXACT                       ((uint32_t)0x00002000)
#define SDIO_IT_TXFIFOHE                    ((uint32_t)0x00004000)
#define SDIO_IT_RXFIFOHF                    ((uint32_t)0x00008000)
#define SDIO_IT_TXFIFOF                     ((uint32_t)0x00010000)
#define SDIO_IT_RXFIFOF                     ((uint32_t)0x00020000)
#define SDIO_IT_TXFIFOE                     ((uint32_t)0x00040000)
#define SDIO_IT_RXFIFOE                     ((uint32_t)0x00080000)
#define SDIO_IT_TXDAVL                      ((uint32_t)0x00100000)
#define SDIO_IT_RXDAVL                      ((uint32_t)0x00200000)
#define SDIO_IT_SDIOIT                      ((uint32_t)0x00400000)
#define SDIO_IT_CEATAEND                    ((uint32_t)0x00800000)
#define IS_SDIO_IT(IT) ((((IT) & (uint32_t)0xFF000000) == 0x00) && ((IT) != (uint32_t)0x00))

#define IS_SDIO_CMD_INDEX(INDEX)            ((INDEX) < 0x40)

#define SDIO_Response_No                    ((uint32_t)0x00000000)
#define SDIO_Response_Short                 ((uint32_t)0x00000040)
#define SDIO_Response_Long                  ((uint32_t)0x000000C0)
#define IS_SDIO_RESPONSE(RESPONSE) (((RESPONSE) == SDIO_Response_No) || \
                                    ((RESPONSE) == SDIO_Response_Short) || \
                                    ((RESPONSE) == SDIO_Response_Long))

#define SDIO_Wait_No                        ((uint32_t)0x00000000) /*!< SDMMC1 No Wait, TimeOut is enabled */
#define SDIO_Wait_IT                        ((uint32_t)0x00000100) /*!< SDMMC1 Wait Interrupt Request */
#define SDIO_Wait_Pend                      ((uint32_t)0x00000200) /*!< SDMMC1 Wait End of transfer */
#define IS_SDIO_WAIT(WAIT) (((WAIT) == SDIO_Wait_No) || ((WAIT) == SDIO_Wait_IT) || \
                            ((WAIT) == SDIO_Wait_Pend))

#define SDIO_CPSM_Disable                    ((uint32_t)0x00000000)
#define SDIO_CPSM_Enable                     ((uint32_t)0x00000400)
#define IS_SDIO_CPSM(CPSM) (((CPSM) == SDIO_CPSM_Enable) || ((CPSM) == SDIO_CPSM_Disable))

#define SDIO_RESP1                          ((uint32_t)0x00000000)
#define SDIO_RESP2                          ((uint32_t)0x00000004)
#define SDIO_RESP3                          ((uint32_t)0x00000008)
#define SDIO_RESP4                          ((uint32_t)0x0000000C)
#define IS_SDIO_RESP(RESP) (((RESP) == SDIO_RESP1) || ((RESP) == SDIO_RESP2) || \
                            ((RESP) == SDIO_RESP3) || ((RESP) == SDIO_RESP4))

#define IS_SDIO_DATA_LENGTH(LENGTH) ((LENGTH) <= 0x01FFFFFF)

#define SDIO_DataBlockSize_1b               ((uint32_t)0x00000000)
#define SDIO_DataBlockSize_2b               ((uint32_t)0x00000010)
#define SDIO_DataBlockSize_4b               ((uint32_t)0x00000020)
#define SDIO_DataBlockSize_8b               ((uint32_t)0x00000030)
#define SDIO_DataBlockSize_16b              ((uint32_t)0x00000040)
#define SDIO_DataBlockSize_32b              ((uint32_t)0x00000050)
#define SDIO_DataBlockSize_64b              ((uint32_t)0x00000060)
#define SDIO_DataBlockSize_128b             ((uint32_t)0x00000070)
#define SDIO_DataBlockSize_256b             ((uint32_t)0x00000080)
#define SDIO_DataBlockSize_512b             ((uint32_t)0x00000090)
#define SDIO_DataBlockSize_1024b            ((uint32_t)0x000000A0)
#define SDIO_DataBlockSize_2048b            ((uint32_t)0x000000B0)
#define SDIO_DataBlockSize_4096b            ((uint32_t)0x000000C0)
#define SDIO_DataBlockSize_8192b            ((uint32_t)0x000000D0)
#define SDIO_DataBlockSize_16384b           ((uint32_t)0x000000E0)
#define IS_SDIO_BLOCK_SIZE(SIZE) (((SIZE) == SDIO_DataBlockSize_1b) || \
                                  ((SIZE) == SDIO_DataBlockSize_2b) || \
                                  ((SIZE) == SDIO_DataBlockSize_4b) || \
                                  ((SIZE) == SDIO_DataBlockSize_8b) || \
                                  ((SIZE) == SDIO_DataBlockSize_16b) || \
                                  ((SIZE) == SDIO_DataBlockSize_32b) || \
                                  ((SIZE) == SDIO_DataBlockSize_64b) || \
                                  ((SIZE) == SDIO_DataBlockSize_128b) || \
                                  ((SIZE) == SDIO_DataBlockSize_256b) || \
                                  ((SIZE) == SDIO_DataBlockSize_512b) || \
                                  ((SIZE) == SDIO_DataBlockSize_1024b) || \
                                  ((SIZE) == SDIO_DataBlockSize_2048b) || \
                                  ((SIZE) == SDIO_DataBlockSize_4096b) || \
                                  ((SIZE) == SDIO_DataBlockSize_8192b) || \
                                  ((SIZE) == SDIO_DataBlockSize_16384b))

#define SDIO_TransferDir_ToCard             ((uint32_t)0x00000000)
#define SDIO_TransferDir_ToSDIO             ((uint32_t)0x00000002)
#define IS_SDIO_TRANSFER_DIR(DIR) (((DIR) == SDIO_TransferDir_ToCard) || \
                                   ((DIR) == SDIO_TransferDir_ToSDIO))

#define SDIO_TransferMode_Block             ((uint32_t)0x00000000)
#define SDIO_TransferMode_Stream            ((uint32_t)0x00000004)
#define IS_SDIO_TRANSFER_MODE(MODE) (((MODE) == SDIO_TransferMode_Stream) || \
                                     ((MODE) == SDIO_TransferMode_Block))

#define SDIO_DPSM_Disable                    ((uint32_t)0x00000000)
#define SDIO_DPSM_Enable                     ((uint32_t)0x00000001)
#define IS_SDIO_DPSM(DPSM) (((DPSM) == SDIO_DPSM_Enable) || ((DPSM) == SDIO_DPSM_Disable))

#define SDIO_FLAG_CCRCFAIL                  ((uint32_t)0x00000001)
#define SDIO_FLAG_DCRCFAIL                  ((uint32_t)0x00000002)
#define SDIO_FLAG_CTIMEOUT                  ((uint32_t)0x00000004)
#define SDIO_FLAG_DTIMEOUT                  ((uint32_t)0x00000008)
#define SDIO_FLAG_TXUNDERR                  ((uint32_t)0x00000010)
#define SDIO_FLAG_RXOVERR                   ((uint32_t)0x00000020)
#define SDIO_FLAG_CMDREND                   ((uint32_t)0x00000040)
#define SDIO_FLAG_CMDSENT                   ((uint32_t)0x00000080)
#define SDIO_FLAG_DATAEND                   ((uint32_t)0x00000100)
#define SDIO_FLAG_STBITERR                  ((uint32_t)0x00000200)
#define SDIO_FLAG_DBCKEND                   ((uint32_t)0x00000400)
#define SDIO_FLAG_CMDACT                    ((uint32_t)0x00000800)
#define SDIO_FLAG_TXACT                     ((uint32_t)0x00001000)
#define SDIO_FLAG_RXACT                     ((uint32_t)0x00002000)
#define SDIO_FLAG_TXFIFOHE                  ((uint32_t)0x00004000)
#define SDIO_FLAG_RXFIFOHF                  ((uint32_t)0x00008000)
#define SDIO_FLAG_TXFIFOF                   ((uint32_t)0x00010000)
#define SDIO_FLAG_RXFIFOF                   ((uint32_t)0x00020000)
#define SDIO_FLAG_TXFIFOE                   ((uint32_t)0x00040000)
#define SDIO_FLAG_RXFIFOE                   ((uint32_t)0x00080000)
#define SDIO_FLAG_TXDAVL                    ((uint32_t)0x00100000)
#define SDIO_FLAG_RXDAVL                    ((uint32_t)0x00200000)
#define SDIO_FLAG_SDIOIT                    ((uint32_t)0x00400000)
#define SDIO_FLAG_CEATAEND                  ((uint32_t)0x00800000)
#define IS_SDIO_FLAG(FLAG) (((FLAG)  == SDIO_FLAG_CCRCFAIL) || \
                            ((FLAG)  == SDIO_FLAG_DCRCFAIL) || \
                            ((FLAG)  == SDIO_FLAG_CTIMEOUT) || \
                            ((FLAG)  == SDIO_FLAG_DTIMEOUT) || \
                            ((FLAG)  == SDIO_FLAG_TXUNDERR) || \
                            ((FLAG)  == SDIO_FLAG_RXOVERR) || \
                            ((FLAG)  == SDIO_FLAG_CMDREND) || \
                            ((FLAG)  == SDIO_FLAG_CMDSENT) || \
                            ((FLAG)  == SDIO_FLAG_DATAEND) || \
                            ((FLAG)  == SDIO_FLAG_STBITERR) || \
                            ((FLAG)  == SDIO_FLAG_DBCKEND) || \
                            ((FLAG)  == SDIO_FLAG_CMDACT) || \
                            ((FLAG)  == SDIO_FLAG_TXACT) || \
                            ((FLAG)  == SDIO_FLAG_RXACT) || \
                            ((FLAG)  == SDIO_FLAG_TXFIFOHE) || \
                            ((FLAG)  == SDIO_FLAG_RXFIFOHF) || \
                            ((FLAG)  == SDIO_FLAG_TXFIFOF) || \
                            ((FLAG)  == SDIO_FLAG_RXFIFOF) || \
                            ((FLAG)  == SDIO_FLAG_TXFIFOE) || \
                            ((FLAG)  == SDIO_FLAG_RXFIFOE) || \
                            ((FLAG)  == SDIO_FLAG_TXDAVL) || \
                            ((FLAG)  == SDIO_FLAG_RXDAVL) || \
                            ((FLAG)  == SDIO_FLAG_SDIOIT) || \
                            ((FLAG)  == SDIO_FLAG_CEATAEND))

#define IS_SDIO_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFF3FF800) == 0x00) && ((FLAG) != (uint32_t)0x00))

#define IS_SDIO_GET_IT(IT) (((IT)  == SDIO_IT_CCRCFAIL) || \
                            ((IT)  == SDIO_IT_DCRCFAIL) || \
                            ((IT)  == SDIO_IT_CTIMEOUT) || \
                            ((IT)  == SDIO_IT_DTIMEOUT) || \
                            ((IT)  == SDIO_IT_TXUNDERR) || \
                            ((IT)  == SDIO_IT_RXOVERR) || \
                            ((IT)  == SDIO_IT_CMDREND) || \
                            ((IT)  == SDIO_IT_CMDSENT) || \
                            ((IT)  == SDIO_IT_DATAEND) || \
                            ((IT)  == SDIO_IT_STBITERR) || \
                            ((IT)  == SDIO_IT_DBCKEND) || \
                            ((IT)  == SDIO_IT_CMDACT) || \
                            ((IT)  == SDIO_IT_TXACT) || \
                            ((IT)  == SDIO_IT_RXACT) || \
                            ((IT)  == SDIO_IT_TXFIFOHE) || \
                            ((IT)  == SDIO_IT_RXFIFOHF) || \
                            ((IT)  == SDIO_IT_TXFIFOF) || \
                            ((IT)  == SDIO_IT_RXFIFOF) || \
                            ((IT)  == SDIO_IT_TXFIFOE) || \
                            ((IT)  == SDIO_IT_RXFIFOE) || \
                            ((IT)  == SDIO_IT_TXDAVL) || \
                            ((IT)  == SDIO_IT_RXDAVL) || \
                            ((IT)  == SDIO_IT_SDIOIT) || \
                            ((IT)  == SDIO_IT_CEATAEND))

#define IS_SDIO_CLEAR_IT(IT) ((((IT) & (uint32_t)0xFF3FF800) == 0x00) && ((IT) != (uint32_t)0x00))

#define SDIO_ReadWaitMode_CLK               ((uint32_t)0x00000000)
#define SDIO_ReadWaitMode_DATA2             ((uint32_t)0x00000001)
#define IS_SDIO_READWAIT_MODE(MODE) (((MODE) == SDIO_ReadWaitMode_CLK) || \
                                     ((MODE) == SDIO_ReadWaitMode_DATA2))

/* ------------ SDMMC1 registers bit address in the alias region ----------- */

/* --- CLKCR Register ---*/
/* Alias word address of CLKEN bit */
#define CLKCR_OFFSET              (SDMMC1_BASE + 0x04)
#define CLKEN_BitNumber           0x08
//#define CLKCR_CLKEN_BB            (PERIPH_BB_BASE + (CLKCR_OFFSET * 32) + (CLKEN_BitNumber * 4))

/* --- CMD Register ---*/
/* Alias word address of SDIOSUSPEND bit */
#define CMD_OFFSET                (SDMMC1_BASE + 0x0C)
#define SDIOSUSPEND_BitNumber     0x0B
//#define CMD_SDIOSUSPEND_BB        (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (SDIOSUSPEND_BitNumber * 4))

/* Alias word address of ENCMDCOMPL bit */
#define ENCMDCOMPL_BitNumber      0x0C
//#define CMD_ENCMDCOMPL_BB         (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (ENCMDCOMPL_BitNumber * 4))

/* Alias word address of NIEN bit */
#define NIEN_BitNumber            0x0D
//#define CMD_NIEN_BB               (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (NIEN_BitNumber * 4))

/* Alias word address of ATACMD bit */
#define ATACMD_BitNumber          0x0E
//#define CMD_ATACMD_BB             (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (ATACMD_BitNumber * 4))

/* --- DCTRL Register ---*/
/* Alias word address of DMAEN bit */
#define DCTRL_OFFSET              (SDMMC1_BASE + 0x2C)
#define DMAEN_BitNumber           0x03
//#define DCTRL_DMAEN_BB            (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (DMAEN_BitNumber * 4))

/* Alias word address of RWSTART bit */
#define RWSTART_BitNumber         0x08
//#define DCTRL_RWSTART_BB          (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (RWSTART_BitNumber * 4))

/* Alias word address of RWSTOP bit */
#define RWSTOP_BitNumber          0x09
//#define DCTRL_RWSTOP_BB           (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (RWSTOP_BitNumber * 4))

/* Alias word address of RWMOD bit */
#define RWMOD_BitNumber           0x0A
//#define DCTRL_RWMOD_BB            (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (RWMOD_BitNumber * 4))

/* Alias word address of SDIOEN bit */
#define SDIOEN_BitNumber          0x0B
//#define DCTRL_SDIOEN_BB           (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (SDIOEN_BitNumber * 4))

/* ---------------------- SDMMC1 registers bit mask ------------------------ */
/* --- CLKCR Register ---*/
/* CLKCR register clear mask */
#define CLKCR_CLEAR_MASK         ((uint32_t)0xFFFF8100)

/* --- PWRCTRL Register ---*/
/* SDMMC1 PWRCTRL Mask */
#define PWR_PWRCTRL_MASK         ((uint32_t)0xFFFFFFFC)

/* --- DCTRL Register ---*/
/* SDMMC1 DCTRL Clear Mask */
#define DCTRL_CLEAR_MASK         ((uint32_t)0xFFFFFF08)

/* --- CMD Register ---*/
/* CMD Register clear mask */
#define CMD_CLEAR_MASK           ((uint32_t)0xFFFFF800)

/* SDMMC1 RESP Registers Address */
#define SDIO_RESP_ADDR           ((uint32_t)(SDMMC1_BASE + 0x14))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup SDIO_Private_Functions
  * @{
  */

  /** @defgroup SDIO_Group1 Initialization and Configuration functions
   *  @brief   Initialization and Configuration functions
   *
  @verbatim
   ===============================================================================
                   Initialization and Configuration functions
   ===============================================================================

  @endverbatim
    * @{
    */

    /**
      * @brief  Deinitializes the SDMMC1 peripheral registers to their default reset values.
      * @param  None
      * @retval None
      */
void SDIO_DeInit(void) {
    RCC->APB2RSTR |= (1 << 11);
    RCC->APB2RSTR &= ~(1 << 11);
}

/**
  * @brief  Initializes the SDMMC1 peripheral according to the specified
  *         parameters in the SDIO_InitStruct.
  * @param  SDIO_InitStruct : pointer to a SDIO_InitTypeDef structure
  *         that contains the configuration information for the SDMMC1 peripheral.
  * @retval None
  */
void SDIO_Init(uint32_t clockDiv, uint32_t clockPowerSave, uint32_t clockBypass, uint32_t clockEdge, uint32_t busWide, uint32_t hardwareFlowControl) {
    uint32_t tmpreg = SDMMC1->CLKCR;;


    /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */
    tmpreg &= CLKCR_CLEAR_MASK;

    /* Set CLKDIV bits according to SDIO_ClockDiv value */
    /* Set PWRSAV bit according to SDIO_ClockPowerSave value */
    /* Set BYPASS bit according to SDIO_ClockBypass value */
    /* Set WIDBUS bits according to SDIO_BusWide value */
    /* Set NEGEDGE bits according to SDIO_ClockEdge value */
    /* Set HWFC_EN bits according to SDIO_HardwareFlowControl value */
    tmpreg |= (clockDiv | clockPowerSave | clockBypass | clockEdge | busWide | hardwareFlowControl);

    /* Write to SDMMC1 CLKCR */
    SDMMC1->CLKCR = tmpreg;
}

/**
  * @brief  Enables or disables the SDMMC1 Clock.
  * @param  newState: new state of the SDMMC1 Clock.
  *         This parameter can be: true or false.
  * @retval None
  */
void SDIO_ClockCmd(bool newState) {

    if (newState) {
        *reinterpret_cast<uint32_t*>(CLKCR_OFFSET) |= (newState << CLKEN_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(CLKCR_OFFSET) &= ~(newState << CLKEN_BitNumber);
    }
}

/**
  * @brief  Sets the power status of the controllerIndex.
  * @param  SDIO_PowerState: new state of the Power state.
  *          This parameter can be one of the following values:
  *            @arg SDIO_PowerState_OFF: SDMMC1 Power OFF
  *            @arg SDIO_PowerState_ON: SDMMC1 Power ON
  * @retval None
  */
void SDIO_SetPowerState(uint32_t SDIO_PowerState) {

    SDMMC1->POWER = SDIO_PowerState;
}

/**
  * @brief  Gets the power status of the controllerIndex.
  * @param  None
  * @retval Power status of the controllerIndex. The returned value can be one of the
  *         following values:
  *            - 0x00: Power OFF
  *            - 0x02: Power UP
  *            - 0x03: Power ON
  */
uint32_t SDIO_GetPowerState(void) {
    return (SDMMC1->POWER & (~PWR_PWRCTRL_MASK));
}

/**
  * @}
  */

  /** @defgroup SDIO_Group2 Command path state machine (CPSM) management functions
   *  @brief   Command path state machine (CPSM) management functions
   *
  @verbatim
   ===============================================================================
                Command path state machine (CPSM) management functions
   ===============================================================================

    This section provide functions allowing to program and read the Command path
    state machine (CPSM).

  @endverbatim
    * @{
    */

    /**
      * @brief  Initializes the SDMMC1 Command according to the specified
      *         parameters in the SDIO_CmdInitStruct and send the command.
      * @param  SDIO_CmdInitStruct : pointer to a SDIO_CmdInitTypeDef
      *         structure that contains the configuration information for the SDMMC1
      *         command.
      * @retval None
      */
void SDIO_SendCommand(uint32_t argument, uint32_t cmdIndex, uint32_t response) {
    uint32_t tmpreg = 0;


    /*---------------------------- SDMMC1 ARG Configuration ------------------------*/
      /* Set the SDMMC1 Argument value */
    SDMMC1->ARG = argument;

    /*---------------------------- SDMMC1 CMD Configuration ------------------------*/
      /* Get the SDMMC1 CMD value */
    tmpreg = SDMMC1->CMD;
    /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, CPSMEN bits */
    tmpreg &= CMD_CLEAR_MASK;
    /* Set CMDINDEX bits according to SDIO_CmdIndex value */
    /* Set WAITRESP bits according to SDIO_Response value */
    /* Set WAITINT and WAITPEND bits according to SDIO_Wait value */
    /* Set CPSMEN bits according to SDIO_CPSM value */
    tmpreg |= cmdIndex | response | SDIO_Wait_No | SDIO_CPSM_Enable;

    /* Write to SDMMC1 CMD */
    SDMMC1->CMD = tmpreg;
}

/**
  * @brief  Returns command index of last command for which response received.
  * @param  None
  * @retval Returns the command index of the last command response received.
  */
uint8_t SDIO_GetCommandResponse(void) {
    return (uint8_t)(SDMMC1->RESPCMD);
}

/**
  * @brief  Returns response received from the card for the last command.
  * @param  SDIO_RESP: Specifies the SDMMC1 response register.
  *          This parameter can be one of the following values:
  *            @arg SDIO_RESP1: Response Register 1
  *            @arg SDIO_RESP2: Response Register 2
  *            @arg SDIO_RESP3: Response Register 3
  *            @arg SDIO_RESP4: Response Register 4
  * @retval The Corresponding response register value.
  */
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP) {
    uint32_t tmp = 0;

    tmp = SDIO_RESP_ADDR + SDIO_RESP;

    return (*(uint32_t *)tmp);
}

/**
  * @}
  */

  /** @defgroup SDIO_Group3 Data path state machine (DPSM) management functions
   *  @brief   Data path state machine (DPSM) management functions
   *
  @verbatim
   ===============================================================================
                Data path state machine (DPSM) management functions
   ===============================================================================

    This section provide functions allowing to program and read the Data path
    state machine (DPSM).

  @endverbatim
    * @{
    */

    /**
      * @brief  Initializes the SDMMC1 data path according to the specified
      *         parameters in the SDIO_DataInitStruct.
      * @param  SDIO_DataInitStruct : pointer to a SDIO_DataInitTypeDef structure
      *         that contains the configuration information for the SDMMC1 command.
      * @retval None
      */

void SDIO_DataConfig(uint32_t dataLength, uint32_t blockSize, uint32_t transferDir) {
    uint32_t tmpreg = 0;

    /*---------------------------- SDMMC1 DTIMER Configuration ---------------------*/
      /* Set the SDMMC1 Data TimeOut value */
    SDMMC1->DTIMER = SD_DATATIMEOUT;

    /*---------------------------- SDMMC1 DLEN Configuration -----------------------*/
      /* Set the SDMMC1 DataLength value */
    SDMMC1->DLEN = dataLength;

    /*---------------------------- SDMMC1 DCTRL Configuration ----------------------*/
      /* Get the SDMMC1 DCTRL value */
    tmpreg = SDMMC1->DCTRL;
    /* Clear DEN, DTMODE, DTDIR and DBCKSIZE bits */
    tmpreg &= DCTRL_CLEAR_MASK;
    /* Set DEN bit according to SDIO_DPSM value */
    /* Set DTMODE bit according to SDIO_TransferMode value */
    /* Set DTDIR bit according to SDIO_TransferDir value */
    /* Set DBCKSIZE bits according to SDIO_DataBlockSize value */
    tmpreg |= (uint32_t)blockSize | transferDir | SDIO_TransferMode_Block | SDIO_DPSM_Enable;

    /* Write to SDMMC1 DCTRL */
    SDMMC1->DCTRL = tmpreg;
}

/**
  * @brief  Returns number of remaining data bytes to be transferred.
  * @param  None
  * @retval Number of remaining data bytes to be transferred
  */
uint32_t SDIO_GetDataCounter(void) {
    return SDMMC1->DCOUNT;
}

/**
  * @brief  Read one data word from Rx FIFO.
  * @param  None
  * @retval Data received
  */
uint32_t SDIO_ReadData(void) {
    return SDMMC1->FIFO;
}

/**
  * @brief  Write one data word to Tx FIFO.
  * @param  Data: 32-bit data word to write.
  * @retval None
  */
void SDIO_WriteData(uint32_t Data) {
    SDMMC1->FIFO = Data;
}

/**
  * @brief  Returns the number of words left to be written to or read from FIFO.
  * @param  None
  * @retval Remaining number of words.
  */
uint32_t SDIO_GetFIFOCount(void) {
    return SDMMC1->FIFOCNT;
}

/**
  * @}
  */

  /** @defgroup SDIO_Group4 SDMMC1 IO Cards mode management functions
   *  @brief   SDMMC1 IO Cards mode management functions
   *
  @verbatim
   ===============================================================================
                SDMMC1 IO Cards mode management functions
   ===============================================================================

    This section provide functions allowing to program and read the SDMMC1 IO Cards.

  @endverbatim
    * @{
    */

    /**
      * @brief  Starts the SD I/O Read Wait operation.
      * @param  newState: new state of the Start SDMMC1 Read Wait operation.
      *         This parameter can be: true or false.
      * @retval None
      */
void SDIO_StartSDIOReadWait(bool newState) {
    if (newState) {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) |= (1 << RWSTART_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) &= ~(1 << RWSTART_BitNumber);
    }
}

/**
  * @brief  Stops the SD I/O Read Wait operation.
  * @param  newState: new state of the Stop SDMMC1 Read Wait operation.
  *         This parameter can be: true or false.
  * @retval None
  */
void SDIO_StopSDIOReadWait(bool newState) {
    if (newState) {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) |= (1 << RWSTOP_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) &= ~(1 << RWSTOP_BitNumber);
    }

}

/**
  * @brief  Sets one of the two options of inserting read wait interval.
  * @param  SDIO_ReadWaitMode: SD I/O Read Wait operation mode.
  *          This parameter can be:
  *            @arg SDIO_ReadWaitMode_CLK: Read Wait control by stopping SDIOCLK
  *            @arg SDIO_ReadWaitMode_DATA2: Read Wait control using SDIO_DATA2
  * @retval None
  */
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode) {
    if (SDIO_ReadWaitMode) {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) |= (1 << RWMOD_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) &= ~(1 << RWMOD_BitNumber);
    }
}

/**
  * @brief  Enables or disables the SD I/O Mode Operation.
  * @param  newState: new state of SDMMC1 specific operation.
  *         This parameter can be: true or false.
  * @retval None
  */
void SDIO_SetSDIOOperation(bool newState) {
    if (newState) {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) |= (1 << SDIOEN_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(DCTRL_OFFSET) &= ~(1 << SDIOEN_BitNumber);
    }
}

/**
  * @brief  Enables or disables the SD I/O Mode suspend command sending.
  * @param  newState: new state of the SD I/O Mode suspend command.
  *         This parameter can be: true or false.
  * @retval None
  */
void SDIO_SendSDIOSuspendCmd(bool newState) {
    if (newState) {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) |= (1 << SDIOSUSPEND_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) &= ~(1 << SDIOSUSPEND_BitNumber);
    }
}

/**
  * @}
  */

  /** @defgroup SDIO_Group5 CE-ATA mode management functions
   *  @brief   CE-ATA mode management functions
   *
  @verbatim
   ===============================================================================
                CE-ATA mode management functions
   ===============================================================================

    This section provide functions allowing to program and read the CE-ATA card.

  @endverbatim
    * @{
    */

    /**
      * @brief  Enables or disables the command completion signal.
      * @param  newState: new state of command completion signal.
      *         This parameter can be: true or false.
      * @retval None
      */
void SDIO_CommandCompletionCmd(bool newState) {
    if (newState) {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) |= (1 << ENCMDCOMPL_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) &= ~(1 << ENCMDCOMPL_BitNumber);
    }
}

/**
  * @brief  Enables or disables the CE-ATA interrupt.
  * @param  newState: new state of CE-ATA interrupt.
  *         This parameter can be: true or false.
  * @retval None
  */
void SDIO_CEATAITCmd(bool newState) {
    if (newState) {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) |= (1 << NIEN_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) &= ~(1 << NIEN_BitNumber);
    }
}

/**
  * @brief  Sends CE-ATA command (CMD61).
  * @param  newState: new state of CE-ATA command.
  *         This parameter can be: true or false.
  * @retval None
  */
void SDIO_SendCEATACmd(bool newState) {
    if (newState) {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) |= (1 << ATACMD_BitNumber);
    }
    else {
        *reinterpret_cast<uint32_t*>(CMD_OFFSET) &= ~(1 << ATACMD_BitNumber);
    }
}

/**
  * @brief  Checks whether the specified SDMMC1 flag is set or not.
  * @param  SDIO_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *            @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *            @arg SDIO_FLAG_STBITERR: Start bit not detected on all data signals in wide bus mode.
  *            @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDIO_FLAG_CMDACT:   Command transfer in progress
  *            @arg SDIO_FLAG_TXACT:    Data transmit in progress
  *            @arg SDIO_FLAG_RXACT:    Data receive in progress
  *            @arg SDIO_FLAG_TXFIFOHE: Transmit FIFO Half Empty
  *            @arg SDIO_FLAG_RXFIFOHF: Receive FIFO Half Full
  *            @arg SDIO_FLAG_TXFIFOF:  Transmit FIFO full
  *            @arg SDIO_FLAG_RXFIFOF:  Receive FIFO full
  *            @arg SDIO_FLAG_TXFIFOE:  Transmit FIFO empty
  *            @arg SDIO_FLAG_RXFIFOE:  Receive FIFO empty
  *            @arg SDIO_FLAG_TXDAVL:   Data available in transmit FIFO
  *            @arg SDIO_FLAG_RXDAVL:   Data available in receive FIFO
  *            @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  *            @arg SDIO_FLAG_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval The new state of SDIO_FLAG (true or false).
  */
bool SDIO_GetFlagStatus(uint32_t SDIO_FLAG) {
    bool bitstatus = false;


    if ((SDMMC1->STA & SDIO_FLAG) != (uint32_t)false) {
        bitstatus = true;
    }
    else {
        bitstatus = false;
    }
    return bitstatus;
}

/**
  * @brief  Clears the SDMMC1's pending flags.
  * @param  SDIO_FLAG: specifies the flag to clear.
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *            @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *            @arg SDIO_FLAG_STBITERR: Start bit not detected on all data signals in wide bus mode
  *            @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  *            @arg SDIO_FLAG_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval None
  */
void SDIO_ClearFlag(uint32_t SDIO_FLAG) {

    SDMMC1->ICR = SDIO_FLAG;
}

/**
  * @brief  Checks whether the specified SDMMC1 interrupt has occurred or not.
  * @param  SDIO_IT: specifies the SDMMC1 interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *            @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide
  *                                   bus mode interrupt
  *            @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *            @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61 interrupt
  * @retval The new state of SDIO_IT (true or false).
  */
bool SDIO_GetITStatus(uint32_t SDIO_IT) {
    bool bitstatus = false;

    if ((SDMMC1->STA & SDIO_IT) != (uint32_t)false) {
        bitstatus = true;
    }
    else {
        bitstatus = false;
    }
    return bitstatus;
}

/**
  * @brief  Clears the SDMMC1's interrupt pending bits.
  * @param  SDIO_IT: specifies the interrupt pending bit to clear.
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, SDIO_DCOUNT, is zero) interrupt
  *            @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide
  *                                   bus mode interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *            @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval None
  */
void SDIO_ClearITPendingBit(uint32_t SDIO_IT) {

    SDMMC1->ICR = SDIO_IT;
}

// sdio_sd

typedef enum {
    /**
      * @brief  SDMMC1 specific error defines
      */
    SD_CMD_CRC_FAIL = (1), /*!< Command response received (but CRC check failed) */
    SD_DATA_CRC_FAIL = (2), /*!< Data bock sent/received (CRC check Failed) */
    SD_CMD_RSP_TIMEOUT = (3), /*!< Command response timeout */
    SD_DATA_TIMEOUT = (4), /*!< Data time out */
    SD_TX_UNDERRUN = (5), /*!< Transmit FIFO under-run */
    SD_RX_OVERRUN = (6), /*!< Receive FIFO over-run */
    SD_START_BIT_ERR = (7), /*!< Start bit not detected on all data signals in widE bus mode */
    SD_CMD_OUT_OF_RANGE = (8), /*!< CMD's argument was out of range.*/
    SD_ADDR_MISALIGNED = (9), /*!< Misaligned address */
    SD_BLOCK_LEN_ERR = (10), /*!< Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
    SD_ERASE_SEQ_ERR = (11), /*!< An error in the sequence of erase command occurs.*/
    SD_BAD_ERASE_PARAM = (12), /*!< An Invalid selection for erase groups */
    SD_WRITE_PROT_VIOLATION = (13), /*!< Attempt to program a write protect block */
    SD_LOCK_UNLOCK_FAILED = (14), /*!< Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
    SD_COM_CRC_FAILED = (15), /*!< CRC check of the previous command failed */
    SD_ILLEGAL_CMD = (16), /*!< Command is not legal for the card state */
    SD_CARD_ECC_FAILED = (17), /*!< Card internal ECC was applied but failed to correct the data */
    SD_CC_ERROR = (18), /*!< Internal card controllerIndex error */
    SD_GENERAL_UNKNOWN_ERROR = (19), /*!< General or Unknown error */
    SD_STREAM_READ_UNDERRUN = (20), /*!< The card could not sustain data transfer in stream read operation. */
    SD_STREAM_WRITE_OVERRUN = (21), /*!< The card could not sustain data programming in stream mode */
    SD_CID_CSD_OVERWRITE = (22), /*!< CID/CSD overwrite error */
    SD_WP_ERASE_SKIP = (23), /*!< only partial address space was erased */
    SD_CARD_ECC_DISABLED = (24), /*!< Command has been executed without using internal ECC */
    SD_ERASE_RESET = (25), /*!< Erase sequence was cleared before executing because an out of erase sequence command was received */
    SD_AKE_SEQ_ERROR = (26), /*!< Error in sequence of authentication. */
    SD_INVALID_VOLTRANGE = (27),
    SD_ADDR_OUT_OF_RANGE = (28),
    SD_SWITCH_ERROR = (29),
    SD_SDIO_DISABLED = (30),
    SD_SDIO_FUNCTION_BUSY = (31),
    SD_SDIO_FUNCTION_FAILED = (32),
    SD_SDIO_UNKNOWN_FUNCTION = (33),

    /**
      * @brief  Standard error defines
      */
    SD_INTERNAL_ERROR,
    SD_NOT_CONFIGURED,
    SD_REQUEST_PENDING,
    SD_REQUEST_NOT_APPLICABLE,
    SD_INVALID_PARAMETER,
    SD_UNSUPPORTED_FEATURE,
    SD_UNSUPPORTED_HW,
    SD_ERROR,
    SD_OK = 0
} SD_Error;

/**
  * @brief  SDMMC1 Transfer state
  */
typedef enum {
    SD_TRANSFER_OK = 0,
    SD_TRANSFER_BUSY = 1,
    SD_TRANSFER_ERROR
} SDTransferState;

/**
  * @brief  SD Card States
  */
typedef enum {
    SD_CARD_READY = ((uint32_t)0x00000001),
    SD_CARD_IDENTIFICATION = ((uint32_t)0x00000002),
    SD_CARD_STANDBY = ((uint32_t)0x00000003),
    SD_CARD_TRANSFER = ((uint32_t)0x00000004),
    SD_CARD_SENDING = ((uint32_t)0x00000005),
    SD_CARD_RECEIVING = ((uint32_t)0x00000006),
    SD_CARD_PROGRAMMING = ((uint32_t)0x00000007),
    SD_CARD_DISCONNECTED = ((uint32_t)0x00000008),
    SD_CARD_ERROR = ((uint32_t)0x000000FF)
}SDCardState;


/**
  * @brief  Card Specific Data: CSD Register
  */
typedef struct {
    uint8_t  CSDStruct;            /*!< CSD structure */
    uint8_t  SysSpecVersion;       /*!< System specification version */
    uint8_t  Reserved1;            /*!< Reserved */
    uint8_t  TAAC;                 /*!< Data read access-time 1 */
    uint8_t  NSAC;                 /*!< Data read access-time 2 in CLK cycles */
    uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency */
    uint16_t CardComdClasses;      /*!< Card command classes */
    uint8_t  RdBlockLen;           /*!< Max. read data block length */
    uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed */
    uint8_t  WrBlockMisalign;      /*!< Write block misalignment */
    uint8_t  RdBlockMisalign;      /*!< Read block misalignment */
    uint8_t  DSRImpl;              /*!< DSR implemented */
    uint8_t  Reserved2;            /*!< Reserved */
    uint32_t DeviceSize;           /*!< Device Size */
    uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min */
    uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max */
    uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min */
    uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max */
    uint8_t  DeviceSizeMul;        /*!< Device size multiplier */
    uint8_t  EraseGrSize;          /*!< Erase group size */
    uint8_t  EraseGrMul;           /*!< Erase group size multiplier */
    uint8_t  WrProtectGrSize;      /*!< Write protect group size */
    uint8_t  WrProtectGrEnable;    /*!< Write protect group enable */
    uint8_t  ManDeflECC;           /*!< Manufacturer default ECC */
    uint8_t  WrSpeedFact;          /*!< Write speed factor */
    uint8_t  MaxWrBlockLen;        /*!< Max. write data block length */
    uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed */
    uint8_t  Reserved3;            /*!< Reserded */
    uint8_t  ContentProtectAppli;  /*!< Content protection application */
    uint8_t  FileFormatGrouop;     /*!< File format group */
    uint8_t  CopyFlag;             /*!< Copy flag (OTP) */
    uint8_t  PermWrProtect;        /*!< Permanent write protection */
    uint8_t  TempWrProtect;        /*!< Temporary write protection */
    uint8_t  FileFormat;           /*!< File Format */
    uint8_t  ECC;                  /*!< ECC code */
    uint8_t  CSD_CRC;              /*!< CSD CRC */
    uint8_t  Reserved4;            /*!< always 1*/
} SD_CSD;

/**
  * @brief  Card Identification Data: CID Register
  */
typedef struct {
    uint8_t  ManufacturerID;       /*!< ManufacturerID */
    uint16_t OEM_AppliID;          /*!< OEM/Application ID */
    uint32_t ProdName1;            /*!< Product Name part1 */
    uint8_t  ProdName2;            /*!< Product Name part2*/
    uint8_t  ProdRev;              /*!< Product Revision */
    uint32_t ProdSN;               /*!< Product Serial Number */
    uint8_t  Reserved1;            /*!< Reserved1 */
    uint16_t ManufactDate;         /*!< Manufacturing Date */
    uint8_t  CID_CRC;              /*!< CID CRC */
    uint8_t  Reserved2;            /*!< always 1 */
} SD_CID;

/**
  * @brief SD Card Status
  */
typedef struct {
    uint8_t DAT_BUS_WIDTH;
    uint8_t SECURED_MODE;
    uint16_t SD_CARD_TYPE;
    uint32_t SIZE_OF_PROTECTED_AREA;
    uint8_t SPEED_CLASS;
    uint8_t PERFORMANCE_MOVE;
    uint8_t AU_SIZE;
    uint16_t ERASE_SIZE;
    uint8_t ERASE_TIMEOUT;
    uint8_t ERASE_OFFSET;
} SD_CardStatus;


/**
  * @brief SD Card information
  */
typedef struct {
    SD_CSD SD_csd;
    SD_CID SD_cid;
    uint64_t CardCapacity;  /*!< Card Capacity */
    uint32_t CardBlockSize; /*!< Card Block Size */
    uint16_t RCA;
    uint8_t CardType;
} SD_CardInfo;

#define SD_CMD_GO_IDLE_STATE                       ((uint8_t)0)
#define SD_CMD_SEND_OP_COND                        ((uint8_t)1)
#define SD_CMD_ALL_SEND_CID                        ((uint8_t)2)
#define SD_CMD_SET_REL_ADDR                        ((uint8_t)3) /*!< SDIO_SEND_REL_ADDR for SD Card */
#define SD_CMD_SET_DSR                             ((uint8_t)4)
#define SD_CMD_SDIO_SEN_OP_COND                    ((uint8_t)5)
#define SD_CMD_HS_SWITCH                           ((uint8_t)6)
#define SD_CMD_SEL_DESEL_CARD                      ((uint8_t)7)
#define SD_CMD_HS_SEND_EXT_CSD                     ((uint8_t)8)
#define SD_CMD_SEND_CSD                            ((uint8_t)9)
#define SD_CMD_SEND_CID                            ((uint8_t)10)
#define SD_CMD_READ_DAT_UNTIL_STOP                 ((uint8_t)11) /*!< SD Card doesn't support it */
#define SD_CMD_STOP_TRANSMISSION                   ((uint8_t)12)
#define SD_CMD_SEND_STATUS                         ((uint8_t)13)
#define SD_CMD_HS_BUSTEST_READ                     ((uint8_t)14)
#define SD_CMD_GO_INACTIVE_STATE                   ((uint8_t)15)
#define SD_CMD_SET_BLOCKLEN                        ((uint8_t)16)
#define SD_CMD_READ_SINGLE_BLOCK                   ((uint8_t)17)
#define SD_CMD_READ_MULT_BLOCK                     ((uint8_t)18)
#define SD_CMD_HS_BUSTEST_WRITE                    ((uint8_t)19)
#define SD_CMD_WRITE_DAT_UNTIL_STOP                ((uint8_t)20) /*!< SD Card doesn't support it */
#define SD_CMD_SET_BLOCK_COUNT                     ((uint8_t)23) /*!< SD Card doesn't support it */
#define SD_CMD_WRITE_SINGLE_BLOCK                  ((uint8_t)24)
#define SD_CMD_WRITE_MULT_BLOCK                    ((uint8_t)25)
#define SD_CMD_PROG_CID                            ((uint8_t)26) /*!< reserved for manufacturers */
#define SD_CMD_PROG_CSD                            ((uint8_t)27)
#define SD_CMD_SET_WRITE_PROT                      ((uint8_t)28)
#define SD_CMD_CLR_WRITE_PROT                      ((uint8_t)29)
#define SD_CMD_SEND_WRITE_PROT                     ((uint8_t)30)
#define SD_CMD_SD_ERASE_GRP_START                  ((uint8_t)32) /*!< To set the address of the first write
                                                                  block to be erased. (For SD card only) */
#define SD_CMD_SD_ERASE_GRP_END                    ((uint8_t)33) /*!< To set the address of the last write block of the
                                                                  continuous range to be erased. (For SD card only) */
#define SD_CMD_ERASE_GRP_START                     ((uint8_t)35) /*!< To set the address of the first write block to be erased.
                                                                  (For MMC card only spec 3.31) */

#define SD_CMD_ERASE_GRP_END                       ((uint8_t)36) /*!< To set the address of the last write block of the
                                                                  continuous range to be erased. (For MMC card only spec 3.31) */

#define SD_CMD_ERASE                               ((uint8_t)38)
#define SD_CMD_FAST_IO                             ((uint8_t)39) /*!< SD Card doesn't support it */
#define SD_CMD_GO_IRQ_STATE                        ((uint8_t)40) /*!< SD Card doesn't support it */
#define SD_CMD_LOCK_UNLOCK                         ((uint8_t)42)
#define SD_CMD_APP_CMD                             ((uint8_t)55)
#define SD_CMD_GEN_CMD                             ((uint8_t)56)
#define SD_CMD_NO_CMD                              ((uint8_t)64)

#define SD_CMD_APP_SD_SET_BUSWIDTH                 ((uint8_t)6)  /*!< For SD Card only */
#define SD_CMD_SD_APP_STAUS                        ((uint8_t)13) /*!< For SD Card only */
#define SD_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS        ((uint8_t)22) /*!< For SD Card only */
#define SD_CMD_SD_APP_OP_COND                      ((uint8_t)41) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CLR_CARD_DETECT          ((uint8_t)42) /*!< For SD Card only */
#define SD_CMD_SD_APP_SEND_SCR                     ((uint8_t)51) /*!< For SD Card only */
#define SD_CMD_SDIO_RW_DIRECT                      ((uint8_t)52) /*!< For SD I/O Card only */
#define SD_CMD_SDIO_RW_EXTENDED                    ((uint8_t)53) /*!< For SD I/O Card only */

#define SD_CMD_SD_APP_GET_MKB                      ((uint8_t)43) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_MID                      ((uint8_t)44) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CER_RN1                  ((uint8_t)45) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_CER_RN2                  ((uint8_t)46) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CER_RES2                 ((uint8_t)47) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_CER_RES1                 ((uint8_t)48) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK   ((uint8_t)18) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK  ((uint8_t)25) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_ERASE                 ((uint8_t)38) /*!< For SD Card only */
#define SD_CMD_SD_APP_CHANGE_SECURE_AREA           ((uint8_t)49) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MKB             ((uint8_t)48) /*!< For SD Card only */

#if !defined (SD_DMA_MODE) && !defined (SD_POLLING_MODE)
                                                                  //#define SD_DMA_MODE                                ((uint32_t)0x00000000) <----------------------------------------------Commented out because of DMA
#define SD_POLLING_MODE                            ((uint32_t)0x00000002)
#endif

#define SD_PRESENT                                 ((uint8_t)0x01)
#define SD_NOT_PRESENT                             ((uint8_t)0x00)

#define SDIO_STD_CAPACITY_SD_CARD_V1_1             ((uint32_t)0x00000000)
#define SDIO_STD_CAPACITY_SD_CARD_V2_0             ((uint32_t)0x00000001)
#define SDIO_HIGH_CAPACITY_SD_CARD                 ((uint32_t)0x00000002)
#define SDIO_MULTIMEDIA_CARD                       ((uint32_t)0x00000003)
#define SDIO_SECURE_DIGITAL_IO_CARD                ((uint32_t)0x00000004)
#define SDIO_HIGH_SPEED_MULTIMEDIA_CARD            ((uint32_t)0x00000005)
#define SDIO_SECURE_DIGITAL_IO_COMBO_CARD          ((uint32_t)0x00000006)
#define SDIO_HIGH_CAPACITY_MMC_CARD                ((uint32_t)0x00000007)

void SD_DeInit(void);
SD_Error SD_Init(void);
SDTransferState SD_GetStatus(void);
SDCardState SD_GetState(void);
uint8_t SD_Detect(void);
SD_Error SD_PowerON(void);
SD_Error SD_PowerOFF(void);
SD_Error SD_InitializeCards(void);
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo);
SD_Error SD_GetCardStatus(SD_CardStatus *cardstatus);
SD_Error SD_EnableWideBusOperation(uint32_t WideMode);
SD_Error SD_SelectDeselect(uint32_t addr);
SD_Error SD_ReadBlock(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize);
SD_Error SD_WriteBlock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize);
SDTransferState SD_GetTransferState(void);
SD_Error SD_StopTransfer(void);
SD_Error SD_SendStatus(uint32_t *pcardstatus);
SD_Error SD_SendSDStatus(uint32_t *psdstatus);

#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40012C80)
#define SDIO_INIT_CLK_DIV                ((uint8_t)0x76)
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x2)
#define SDIO_STATIC_FLAGS               ((uint32_t)0x000005FF)
#define SDIO_CMD0TIMEOUT                ((uint32_t)0x00010000)

#define SD_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRIETE       ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)

#define SD_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SD_HIGH_CAPACITY                ((uint32_t)0x40000000)
#define SD_STD_CAPACITY                 ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                ((uint32_t)0x000001AA)

#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SD_ALLZERO                      ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT           ((uint32_t)0x00010000)
#define SD_CARD_LOCKED                  ((uint32_t)0x02000000)

#define SD_0TO7BITS                     ((uint32_t)0x000000FF)
#define SD_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                   ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SD_HALFFIFO                     ((uint32_t)0x00000008)
#define SD_HALFFIFOBYTES                ((uint32_t)0x00000020)

#define SD_CCCC_LOCK_UNLOCK             ((uint32_t)0x00000080)
#define SD_CCCC_WRITE_PROT              ((uint32_t)0x00000040)
#define SD_CCCC_ERASE                   ((uint32_t)0x00000020)

#define SDIO_SEND_IF_COND               ((uint32_t)0x00000008)

static uint32_t CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1;
static uint32_t CSD_Tab[4], CID_Tab[4], RCA = 0;
static uint8_t SDSTATUS_Tab[16];
uint32_t StopCondition = 0;
SD_Error TransferError = SD_OK;
uint32_t TransferEnd = 0, DMAEndOfTransfer = 0;
SD_CardInfo SDCardInfo;

static SD_Error CmdError(void);
static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca);
static SD_Error SDEnWideBus(FunctionalState newState);
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr);

/** @defgroup STM324xG_EVAL_SDIO_SD_Private_Functions
  * @{
  */

  /**
    * @brief  DeInitializes the SDMMC1 interface.
    * @param  None
    * @retval None
    */
void SD_DeInit(void) {
    SDIO_DeInit();

    SD_PowerOFF();
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for data
  *         transfer).
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_Init(void) {
    SD_Error errorstatus = SD_OK;

    errorstatus = SD_PowerON();

    if (errorstatus != SD_OK) {
        /*!< CMD Response TimeOut (wait for CMDSENT flag) */
        return(errorstatus);
    }

    errorstatus = SD_InitializeCards();

    if (errorstatus != SD_OK) {
        /*!< CMD Response TimeOut (wait for CMDSENT flag) */
        return(errorstatus);
    }

    /*!< Configure the SDMMC1 peripheral */
    /*!< SDIO_CK = SDIOCLK / (SDIO_TRANSFER_CLK_DIV + 2) */
    /*!< on STM32F7xx devices, SDIOCLK is fixed to 48MHz */

    SDIO_Init(SDIO_TRANSFER_CLK_DIV, SDIO_ClockPowerSave_Disable, SDIO_ClockBypass_Disable, SDIO_ClockEdge_Rising, SDIO_BusWide_1b, SDIO_HardwareFlowControl_Disable);

    /*----------------- Read CSD/CID MSD registers ------------------*/
    errorstatus = SD_GetCardInfo(&SDCardInfo);

    if (errorstatus == SD_OK) {
        /*----------------- Select Card --------------------------------*/
        errorstatus = SD_SelectDeselect((uint32_t)(SDCardInfo.RCA << 16));
    }

    if (errorstatus == SD_OK) {
        errorstatus = SD_EnableWideBusOperation(SDIO_BusWide_4b);
    }

    return(errorstatus);
}

/**
  * @brief  Gets the cuurent sd card data transfer status.
  * @param  None
  * @retval SDTransferState: Data Transfer state.
  *   This value can be:
  *        - SD_TRANSFER_OK: No data transfer is acting
  *        - SD_TRANSFER_BUSY: Data transfer is acting
  */
SDTransferState SD_GetStatus(void) {
    SDCardState cardstate = SD_CARD_TRANSFER;

    cardstate = SD_GetState();

    if (cardstate == SD_CARD_TRANSFER) {
        return(SD_TRANSFER_OK);
    }
    else if (cardstate == SD_CARD_ERROR) {
        return (SD_TRANSFER_ERROR);
    }
    else {
        return(SD_TRANSFER_BUSY);
    }
}

/**
  * @brief  Returns the current card's state.
  * @param  None
  * @retval SDCardState: SD Card Error or SD Card Current State.
  */
SDCardState SD_GetState(void) {
    uint32_t resp1 = 0;

    if (SD_Detect() == SD_PRESENT) {
        if (SD_SendStatus(&resp1) != SD_OK) {
            return SD_CARD_ERROR;
        }
        else {
            return (SDCardState)((resp1 >> 9) & 0x0F);
        }
    }
    else {
        return SD_CARD_ERROR;
    }
}

/**
 * @brief  Detect if SD card is correctly plugged in the memory slot.
 * @param  None
 * @retval Return if SD is detected or not
 */
uint8_t SD_Detect(void) {
    uint8_t status = SD_PRESENT;

    return status;
}

/**
  * @brief  Enquires cards about their operating voltage and configures
  *   clock controls.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_PowerON(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t response = 0, count = 0, validvoltage = 0;
    uint32_t SDType = SD_STD_CAPACITY;

    /*!< Power ON Sequence -----------------------------------------------------*/
    /*!< Configure the SDMMC1 peripheral */
    /*!< SDIO_CK = SDIOCLK / (SDIO_INIT_CLK_DIV + 2) */
    /*!< on STM32F7xx devices, SDIOCLK is fixed to 48MHz */
    /*!< SDIO_CK for initialization should not exceed 400 KHz */
    SDIO_Init(SDIO_INIT_CLK_DIV, SDIO_ClockPowerSave_Disable, SDIO_ClockBypass_Disable, SDIO_ClockEdge_Rising, SDIO_BusWide_1b, SDIO_HardwareFlowControl_Disable);

    /*!< Set Power State to ON */
    SDIO_SetPowerState(SDIO_PowerState_ON);

    /*!< Enable SDMMC1 Clock */
    SDIO_ClockCmd(ENABLE);

    /*!< CMD0: GO_IDLE_STATE ---------------------------------------------------*/
    /*!< No CMD response required */
    SDIO_SendCommand(0, SD_CMD_GO_IDLE_STATE, SDIO_Response_No);

    errorstatus = CmdError();

    if (errorstatus != SD_OK) {
        /*!< CMD Response TimeOut (wait for CMDSENT flag) */
        return(errorstatus);
    }

    /*!< CMD8: SEND_IF_COND ----------------------------------------------------*/
    /*!< Send CMD8 to verify SD card interface operating condition */
    /*!< Argument: - [31:12]: Reserved (shall be set to '0')
                 - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
                 - [7:0]: Check Pattern (recommended 0xAA) */
                 /*!< CMD Response: R7 */
    SDIO_SendCommand(SD_CHECK_PATTERN, SDIO_SEND_IF_COND, SDIO_Response_Short);

    errorstatus = CmdResp7Error();

    if (errorstatus == SD_OK) {
        CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /*!< SD Card 2.0 */
        SDType = SD_HIGH_CAPACITY;
    }
    else {
        /*!< CMD55 */
        SDIO_SendCommand(0x00, SD_CMD_APP_CMD, SDIO_Response_Short);

        errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
    }
    /*!< CMD55 */
    SDIO_SendCommand(0x00, SD_CMD_APP_CMD, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    /*!< If errorstatus is Command TimeOut, it is a MMC card */
    /*!< If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
       or SD card 1.x */
    if (errorstatus == SD_OK) {
        /*!< SD CARD */
        /*!< Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL)) {

            /*!< SEND CMD55 APP_CMD with RCA as 0 */
            SDIO_SendCommand(0x00, SD_CMD_APP_CMD, SDIO_Response_Short);

            errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

            if (errorstatus != SD_OK) {
                return(errorstatus);
            }
            SDIO_SendCommand(SD_VOLTAGE_WINDOW_SD | SDType, SD_CMD_SD_APP_OP_COND, SDIO_Response_Short);

            errorstatus = CmdResp3Error();
            if (errorstatus != SD_OK) {
                return(errorstatus);
            }

            response = SDIO_GetResponse(SDIO_RESP1);
            validvoltage = (((response >> 31) == 1) ? 1 : 0);
            count++;
        }
        if (count >= SD_MAX_VOLT_TRIAL) {
            errorstatus = SD_INVALID_VOLTRANGE;
            return(errorstatus);
        }

        if (response &= SD_HIGH_CAPACITY) {
            CardType = SDIO_HIGH_CAPACITY_SD_CARD;
        }

    }/*!< else MMC Card */

    return(errorstatus);
}

/**
  * @brief  Turns the SDMMC1 output signals off.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_PowerOFF(void) {
    SD_Error errorstatus = SD_OK;

    /*!< Set Power State to OFF */
    SDIO_SetPowerState(SDIO_PowerState_OFF);

    return(errorstatus);
}

/**
  * @brief  Intialises all cards or single card as the case may be Card(s) come
  *         into standby state.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_InitializeCards(void) {
    SD_Error errorstatus = SD_OK;
    uint16_t rca = 0x01;

    if (SDIO_GetPowerState() == SDIO_PowerState_OFF) {
        errorstatus = SD_REQUEST_NOT_APPLICABLE;
        return(errorstatus);
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != CardType) {
        /*!< Send CMD2 ALL_SEND_CID */
        SDIO_SendCommand(0x00, SD_CMD_ALL_SEND_CID, SDIO_Response_Long);

        errorstatus = CmdResp2Error();

        if (SD_OK != errorstatus) {
            return(errorstatus);
        }

        CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
        CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
        CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
        CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
    }
    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
        || (SDIO_HIGH_CAPACITY_SD_CARD == CardType)) {
        /*!< Send CMD3 SET_REL_ADDR with argument 0 */
        /*!< SD Card publishes its RCA. */
        SDIO_SendCommand(0x00, SD_CMD_SET_REL_ADDR, SDIO_Response_Short);

        errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);

        if (SD_OK != errorstatus) {
            return(errorstatus);
        }
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != CardType) {
        RCA = rca;

        /*!< Send CMD9 SEND_CSD with argument as card's RCA */
        SDIO_SendCommand((uint32_t)(rca << 16), SD_CMD_SEND_CSD, SDIO_Response_Long);

        errorstatus = CmdResp2Error();

        if (SD_OK != errorstatus) {
            return(errorstatus);
        }

        CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
        CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
        CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
        CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
    }

    errorstatus = SD_OK; /*!< All cards get intialized */

    return(errorstatus);
}

/**
  * @brief  Returns information about specific card.
  * @param  cardinfo: pointer to a SD_CardInfo structure that contains all SD card
  *         information.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo) {
    SD_Error errorstatus = SD_OK;
    uint8_t tmp = 0;

    cardinfo->CardType = (uint8_t)CardType;
    cardinfo->RCA = (uint16_t)RCA;

    /*!< Byte 0 */
    tmp = (uint8_t)((CSD_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
    cardinfo->SD_csd.Reserved1 = tmp & 0x03;

    /*!< Byte 1 */
    tmp = (uint8_t)((CSD_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.TAAC = tmp;

    /*!< Byte 2 */
    tmp = (uint8_t)((CSD_Tab[0] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.NSAC = tmp;

    /*!< Byte 3 */
    tmp = (uint8_t)(CSD_Tab[0] & 0x000000FF);
    cardinfo->SD_csd.MaxBusClkFrec = tmp;

    /*!< Byte 4 */
    tmp = (uint8_t)((CSD_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CardComdClasses = tmp << 4;

    /*!< Byte 5 */
    tmp = (uint8_t)((CSD_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
    cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

    /*!< Byte 6 */
    tmp = (uint8_t)((CSD_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.Reserved2 = 0; /*!< Reserved */

    if ((CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0)) {
        cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

        /*!< Byte 7 */
        tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

        /*!< Byte 8 */
        tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);
        cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

        cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

        /*!< Byte 9 */
        tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);
        cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
        /*!< Byte 10 */
        tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
        cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

        cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1);
        cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
        cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
        cardinfo->CardCapacity *= cardinfo->CardBlockSize;
    }
    else if (CardType == SDIO_HIGH_CAPACITY_SD_CARD) {
        /*!< Byte 7 */
        tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

        /*!< Byte 8 */
        tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);

        cardinfo->SD_csd.DeviceSize |= (tmp << 8);

        /*!< Byte 9 */
        tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);

        cardinfo->SD_csd.DeviceSize |= (tmp);

        /*!< Byte 10 */
        tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);

        cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
        cardinfo->CardBlockSize = 512;
    }


    cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

    /*!< Byte 11 */
    tmp = (uint8_t)(CSD_Tab[2] & 0x000000FF);
    cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

    /*!< Byte 12 */
    tmp = (uint8_t)((CSD_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
    cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

    /*!< Byte 13 */
    tmp = (uint8_t)((CSD_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.Reserved3 = 0;
    cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

    /*!< Byte 14 */
    tmp = (uint8_t)((CSD_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
    cardinfo->SD_csd.ECC = (tmp & 0x03);

    /*!< Byte 15 */
    tmp = (uint8_t)(CSD_Tab[3] & 0x000000FF);
    cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_csd.Reserved4 = 1;


    /*!< Byte 0 */
    tmp = (uint8_t)((CID_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ManufacturerID = tmp;

    /*!< Byte 1 */
    tmp = (uint8_t)((CID_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.OEM_AppliID = tmp << 8;

    /*!< Byte 2 */
    tmp = (uint8_t)((CID_Tab[0] & 0x000000FF00) >> 8);
    cardinfo->SD_cid.OEM_AppliID |= tmp;

    /*!< Byte 3 */
    tmp = (uint8_t)(CID_Tab[0] & 0x000000FF);
    cardinfo->SD_cid.ProdName1 = tmp << 24;

    /*!< Byte 4 */
    tmp = (uint8_t)((CID_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdName1 |= tmp << 16;

    /*!< Byte 5 */
    tmp = (uint8_t)((CID_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdName1 |= tmp << 8;

    /*!< Byte 6 */
    tmp = (uint8_t)((CID_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdName1 |= tmp;

    /*!< Byte 7 */
    tmp = (uint8_t)(CID_Tab[1] & 0x000000FF);
    cardinfo->SD_cid.ProdName2 = tmp;

    /*!< Byte 8 */
    tmp = (uint8_t)((CID_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdRev = tmp;

    /*!< Byte 9 */
    tmp = (uint8_t)((CID_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdSN = tmp << 24;

    /*!< Byte 10 */
    tmp = (uint8_t)((CID_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdSN |= tmp << 16;

    /*!< Byte 11 */
    tmp = (uint8_t)(CID_Tab[2] & 0x000000FF);
    cardinfo->SD_cid.ProdSN |= tmp << 8;

    /*!< Byte 12 */
    tmp = (uint8_t)((CID_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdSN |= tmp;

    /*!< Byte 13 */
    tmp = (uint8_t)((CID_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
    cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

    /*!< Byte 14 */
    tmp = (uint8_t)((CID_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ManufactDate |= tmp;

    /*!< Byte 15 */
    tmp = (uint8_t)(CID_Tab[3] & 0x000000FF);
    cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_cid.Reserved2 = 1;

    return(errorstatus);
}

/**
  * @brief  Enables wide bus opeartion for the requeseted card if supported by
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode.
  *   This parameter can be one of the following values:
  *     @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
  *     @arg SDIO_BusWide_4b: 4-bit data transfer
  *     @arg SDIO_BusWide_1b: 1-bit data transfer
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_GetCardStatus(SD_CardStatus *cardstatus) {
    SD_Error errorstatus = SD_OK;
    uint8_t tmp = 0;

    errorstatus = SD_SendSDStatus((uint32_t *)SDSTATUS_Tab);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    /*!< Byte 0 */
    tmp = (uint8_t)((SDSTATUS_Tab[0] & 0xC0) >> 6);
    cardstatus->DAT_BUS_WIDTH = tmp;

    /*!< Byte 0 */
    tmp = (uint8_t)((SDSTATUS_Tab[0] & 0x20) >> 5);
    cardstatus->SECURED_MODE = tmp;

    /*!< Byte 2 */
    tmp = (uint8_t)((SDSTATUS_Tab[2] & 0xFF));
    cardstatus->SD_CARD_TYPE = tmp << 8;

    /*!< Byte 3 */
    tmp = (uint8_t)((SDSTATUS_Tab[3] & 0xFF));
    cardstatus->SD_CARD_TYPE |= tmp;

    /*!< Byte 4 */
    tmp = (uint8_t)(SDSTATUS_Tab[4] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA = tmp << 24;

    /*!< Byte 5 */
    tmp = (uint8_t)(SDSTATUS_Tab[5] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 16;

    /*!< Byte 6 */
    tmp = (uint8_t)(SDSTATUS_Tab[6] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 8;

    /*!< Byte 7 */
    tmp = (uint8_t)(SDSTATUS_Tab[7] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp;

    /*!< Byte 8 */
    tmp = (uint8_t)((SDSTATUS_Tab[8] & 0xFF));
    cardstatus->SPEED_CLASS = tmp;

    /*!< Byte 9 */
    tmp = (uint8_t)((SDSTATUS_Tab[9] & 0xFF));
    cardstatus->PERFORMANCE_MOVE = tmp;

    /*!< Byte 10 */
    tmp = (uint8_t)((SDSTATUS_Tab[10] & 0xF0) >> 4);
    cardstatus->AU_SIZE = tmp;

    /*!< Byte 11 */
    tmp = (uint8_t)(SDSTATUS_Tab[11] & 0xFF);
    cardstatus->ERASE_SIZE = tmp << 8;

    /*!< Byte 12 */
    tmp = (uint8_t)(SDSTATUS_Tab[12] & 0xFF);
    cardstatus->ERASE_SIZE |= tmp;

    /*!< Byte 13 */
    tmp = (uint8_t)((SDSTATUS_Tab[13] & 0xFC) >> 2);
    cardstatus->ERASE_TIMEOUT = tmp;

    /*!< Byte 13 */
    tmp = (uint8_t)((SDSTATUS_Tab[13] & 0x3));
    cardstatus->ERASE_OFFSET = tmp;

    return(errorstatus);
}

/**
  * @brief  Enables wide bus opeartion for the requeseted card if supported by
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode.
  *   This parameter can be one of the following values:
  *     @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
  *     @arg SDIO_BusWide_4b: 4-bit data transfer
  *     @arg SDIO_BusWide_1b: 1-bit data transfer
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_EnableWideBusOperation(uint32_t WideMode) {
    SD_Error errorstatus = SD_OK;

    /*!< MMC Card doesn't support this feature */
    if (SDIO_MULTIMEDIA_CARD == CardType) {
        errorstatus = SD_UNSUPPORTED_FEATURE;
        return(errorstatus);
    }
    else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType)) {
        if (SDIO_BusWide_8b == WideMode) {
            errorstatus = SD_UNSUPPORTED_FEATURE;
            return(errorstatus);
        }
        else if (SDIO_BusWide_4b == WideMode) {
            errorstatus = SDEnWideBus(ENABLE);

            if (SD_OK == errorstatus) {
                /*!< Configure the SDMMC1 peripheral */
                SDIO_Init(SDIO_TRANSFER_CLK_DIV, SDIO_ClockPowerSave_Disable, SDIO_ClockBypass_Disable, SDIO_ClockEdge_Rising, SDIO_BusWide_4b, SDIO_HardwareFlowControl_Disable);
            }
        }
        else {
            errorstatus = SDEnWideBus(DISABLE);

            if (SD_OK == errorstatus) {
                /*!< Configure the SDMMC1 peripheral */
                SDIO_Init(SDIO_TRANSFER_CLK_DIV, SDIO_ClockPowerSave_Disable, SDIO_ClockBypass_Disable, SDIO_ClockEdge_Rising, SDIO_BusWide_1b, SDIO_HardwareFlowControl_Disable);
            }
        }
    }

    return(errorstatus);
}

/**
  * @brief  Selects od Deselects the corresponding card.
  * @param  addr: Address of the Card to be selected.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_SelectDeselect(uint32_t addr) {
    SD_Error errorstatus = SD_OK;

    /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
    SDIO_SendCommand(addr, SD_CMD_SEL_DESEL_CARD, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);

    return(errorstatus);
}

/**
  * @brief  Allows to read one block from a specified address in a card. The Data
  *         transfer can be managed by DMA mode or Polling mode.
  * @note   This operation should be followed by two functions to check if the
  *         DMA Controller and SD Card status.
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controllerIndex has finished all data transfer.
  *          - SD_GetStatus(): to check that the SD Card has finished the
  *            data transfer and it is ready for data.
  * @param  readbuff: pointer to the buffer that will contain the received data
  * @param  ReadAddr: Address from where data are to be read.
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_ReadBlock(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize) {
    SD_Error errorstatus = SD_OK;
#if defined (SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t *)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd = 0;
    StopCondition = 0;
    //while(SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOE) != RESET);
    SDMMC1->DCTRL = 0x0;


    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD) {
        BlockSize = 512;
        ReadAddr /= 512;
    }

    /* Set Block Size for Card */
    SDIO_SendCommand((uint32_t)BlockSize, SD_CMD_SET_BLOCKLEN, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus) {
        return(errorstatus);
    }

    SDIO_DataConfig(BlockSize, (uint32_t)9 << 4, SDIO_TransferDir_ToSDIO);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_SendCommand((uint32_t)ReadAddr, SD_CMD_READ_SINGLE_BLOCK, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

#if defined (SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDMMC1->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR))) {
        if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET) {
            for (count = 0; count < 8; count++) {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        return(SD_DATA_TIMEOUT);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        return(SD_DATA_CRC_FAIL);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
        //    errorstatus = SD_RX_OVERRUN;
        //    return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        return(SD_START_BIT_ERR);
    }
    count = SD_DATATIMEOUT;
    while ((SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) && (count > 0)) {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
        count--;
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

#elif defined (SD_DMA_MODE)
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_LowLevel_DMA_RxConfig((uint32_t *)readbuff, BlockSize);
#endif

    return(errorstatus);
}

/**
  * @brief  Allows to write one block starting from a specified address in a card.
  *         The Data transfer can be managed by DMA mode or Polling mode.
  * @note   This operation should be followed by two functions to check if the
  *         DMA Controller and SD Card status.
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controllerIndex has finished all data transfer.
  *          - SD_GetStatus(): to check that the SD Card has finished the
  *            data transfer and it is ready for data.
  * @param  writebuff: pointer to the buffer that contain the data to be transferred.
  * @param  WriteAddr: Address from where data are to be read.
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_WriteBlock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize) {
    SD_Error errorstatus = SD_OK;

#if defined (SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t *tempbuff = (uint32_t *)writebuff;
#endif

    TransferError = SD_OK;
    TransferEnd = 0;
    StopCondition = 0;

    SDMMC1->DCTRL = 0x0;


    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD) {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /* Set Block Size for Card */
    SDIO_SendCommand((uint32_t)BlockSize, SD_CMD_SET_BLOCKLEN, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus) {
        return(errorstatus);
    }

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_SendCommand((uint32_t)WriteAddr, SD_CMD_WRITE_SINGLE_BLOCK, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    SDIO_DataConfig(BlockSize, (uint32_t)9 << 4, SDIO_TransferDir_ToCard);

    /*!< In case of single data block transfer no need of stop command at all */
#if defined (SD_POLLING_MODE)
    while (!(SDMMC1->STA & (SDIO_FLAG_DBCKEND | SDIO_FLAG_TXUNDERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR))) {
        if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET) {
            if ((512 - bytestransferred) < 32) {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4) : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4) {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else {
                for (count = 0; count < 8; count++) {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
            }
        }
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        return(SD_DATA_TIMEOUT);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        return(SD_DATA_CRC_FAIL);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
        // errorstatus = SD_TX_UNDERRUN;
        // return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        return(SD_START_BIT_ERR);
    }
#elif defined (SD_DMA_MODE)
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
    SD_LowLevel_DMA_TxConfig((uint32_t *)writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
#endif

    return(errorstatus);
}

/**
  * @brief  Gets the cuurent data transfer state.
  * @param  None
  * @retval SDTransferState: Data Transfer state.
  *   This value can be:
  *        - SD_TRANSFER_OK: No data transfer is acting
  *        - SD_TRANSFER_BUSY: Data transfer is acting
  */
SDTransferState SD_GetTransferState(void) {
    if (SDMMC1->STA & (SDIO_FLAG_TXACT | SDIO_FLAG_RXACT)) {
        return(SD_TRANSFER_BUSY);
    }
    else {
        return(SD_TRANSFER_OK);
    }
}

/**
  * @brief  Aborts an ongoing data transfer.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_StopTransfer(void) {
    SD_Error errorstatus = SD_OK;

    /*!< Send CMD12 STOP_TRANSMISSION  */
    SDIO_SendCommand(0x0, SD_CMD_STOP_TRANSMISSION, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);

    return(errorstatus);
}

/**
  * @brief  Returns the current card's status.
  * @param  pcardstatus: pointer to the buffer that will contain the SD card
  *         status (Card Status register).
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_SendStatus(uint32_t *pcardstatus) {
    SD_Error errorstatus = SD_OK;

    if (pcardstatus == nullptr) {
        errorstatus = SD_INVALID_PARAMETER;
        return(errorstatus);
    }

    SDIO_SendCommand((uint32_t)RCA << 16, SD_CMD_SEND_STATUS, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    *pcardstatus = SDIO_GetResponse(SDIO_RESP1);

    return(errorstatus);
}

/**
  * @brief  Returns the current SD card's status.
  * @param  psdstatus: pointer to the buffer that will contain the SD card status
  *         (SD Status register).
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_SendSDStatus(uint32_t *psdstatus) {
    SD_Error errorstatus = SD_OK;
    uint32_t count = 0;

    if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED) {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return(errorstatus);
    }

    /*!< Set block size for card if it is not equal to current block size for card. */
    SDIO_SendCommand(64, SD_CMD_SET_BLOCKLEN, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    /*!< CMD55 */
    SDIO_SendCommand((uint32_t)RCA << 16, SD_CMD_APP_CMD, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    SDIO_DataConfig(64, SDIO_DataBlockSize_64b, SDIO_TransferDir_ToSDIO);

    /*!< Send ACMD13 SD_APP_STAUS  with argument as card's RCA.*/
    SDIO_SendCommand(0, SD_CMD_SD_APP_STAUS, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SD_APP_STAUS);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    while (!(SDMMC1->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR))) {
        if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET) {
            for (count = 0; count < 8; count++) {
                *(psdstatus + count) = SDIO_ReadData();
            }
            psdstatus += 8;
        }
    }

    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        return(SD_DATA_TIMEOUT);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        return(SD_DATA_CRC_FAIL);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
        return(SD_RX_OVERRUN);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        return(SD_START_BIT_ERR);
    }

    count = SD_DATATIMEOUT;
    while ((SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) && (count > 0)) {
        *psdstatus = SDIO_ReadData();
        psdstatus++;
        count--;
    }
    /*!< Clear all the static status flags*/
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

/**
  * @brief  Checks for error conditions for CMD0.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdError(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t timeout;

    timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */

    while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET)) {
        timeout--;
    }

    if (timeout == 0) {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        return(errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R7 response.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp7Error(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t timeout = SDIO_CMD0TIMEOUT;

    status = SDMMC1->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0)) {
        timeout--;
        status = SDMMC1->STA;
    }

    if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT)) {
        /*!< Card is not V2.0 complient or card does not support the set voltage range */
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }

    if (status & SDIO_FLAG_CMDREND) {
        /*!< Card is SD V2.0 compliant */
        errorstatus = SD_OK;
        SDIO_ClearFlag(SDIO_FLAG_CMDREND);
        return(errorstatus);
    }
    return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R1 response.
  * @param  cmd: The sent command index.
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp1Error(uint8_t cmd) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t response_r1;

    status = SDMMC1->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT))) {
        status = SDMMC1->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT) {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL) {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        return(errorstatus);
    }

    /*!< Check response received is of desired command */
    if (SDIO_GetCommandResponse() != cmd) {
        errorstatus = SD_ILLEGAL_CMD;
        return(errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    /*!< We have received response, retrieve it for analysis  */
    response_r1 = SDIO_GetResponse(SDIO_RESP1);

    if ((response_r1 & SD_OCR_ERRORBITS) == SD_ALLZERO) {
        return(errorstatus);
    }

    if (response_r1 & SD_OCR_ADDR_OUT_OF_RANGE) {
        return(SD_ADDR_OUT_OF_RANGE);
    }

    if (response_r1 & SD_OCR_ADDR_MISALIGNED) {
        return(SD_ADDR_MISALIGNED);
    }

    if (response_r1 & SD_OCR_BLOCK_LEN_ERR) {
        return(SD_BLOCK_LEN_ERR);
    }

    if (response_r1 & SD_OCR_ERASE_SEQ_ERR) {
        return(SD_ERASE_SEQ_ERR);
    }

    if (response_r1 & SD_OCR_BAD_ERASE_PARAM) {
        return(SD_BAD_ERASE_PARAM);
    }

    if (response_r1 & SD_OCR_WRITE_PROT_VIOLATION) {
        return(SD_WRITE_PROT_VIOLATION);
    }

    if (response_r1 & SD_OCR_LOCK_UNLOCK_FAILED) {
        return(SD_LOCK_UNLOCK_FAILED);
    }

    if (response_r1 & SD_OCR_COM_CRC_FAILED) {
        return(SD_COM_CRC_FAILED);
    }

    if (response_r1 & SD_OCR_ILLEGAL_CMD) {
        return(SD_ILLEGAL_CMD);
    }

    if (response_r1 & SD_OCR_CARD_ECC_FAILED) {
        return(SD_CARD_ECC_FAILED);
    }

    if (response_r1 & SD_OCR_CC_ERROR) {
        return(SD_CC_ERROR);
    }

    if (response_r1 & SD_OCR_GENERAL_UNKNOWN_ERROR) {
        return(SD_GENERAL_UNKNOWN_ERROR);
    }

    if (response_r1 & SD_OCR_STREAM_READ_UNDERRUN) {
        return(SD_STREAM_READ_UNDERRUN);
    }

    if (response_r1 & SD_OCR_STREAM_WRITE_OVERRUN) {
        return(SD_STREAM_WRITE_OVERRUN);
    }

    if (response_r1 & SD_OCR_CID_CSD_OVERWRIETE) {
        return(SD_CID_CSD_OVERWRITE);
    }

    if (response_r1 & SD_OCR_WP_ERASE_SKIP) {
        return(SD_WP_ERASE_SKIP);
    }

    if (response_r1 & SD_OCR_CARD_ECC_DISABLED) {
        return(SD_CARD_ECC_DISABLED);
    }

    if (response_r1 & SD_OCR_ERASE_RESET) {
        return(SD_ERASE_RESET);
    }

    if (response_r1 & SD_OCR_AKE_SEQ_ERROR) {
        return(SD_AKE_SEQ_ERROR);
    }
    return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R3 (OCR) response.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp3Error(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDMMC1->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT))) {
        status = SDMMC1->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT) {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);
    return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R2 (CID or CSD) response.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp2Error(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDMMC1->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND))) {
        status = SDMMC1->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT) {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL) {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        return(errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R6 (RCA) response.
  * @param  cmd: The sent command index.
  * @param  prca: pointer to the variable that will contain the SD card relative
  *         address RCA.
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t response_r1;

    status = SDMMC1->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND))) {
        status = SDMMC1->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT) {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL) {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        return(errorstatus);
    }

    /*!< Check response received is of desired command */
    if (SDIO_GetCommandResponse() != cmd) {
        errorstatus = SD_ILLEGAL_CMD;
        return(errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    /*!< We have received response, retrieve it.  */
    response_r1 = SDIO_GetResponse(SDIO_RESP1);

    if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED))) {
        *prca = (uint16_t)(response_r1 >> 16);
        return(errorstatus);
    }

    if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR) {
        return(SD_GENERAL_UNKNOWN_ERROR);
    }

    if (response_r1 & SD_R6_ILLEGAL_CMD) {
        return(SD_ILLEGAL_CMD);
    }

    if (response_r1 & SD_R6_COM_CRC_FAILED) {
        return(SD_COM_CRC_FAILED);
    }

    return(errorstatus);
}

/**
  * @brief  Enables or disables the SDMMC1 wide bus mode.
  * @param  newState: new state of the SDMMC1 wide bus mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error SDEnWideBus(FunctionalState newState) {
    SD_Error errorstatus = SD_OK;

    uint32_t scr[2] = { 0, 0 };

    if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED) {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return(errorstatus);
    }

    /*!< Get SCR Register */
    errorstatus = FindSCR(RCA, scr);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    /*!< If wide bus operation to be enabled */
    if (newState == ENABLE) {
        /*!< If requested card supports wide bus operation */
        if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO) {
            /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
            SDIO_SendCommand((uint32_t)RCA << 16, SD_CMD_APP_CMD, SDIO_Response_Short);

            errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

            if (errorstatus != SD_OK) {
                return(errorstatus);
            }

            /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
            SDIO_SendCommand(0x2, SD_CMD_APP_SD_SET_BUSWIDTH, SDIO_Response_Short);

            errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

            if (errorstatus != SD_OK) {
                return(errorstatus);
            }
            return(errorstatus);
        }
        else {
            errorstatus = SD_REQUEST_NOT_APPLICABLE;
            return(errorstatus);
        }
    }   /*!< If wide bus operation to be disabled */
    else {
        /*!< If requested card supports 1 bit mode operation */
        if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO) {
            /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
            SDIO_SendCommand((uint32_t)RCA << 16, SD_CMD_APP_CMD, SDIO_Response_Short);


            errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

            if (errorstatus != SD_OK) {
                return(errorstatus);
            }

            /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
            SDIO_SendCommand(0x00, SD_CMD_APP_SD_SET_BUSWIDTH, SDIO_Response_Short);

            errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

            if (errorstatus != SD_OK) {
                return(errorstatus);
            }

            return(errorstatus);
        }
        else {
            errorstatus = SD_REQUEST_NOT_APPLICABLE;
            return(errorstatus);
        }
    }
}

/**
  * @brief  Find the SD card SCR register value.
  * @param  rca: selected card address.
  * @param  pscr: pointer to the buffer that will contain the SCR value.
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr) {
    uint32_t index = 0;
    SD_Error errorstatus = SD_OK;
    uint32_t tempscr[2] = { 0, 0 };

    /*!< Set Block Size To 8 Bytes */
    /*!< Send CMD55 APP_CMD with argument as card's RCA */
    SDIO_SendCommand(0x8, SD_CMD_SET_BLOCKLEN, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    /*!< Send CMD55 APP_CMD with argument as card's RCA */
    SDIO_SendCommand((uint32_t)RCA << 16, SD_CMD_APP_CMD, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    SDIO_DataConfig(8, SDIO_DataBlockSize_8b, SDIO_TransferDir_ToSDIO);


    /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
    SDIO_SendCommand(0x0, SD_CMD_SD_APP_SEND_SCR, SDIO_Response_Short);

    errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);

    if (errorstatus != SD_OK) {
        return(errorstatus);
    }

    while (!(SDMMC1->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR))) {
        if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) {
            *(tempscr + index) = SDIO_ReadData();
            index++;
        }
    }

    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        return(SD_DATA_TIMEOUT);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        return(SD_DATA_CRC_FAIL);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
        return(SD_RX_OVERRUN);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        return(SD_START_BIT_ERR);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

    *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

    return(errorstatus);
}

// stm32f7

#define STM32F7_SD_SECTOR_SIZE 512
#define STM32F7_SD_TIMEOUT 5000000
#define TOTAL_SDCARD_CONTROLLERS 1

static TinyCLR_Storage_Controller sdCardControllers[TOTAL_SDCARD_CONTROLLERS];
static TinyCLR_Api_Info sdCardApi[TOTAL_SDCARD_CONTROLLERS];

struct SdCardState {
    int32_t controllerIndex;

    uint64_t *regionAddresses;
    size_t  *regionSizes;

    TinyCLR_Storage_Descriptor descriptor;

    bool isOpened = false;
};

static const STM32F7_Gpio_Pin sdCardData0Pins[] = STM32F7_SD_DATA0_PINS;
static const STM32F7_Gpio_Pin sdCardData1Pins[] = STM32F7_SD_DATA1_PINS;
static const STM32F7_Gpio_Pin sdCardData2Pins[] = STM32F7_SD_DATA2_PINS;
static const STM32F7_Gpio_Pin sdCardData3Pins[] = STM32F7_SD_DATA3_PINS;
static const STM32F7_Gpio_Pin sdCardClkPins[] = STM32F7_SD_CLK_PINS;
static const STM32F7_Gpio_Pin sdCardCmdPins[] = STM32F7_SD_CMD_PINS;

static SdCardState sdCardStates[TOTAL_SDCARD_CONTROLLERS];

void STM32F7_SdCard_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_SDCARD_CONTROLLERS; i++) {
        sdCardControllers[i].ApiInfo = &sdCardApi[i];
        sdCardControllers[i].Acquire = &STM32F7_SdCard_Acquire;
        sdCardControllers[i].Release = &STM32F7_SdCard_Release;
        sdCardControllers[i].Open = &STM32F7_SdCard_Open;
        sdCardControllers[i].Close = &STM32F7_SdCard_Close;
        sdCardControllers[i].Write = &STM32F7_SdCard_Write;
        sdCardControllers[i].Read = &STM32F7_SdCard_Read;
        sdCardControllers[i].Erase = &STM32F7_SdCard_Erases;
        sdCardControllers[i].IsErased = &STM32F7_SdCard_IsErased;
        sdCardControllers[i].GetDescriptor = &STM32F7_SdCard_GetDescriptor;
        sdCardControllers[i].IsPresent = &STM32F7_SdCard_IsPresent;
        sdCardControllers[i].SetPresenceChangedHandler = &STM32F7_SdCard_SetPresenceChangedHandler;

        sdCardApi[i].Author = "GHI Electronics, LLC";
        sdCardApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.SdCardStorageController";
        sdCardApi[i].Type = TinyCLR_Api_Type::StorageController;
        sdCardApi[i].Version = 0;
        sdCardApi[i].Implementation = &sdCardControllers[i];
        sdCardApi[i].State = &sdCardStates[i];

        sdCardStates[i].controllerIndex = i;
    }

    
}

TinyCLR_Result STM32F7_SdCard_Acquire(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<SdCardState*>(self->ApiInfo->State);

    if (state->isOpened) return TinyCLR_Result::SharingViolation;

    auto controllerIndex = state->controllerIndex;

    auto d0 = sdCardData0Pins[controllerIndex];
    auto d1 = sdCardData1Pins[controllerIndex];
    auto d2 = sdCardData2Pins[controllerIndex];
    auto d3 = sdCardData3Pins[controllerIndex];
    auto clk = sdCardClkPins[controllerIndex];
    auto cmd = sdCardCmdPins[controllerIndex];

    if (!STM32F7_GpioInternal_OpenPin(d0.number)
        || !STM32F7_GpioInternal_OpenPin(d1.number)
        || !STM32F7_GpioInternal_OpenPin(d2.number)
        || !STM32F7_GpioInternal_OpenPin(d3.number)
        || !STM32F7_GpioInternal_OpenPin(clk.number)
        || !STM32F7_GpioInternal_OpenPin(cmd.number)
        )
        return TinyCLR_Result::SharingViolation;

    STM32F7_GpioInternal_ConfigurePin(d0.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::High, STM32F7_Gpio_PullDirection::PullUp, d0.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(d1.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::High, STM32F7_Gpio_PullDirection::PullUp, d1.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(d2.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::High, STM32F7_Gpio_PullDirection::PullUp, d2.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(d3.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::High, STM32F7_Gpio_PullDirection::PullUp, d3.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(clk.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::High, STM32F7_Gpio_PullDirection::None, clk.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(cmd.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::High, STM32F7_Gpio_PullDirection::PullUp, cmd.alternateFunction);

    RCC->APB2ENR |= (1 << 11);

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    state->regionAddresses = (uint64_t*)memoryProvider->Allocate(memoryProvider, sizeof(uint64_t));
    state->regionSizes = (size_t*)memoryProvider->Allocate(memoryProvider, sizeof(size_t));

    state->descriptor.CanReadDirect = true;
    state->descriptor.CanWriteDirect = true;
    state->descriptor.CanExecuteDirect = false;
    state->descriptor.EraseBeforeWrite = false;
    state->descriptor.Removable = true;
    state->descriptor.RegionsRepeat = true;

    state->descriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(state->regionAddresses);
    state->descriptor.RegionSizes = reinterpret_cast<const size_t*>(state->regionSizes);

    SD_DeInit();

    if (SD_Init() == SD_OK) {
        state->isOpened = true;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result STM32F7_SdCard_Release(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<SdCardState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    auto d0 = sdCardData0Pins[controllerIndex];
    auto d1 = sdCardData1Pins[controllerIndex];
    auto d2 = sdCardData2Pins[controllerIndex];
    auto d3 = sdCardData3Pins[controllerIndex];
    auto clk = sdCardClkPins[controllerIndex];
    auto cmd = sdCardCmdPins[controllerIndex];

    SD_DeInit();

    RCC->APB2ENR &= ~(1 << 11);

    if (state->isOpened) {
        auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

        memoryProvider->Free(memoryProvider, state->regionSizes);
        memoryProvider->Free(memoryProvider, state->regionAddresses);
    }

    STM32F7_GpioInternal_ClosePin(d0.number);
    STM32F7_GpioInternal_ClosePin(d1.number);
    STM32F7_GpioInternal_ClosePin(d2.number);
    STM32F7_GpioInternal_ClosePin(d3.number);
    STM32F7_GpioInternal_ClosePin(clk.number);
    STM32F7_GpioInternal_ClosePin(cmd.number);

    state->isOpened = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    int32_t index = 0;

    int32_t to;

    auto sectorCount = count;

    auto sectorNum = address;

    uint8_t* pData = (uint8_t*)data;

    while (sectorCount) {
        to = timeout;

        while (SD_GetStatus() != SD_TRANSFER_OK && to--) {
            STM32F7_Time_Delay(nullptr, 1);
        }

        if (to > 0 && SD_WriteBlock(&pData[index], sectorNum * STM32F7_SD_SECTOR_SIZE, STM32F7_SD_SECTOR_SIZE) == SD_OK) {
            index += STM32F7_SD_SECTOR_SIZE;
            sectorNum++;
            sectorCount--;
        }
        else {
            SD_StopTransfer();
        }

        if (!to) {
            return TinyCLR_Result::TimedOut;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    int32_t index = 0;

    int32_t to;

    auto sectorCount = count;

    auto sectorNum = address;

    while (sectorCount) {
        to = timeout;

        while (SD_GetStatus() != SD_TRANSFER_OK && to--) {
            STM32F7_Time_Delay(nullptr, 1);
        }

        if (to > 0 && SD_ReadBlock(&data[index], sectorNum * STM32F7_SD_SECTOR_SIZE, STM32F7_SD_SECTOR_SIZE) == SD_OK) {
            index += STM32F7_SD_SECTOR_SIZE;
            sectorNum++;
            sectorCount--;
        }
        else {
            SD_StopTransfer();
        }

        if (!to) {
            return TinyCLR_Result::TimedOut;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, bool& erased) {
    erased = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_Erases(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    auto state = reinterpret_cast<SdCardState*>(self->ApiInfo->State);

    state->regionSizes[0] = STM32F7_SD_SECTOR_SIZE;
    state->descriptor.RegionCount = SDCardInfo.CardCapacity / STM32F7_SD_SECTOR_SIZE;

    descriptor = reinterpret_cast<const TinyCLR_Storage_Descriptor*>(&state->descriptor);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_Open(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_Close(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_IsPresent(const TinyCLR_Storage_Controller* self, bool& present) {
    present = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_SdCard_Reset() {
    return TinyCLR_Result::Success;
}
#endif // INCLUDE_SD