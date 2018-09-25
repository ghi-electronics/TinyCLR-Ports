// Copyright Microsoft Corporation
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

#include <string.h>

#include "LPC24.h"

#ifdef INCLUDE_SD
//LPC24
#define MCIPower (*(volatile unsigned long *)0xE008C000)
#define MCIPower_OFFSET 0x0
#define MCIPower_Ctrl_MASK 0x3
#define MCIPower_Ctrl_BIT 0
#define MCIPower_OpenDrain_MASK 0x40
#define MCIPower_OpenDrain 0x40
#define MCIPower_OpenDrain_BIT 6
#define MCIPower_Rod_MASK 0x80
#define MCIPower_Rod 0x80
#define MCIPower_Rod_BIT 7

#define MCIClock (*(volatile unsigned long *)0xE008C004)
#define MCIClock_OFFSET 0x4
#define MCIClock_ClkDiv_MASK 0xFF
#define MCIClock_ClkDiv_BIT 0
#define MCIClock_Enable_MASK 0x100
#define MCIClock_Enable 0x100
#define MCIClock_Enable_BIT 8
#define MCIClock_PwrSave_MASK 0x200
#define MCIClock_PwrSave 0x200
#define MCIClock_PwrSave_BIT 9
#define MCIClock_Bypass_MASK 0x400
#define MCIClock_Bypass 0x400
#define MCIClock_Bypass_BIT 10
#define MCIClock_WideBus_MASK 0x800
#define MCIClock_WideBus 0x800
#define MCIClock_WideBus_BIT 11

#define MCIArgument (*(volatile unsigned long *)0xE008C008)
#define MCIArgument_OFFSET 0x8

#define MCICommand (*(volatile unsigned long *)0xE008C00C)
#define MCICommand_OFFSET 0xC
#define MCICommand_CmdIndex_MASK 0x3F
#define MCICommand_CmdIndex_BIT 0
#define MCICommand_Response_MASK 0x40
#define MCICommand_Response 0x40
#define MCICommand_Response_BIT 6
#define MCICommand_LongRsp_MASK 0x80
#define MCICommand_LongRsp 0x80
#define MCICommand_LongRsp_BIT 7
#define MCICommand_Interrupt_MASK 0x100
#define MCICommand_Interrupt 0x100
#define MCICommand_Interrupt_BIT 8
#define MCICommand_Pending_MASK 0x200
#define MCICommand_Pending 0x200
#define MCICommand_Pending_BIT 9
#define MCICommand_Enable_MASK 0x400
#define MCICommand_Enable 0x400
#define MCICommand_Enable_BIT 10

#define MCIRespCmd (*(volatile unsigned long *)0xE008C010)
#define MCIRespCmd_OFFSET 0x10
#define MCIRespCmd_RespCmd_MASK 0x3F
#define MCIRespCmd_RespCmd_BIT 0

#define MCIResponse20 (*(volatile unsigned long *)0xE008C014)
#define MCIResponse20_OFFSET 0x14

#define MCIResponse24 (*(volatile unsigned long *)0xE008C014)
#define MCIResponse24_OFFSET 0x14

#define MCIResponse28 (*(volatile unsigned long *)0xE008C014)
#define MCIResponse28_OFFSET 0x14

#define MCIResponse32 (*(volatile unsigned long *)0xE008C014)
#define MCIResponse32_OFFSET 0x14

#define MCIDataTimer (*(volatile unsigned long *)0xE008C024)
#define MCIDataTimer_OFFSET 0x24

#define MCIDataLength (*(volatile unsigned long *)0xE008C028)
#define MCIDataLength_OFFSET 0x28
#define MCIDataLength_DataLength_MASK 0xFFFF
#define MCIDataLength_DataLength_BIT 0

#define MCIDataCtrl (*(volatile unsigned long *)0xE008C02C)
#define MCIDataCtrl_OFFSET 0x2C
#define MCIDataCtrl_Enable_MASK 0x1
#define MCIDataCtrl_Enable 0x1
#define MCIDataCtrl_Enable_BIT 0
#define MCIDataCtrl_Direction_MASK 0x2
#define MCIDataCtrl_Direction 0x2
#define MCIDataCtrl_Direction_BIT 1
#define MCIDataCtrl_Mode_MASK 0x4
#define MCIDataCtrl_Mode 0x4
#define MCIDataCtrl_Mode_BIT 2
#define MCIDataCtrl_DMAEnable_MASK 0x8
#define MCIDataCtrl_DMAEnable 0x8
#define MCIDataCtrl_DMAEnable_BIT 3
#define MCIDataCtrl_BlockSize_MASK 0xF0
#define MCIDataCtrl_BlockSize_BIT 4

#define MCIDataCnt (*(volatile unsigned long *)0xE008C030)
#define MCIDataCnt_OFFSET 0x30
#define MCIDataCnt_DataCount_MASK 0xFFFF
#define MCIDataCnt_DataCount_BIT 0

#define MCIStatus (*(volatile unsigned long *)0xE008C034)
#define MCIStatus_OFFSET 0x34
#define MCIStatus_CmdCrcFail_MASK 0x1
#define MCIStatus_CmdCrcFail 0x1
#define MCIStatus_CmdCrcFail_BIT 0
#define MCIStatus_DataCrcFail_MASK 0x2
#define MCIStatus_DataCrcFail 0x2
#define MCIStatus_DataCrcFail_BIT 1
#define MCIStatus_CmdTimeOut_MASK 0x4
#define MCIStatus_CmdTimeOut 0x4
#define MCIStatus_CmdTimeOut_BIT 2
#define MCIStatus_DataTimeOut_MASK 0x8
#define MCIStatus_DataTimeOut 0x8
#define MCIStatus_DataTimeOut_BIT 3
#define MCIStatus_TxUnderrun_MASK 0x10
#define MCIStatus_TxUnderrun 0x10
#define MCIStatus_TxUnderrun_BIT 4
#define MCIStatus_RxOverrun_MASK 0x20
#define MCIStatus_RxOverrun 0x20
#define MCIStatus_RxOverrun_BIT 5
#define MCIStatus_CmdRespEnd_MASK 0x40
#define MCIStatus_CmdRespEnd 0x40
#define MCIStatus_CmdRespEnd_BIT 6
#define MCIStatus_CmdSent_MASK 0x80
#define MCIStatus_CmdSent 0x80
#define MCIStatus_CmdSent_BIT 7
#define MCIStatus_DataEnd_MASK 0x100
#define MCIStatus_DataEnd 0x100
#define MCIStatus_DataEnd_BIT 8
#define MCIStatus_StartBitErr_MASK 0x200
#define MCIStatus_StartBitErr 0x200
#define MCIStatus_StartBitErr_BIT 9
#define MCIStatus_DataBlockEnd_MASK 0x400
#define MCIStatus_DataBlockEnd 0x400
#define MCIStatus_DataBlockEnd_BIT 10
#define MCIStatus_CmdActive_MASK 0x800
#define MCIStatus_CmdActive 0x800
#define MCIStatus_CmdActive_BIT 11
#define MCIStatus_TxActive_MASK 0x1000
#define MCIStatus_TxActive 0x1000
#define MCIStatus_TxActive_BIT 12
#define MCIStatus_RxActive_MASK 0x2000
#define MCIStatus_RxActive 0x2000
#define MCIStatus_RxActive_BIT 13
#define MCIStatus_TxFifoHalfEmpty_MASK 0x4000
#define MCIStatus_TxFifoHalfEmpty 0x4000
#define MCIStatus_TxFifoHalfEmpty_BIT 14
#define MCIStatus_RxFifoHalfEmpty_MASK 0x8000
#define MCIStatus_RxFifoHalfEmpty 0x8000
#define MCIStatus_RxFifoHalfEmpty_BIT 15
#define MCIStatus_TxFifoFull_MASK 0x10000
#define MCIStatus_TxFifoFull 0x10000
#define MCIStatus_TxFifoFull_BIT 16
#define MCIStatus_RxFifoFull_MASK 0x20000
#define MCIStatus_RxFifoFull 0x20000
#define MCIStatus_RxFifoFull_BIT 17
#define MCIStatus_TxFifoEmpty_MASK 0x40000
#define MCIStatus_TxFifoEmpty 0x40000
#define MCIStatus_TxFifoEmpty_BIT 18
#define MCIStatus_RxFifoEmpty_MASK 0x80000
#define MCIStatus_RxFifoEmpty 0x80000
#define MCIStatus_RxFifoEmpty_BIT 19
#define MCIStatus_TxDataAvlbl_MASK 0x100000
#define MCIStatus_TxDataAvlbl 0x100000
#define MCIStatus_TxDataAvlbl_BIT 20
#define MCIStatus_RxDataAvlbl_MASK 0x200000
#define MCIStatus_RxDataAvlbl 0x200000
#define MCIStatus_RxDataAvlbl_BIT 21

#define MCIClear (*(volatile unsigned long *)0xE008C038)
#define MCIClear_OFFSET 0x38
#define MCIClear_CmdCrcFailClr_MASK 0x1
#define MCIClear_CmdCrcFailClr 0x1
#define MCIClear_CmdCrcFailClr_BIT 0
#define MCIClear_DataCrcFailClr_MASK 0x2
#define MCIClear_DataCrcFailClr 0x2
#define MCIClear_DataCrcFailClr_BIT 1
#define MCIClear_CmdTimeOutClr_MASK 0x4
#define MCIClear_CmdTimeOutClr 0x4
#define MCIClear_CmdTimeOutClr_BIT 2
#define MCIClear_DataTimeOutClr_MASK 0x8
#define MCIClear_DataTimeOutClr 0x8
#define MCIClear_DataTimeOutClr_BIT 3
#define MCIClear_TxUnderrunClr_MASK 0x10
#define MCIClear_TxUnderrunClr 0x10
#define MCIClear_TxUnderrunClr_BIT 4
#define MCIClear_RxOverrunClr_MASK 0x20
#define MCIClear_RxOverrunClr 0x20
#define MCIClear_RxOverrunClr_BIT 5
#define MCIClear_CmdRespEndClr_MASK 0x40
#define MCIClear_CmdRespEndClr 0x40
#define MCIClear_CmdRespEndClr_BIT 6
#define MCIClear_CmdSentClr_MASK 0x80
#define MCIClear_CmdSentClr 0x80
#define MCIClear_CmdSentClr_BIT 7
#define MCIClear_DataEndClr_MASK 0x100
#define MCIClear_DataEndClr 0x100
#define MCIClear_DataEndClr_BIT 8
#define MCIClear_StartBitErrClr_MASK 0x200
#define MCIClear_StartBitErrClr 0x200
#define MCIClear_StartBitErrClr_BIT 9
#define MCIClear_DataBlockEndClr_MASK 0x400
#define MCIClear_DataBlockEndClr 0x400
#define MCIClear_DataBlockEndClr_BIT 10

#define MCIMask0 (*(volatile unsigned long *)0xE008C03C)
#define MCIMask0_OFFSET 0x3C

#define MCIMask1 (*(volatile unsigned long *)0xE008C040)
#define MCIMask1_OFFSET 0x40

#define MCIFifoCnt (*(volatile unsigned long *)0xE008C048)
#define MCIFifoCnt_OFFSET 0x48
#define MCIFifoCnt_DataCount_MASK 0x7FFF
#define MCIFifoCnt_DataCount_BIT 0

#define MCIFIFO (*(volatile unsigned long *)0xE008C080)
#define MCIFIFO_OFFSET 0x80

// DMA
#define DATA_BLOCK_LEN        9
#define BLOCK_LENGTH        (1 << DATA_BLOCK_LEN)

#define BLOCK_NUM            0x80
#define FIFO_SIZE            16

#define MAX_GPDMA_CHANNELS 8

#define DMA_SRC            ((0x7FD04000 - 512))

#define DMA_DST            DMA_SRC

#define DMA_MCIFIFO        0xE008C080
#define DMA_SIZE        BLOCK_LENGTH

/* DMA mode */
#define M2M                0x00
#define M2P                0x01
#define P2M                0x02
#define P2P                0x03

/* General-purpose DMA Controller */
#define DMA_BASE_ADDR        0xFFE04000
#define GPDMA_INT_STAT         (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x000))
#define GPDMA_INT_TCSTAT       (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x004))
#define GPDMA_INT_TCCLR        (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x008))
#define GPDMA_INT_ERR_STAT     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x00C))
#define GPDMA_INT_ERR_CLR      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x010))
#define GPDMA_RAW_INT_TCSTAT   (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x014))
#define GPDMA_RAW_INT_ERR_STAT (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x018))
#define GPDMA_ENABLED_CHNS     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x01C))
#define GPDMA_SOFT_BREQ        (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x020))
#define GPDMA_SOFT_SREQ        (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x024))
#define GPDMA_SOFT_LBREQ       (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x028))
#define GPDMA_SOFT_LSREQ       (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x02C))
#define GPDMA_CONFIG           (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x030))
#define GPDMA_SYNC             (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x034))

/* DMA channel 0 registers */
#define GPDMA_CH0_SRC      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x100))
#define GPDMA_CH0_DEST     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x104))
#define GPDMA_CH0_LLI      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x108))
#define GPDMA_CH0_CTRL     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x10C))
#define GPDMA_CH0_CFG      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x110))

/* DMA channel 1 registers */
#define GPDMA_CH1_SRC      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x120))
#define GPDMA_CH1_DEST     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x124))
#define GPDMA_CH1_LLI      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x128))
#define GPDMA_CH1_CTRL     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x12C))
#define GPDMA_CH1_CFG      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x130))

#define GPDMA_Source_Register_Channel(ChannelNumber)            (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x100 + (ChannelNumber * 0x20)))
#define GPDMA_Destination_Register_Channel(ChannelNumber)        (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x104 + (ChannelNumber * 0x20)))
#define GPDMA_LinkedListItem_Register_Channel(ChannelNumber)    (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x108 + (ChannelNumber * 0x20)))
#define GPDMA_Control_Register_Channel(ChannelNumber)            (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x10C + (ChannelNumber * 0x20)))
#define GPDMA_Config_Register_Channel(ChannelNumber)            (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x110 + (ChannelNumber * 0x20)))

/******************************************************************************
** Function name:        DMAHandler
**
** Descriptions:        DMA interrupt handler for MCI
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/

void DMAHandler(void * p) {
    uint32_t regVal;
    static uint32_t DMATCCount, DMAErrCount;

    regVal = GPDMA_INT_TCSTAT;
    if (regVal) {
        DMATCCount++;
        GPDMA_INT_TCCLR |= regVal;
    }

    regVal = GPDMA_INT_ERR_STAT;
    if (regVal) {
        DMAErrCount++;
        GPDMA_INT_ERR_CLR |= regVal;
    }
}

/******************************************************************************
** Function name:        DMA_Init
**
** Descriptions:        Setup GPDMA clock and install interrupt handler
**
** parameters:            None
** Returned value:        true or false, false if ISR can't be installed.
**
******************************************************************************/
void DMA_Init(void) {
    LPC24XX::SYSCON().PCONP |= (1 << 29);

    GPDMA_INT_TCCLR = 0xFF;
    GPDMA_INT_ERR_CLR = 0xFF;

    GPDMA_CONFIG = 0x01;

    while (!(GPDMA_CONFIG & 0x01));

    LPC24_InterruptInternal_Activate(LPC24XX_VIC::c_IRQ_INDEX_DMA, (uint32_t*)&DMAHandler, (void*)0);
}

/******************************************************************************
** Function name:        DMA_Move
**
** Descriptions:        Setup GPDMA for MCI DMA transfer
**                        including mode, M2P or M2M, or P2M,
**                        src and dest. address, control reg. etc.
**
** parameters:            Channel number, DMA mode
** Returned value:        true or false
**
******************************************************************************/
uint32_t DMA_Move(uint32_t ChannelNum, uint32_t DMAMode) {

    GPDMA_INT_TCCLR = 0xFF;
    GPDMA_INT_ERR_CLR = 0xFF;

    if (DMAMode == M2M) {
        GPDMA_Source_Register_Channel(ChannelNum) = DMA_SRC;
        GPDMA_Destination_Register_Channel(ChannelNum) = DMA_DST;

        GPDMA_Control_Register_Channel(ChannelNum) = (0x80000000) |
            (0x01 << 27) |
            (0x01 << 26) |
            (0x02 << 21) |
            (0x02 << 18) |
            (0x04 << 15) |
            (0x04 << 12) |
            (DMA_SIZE & 0x0FFF);
    }
    else if (DMAMode == M2P) {
        GPDMA_Source_Register_Channel(ChannelNum) = DMA_SRC;
        GPDMA_Destination_Register_Channel(ChannelNum) = DMA_MCIFIFO;

        GPDMA_Control_Register_Channel(ChannelNum) = (0x80000000) |
            (0x01 << 26) |
            (0x02 << 21) |
            (0x02 << 18) |
            (0x02 << 15) |
            (0x04 << 12) |
            (DMA_SIZE & 0x0FFF);


        GPDMA_Config_Register_Channel(ChannelNum) = (0x01 << 16) |
            (0x05 << 11) |
            (0x01 << 6) |
            (0x00 << 1) |
            (0x01 << 0);
    }
    else if (DMAMode == P2M) {

        GPDMA_Source_Register_Channel(ChannelNum) = DMA_MCIFIFO;
        GPDMA_Destination_Register_Channel(ChannelNum) = DMA_DST;

        GPDMA_Control_Register_Channel(ChannelNum) = (0x80000000) |
            (0x01 << 27) |
            (0x02 << 21) |
            (0x02 << 18) |
            (0x04 << 15) |
            (0x02 << 12) |
            (DMA_SIZE & 0x0FFF);

        GPDMA_Config_Register_Channel(ChannelNum) = (0x01 << 16) |
            (0x06 << 11) |
            (0x00 << 6) |
            (0x01 << 1) |
            (0x01 << 0);
    }
    else {
        return (false);
    }

    GPDMA_CONFIG = 0x01;

    while (!(GPDMA_CONFIG & 0x01));

    return (true);
}

// MCI
#define TIME_OUT 2000
#define READ_TIME_OUT 100
#define WRITE_TIME_OUT 100
#define CheckStatus_TIME_OUT 50
#define OP_COND_COUNT 0x200
#define ACMD_OP_COND_COUNT 0x40
#define Send_Status_COUNT 0x100

#define MCI_FIFO_BASE        MCI_BASE_ADDR + 0x80
#define MCI_FIFO_INDEX        0x04

#define GO_IDLE_STATE        0        /* GO_IDLE_STATE(MMC) or RESET(SD) */
#define SEND_OP_COND        1        /* SEND_OP_COND(MMC) or ACMD41(SD) */
#define ALL_SEND_CID        2        /* ALL SEND_CID */
#define SET_RELATIVE_ADDR    3        /* SET_RELATE_ADDR */
#define SET_ACMD_BUS_WIDTH    6
#define SELECT_CARD            7        /* SELECT/DESELECT_CARD */
#define SEND_IF_COND        8        /* Send Interface Condition */
#define SEND_CSD            9        /* SEND_CSD */
#define STOP_TRANSMISSION    12        /* Stop either READ or WRITE operation */
#define SEND_STATUS            13        /* SEND_STATUS */
#define SET_BLOCK_LEN        16        /* SET_BLOCK_LEN */
#define READ_SINGLE_BLOCK    17        /* READ_SINGLE_BLOCK */
#define WRITE_BLOCK            24        /* WRITE_BLOCK */
#define SEND_APP_OP_COND    41        /* ACMD41 for SD card */
#define APP_CMD                55        /* APP_CMD, the following will a ACMD */

#define OCR_INDEX            0x00FF8000

#define CARD_STATUS_ACMD_ENABLE        1 << 5
#define CARD_STATUS_RDY_DATA        1 << 8
#define CARD_STATUS_CURRENT_STATE    0x0F << 9
#define CARD_STATUS_ERASE_RESET        1 << 13

#define SLOW_RATE            1
#define NORMAL_RATE            2
#define MMC_RATE            3

#define SD_1_BIT             0
#define SD_4_BIT            1

#define CARD_UNKNOWN        0
#define MMC_CARD            1
#define SD_CARD                2

#define MCI_CLK_300KHZ			59
#define MCI_CLK_9MHz_DEFAULT            1

#define DATA_TIMER_VALUE    5000000

#define EXPECT_NO_RESP        0
#define EXPECT_SHORT_RESP    1
#define EXPECT_LONG_RESP    2


#define BUS_WIDTH_1BIT        0
#define BUS_WIDTH_4BITS        10

/* MCI Status register bit information */
#define MCI_CMD_CRC_FAIL    (1 << 0)
#define MCI_DATA_CRC_FAIL    (1 << 1)
#define MCI_CMD_TIMEOUT        (1 << 2)
#define MCI_DATA_TIMEOUT    (1 << 3)
#define MCI_TX_UNDERRUN        (1 << 4)
#define MCI_RX_OVERRUN        (1 << 5)
#define MCI_CMD_RESP_END    (1 << 6)
#define MCI_CMD_SENT        (1 << 7)
#define MCI_DATA_END        (1 << 8)
#define MCI_START_BIT_ERR    (1 << 9)
#define MCI_DATA_BLK_END    (1 << 10)
#define MCI_CMD_ACTIVE        (1 << 11)
#define MCI_TX_ACTIVE        (1 << 12)
#define MCI_RX_ACTIVE        (1 << 13)
#define MCI_TX_HALF_EMPTY    (1 << 14)
#define MCI_RX_HALF_FULL    (1 << 15)
#define MCI_TX_FIFO_FULL    (1 << 16)
#define MCI_RX_FIFO_FULL    (1 << 17)
#define MCI_TX_FIFO_EMPTY    (1 << 18)
#define MCI_RX_FIFO_EMPTY    (1 << 19)
#define MCI_TX_DATA_AVAIL    (1 << 20)
#define MCI_RX_DATA_AVAIL    (1 << 21)

#define CMD_INT_MASK      (MCI_CMD_CRC_FAIL | MCI_CMD_TIMEOUT | MCI_CMD_RESP_END \
                         | MCI_CMD_SENT     | MCI_CMD_ACTIVE)

#define DATA_ERR_INT_MASK    (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_TX_UNDERRUN \
                           | MCI_RX_OVERRUN | MCI_START_BIT_ERR)

#define ACTIVE_INT_MASK ( MCI_TX_ACTIVE | MCI_RX_ACTIVE)

#define FIFO_INT_MASK        (MCI_TX_HALF_EMPTY | MCI_RX_HALF_FULL \
                           | MCI_TX_FIFO_FULL  | MCI_RX_FIFO_FULL \
                           | MCI_TX_FIFO_EMPTY | MCI_RX_FIFO_EMPTY \
                           | MCI_DATA_BLK_END )

#define    FIFO_TX_INT_MASK (MCI_TX_HALF_EMPTY )
#define    FIFO_RX_INT_MASK (MCI_RX_HALF_FULL  )

#define DATA_END_INT_MASK    (MCI_DATA_END | MCI_DATA_BLK_END)

#define ERR_TX_INT_MASK (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_TX_UNDERRUN | MCI_START_BIT_ERR)
#define ERR_RX_INT_MASK (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_RX_OVERRUN  | MCI_START_BIT_ERR)

/* Error code on the command response. */
#define INVALID_RESPONSE    0xFFFFFFFF

extern void MCI_TXEnable(void);
extern void MCI_TXDisable(void);
extern void MCI_RXEnable(void);
extern void MCI_RXDisable(void);

extern void MCI_Init(void);
extern void  MCI_SendCmd(uint32_t CmdIndex, uint32_t Argument, uint32_t ExpectResp, uint32_t AllowTimeout);
extern uint32_t MCI_GetCmdResp(uint32_t CmdIndex, uint32_t NeedRespFlag, uint32_t *CmdRespStatus);

extern void  MCI_Set_MCIClock(uint32_t clockrate);
extern bool SD_Set_BusWidth(uint32_t width);

extern uint32_t MCI_CardInit(void);
extern bool MCI_Go_Idle_State(void);
extern bool MCI_Check_CID(void);
extern bool MCI_Set_Address(void);
extern bool MCI_Send_CSD(void);
extern bool MCI_Select_Card(void);
extern uint32_t MCI_Send_Status(void);
extern bool MCI_Set_BlockLen(uint32_t blockLength);
extern bool MCI_Send_ACMD_Bus_Width(uint32_t buswidth);
extern bool MCI_Send_Stop(void);

typedef void(*MCI_DATA_END_CALLBACK)();

extern bool MCI_Write_Block(uint32_t blockNum, MCI_DATA_END_CALLBACK MCI_DATA_END_Callback);
extern bool MCI_Read_Block(uint32_t blockNum, MCI_DATA_END_CALLBACK MCI_DATA_END_Callback);

bool MCI_And_Card_initialize();

bool MCI_ReadSector(uint32_t sector, uint8_t *readbuffer);
bool MCI_WriteSector(uint32_t sector, uint8_t *writebuffer);

uint64_t sdMediaSize = 0;
uint32_t sdSectorsPerBlock = 0;
bool isSDHC;


typedef enum mci_func_error {
    MCI_FUNC_OK = 0,
    MCI_FUNC_FAILED = -1,
    MCI_FUNC_BAD_PARAMETERS = -2,
    MCI_FUNC_BUS_NOT_IDLE = -3,
    MCI_FUNC_TIMEOUT = -3,
    MCI_FUNC_ERR_STATE = -4,
    MCI_FUNC_NOT_READY = -5,
}en_Mci_Func_Error;

/* MultiMedia Card Interface(MCI) Controller */
#define MCI_BASE_ADDR        0xE008C000
#define MCI_POWER      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x00))
#define MCI_CLOCK      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x04))
#define MCI_ARGUMENT   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x08))
#define MCI_COMMAND    (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x0C))
#define MCI_RESP_CMD   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x10))
#define MCI_RESP0      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x14))
#define MCI_RESP1      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x18))
#define MCI_RESP2      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x1C))
#define MCI_RESP3      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x20))
#define MCI_DATA_TMR   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x24))
#define MCI_DATA_LEN   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x28))
#define MCI_DATA_CTRL  (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x2C))
#define MCI_DATA_CNT   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x30))
#define MCI_STATUS     (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x34))
#define MCI_CLEAR      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x38))
#define MCI_MASK0      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x3C))
#define MCI_FIFO_CNT   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x48))
#define MCI_FIFO       (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x80))

volatile uint32_t MCI_DataErrorProcess_count = 0;
volatile uint32_t MCI_DATA_END_InterruptService_count = 0;
volatile uint32_t MCI_FIFOInterruptService_count = 0;
volatile uint32_t MCI_CmdProcess_count = 0;

volatile uint32_t CmdCRCErrCount = 0;
volatile uint32_t CmdTimeoutErrCount = 0;
volatile uint32_t CmdRespEndCount = 0;
volatile uint32_t CmdSentCount = 0;
volatile uint32_t CmdActiveCount = 0;

volatile uint32_t DataCRCErrCount = 0;
volatile uint32_t DataTimeoutErrCount = 0;
volatile uint32_t DataTxUnderrunErrCount = 0;
volatile uint32_t DataRxOverrunErrCount = 0;
volatile uint32_t DataStartbitErrCount = 0;

volatile uint32_t DataEndCount = 0;
volatile uint32_t DataBlockEndCount = 0;
volatile uint32_t MCI_Block_End_Flag = 0;

volatile uint32_t DataTxActiveCount = 0;
volatile uint32_t DataRxActiveCount = 0;

volatile uint32_t DataFIFOCount = 0;
volatile uint32_t DataRxFIFOCount = 0;
volatile uint32_t DataTxFIFOCount = 0;

volatile uint32_t CardRCA;
volatile uint32_t MCI_CardType;

uint32_t MCI_ReadFifo(volatile uint32_t * dest) {
    int i;

    for (i = 0; i < 8; i++)
        dest[i] = MCIFIFO;

    return 0;
}

uint32_t MCI_WriteFifo(volatile uint32_t * src) {
    int i;

    for (i = 0; i < 8; i++)
        MCIFIFO = src[i];

    return 0;
}


uint8_t *WriteBlock = (uint8_t *)(DMA_SRC);
uint8_t *ReadBlock = (uint8_t *)(DMA_DST);

volatile uint32_t TXBlockCounter = 0, RXBlockCounter = 0;
/******************************************************************************
** Function name:        MCI_Interrupt related
**
** Descriptions:        MCI interrupt handler and related APIs
**
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/
void MCI_TXEnable(void) {
    MCI_MASK0 |= ((DATA_END_INT_MASK) | (ERR_TX_INT_MASK));
}
/*****************************************************************/

void MCI_TXDisable(void) {
    MCI_MASK0 &= ~((DATA_END_INT_MASK) | (ERR_TX_INT_MASK));
}

/*****************************************************************/
void MCI_RXEnable(void) {
    MCI_MASK0 |= ((DATA_END_INT_MASK) | (ERR_RX_INT_MASK));
}

/*****************************************************************/

void MCI_RXDisable(void) {
    MCI_MASK0 &= ~((DATA_END_INT_MASK) | (ERR_RX_INT_MASK));
}

/******************************************************************************
** Function name:        MCI_CheckStatus
**
** Descriptions:        MCI Check status before and after the block read and
**                        write. Right after the block read and write, this routine
**                        is important that, even the FIFO is empty, complete
**                        block has been sent, but, data is still being written
**                        to the card, this routine is to ensure that the data
**                        has been written based on the state of the card, not
**                        by the length being set.
**
** parameters:            None
** Returned value:        true or false
**
******************************************************************************/
bool MCI_CheckStatus(void) {
    uint32_t respValue;
    uint32_t i;
    i = 0;
    while (i < CheckStatus_TIME_OUT) {
        if ((respValue = MCI_Send_Status()) == INVALID_RESPONSE) {
            break;
        }
        else {

            if ((respValue & (0x0F << 8)) == 0x0900) {
                return (true);
            }
        }

        i++;

        LPC24_Time_Delay(nullptr, 1000);
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_CmdProcess
**
** Descriptions:        Called by MCI interrupt handler
**                        To simplify the process, for card initialization, the
**                        CMD interrupts are disabled.
**
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/
void MCI_CmdProcess(void) {
    uint32_t MCIStatus;

    MCIStatus = MCI_STATUS;
    if (MCIStatus &  MCI_CMD_CRC_FAIL) {
        CmdCRCErrCount++;
        MCI_CLEAR = MCI_CMD_CRC_FAIL;
    }
    if (MCIStatus &  MCI_CMD_TIMEOUT) {
        CmdTimeoutErrCount++;
        MCI_CLEAR = MCI_CMD_TIMEOUT;
    }
    /* Cmd Resp End or Cmd Sent */
    if (MCIStatus &  MCI_CMD_RESP_END) {
        CmdRespEndCount++;
        MCI_CLEAR = MCI_CMD_RESP_END;
    }
    if (MCIStatus &  MCI_CMD_SENT) {
        CmdSentCount++;
        MCI_CLEAR = MCI_CMD_SENT;
    }
    if (MCIStatus &  MCI_CMD_ACTIVE) {
        CmdActiveCount++;
        MCI_CLEAR = MCI_CMD_ACTIVE;
    }
    return;
}

/******************************************************************************
** Function name:        MCI_DataErrorProcess
**
** Descriptions:        Called by MCI interrupt handler
**                        Process data error.
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/
void MCI_DataErrorProcess(void) {
    uint32_t MCIStatus;

    MCIStatus = MCI_STATUS;
    if (MCIStatus &  MCI_DATA_CRC_FAIL) {
        DataCRCErrCount++;
        MCI_CLEAR = MCI_DATA_CRC_FAIL;
    }
    if (MCIStatus &  MCI_DATA_TIMEOUT) {
        DataTimeoutErrCount++;
        MCI_CLEAR = MCI_DATA_TIMEOUT;
    }
    /* Underrun or overrun */
    if (MCIStatus &  MCI_TX_UNDERRUN) {
        DataTxUnderrunErrCount++;
        MCI_CLEAR = MCI_TX_UNDERRUN;
    }
    if (MCIStatus &  MCI_RX_OVERRUN) {
        DataRxOverrunErrCount++;
        MCI_CLEAR = MCI_RX_OVERRUN;
    }
    /* Start bit error on data signal */
    if (MCIStatus &  MCI_START_BIT_ERR) {
        DataStartbitErrCount++;
        MCI_CLEAR = MCI_START_BIT_ERR;
    }
    return;
}

/******************************************************************************
** Function name:        MCI_DATA_END_InterruptService
**
** Descriptions:        Called by MCI interrupt handler
**                        This is the last interrupt module processing
**                      the block write and    read to and from the MM card.
**
**                      FIFO interrupts are also used when DMA is disabled
**                        This routine simply clears the
**                      MCI_Block_End_Flag, and increments counters for debug
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/
MCI_DATA_END_CALLBACK MCI_DATA_END_Callback;
MCI_DATA_END_CALLBACK MCI_DATA_END_Callback_temp;
void MCI_DATA_END_InterruptService(void) {
    uint32_t MCIStatus;

    MCIStatus = MCI_STATUS;
    if (MCIStatus &  MCI_DATA_END)        /* Data end, and Data block end  */
    {
        DataEndCount++;
        MCI_CLEAR = MCI_DATA_END;
        return;
    }
    if (MCIStatus &  MCI_DATA_BLK_END) {
        DataBlockEndCount++;
        MCI_CLEAR = MCI_DATA_BLK_END;
        MCI_TXDisable();
        if (MCI_DATA_END_Callback) {
            MCI_DATA_END_Callback_temp = MCI_DATA_END_Callback;
            MCI_DATA_END_Callback = NULL;
            MCI_DATA_END_Callback_temp();

        }
        MCI_Block_End_Flag = 0;

        return;
    }

    /* Tx active  */
    if (MCIStatus & MCI_TX_ACTIVE) {
        DataTxActiveCount++;
    }
    /* Rx active  */
    if (MCIStatus & MCI_RX_ACTIVE) {
        DataRxActiveCount++;
    }
    return;
}

/******************************************************************************
** Function name:    MCI_FIFOInterruptService
**
** Descriptions:    Called by MCI interrupt handler when using FIFO
**                    interrupts and DMA is disabled
**
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/
void MCI_FIFOInterruptService(void) {
    DataFIFOCount++;
}

/******************************************************************************
** Function name:        MCI_IRQHandler
**
** Descriptions:        MCI interrupt handler
**                        The handler to handle the block data write and read
**                        not for the commands.
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/
void MCI_IRQHandler(void * p) {
    uint32_t MCI_Status;

    MCI_Status = MCIStatus;

    /* handle MCIStatus interrupt */
    if (MCI_Status & DATA_ERR_INT_MASK) {
        MCI_DataErrorProcess();
        MCI_DataErrorProcess_count++;
        //VICVectAddr = 0;        /* Acknowledge Interrupt */
        return;
    }
    if (MCI_Status & DATA_END_INT_MASK) {
        MCI_DATA_END_InterruptService();
        MCI_DATA_END_InterruptService_count++;
        return;
    }
    else if (MCI_Status & FIFO_INT_MASK) {
        MCI_FIFOInterruptService();
        MCI_FIFOInterruptService_count++;
        return;
    }
    else if (MCI_Status & CMD_INT_MASK) {
        MCI_CmdProcess();
        MCI_CmdProcess_count++;
        return;
    }
    return;
}

/******************************************************************************
** Function name:        MCI_Set_MCIClock
**
** Descriptions:        Set MCI clock rate, during initialization phase < 400K
**                        during data phase < 20Mhz.
**
** parameters:            Clock rate to be set
** Returned value:        None
**
******************************************************************************/
void MCI_Set_MCIClock(uint32_t ClockRate) {
    uint32_t i, ClkValue = 0;

    ClkValue |= ClockRate;/* normal MMC clock */

    MCI_CLOCK &= ~(0xFF); /* clear clock divider */
    MCI_CLOCK |= (1 << 8) | ClkValue | (1 << 9);

    LPC24_Time_Delay(nullptr, 1 * 1000);
}

/******************************************************************************
** Function name:        SD_Set_BusWidth
**
** Descriptions:        1-bit bus or 4-bit bus.
**
** parameters:            bus width
** Returned value:        true or false
**
******************************************************************************/
bool SD_Set_BusWidth(uint32_t width) {
    uint32_t i;

    for (i = 0; i < 0x10; i++);    /* delay 3MCLK + 2PCLK  */

    if (width == SD_1_BIT) {
        MCI_CLOCK &= ~(1 << 11);    /* 1 bit bus */
        if (MCI_Send_ACMD_Bus_Width(BUS_WIDTH_1BIT) == false) {
            return(false);
        }
    }
    else if (width == SD_4_BIT) {
        MCI_CLOCK |= (1 << 11);/* 4 bit bus */
        if (MCI_Send_ACMD_Bus_Width(BUS_WIDTH_4BITS) == false) {
            return(false);
        }
    }


    return true;
}

/******************************************************************************
** Function name:        MCI_Init
**
** Descriptions:        Set MCI clock and power registers, setup VIC for
**                        data interrupt.
**
** parameters:            None
** Returned value:        true or fase, if VIC table is full, return false
**
******************************************************************************/
void MCI_Init(void) {
    volatile uint32_t i;

    LPC24_Time_Delay(nullptr, 50 * 1000); // delay 50ms


    LPC24XX::SYSCON().PCONP |= (1 << 28);            /* Enable clock to the MCI block */




    if (MCI_CLOCK & (1 << 8)) {
        MCI_CLOCK &= ~(1 << 8);
    }

    if (MCI_POWER & 0x02) {
        MCI_POWER = 0x00;
    }

    LPC24_Time_Delay(nullptr, 1000); // delay 1ms

    /* Disable all interrupts for now */
    MCI_MASK0 = 0;

    LPC24XX::SYSCON().SCS |= 0x08;

    //Setting for timeout problem
    MCI_DATA_TMR = 0x1FFFFFFF;

    /*set up clocking default mode, clear any registers as needed */
    MCI_COMMAND = 0;
    MCI_DATA_CTRL = 0;
    MCI_CLEAR = 0x7FF;

    MCI_POWER = 0x02;

    while (!(MCI_POWER & 0x02));

    LPC24_Time_Delay(nullptr, 1000); // delay 1ms

    MCI_Set_MCIClock(MCI_CLK_300KHZ);

    LPC24_Time_Delay(nullptr, 1000); // delay 1ms

    MCI_POWER |= 0x01;

    LPC24_Time_Delay(nullptr, 2000); // delay 2ms

    LPC24_InterruptInternal_Activate(LPC24XX_VIC::c_IRQ_INDEX_SD, (uint32_t*)&MCI_IRQHandler, (void*)0);
}

/******************************************************************************
** Function name:        MCI_SendCmd
**
** Descriptions:        The routine is used to send a CMD to the card
**
** parameters:            CmdIndex, Argument, ExpectResp Flag, AllowTimeout flag
** Returned value:        None
**
******************************************************************************/
void MCI_SendCmd(uint32_t CmdIndex, uint32_t Argument, uint32_t ExpectResp, uint32_t AllowTimeout) {
    uint32_t i, CmdData = 0;
    uint32_t CmdStatus;

    while ((CmdStatus = MCI_STATUS) & MCI_CMD_ACTIVE)    /* Command in progress. */
    {
        MCI_COMMAND = 0;
        MCI_CLEAR = CmdStatus | MCI_CMD_ACTIVE;
    }

    /*set the command details, the CmdIndex should 0 through 0x3F only */
    CmdData |= (CmdIndex & 0x3F);    /* bit 0 through 5 only */
    if (ExpectResp == EXPECT_NO_RESP)            /* no response */
    {
        CmdData &= ~((1 << 6) | (1 << 7));        /* Clear long response bit as well */
    }
    else if (ExpectResp == EXPECT_SHORT_RESP)    /* expect short response */
    {
        CmdData |= (1 << 6);
    }
    else if (ExpectResp == EXPECT_LONG_RESP)    /* expect long response */
    {
        CmdData |= (1 << 6) | (1 << 7);
    }

    if (AllowTimeout)            /* allow timeout or not */
    {
        CmdData |= (1 << 8);
    }
    else {
        CmdData &= ~(1 << 8);
    }

    /*send the command*/
    CmdData |= (1 << 10);        /* This bit needs to be set last. */
    MCI_ARGUMENT = Argument;    /* Set the argument first, finally command */
    MCI_COMMAND = CmdData;
}

/******************************************************************************
** Function name:        MCI_GetCmdResp
**
** Descriptions:        Get response from the card. This module is always used
**                        in pair with MCI_SendCmd()
**
** parameters:            Expected cmd data, expect response flag, pointer to the
**                        response
**                        Expected cmd data should be the same as that in SendCmd()
**                        expect response flag could be    EXPECT_NO_RESP
**                                                        EXPECT_SHORT_RESP
**                                                        EXPECT_LONG_RESP
**                        if GetCmdResp() is 0, check the pointer to the response
**                        field to get the response value, if GetCmdResp() returns
**                        non-zero, no need to check the response field, just resend
**                        command or bailout.
** Returned value:        Response status, 0 is valid response.
**
******************************************************************************/
uint32_t MCI_GetCmdResp(uint32_t ExpectCmdData, uint32_t ExpectResp, uint32_t *CmdResp) {
    uint32_t CmdRespStatus = 0;
    uint32_t LastCmdIndex;
    uint32_t i;

    int32_t retry = 0xFFFF;

    if (ExpectResp == EXPECT_NO_RESP) {
        return (0);
    }

    bool stop = false;

    while (!stop) {
        CmdRespStatus = MCI_STATUS;
        if (CmdRespStatus & (MCI_CMD_TIMEOUT)) {
            MCI_CLEAR = CmdRespStatus | MCI_CMD_TIMEOUT;
            MCI_COMMAND = 0;
            MCI_ARGUMENT = 0xFFFFFFFF;
            return (CmdRespStatus);
        }
        if (CmdRespStatus & MCI_CMD_CRC_FAIL) {
            MCI_CLEAR = CmdRespStatus | MCI_CMD_CRC_FAIL;

            while (retry-- > 0) {
                LPC24_Time_Delay(nullptr, 1); // delay 1us

                LastCmdIndex = MCI_COMMAND & 0x003F;

                if ((LastCmdIndex == SEND_OP_COND) || (LastCmdIndex == SEND_APP_OP_COND) || (LastCmdIndex == STOP_TRANSMISSION)) {

                    MCI_COMMAND = 0;
                    MCI_ARGUMENT = 0xFFFFFFFF;

                    stop = true;
                    break;
                }
            }

            if (retry == 0)
                return (CmdRespStatus);
        }
        else if (CmdRespStatus & MCI_CMD_RESP_END) {
            MCI_CLEAR = CmdRespStatus | MCI_CMD_RESP_END;
            break;
        }
    }

    if ((MCI_RESP_CMD & 0x3F) != ExpectCmdData) {
        if ((ExpectCmdData != SEND_OP_COND) && (ExpectCmdData != SEND_APP_OP_COND)
            && (ExpectCmdData != ALL_SEND_CID) && (ExpectCmdData != SEND_CSD)) {
            CmdRespStatus = INVALID_RESPONSE;
            return (INVALID_RESPONSE);
        }
    }

    if (ExpectResp == EXPECT_SHORT_RESP) {
        *CmdResp = MCI_RESP0;
        *(CmdResp + 1) = 0;
        *(CmdResp + 2) = 0;
        *(CmdResp + 3) = 0;
    }
    else if (ExpectResp == EXPECT_LONG_RESP) {
        *CmdResp = MCI_RESP0;
        *(CmdResp + 1) = MCI_RESP1;
        *(CmdResp + 2) = MCI_RESP2;
        *(CmdResp + 3) = MCI_RESP3;
    }
    return (0);
}

/******************************************************************************
** Function name:        MCI_Go_Idle_State
**
** Descriptions:        CMD0, the very first command to be sent to initialize
**                        either MMC or SD card.
**
** parameters:            None
** Returned value:        true or false, true if card has been initialized.
**
******************************************************************************/
bool MCI_Go_Idle_State(void) {
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    retryCount = 0x20;

    while (retryCount > 0) {
        MCI_SendCmd(GO_IDLE_STATE, 0x00000000, EXPECT_NO_RESP, 0);
        respStatus = MCI_GetCmdResp(GO_IDLE_STATE, EXPECT_NO_RESP, (uint32_t *)respValue);
        if (respStatus == 0) {
            break;
        }
        retryCount--;
    }

    if (respStatus != 0)        /* timeout, give up */
    {
        return (false);
    }

    return(true);
}

/******************************************************************************
** Function name:        MCI_Send_ACMD
**
** Descriptions:        CMD55, before sending an ACMD, call this routine first
**
** parameters:            None
** Returned value:        true or false, true if card has responded before timeout.
**                        false is timeout.
**
******************************************************************************/
uint32_t MCI_Send_ACMD(void) {
    uint32_t i, retryCount;
    uint32_t CmdArgument;
    uint32_t respStatus;
    uint32_t respValue[4];

    if (MCI_CardType == SD_CARD) {
        CmdArgument = CardRCA;
    }
    else {
        CmdArgument = 0x00000000;
    }

    retryCount = 20;

    while (retryCount > 0) {
        /* Send CMD55 command followed by an ACMD */
        MCI_SendCmd(APP_CMD, CmdArgument, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(APP_CMD, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        if (!respStatus && (respValue[0] & CARD_STATUS_ACMD_ENABLE)) {
            return(true);
        }

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms

        retryCount--;
    }

    return(false);
}

/******************************************************************************
** Function name:        MCI_Send_OP_Cond
**
** Descriptions:        CMD1 for MMC
**
** parameters:            None
** Returned value:        true or false, true if card has response back before
**                        timeout, false is timeout on the command.
**
******************************************************************************/
uint32_t MCI_Send_OP_Cond(void) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    retryCount = OP_COND_COUNT;

    while (retryCount > 0) {
        /* Send CMD1 command repeatedly until the response is back correctly */
        MCI_SendCmd(SEND_OP_COND, OCR_INDEX, EXPECT_SHORT_RESP, 0/*0*/);
        respStatus = MCI_GetCmdResp(SEND_OP_COND, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        /* bit 0 and bit 2 must be zero, or it's timeout or CRC error */
        if (!(respStatus & MCI_CMD_TIMEOUT) && (respValue[0] & 0x80000000)) {
            return (true);    /* response is back and correct. */
        }

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms

        retryCount--;
    }

    return(false);
}

/******************************************************************************
** Function name:        MCI_Send_ACMD_OP_Cond
**
** Descriptions:        If Send_OP_Cond is timeout, it's not a MMC card, try
**                        this combination to see if we can communicate with
**                        a SD card.
**
** parameters:            None
** Returned value:        true or false, true if card has been initialized.
**
******************************************************************************/
uint32_t MCI_Send_ACMD_OP_Cond(void) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    retryCount = ACMD_OP_COND_COUNT;

    while (retryCount > 0) {
        MCI_POWER &= ~(1 << 6);

        LPC24_Time_Delay(nullptr, 15000);

        if (MCI_Send_ACMD() == false) {
            retryCount--;

            continue;
        }

        /* Send ACMD41 command repeatedly until the response is back correctly */
        MCI_SendCmd(SEND_APP_OP_COND, OCR_INDEX, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(SEND_APP_OP_COND, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);

        if (!(respStatus & MCI_CMD_TIMEOUT) && (respValue[0] & 0x80000000)) {
            return (true);    /* response is back and correct. */
        }

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms

        retryCount--;
    }
    return(false);
}



uint32_t MCI_Send_SDHC(void) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    MCI_SendCmd(8, 0x1aa, EXPECT_SHORT_RESP, 0);
    respStatus = MCI_GetCmdResp(8, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
    if (respStatus) {
        return (false);
    }

    retryCount = ACMD_OP_COND_COUNT;
    MCI_POWER &= ~(1 << 6);

    LPC24_Time_Delay(nullptr, 1000); // delay 1ms

    while (retryCount > 0) {


        if (MCI_Send_ACMD() == false) {
            retryCount--;
            continue;
        }

        /* Send ACMD41 command repeatedly until the response is back correctly */
        MCI_SendCmd(SEND_APP_OP_COND, OCR_INDEX | 1 << 30, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(SEND_APP_OP_COND, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        if (!(respStatus & MCI_CMD_TIMEOUT) && (respValue[0] & 0x80000000)) {
            if (respValue[0] & 0x40000000)
                isSDHC = true;
            return (true);
        }
        LPC24_Time_Delay(nullptr, 1000); // delay 1ms
        retryCount--;
    }
    return(false);
}

/******************************************************************************
** Function name:        MCI_CardInit
**
** Descriptions:        Try CMD1 first for MMC, if it's timeout, try CMD55
**                        and CMD41 for SD, if both failed, initialization faliure,
**                        bailout with unknown card type. Otherwise, return the
**                        card type, either MMC or SD.
**
** parameters:            None
** Returned value:        Card type.
**
******************************************************************************/
uint32_t MCI_CardInit(void) {
    uint32_t i, CardType;

    isSDHC = false;

    MCI_CardType = CARD_UNKNOWN;

    if (MCI_Go_Idle_State() == false) {
        return(CARD_UNKNOWN);
    }

    MCI_POWER &= ~(1 << 6);

    LPC24_Time_Delay(nullptr, 1000); // delay 1ms

    if (MCI_Send_SDHC() == true) {

        CardType = SD_CARD;
        return (CardType);
    }
    else if (MCI_Send_ACMD_OP_Cond() == true) {
        CardType = SD_CARD;
        return (CardType);
    }

    MCI_POWER |= (1 << 6);

    if (MCI_Send_OP_Cond() == true) {
        CardType = MMC_CARD;
        MCI_POWER &= ~(1 << 6);

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms

        return (CardType);
    }

    return (CARD_UNKNOWN);
}

/******************************************************************************
** Function name:        MCI_Check_CID
**
** Descriptions:        Send CMD2, ALL_SEND_CID
**
** parameters:            None
** Returned value:        If not timeout, return true.
**
******************************************************************************/
bool MCI_Check_CID(void) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];


    retryCount = 0x20;
    while (retryCount > 0) {
        /* Send CMD2 command repeatedly until the response is back correctly */
        MCI_SendCmd(ALL_SEND_CID, 0, EXPECT_LONG_RESP, 0);
        respStatus = MCI_GetCmdResp(ALL_SEND_CID, EXPECT_LONG_RESP, (uint32_t *)&respValue[0]);

        /* bit 0 and bit 2 must be zero, or it's timeout or CRC error */
        if (!(respStatus & MCI_CMD_TIMEOUT)) {
            return (true);
        }

        LPC24_Time_Delay(nullptr, 1000);
        retryCount--;
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_Set_Address
**
** Descriptions:        Send CMD3, STE_RELATIVE_ADDR, should after CMD2
**
** parameters:            None
** Returned value:        true if response is back before timeout.
**
******************************************************************************/
bool MCI_Set_Address(void) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;

    if (MCI_CardType == SD_CARD) {
        CmdArgument = 0;
    }
    else {
        CmdArgument = 0x00010000;
    }

    retryCount = 0x20;
    while (retryCount > 0) {
        /* Send CMD3 command repeatedly until the response is back correctly */
        MCI_SendCmd(SET_RELATIVE_ADDR, CmdArgument, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(SET_RELATIVE_ADDR, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);

        if (!(respStatus & MCI_CMD_TIMEOUT) && ((respValue[0] & (0x0F << 8)) == 0x0500)) {
            CardRCA = respValue[0] & 0xFFFF0000;
            return (true);
        }

        LPC24_Time_Delay(nullptr, 1000);
        retryCount--;
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_Send_CSD
**
** Descriptions:        CMD9, SEND_CSD cmd, it should be sent only at
**                        STBY state and after CMD3. See MMC and SD spec. state
**                        diagram.
**
** parameters:            None
** Returned value:        Response value
**
******************************************************************************/
bool MCI_Send_CSD(void) {
    uint32_t i, retryCount, temp;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;
    unsigned char *regCSD = (unsigned char *)respValue;

    if (MCI_CardType == SD_CARD) {
        CmdArgument = CardRCA;
    }
    else {
        CmdArgument = 0x00010000;
    }

    retryCount = 0x20;
    while (retryCount > 0) {
        /* Send SET_BLOCK_LEN command before read and write */
        MCI_CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);
        MCI_SendCmd(SEND_CSD, CmdArgument, EXPECT_LONG_RESP, 0);
        respStatus = MCI_GetCmdResp(SEND_CSD, EXPECT_LONG_RESP, (uint32_t *)&respValue[0]);
        if (!respStatus) {

            for (i = 0; i <= 12; i += 4) {
                temp = regCSD[i + 0];
                regCSD[i + 0] = regCSD[i + 3];
                regCSD[i + 3] = temp;

                temp = regCSD[i + 1];
                regCSD[i + 1] = regCSD[i + 2];
                regCSD[i + 2] = temp;
            }

            uint8_t SECTOR_SIZE, READ_BL_LEN;
            uint8_t C_SIZE_MULT = 0;
            bool ERASE_BL_EN;
            uint32_t C_SIZE;
            uint64_t MemCapacity = 0;

            READ_BL_LEN = regCSD[5] & 0x0F;

            if (regCSD[0] == 0x00) {
                C_SIZE = ((regCSD[6] & 0x3) << 10) | (regCSD[7] << 2) | ((regCSD[8] & 0xC0) >> 6);

                C_SIZE_MULT = ((regCSD[9] & 0x03) << 1) | ((regCSD[10] & 0x80) >> 7);

                ERASE_BL_EN = ((regCSD[10] & 0x40) == 0x00) ? false : true;

                SECTOR_SIZE = ((regCSD[10] & 0x3F) << 1) | ((regCSD[11] & 0x80) >> 7);

                MemCapacity = (uint64_t)(C_SIZE + 1)*(0x1 << (C_SIZE_MULT + 2))*(0x1 << READ_BL_LEN);
            }
            else {
                C_SIZE = ((regCSD[7] & 0x3F) << 16) | (regCSD[8] << 8) | regCSD[9];

                ERASE_BL_EN = ((regCSD[10] & 0x40) == 0x00) ? false : true;

                SECTOR_SIZE = ((regCSD[10] & 0x3F) << 1) | ((regCSD[11] & 0x80) >> 7);

                MemCapacity = (uint64_t)(C_SIZE + 1) * 512 * 1024;
            }

            sdSectorsPerBlock = (ERASE_BL_EN == true) ? 1 : (SECTOR_SIZE + 1);
            sdMediaSize = MemCapacity;

            return (true);
        }

        LPC24_Time_Delay(nullptr, 1000);

        retryCount--;
    }

    return (false);
}

/******************************************************************************
** Function name:        MCI_Select_Card
**
** Descriptions:        CMD7, SELECT_CARD, should be after CMD9, the state
**                        will be inter-changed between STBY and TRANS after
**                        this cmd.
**
** parameters:            None
** Returned value:        return false if response times out.
**
******************************************************************************/
bool MCI_Select_Card(void) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;

    if (MCI_CardType == SD_CARD) {
        CmdArgument = CardRCA;
    }
    else {
        CmdArgument = 0x00010000;
    }

    retryCount = 0x20;

    while (retryCount > 0) {
        /* Send SELECT_CARD command before read and write */
        MCI_CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);
        MCI_SendCmd(SELECT_CARD, CmdArgument, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(SELECT_CARD, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        if (!respStatus && ((respValue[0] & (0x0F << 8)) == 0x0700)) {
            return (true);
        }

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms

        retryCount--;
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_Send_Status
**
** Descriptions:        CMD13, SEND_STATUS, the most important cmd to
**                        debug the state machine of the card.
**
** parameters:            None
** Returned value:        Response value(card status), true if the ready bit
**                        is set in the card status register, if timeout, return
**                        INVALID_RESPONSE 0xFFFFFFFF.
**
******************************************************************************/
uint32_t MCI_Send_Status(void) {
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;

    if (MCI_CardType == SD_CARD) {
        CmdArgument = CardRCA;
    }
    else {
        CmdArgument = 0x00010000;
    }

    retryCount = Send_Status_COUNT;
    while (retryCount > 0) {
        /* Send SELECT_CARD command before read and write */
        MCI_CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);
        MCI_SendCmd(SEND_STATUS, CmdArgument, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(SEND_STATUS, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        if (!respStatus && (respValue[0] & (1 << 8))) {
            return (respValue[0]);
        }
        retryCount--;

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms
    }
    return (INVALID_RESPONSE);
}

/******************************************************************************
** Function name:        MCI_Set_BlockLen
**
** Descriptions:        CMD16, SET_BLOCKLEN, called after CMD7(SELECT_CARD)
**                        called in the TRANS state.
**
** parameters:            The length of the data block to be written or read.
** Returned value:        true or false, return true if ready bit is set, and it's
**                        in TRANS state.
**
******************************************************************************/
bool MCI_Set_BlockLen(uint32_t blockLength) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    retryCount = 0x20;

    while (retryCount > 0) {
        /* Send SET_BLOCK_LEN command before read and write */
        MCI_CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);
        MCI_SendCmd(SET_BLOCK_LEN, blockLength, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(SET_BLOCK_LEN, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);

        /* bit 9 through 12 should be in transfer state now. bit 8 is ready. */
        if (!respStatus && ((respValue[0] & (0x0F << 8)) == 0x0900)) {
            return (true);
        }

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms

        retryCount--;
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_Send_ACMD_Bus_Width
**
** Descriptions:        ACMD6, SET_BUS_WIDTH, if it's SD card, we can
**                        use the 4-bit bus instead of 1-bit. This cmd
**                        can only be called during TRANS state.
**                        Since it's a ACMD, CMD55 APP_CMD needs to be
**                        sent out first.
**
** parameters:            Bus width value, 1-bit is 0, 4-bit is 10
** Returned value:        true or false, true if the card is still in the
**                        TRANS state after the cmd.
**
******************************************************************************/
bool MCI_Send_ACMD_Bus_Width(uint32_t buswidth) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    retryCount = 0x20;
    while (retryCount > 0) {
        if (MCI_Send_ACMD() == false) {
            continue;
        }
        /* Send ACMD6 command to set the bus width */
        MCI_SendCmd(SET_ACMD_BUS_WIDTH, buswidth, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(SET_ACMD_BUS_WIDTH, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        if (!respStatus && ((respValue[0] & (0x0F << 8)) == 0x0900)) {
            return (true);
        }

        LPC24_Time_Delay(nullptr, 1000);

        retryCount--;
    }
    return(false);
}

/******************************************************************************
** Function name:        MCI_Send_Stop
**
** Descriptions:        CMD12, STOP_TRANSMISSION. if that happens, the card is
**                        maybe in a unknown state that need a warm reset.
**
** parameters:            None
** Returned value:        true or false, true if, at least, the card status
**                        shows ready bit is set.
**
******************************************************************************/
bool MCI_Send_Stop(void) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    retryCount = 0x20;
    while (retryCount > 0) {
        MCI_CLEAR = 0x7FF;
        MCI_SendCmd(STOP_TRANSMISSION, 0x00000000, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(STOP_TRANSMISSION, EXPECT_SHORT_RESP, (uint32_t *)respValue);
        /* ready bit, bit 8, should be set in the card status register */
        if (!respStatus && (respValue[0] & (1 << 8))) {
            return(true);
        }

        LPC24_Time_Delay(nullptr, 1000);

        retryCount--;
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_Send_Write_Block
**
** Descriptions:        CMD24, WRITE_BLOCK, send this cmd in the TRANS state
**                        to write a block of data to the card.
**
** parameters:            block number
** Returned value:        Response value
**
******************************************************************************/
uint32_t MCI_Send_Write_Block(uint32_t blockNum) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    if (!isSDHC)
        blockNum *= BLOCK_LENGTH;

    retryCount = 0x20;
    while (retryCount > 0) {
        MCI_CLEAR = 0x7FF;
        MCI_SendCmd(WRITE_BLOCK, blockNum, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(WRITE_BLOCK, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        /* it should be in the transfer state, bit 9~12 is 0x0100 and bit 8 is 1 */
        if (!respStatus && ((respValue[0] & (0x0F << 8)) == 0x0900)) {
            return(true);
        }

        LPC24_Time_Delay(nullptr, 1000);

        retryCount--;
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_Send_Read_Block
**
** Descriptions:        CMD17, READ_SINGLE_BLOCK, send this cmd in the TRANS
**                        state to read a block of data from the card.
**
** parameters:            block number
** Returned value:        Response value
**
******************************************************************************/
uint32_t MCI_Send_Read_Block(uint32_t blockNum) {
    uint32_t i, retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    if (!isSDHC)
        blockNum *= BLOCK_LENGTH;
    retryCount = 0x20;
    while (retryCount > 0) {
        MCI_CLEAR = 0x7FF;
        MCI_SendCmd(READ_SINGLE_BLOCK, blockNum, EXPECT_SHORT_RESP, 0);
        respStatus = MCI_GetCmdResp(READ_SINGLE_BLOCK, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);
        /* it should be in the transfer state, bit 9~12 is 0x0100 and bit 8 is 1 */
        if (!respStatus && ((respValue[0] & (0x0F << 8)) == 0x0900)) {
            return(true);
        }

        LPC24_Time_Delay(nullptr, 1000);
        retryCount--;
    }
    return (false);
}

/******************************************************************************
** Function name:        MCI_Write_Block
**
** Descriptions:        Set MCI data control register, data length and data
**                        timeout, send WRITE_BLOCK cmd, finally, enable
**                        interrupt. On completion of WRITE_BLOCK cmd, TX_ACTIVE
**                        interrupt will occurs, data can be written continuously
**                        into the FIFO until the block data length is reached.
**
** parameters:            block number
** Returned value:        true or false, if cmd times out, return false and no
**                        need to continue.
**
******************************************************************************/

bool MCI_Write_Block(uint32_t blockNum, MCI_DATA_END_CALLBACK Write_end_Callback) {
    uint32_t i;
    uint32_t DataCtrl = 0;

    MCI_CLEAR = 0x7FF;
    MCI_DATA_CTRL = 0;

    if (MCI_CheckStatus() != true) {
        MCI_Send_Stop();
        return(false);
    }

    MCI_DATA_TMR = DATA_TIMER_VALUE;
    MCI_DATA_LEN = BLOCK_LENGTH;
    MCI_Block_End_Flag = 1;

    MCI_DATA_END_Callback = Write_end_Callback;

    MCI_TXEnable();
    if (MCI_Send_Write_Block(blockNum) == false) {
        return (false);
    }

    DMA_Move(0, M2P);
    GPDMA_CH0_CFG = 0x10001 | (0x00 << 1) | (0x04 << 6) | (0x05 << 11);
    DataCtrl = ((1 << 0) | (1 << 3) | (DATA_BLOCK_LEN << 4));

    MCI_DATA_CTRL = DataCtrl;

    return (true);
}

/******************************************************************************
** Function name:        MCI_Read_Block
**
** Descriptions:        Set MCI data control register, data length and data
**                        timeout, send READ_SINGLE_BLOCK cmd, finally, enable
**                        interrupt. On completion of READ_SINGLE_BLOCK cmd,
**                        RX_ACTIVE interrupt will occurs, data can be read
**                        continuously into the FIFO until the block data
**                        length is reached.
**
** parameters:            block number
** Returned value:        true or false, if cmd times out, return false and no
**                        need to continue.
**
**
******************************************************************************/
bool MCI_Read_Block(uint32_t blockNum, MCI_DATA_END_CALLBACK read_end_Callback) {
    uint32_t i;
    uint32_t DataCtrl = 0;

    MCI_CLEAR = 0x7FF;
    MCI_DATA_CTRL = 0;

    if (MCI_CheckStatus() != true) {

        MCI_Send_Stop();
        return(false);
    }
    MCI_RXEnable();

    MCI_DATA_TMR = DATA_TIMER_VALUE;
    MCI_DATA_LEN = BLOCK_LENGTH;
    MCI_Block_End_Flag = 1;

    MCI_DATA_END_Callback = read_end_Callback;

    if (MCI_Send_Read_Block(blockNum) == false) {
        return (false);
    }

    DMA_Move(0, P2M);
    GPDMA_CH0_CFG = 0x10001 | (0x04 << 1) | (0x00 << 6) | (0x06 << 11);
    DataCtrl = ((1 << 0) | (1 << 1) | (1 << 3) | (DATA_BLOCK_LEN << 4));

    MCI_DATA_CTRL = DataCtrl;

    return (true);
}

/******************************************************************************
** Function name:        MCI_And_Card_initialize
**
** Descriptions:        Intialize Memory Card Interface and then Initialize Card
**
**
** parameters:            None
** Returned value:        false if succeeded. Nuber  if Faild
**
******************************************************************************/
bool MCI_And_Card_initialize(void) {
    int err = 0;
    sdMediaSize = 0;
    sdSectorsPerBlock = 0;

    DMA_Init();

    MCI_Init();

    // Allow the card to power up...
    LPC24_Time_Delay(nullptr, 50 * 1000); // delay 50ms

    if (!err) {
        MCI_CardType = MCI_CardInit();
        if (MCI_CardType == CARD_UNKNOWN) {
            err++;
        }
    }

    if (err || MCI_Check_CID() == false) {
        err++;
    }

    if (err || MCI_Set_Address() == false) {
        err++;
    }

    if (err || MCI_Send_CSD() == false) {

        err++;
    }
    if (err || MCI_Select_Card() == false) {

        err++;
    }

    if (!err) {
        MCI_Set_MCIClock(MCI_CLK_9MHz_DEFAULT);

        if (MCI_CardType == SD_CARD) {

            if (SD_Set_BusWidth(SD_4_BIT) != true) {
                err++;
            }
        }
        else {
            MCI_CLOCK &= ~(1 << 11);
        }
    }

    if (err || MCI_Set_BlockLen(BLOCK_LENGTH) == false) {
        err++;
    }

    return err == 0 ? true : false;
}

/******************************************************************************
** Function name:        MCI_ReadSector
**
** parameters:            block number, Read Buffer
** Returned value:        false if Succeeded
**
**
******************************************************************************/
volatile uint8_t Read_Flag = 0;

void Read_end_Callback() {
    Read_Flag = 1;
}

bool MCI_ReadSector(
    uint32_t sector,
    uint8_t *buff) {
    uint32_t i;
    uint8_t temp;

    ReadBlock = (uint8_t *)(DMA_DST);

    if (MCI_Read_Block(sector, Read_end_Callback) == false) {
        return false; // Error
    }

    temp = 0;
    i = 0;
    while (i < READ_TIME_OUT) {
        if (Read_Flag == 1) {
            temp = 1;
            break;
        }

        i++;
        LPC24_Time_Delay(nullptr, 1000); // READ_TIME_OUT uint is ms;
    }
    Read_Flag = 0;

    if (temp == 0)
        return false; // Error

    memcpy(buff, ReadBlock, BLOCK_LENGTH);

    return true;// No Error
}

/******************************************************************************
** Function name:        MCI_WriteSector
**
** parameters:            block number, Write Buffer
** Returned value:        false if Succeeded
**
**
******************************************************************************/

volatile uint8_t Flag_write = 0;

void Write_end_Callback() {
    Flag_write = 1;
}

bool MCI_WriteSector(
    uint32_t sector,        /* Sector number (LBA) */
    uint8_t *buff    /* Data to be written */) {
    uint32_t i;
    uint8_t temp;

    WriteBlock = (uint8_t *)(DMA_SRC);
    memcpy(WriteBlock, buff, BLOCK_LENGTH);


    if (MCI_Write_Block(sector, Write_end_Callback) == false) {
        /* Fatal error */
        return false; // Error
    }

    temp = 0;
    i = 0;

    while (i < WRITE_TIME_OUT) {
        if (Flag_write == 1) {
            temp = 1;
            break;
        }

        i++;
        LPC24_Time_Delay(nullptr, 1000); // WRITE_TIME_OUT unit is ms = 1000*x;
    }

    Flag_write = 0;

    if (temp == 0)
        return false; // Error

    return true;// No Error
}

/************************************************************************//**
 * @brief         Send CMD8 (SEND_IF_COND) for interface condition to card.
 *
 * @param        None
 *
 * @return         MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t MCI_Cmd_SendIfCond(void) {
    volatile uint32_t i;
    uint32_t retryCount;
    uint32_t CmdArgument;
    uint32_t respStatus;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    uint8_t voltageSupplied = 0x01;//in range 2.7-3.6V
    uint8_t checkPattern = 0xAA;

    CmdArgument = (voltageSupplied << 8 /* Voltage Supplied Bit*/) | checkPattern;

    retryCount = 20;

    while (retryCount > 0) {

        MCI_SendCmd(SEND_IF_COND, CmdArgument, EXPECT_SHORT_RESP, 1/*Allow a command timeout*/);
        respStatus = MCI_GetCmdResp(SEND_IF_COND, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0]);

        if (respStatus & MCI_CMD_TIMEOUT) {
            //Consider as no response
            retval = MCI_FUNC_TIMEOUT;
        }
        else if ((respValue[0] & 0xFF /*Checkerpattern Bit Mask*/) != checkPattern) {
            retval = MCI_FUNC_FAILED;
        }
        else if (((respValue[0] >> 8 /* Voltage Supplied Bit*/) & 0xFF/* Voltage Supplied Bit Mask*/)
            != voltageSupplied) {
            retval = MCI_FUNC_BAD_PARAMETERS;
        }
        else {
            retval = MCI_FUNC_OK;
            break;
        }

        LPC24_Time_Delay(nullptr, 1000); // delay 1ms

        retryCount--;
    }

    return retval;
}

// LPC24
#define LPC24_SD_SECTOR_SIZE 512
#define LPC24_SD_TIMEOUT 5000000
#define TOTAL_SDCARD_CONTROLLERS 1

static TinyCLR_Storage_Controller sdCardControllers[TOTAL_SDCARD_CONTROLLERS];
static TinyCLR_Api_Info sdCardApi[TOTAL_SDCARD_CONTROLLERS];

struct SdCardState {
    int32_t controllerIndex;

    uint64_t *regionAddresses;
    size_t  *regionSizes;

    TinyCLR_Storage_Descriptor descriptor;

    uint16_t initializeCount;
};

static const LPC24_Gpio_Pin sdCardData0Pins[] = LPC24_SD_DATA0_PINS;
static const LPC24_Gpio_Pin sdCardData1Pins[] = LPC24_SD_DATA1_PINS;
static const LPC24_Gpio_Pin sdCardData2Pins[] = LPC24_SD_DATA2_PINS;
static const LPC24_Gpio_Pin sdCardData3Pins[] = LPC24_SD_DATA3_PINS;
static const LPC24_Gpio_Pin sdCardClkPins[] = LPC24_SD_CLK_PINS;
static const LPC24_Gpio_Pin sdCardCmdPins[] = LPC24_SD_CMD_PINS;

static SdCardState sdCardStates[TOTAL_SDCARD_CONTROLLERS];

const char* sdCardApiNames[TOTAL_SDCARD_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC24.SdCardStorageController\\0"
};

void LPC24_SdCard_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_SDCARD_CONTROLLERS; i++) {
        sdCardControllers[i].ApiInfo = &sdCardApi[i];
        sdCardControllers[i].Acquire = &LPC24_SdCard_Acquire;
        sdCardControllers[i].Release = &LPC24_SdCard_Release;
        sdCardControllers[i].Open = &LPC24_SdCard_Open;
        sdCardControllers[i].Close = &LPC24_SdCard_Close;
        sdCardControllers[i].Write = &LPC24_SdCard_Write;
        sdCardControllers[i].Read = &LPC24_SdCard_Read;
        sdCardControllers[i].Erase = &LPC24_SdCard_Erases;
        sdCardControllers[i].IsErased = &LPC24_SdCard_IsErased;
        sdCardControllers[i].GetDescriptor = &LPC24_SdCard_GetDescriptor;
        sdCardControllers[i].IsPresent = &LPC24_SdCard_IsPresent;
        sdCardControllers[i].SetPresenceChangedHandler = &LPC24_SdCard_SetPresenceChangedHandler;

        sdCardApi[i].Author = "GHI Electronics, LLC";
        sdCardApi[i].Name = sdCardApiNames[i];
        sdCardApi[i].Type = TinyCLR_Api_Type::StorageController;
        sdCardApi[i].Version = 0;
        sdCardApi[i].Implementation = &sdCardControllers[i];
        sdCardApi[i].State = &sdCardStates[i];

        sdCardStates[i].controllerIndex = i;

        apiManager->Add(apiManager, &sdCardApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::StorageController, sdCardApi[0].Name);
}

TinyCLR_Result LPC24_SdCard_Acquire(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<SdCardState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        auto d0 = sdCardData0Pins[controllerIndex];
        auto d1 = sdCardData1Pins[controllerIndex];
        auto d2 = sdCardData2Pins[controllerIndex];
        auto d3 = sdCardData3Pins[controllerIndex];
        auto clk = sdCardClkPins[controllerIndex];
        auto cmd = sdCardCmdPins[controllerIndex];

        if (!LPC24_Gpio_OpenPin(d0.number)
            || !LPC24_Gpio_OpenPin(d1.number)
            || !LPC24_Gpio_OpenPin(d2.number)
            || !LPC24_Gpio_OpenPin(d3.number)
            || !LPC24_Gpio_OpenPin(clk.number)
            || !LPC24_Gpio_OpenPin(cmd.number)
            )
            return TinyCLR_Result::SharingViolation;

        LPC24_Gpio_ConfigurePin(d0.number, LPC24_Gpio_Direction::Input, d0.pinFunction, LPC24_Gpio_PinMode::PullUp);
        LPC24_Gpio_ConfigurePin(d1.number, LPC24_Gpio_Direction::Input, d1.pinFunction, LPC24_Gpio_PinMode::PullUp);
        LPC24_Gpio_ConfigurePin(d2.number, LPC24_Gpio_Direction::Input, d2.pinFunction, LPC24_Gpio_PinMode::PullUp);
        LPC24_Gpio_ConfigurePin(d3.number, LPC24_Gpio_Direction::Input, d3.pinFunction, LPC24_Gpio_PinMode::PullUp);
        LPC24_Gpio_ConfigurePin(clk.number, LPC24_Gpio_Direction::Input, clk.pinFunction, LPC24_Gpio_PinMode::Inactive);
        LPC24_Gpio_ConfigurePin(cmd.number, LPC24_Gpio_Direction::Input, cmd.pinFunction, LPC24_Gpio_PinMode::PullUp);

        auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

        state->regionAddresses = (uint64_t*)memoryProvider->Allocate(memoryProvider, sizeof(uint64_t));
        state->regionSizes = (size_t*)memoryProvider->Allocate(memoryProvider, sizeof(size_t));

        state->descriptor.CanReadDirect = true;
        state->descriptor.CanWriteDirect = true;
        state->descriptor.CanExecuteDirect = false;
        state->descriptor.EraseBeforeWrite = false;
        state->descriptor.Removable = true;
        state->descriptor.RegionsContiguous = false;
        state->descriptor.RegionsEqualSized = false;

        state->descriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(state->regionAddresses);
        state->descriptor.RegionSizes = reinterpret_cast<const size_t*>(state->regionSizes);

        if (!MCI_And_Card_initialize())
            return TinyCLR_Result::InvalidOperation;
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_Release(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<SdCardState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        auto d0 = sdCardData0Pins[controllerIndex];
        auto d1 = sdCardData1Pins[controllerIndex];
        auto d2 = sdCardData2Pins[controllerIndex];
        auto d3 = sdCardData3Pins[controllerIndex];
        auto clk = sdCardClkPins[controllerIndex];
        auto cmd = sdCardCmdPins[controllerIndex];

        LPC24XX::SYSCON().PCONP &= ~(1 << 28); /* Disable clock to the Mci block */

        LPC24XX::SYSCON().PCONP &= ~(1 << 29); /* Disable clock to the Dma block */

        LPC24_InterruptInternal_Deactivate(LPC24XX_VIC::c_IRQ_INDEX_SD); /* Disable Interrupt */

        auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

        memoryProvider->Free(memoryProvider, state->regionSizes);
        memoryProvider->Free(memoryProvider, state->regionAddresses);

        LPC24_Gpio_ClosePin(d0.number);
        LPC24_Gpio_ClosePin(d1.number);
        LPC24_Gpio_ClosePin(d2.number);
        LPC24_Gpio_ClosePin(d3.number);
        LPC24_Gpio_ClosePin(clk.number);
        LPC24_Gpio_ClosePin(cmd.number);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    int32_t index = 0;

    int32_t to = timeout;

    auto sectorCount = count;

    auto sectorNum = address;

    uint8_t* pData = (uint8_t*)data;

    while (sectorCount) {
        if (MCI_WriteSector(sectorNum, &pData[index]) == true) {
            index += LPC24_SD_SECTOR_SIZE;
            sectorNum++;
            sectorCount--;
        }
        else {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    if (!to) {
        return TinyCLR_Result::TimedOut;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    int32_t index = 0;

    int32_t to = timeout;

    auto sectorCount = count;

    auto sectorNum = address;

    while (sectorCount) {
        if (MCI_ReadSector(sectorNum, &data[index]) == true) {
            index += LPC24_SD_SECTOR_SIZE;
            sectorNum++;
            sectorCount--;
        }
        else {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    if (!to) {
        return TinyCLR_Result::TimedOut;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased) {
    erased = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_Erases(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    auto state = reinterpret_cast<SdCardState*>(self->ApiInfo->State);

    state->regionSizes[0] = LPC24_SD_SECTOR_SIZE;
    state->descriptor.RegionCount = sdMediaSize / LPC24_SD_SECTOR_SIZE;

    descriptor = reinterpret_cast<const TinyCLR_Storage_Descriptor*>(&state->descriptor);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_Open(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_Close(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_IsPresent(const TinyCLR_Storage_Controller* self, bool& present) {
    present = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_SdCard_Reset() {
    for (auto i = 0; i < TOTAL_SDCARD_CONTROLLERS; i++) {
        LPC24_SdCard_Close(&sdCardControllers[i]);
        LPC24_SdCard_Release(&sdCardControllers[i]);
        sdCardStates[i].initializeCount = 0;
    }

    return TinyCLR_Result::Success;
}
#endif // INCLUDE_SD