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

#include "AT91.h"

#include <string.h>

#ifdef INCLUDE_SD
//SD timeout command
#define TRANSFER_CMD_TIMEOUT 2000000

// DMA
#define MAX_GPDMA_CHANNELS 8
#define DMA_SIZE        BLOCK_LENGTH    /* DMA_SIZE is the same BLOCK_LENGTH defined in mci.h */

/* DMA mode */
#define M2M                0x00
#define M2P                0x01
#define P2M                0x02
#define P2P                0x03

#define DMAC_CFG_SRC_PER_Pos 0
#define DMAC_CFG_SRC_PER_Msk (0xfu << DMAC_CFG_SRC_PER_Pos) /**< \brief (DMAC_CFG) Source with Peripheral identifier */
#define DMAC_CFG_SRC_PER(value) ((DMAC_CFG_SRC_PER_Msk & ((value) << DMAC_CFG_SRC_PER_Pos)))

#define HSMCI_RECEIVE_DATA_ADDRESS 0xF0008030
#define HSMCI_TRANSMIT_DATA_ADDRESS 0xF0008034

#define DMAC0_PERHERIAL 20
#define DMAC0_INTERRUP_ID DMAC0_PERHERIAL

#define DMAC0_EBCIER_REG                    (*(volatile uint32_t *)(0xFFFFEC18))
#define DMAC0_EN_REG                         (*(volatile uint32_t *)(0xFFFFEC04))
#define DMAC0_CHER_REG                         (*(volatile uint32_t *)(0xFFFFEC28)) //DMAC0 Channel Handler Enable Register
#define DMAC0_CHDR_REG                         (*(volatile uint32_t *)(0xFFFFEC2C)) //DMAC0 Channel Handler Disable Register
#define DMAC0_EBCISR_REG                     (*(volatile uint32_t *)(0xFFFFEC24)) //DMAC0 Error, Buffer Transfer and Chained Buffer Transfer Status Register
#define DMAC0_CTRLA_REG                     (*(volatile uint32_t *)(0xFFFFEC48)) //Control A Register
#define DMAC0_CTRLB_REG                     (*(volatile uint32_t *)(0xFFFFEC4C)) //Control B Register
#define DMAC0_CFG_REG                         (*(volatile uint32_t *)(0xFFFFEC50)) //Config Register
#define DMAC0_SREQ_REG                                          (*(volatile uint32_t *)(0xFFFFEC08)) //DMAC0 Software Single Request Register
#define DMAC0_LAST_REG                                          (*(volatile uint32_t *)(0xFFFFEC10)) //DMAC0 Software Single Request Register
#define DMAC0_CHSR_REG                                          (*(volatile uint32_t *)(0xFFFFEC30)) //DMAC0 Channel Handler Status Register
#define DMAC0_DSCR_REG                                          (*(volatile uint32_t *)(0xFFFFEC44)) //DMAC0 Channel Handler Status Register

#define HSMCI_SR_REG                         (*(volatile uint32_t *)(0xF0008040)) //HSMCI Status Register

#define GPDMA_Source_Register_Channel            (*(volatile uint32_t *)(0xFFFFEC3C)) // chanel 0 default
#define GPDMA_Destination_Register_Channel        (*(volatile uint32_t *)(0xFFFFEC40)) // chanel 0 default

void DMA_Config(uint32_t DMAMode, uint8_t* pData);
void DMA_Init(void);
void DMA_EnableChannel(void);
void DMA_DiableChannel(void);
void DMA_Enable(void);

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
    AT91_PMC &pmc = AT91::PMC();
    pmc.EnablePeriphClock(AT91C_ID_DMAC0);

    DMA_Enable();
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
void DMA_EnableChannel() {
    DMAC0_CHER_REG |= (1 << 0);                         // Enable chanel 0
    while ((DMAC0_CHSR_REG & (1 << 0)) == 0);

}
void DMA_DiableChannel() {
    DMAC0_CHDR_REG |= (1 << 0);                         // Enable chanel 0
    while ((DMAC0_CHSR_REG & (1 << 0)) == 1);
}

void DMA_Enable() {
    DMAC0_EN_REG = 0x01;    /* Enable DMA channels, little endian */
    while (!(DMAC0_EN_REG & 0x01));
}

void DMA_Config(uint32_t DMAMode, uint8_t* pData) {
    volatile uint32_t error_status = DMAC0_EBCISR_REG; // dump register

    if (DMAMode == P2M) // for read
    {
        DMAC0_CFG_REG =     /*HSMCI0_PERHERIAL*/(DMAC_CFG_SRC_PER(0) << 0) |                 // SRC_PER code
            (0 << 4) |                                            // DST_PER code
            (1 << 16) |                                                // Stop on done
            (0 << 28) |                         // FIFOCFG defines the watermark of the DMA channel FIFO.
            (1 << 9);                        // SRC_H2SEL is set to true to enable hardware handshaking on the destination.

        GPDMA_Source_Register_Channel = HSMCI_RECEIVE_DATA_ADDRESS;

        GPDMA_Destination_Register_Channel = (uint32_t)pData;

        DMAC0_CTRLA_REG = (512 >> 2) |                                                               //BTSIZE is programmed with block_length/4.
            (0 << 16) |                         //SCSIZE must be set according to the value of HSMCI_DMA, CHKSIZE field. 4
            (0 << 20) |                        //DCSIZE must be set according to the value of HSMCI_DMA, CHKSIZE field. 4
            (2 << 24) |                         //SRC_WIDTH is set to WORD.
            (2 << 28);                        //DST_WIDTH is set to WORD.

        DMAC0_CTRLB_REG = (1 << 16) |                                                               // both DST_DSCR and SRC_DSCR are set to 1: 1 is disable
            (1 << 20) |                        // both DST_DSCR and SRC_DSCR are set to 1
            (2 << 21) |                        // FC field is programmed with peripheral to memory flow control mode. P2M
            (2 << 24) |                         // SCR_INCR is set to INCR => fixed
            (0 << 28);                        // DST_INCR is set to INCR=>inscrease

        DMAC0_DSCR_REG = 0; //Set the DMAC Channel Descriptor Address Register as “0” as no more DMA descriptor is needed
    }
    if (DMAMode == M2P) // for write
    {
        DMAC0_CFG_REG =     /*HSMCI0_PERHERIAL*/(0 << 0) |                                                    // SRC_PER code
            (DMAC_CFG_SRC_PER(0) << 4) |                                  // DST_PER code
            (1 << 16) |                                    // Stop on done
            (0 << 28) |                                   // FIFOCFG defines the watermark of the DMA channel FIFO.
            (1 << 13);                                    // DST_H2SEL is set to true to enable hardware handshaking on the destination.

        GPDMA_Source_Register_Channel = (uint32_t)pData;
        GPDMA_Destination_Register_Channel = HSMCI_TRANSMIT_DATA_ADDRESS;

        DMAC0_CTRLA_REG = (512 >> 2) |                                                                       //BTSIZE is programmed with block_length/4.
            (0 << 16) |                         //SCSIZE must be set according to the value of HSMCI_DMA, CHKSIZE field. 4
            (0 << 20) |                        //DCSIZE must be set according to the value of HSMCI_DMA, CHKSIZE field. 4
            (2 << 24) |                         //SRC_WIDTH is set to WORD.
            (2 << 28);                        //DST_WIDTH is set to WORD.

        DMAC0_CTRLB_REG = (1 << 16) |                                               // both DST_DSCR and SRC_DSCR are set to 1: 1 is disable
            (1 << 20) |                        // both DST_DSCR and SRC_DSCR are set to 1
            (1 << 21) |                        // FC field is programmed with peripheral to memory flow control mode. M2P
            (0 << 24) |                         // SCR_INCR is set to INCR => increase
            (2 << 28);                        // DST_INCR is set to INCR=> fixed
        DMAC0_DSCR_REG = 0;//Set the DMAC Channel Descriptor Address Register as “0” as no more DMA descriptor is needed

    }

    DMA_EnableChannel();
}

// MCI

// -------- MCI_CR : (MCI Offset: 0x0) MCI Control Register --------
#define AT91C_MCI_MCIEN       (0x1 <<  0) // (MCI) Multimedia Interface Enable
#define AT91C_MCI_MCIDIS      (0x1 <<  1) // (MCI) Multimedia Interface Disable
#define AT91C_MCI_PWSEN       (0x1 <<  2) // (MCI) Power Save Mode Enable
#define AT91C_MCI_PWSDIS      (0x1 <<  3) // (MCI) Power Save Mode Disable
#define AT91C_MCI_SWRST       (0x1 <<  7) // (MCI) MCI Software reset
// -------- MCI_MR : (MCI Offset: 0x4) MCI Mode Register --------
#define AT91C_MCI_CLKDIV      (0xFF <<  0) // (MCI) Clock Divider
#define AT91C_MCI_PWSDIV      (0x7 <<  8) // (MCI) Power Saving Divider
#define AT91C_MCI_RDPROOF     (0x1 << 11) // (MCI) Read Proof Enable
#define AT91C_MCI_WRPROOF     (0x1 << 12) // (MCI) Write Proof Enable
#define AT91C_MCI_PDCFBYTE    (0x1 << 13) // (MCI) PDC Force Byte Transfer
#define AT91C_MCI_PDCPADV     (0x1 << 14) // (MCI) PDC Padding Value
//#define AT91C_MCI_PDCMODE     (0x1 << 15) // (MCI) PDC Oriented Mode
//#define AT91C_MCI_BLKLEN      (0xFFFF << 16) // (MCI) Data Block Length
// -------- MCI_DTOR : (MCI Offset: 0x8) MCI Data Timeout Register --------
#define AT91C_MCI_DTOCYC      (0xF <<  0) // (MCI) Data Timeout Cycle Number
#define AT91C_MCI_DTOMUL      (0x7 <<  4) // (MCI) Data Timeout Multiplier
#define     AT91C_MCI_DTOMUL_1                    (0x0 <<  4) // (MCI) DTOCYC x 1
#define     AT91C_MCI_DTOMUL_16                   (0x1 <<  4) // (MCI) DTOCYC x 16
#define     AT91C_MCI_DTOMUL_128                  (0x2 <<  4) // (MCI) DTOCYC x 128
#define     AT91C_MCI_DTOMUL_256                  (0x3 <<  4) // (MCI) DTOCYC x 256
#define     AT91C_MCI_DTOMUL_1024                 (0x4 <<  4) // (MCI) DTOCYC x 1024
#define     AT91C_MCI_DTOMUL_4096                 (0x5 <<  4) // (MCI) DTOCYC x 4096
#define     AT91C_MCI_DTOMUL_65536                (0x6 <<  4) // (MCI) DTOCYC x 65536
#define     AT91C_MCI_DTOMUL_1048576              (0x7 <<  4) // (MCI) DTOCYC x 1048576
// -------- MCI_SDCR : (MCI Offset: 0xc) MCI SD Card Register --------
#define AT91C_MCI_SCDSEL      (0x3 <<  0) // (MCI) SD Card Selector
#define AT91C_MCI_SCDBUS      (0x1 <<  7) // (MCI) SDCard/SDIO Bus Width
// -------- MCI_CMDR : (MCI Offset: 0x14) MCI Command Register --------
#define AT91C_MCI_CMDNB       (0x3F <<  0) // (MCI) Command Number
#define AT91C_MCI_RSPTYP      (0x3 <<  6) // (MCI) Response Type
#define     AT91C_MCI_RSPTYP_NO                   (0x0 <<  6) // (MCI) No response
#define     AT91C_MCI_RSPTYP_48                   (0x1 <<  6) // (MCI) 48-bit response
#define     AT91C_MCI_RSPTYP_136                  (0x2 <<  6) // (MCI) 136-bit response
#define AT91C_MCI_SPCMD       (0x7 <<  8) // (MCI) Special CMD
#define     AT91C_MCI_SPCMD_NONE                 (0x0 <<  8) // (MCI) Not a special CMD
#define     AT91C_MCI_SPCMD_INIT                 (0x1 <<  8) // (MCI) Initialization CMD
#define     AT91C_MCI_SPCMD_SYNC                 (0x2 <<  8) // (MCI) Synchronized CMD
#define     AT91C_MCI_SPCMD_IT_CMD               (0x4 <<  8) // (MCI) Interrupt command
#define     AT91C_MCI_SPCMD_IT_REP               (0x5 <<  8) // (MCI) Interrupt response
#define AT91C_MCI_OPDCMD      (0x1 << 11) // (MCI) Open Drain Command
#define AT91C_MCI_MAXLAT      (0x1 << 12) // (MCI) Maximum Latency for Command to respond
#define AT91C_MCI_TRCMD       (0x3 << 16) // (MCI) Transfer CMD
#define     AT91C_MCI_TRCMD_NO                   (0x0 << 16) // (MCI) No transfer
#define     AT91C_MCI_TRCMD_START                (0x1 << 16) // (MCI) Start transfer
#define     AT91C_MCI_TRCMD_STOP                 (0x2 << 16) // (MCI) Stop transfer
#define AT91C_MCI_TRDIR       (0x1 << 18) // (MCI) Transfer Direction
#define AT91C_MCI_TRTYP       (0x7 << 19) // (MCI) Transfer Type
#define     AT91C_MCI_TRTYP_BLOCK                (0x0 << 19) // (MCI) MMC/SDCard Single Block Transfer type
#define     AT91C_MCI_TRTYP_MULTIPLE             (0x1 << 19) // (MCI) MMC/SDCard Multiple Block transfer type
#define     AT91C_MCI_TRTYP_STREAM               (0x2 << 19) // (MCI) MMC Stream transfer type
#define     AT91C_MCI_TRTYP_SDIO_BYTE            (0x4 << 19) // (MCI) SDIO Byte transfer type
#define     AT91C_MCI_TRTYP_SDIO_BLOCK           (0x5 << 19) // (MCI) SDIO Block transfer type
#define AT91C_MCI_IOSPCMD     (0x3 << 24) // (MCI) SDIO Special Command
#define     AT91C_MCI_IOSPCMD_NONE                 (0x0 << 24) // (MCI) NOT a special command
#define     AT91C_MCI_IOSPCMD_SUSPEND              (0x1 << 24) // (MCI) SDIO Suspend Command
#define     AT91C_MCI_IOSPCMD_RESUME               (0x2 << 24) // (MCI) SDIO Resume Command
// -------- MCI_BLKR : (MCI Offset: 0x18) MCI Block Register --------
#define AT91C_MCI_BCNT        (0xFFFF <<  0) // (MCI) MMC/SDIO Block Count / SDIO Byte Count
// -------- MCI_SR : (MCI Offset: 0x40) MCI Status Register --------
#define AT91C_MCI_CMDRDY      (0x1 <<  0) // (MCI) Command Ready flag
#define AT91C_MCI_RXRDY       (0x1 <<  1) // (MCI) RX Ready flag
#define AT91C_MCI_TXRDY       (0x1 <<  2) // (MCI) TX Ready flag
#define AT91C_MCI_BLKE        (0x1 <<  3) // (MCI) Data Block Transfer Ended flag
#define AT91C_MCI_DTIP        (0x1 <<  4) // (MCI) Data Transfer in Progress flag
#define AT91C_MCI_NOTBUSY     (0x1 <<  5) // (MCI) Data Line Not Busy flag
//#define AT91C_MCI_ENDRX       (0x1 <<  6) // (MCI) End of RX Buffer flag
//#define AT91C_MCI_ENDTX       (0x1 <<  7) // (MCI) End of TX Buffer flag
#define AT91C_MCI_SDIOIRQA    (0x1 <<  8) // (MCI) SDIO Interrupt for Slot A
//#define AT91C_MCI_SDIOIRQB    (0x1 <<  9) // (MCI) SDIO Interrupt for Slot B
//#define AT91C_MCI_SDIOIRQC    (0x1 << 10) // (MCI) SDIO Interrupt for Slot C
//#define AT91C_MCI_SDIOIRQD    (0x1 << 11) // (MCI) SDIO Interrupt for Slot D
//#define AT91C_MCI_RXBUFF      (0x1 << 14) // (MCI) RX Buffer Full flag
//#define AT91C_MCI_TXBUFE      (0x1 << 15) // (MCI) TX Buffer Empty flag
#define AT91C_MCI_RINDE       (0x1 << 16) // (MCI) Response Index Error flag
#define AT91C_MCI_RDIRE       (0x1 << 17) // (MCI) Response Direction Error flag
#define AT91C_MCI_RCRCE       (0x1 << 18) // (MCI) Response CRC Error flag
#define AT91C_MCI_RENDE       (0x1 << 19) // (MCI) Response End Bit Error flag
#define AT91C_MCI_RTOE        (0x1 << 20) // (MCI) Response Time-out Error flag
#define AT91C_MCI_DCRCE       (0x1 << 21) // (MCI) data CRC Error flag
#define AT91C_MCI_DTOE        (0x1 << 22) // (MCI) Data timeout Error flag
#define AT91C_MCI_CSTOE        (0x1 << 23) // (MCI)
#define AT91C_MCI_BLKOVRE        (0x1 << 24) // (MCI)
#define AT91C_MCI_DMADONE        (0x1 << 25) // (MCI) DMA done flag
#define AT91C_MCI_XFRDONE        (0x1 << 27) // (MCI) Transfer done flag

#define AT91C_MCI_ACKRCVE        (0x1 << 29) // (MCI)
#define AT91C_MCI_OVRE        (0x1 << 30) // (MCI) Overrun flag
#define AT91C_MCI_UNRE        (0x1 << 31) // (MCI) Underrun flag
// -------- MCI_IER : (MCI Offset: 0x44) MCI Interrupt Enable Register --------
// -------- MCI_IDR : (MCI Offset: 0x48) MCI Interrupt Disable Register --------
// -------- MCI_IMR : (MCI Offset: 0x4c) MCI Interrupt Mask Register --------

#define AT91C_MCI_SCDBUS      (0x1 <<  7) // (MCI) SDCard/SDIO Bus Width

/// Transfer is pending.
#define MCI_STATUS_PENDING      1
/// Transfer has been aborted because an error occured.
#define MCI_STATUS_ERROR        2
/// Card did not answer command.
#define MCI_STATUS_NORESPONSE   3

/// MCI driver is currently in use.
#define MCI_ERROR_LOCK    1

/// MCI configuration with 1-bit data bus on slot A (for MMC cards).
#define MCI_MMC_SLOTA            0
/// MCI configuration with 1-bit data bus on slot B (for MMC cards).
#define MCI_MMC_SLOTB            1
/// MCI configuration with 4-bit data bus on slot A (for SD cards).
#define MCI_SD_SLOTA            AT91C_MCI_SCDBUS
/// MCI configuration with 4-bit data bus on slot B (for SD cards).
#define MCI_SD_SLOTB            (AT91C_MCI_SCDBUS | 1)

/// Start new data transfer
#define MCI_NEW_TRANSFER        0
/// Continue data transfer
#define MCI_CONTINUE_TRANSFER   1

/// MCI SD Bus Width 1-bit
#define MCI_SDCBUS_1BIT (0 << 7)
/// MCI SD Bus Width 4-bit
#define MCI_SDCBUS_4BIT (1 << 7)

//------------------------------------------------------------------------------
//         Types
//------------------------------------------------------------------------------

/// MCI end-of-transfer callback function.
typedef void(*MciCallback)(uint8_t status, void *pCommand);

//------------------------------------------------------------------------------
/// MCI Transfer Request prepared by the application upper layer. This structure
/// is sent to the MCI_SendCommand function to start the transfer. At the end of
/// the transfer, the callback is invoked by the interrupt handler.
//------------------------------------------------------------------------------
typedef struct _MciCmd {

    /// Command status.
    volatile int8_t status;
    /// Command code.
    uint32_t cmd;
    /// Command argument.
    uint32_t arg;
    /// Data buffer.
    uint8_t *pData;
    /// Size of data buffer in bytes.
    uint16_t blockSize;
    /// Number of blocks to be transfered
    uint16_t nbBlock;
    /// Indicate if continue to transfer data
    uint8_t conTrans;
    /// Indicates if the command is a read operation.
    uint8_t isRead;
    /// Response buffer.
    uint32_t  *pResp;
    /// SD card response type.
    uint8_t  resType;
    /// Optional user-provided callback function.
    MciCallback callback;
    /// Optional argument to the callback function.
    void *pArg;

} MciCmd;

//------------------------------------------------------------------------------
/// MCI driver structure. Holds the internal state of the MCI driver and
/// prevents parallel access to a MCI peripheral.
//------------------------------------------------------------------------------
typedef struct {

    /// Pointer to a MCI peripheral.
    AT91S_MCI *pMciHw;
    /// MCI peripheral identifier.
    uint8_t mciId;
    /// Pointer to currently executing command.
    MciCmd *pCommand;
    /// Mutex.
    volatile int8_t semaphore;

} Mci;

//------------------------------------------------------------------------------
//         Local constants
//------------------------------------------------------------------------------

/// Bit mask for status register errors.
#define STATUS_ERRORS (AT91C_MCI_UNRE  \
                       | AT91C_MCI_OVRE \
                       | AT91C_MCI_DTOE \
                       | AT91C_MCI_DCRCE \
                       | AT91C_MCI_RTOE \
                       | AT91C_MCI_RENDE \
                       | AT91C_MCI_RCRCE \
                       | AT91C_MCI_RDIRE \
                       | AT91C_MCI_RINDE)

/// MCI data timeout configuration with 1048576 MCK cycles between 2 data transfers.
#define DTOR_1MEGA_CYCLES           (AT91C_MCI_DTOCYC | AT91C_MCI_DTOMUL)

/// MCI MR: disable MCI Clock when FIFO is full
#ifndef AT91C_MCI_WRPROOF
#define AT91C_MCI_WRPROOF 0
#endif
#ifndef AT91C_MCI_RDPROOF
#define AT91C_MCI_RDPROOF 0
#endif

#define SDCARD_APP_OP_COND_CMD      (41 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO )
#define MMC_SEND_OP_COND_CMD        (1  | AT91C_MCI_TRCMD_NO    | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48 | AT91C_MCI_OPDCMD)

//------------------------------------------------------------------------------
//         Local macros
//------------------------------------------------------------------------------

/// Used to write in PMC registers.
#define WRITE_PMC(pPmc, regName, value)     pmc.regName = (value)

/// Used to write in MCI registers.
#define WRITE_MCI(pMci, regName, value)     pMci->regName = (value)

/// Used to read from MCI registers.
#define READ_MCI(pMci, regName)             (pMci->regName)

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

void MCI_SetSpeed(Mci *pMci, uint32_t mciSpeed);

void MCI_SendCommand(Mci *pMci, MciCmd *pMciCmd);

void MCI_Handler(Mci *pMci);

bool MCI_IsTxComplete(MciCmd *pMciCmd);

bool MCI_CheckBusy(Mci *pMci);

void MCI_Close(Mci *pMci);

void MCI_SetBusWidth(Mci *pMci, uint8_t busWidth);

//------------------------------------------------------------------------------
/// Enable/disable a MCI driver instance.
/// \param pMci  Pointer to a MCI driver instance.
/// \param enb  0 for disable MCI and 1 for enable MCI.
//------------------------------------------------------------------------------
void MCI_Enable(Mci *pMci, bool enb) {
    AT91S_MCI *pMciHw = pMci->pMciHw;

    // Set the Control Register: Enable/Disable MCI interface clock
    if (enb == false) {
        WRITE_MCI(pMciHw, MCI_CR, AT91C_MCI_MCIDIS);
    }
    else {
        WRITE_MCI(pMciHw, MCI_CR, AT91C_MCI_MCIEN);
    }
}

//------------------------------------------------------------------------------
/// Initializes a MCI driver instance and the underlying peripheral.
/// \param pMci  Pointer to a MCI driver instance.
/// \param pMciHw  Pointer to a MCI peripheral.
/// \param mciId  MCI peripheral identifier.
/// \param mode  Slot and type of connected card.
//------------------------------------------------------------------------------
void MCI_Init(Mci *pMci, AT91S_MCI *pMciHw, uint8_t mciId, uint32_t mode) {
    uint16_t clkDiv;

    // Initialize the MCI driver structure
    pMci->pMciHw = pMciHw;
    pMci->mciId = mciId;
    pMci->semaphore = 1;
    pMci->pCommand = 0;

    AT91_PMC &pmc = AT91::PMC();

    // Enable the MCI clock
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCER, (1 << mciId));

    // Reset the MCI
    WRITE_MCI(pMciHw, MCI_CR, AT91C_MCI_SWRST);

    // Disable the MCI
    WRITE_MCI(pMciHw, MCI_CR, AT91C_MCI_MCIDIS | AT91C_MCI_PWSDIS);

    // Disable all the interrupts
    WRITE_MCI(pMciHw, MCI_IDR, 0xFFFFFFFF);

    // Set the Data Timeout Register
    WRITE_MCI(pMciHw, MCI_DTOR, DTOR_1MEGA_CYCLES);

    // Set the Mode Register: 400KHz to init the card
    clkDiv = (AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / (400000 * 2)) - 1;
    WRITE_MCI(pMciHw, MCI_MR, (clkDiv | (AT91C_MCI_PWSDIV & (0x7 << 8))));

    // Set the SDCard Register
    //mode = 0x0;                                <-- There to force 1 bit mode
    WRITE_MCI(pMciHw, MCI_SDCR, mode);

    // Enable the MCI and the Power Saving
    WRITE_MCI(pMciHw, MCI_CR, AT91C_MCI_MCIEN);

    // Disable the MCI peripheral clock.
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCDR, (1 << mciId));
}

//------------------------------------------------------------------------------
/// Close a MCI driver instance and the underlying peripheral.
/// \param pMci  Pointer to a MCI driver instance.
/// \param pMciHw  Pointer to a MCI peripheral.
/// \param mciId  MCI peripheral identifier.
//------------------------------------------------------------------------------
void MCI_Close(Mci *pMci) {
    AT91S_MCI *pMciHw = pMci->pMciHw;

    // Initialize the MCI driver structure
    pMci->semaphore = 1;
    pMci->pCommand = 0;

    AT91_PMC &pmc = AT91::PMC();

    // Disable the MCI peripheral clock.
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCDR, (1 << pMci->mciId));

    // Disable the MCI
    WRITE_MCI(pMciHw, MCI_CR, AT91C_MCI_MCIDIS);

    // Disable all the interrupts
    WRITE_MCI(pMciHw, MCI_IDR, 0xFFFFFFFF);
}

//------------------------------------------------------------------------------
/// Configure the  MCI CLKDIV in the MCI_MR register. The max. for MCI clock is
/// MCK/2 and corresponds to CLKDIV = 0
/// \param pMci  Pointer to the low level MCI driver.
/// \param mciSpeed  MCI clock speed in Hz.
//------------------------------------------------------------------------------
void MCI_SetSpeed(Mci *pMci, uint32_t mciSpeed) {
    AT91S_MCI *pMciHw = pMci->pMciHw;
    uint32_t mciMr;
    uint32_t clkdiv;

    mciMr = READ_MCI(pMciHw, MCI_MR) & (~AT91C_MCI_CLKDIV);

    if (mciSpeed > 0) {
        clkdiv = (AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / (mciSpeed * 2));

        if (clkdiv > 0) {
            clkdiv -= 1;
        }
    }
    else {
        clkdiv = 0;
    }

    WRITE_MCI(pMciHw, MCI_MR, mciMr | clkdiv);
}

//------------------------------------------------------------------------------
/// Configure the  MCI SDCBUS in the MCI_SDCR register. Only two modes available
///
/// \param pMci  Pointer to the low level MCI driver.
/// \param busWidth  MCI bus width mode.
//------------------------------------------------------------------------------
void MCI_SetBusWidth(Mci *pMci, uint8_t busWidth) {
    AT91S_MCI *pMciHw = pMci->pMciHw;
    uint32_t mciSdcr;

    mciSdcr = (READ_MCI(pMciHw, MCI_SDCR) & ~(AT91C_MCI_SCDBUS));

    WRITE_MCI(pMciHw, MCI_SDCR, mciSdcr | busWidth);
}



void MCI_PreConfig(Mci *pMci, MciCmd *pCommand) {
    volatile uint32_t* ctrl = (volatile uint32_t *)(0xF0008054);

    uint32_t block_reg = (((pCommand->blockSize) << 16) | pCommand->nbBlock);
    uint32_t dma_config = 0 |            //OFFSET is 0
        (0 << 4) |        //CHKSIZE is 4
        (1 << 8) |        // Enable HSMCI DMA
        (1 << 12);    // ROPT is 0
    AT91PS_MCI pMciHw = pMci->pMciHw;

    WRITE_MCI(pMciHw, MCI_BLKR, block_reg); // set block size, block num
    WRITE_MCI(pMciHw, MCI_DMA, dma_config);
    *ctrl = 1 << 4;
}
//------------------------------------------------------------------------------
/// Starts a MCI  transfer. This is a non blocking function. It will return
/// as soon as the transfer is started.
/// Return 0 if successful; otherwise returns MCI_ERROR_LOCK if the driver is
/// already in use.
/// \param pMci  Pointer to an MCI driver instance.
/// \param pCommand  Pointer to the command to execute.
//------------------------------------------------------------------------------
void MCI_SendCommand(Mci *pMci, MciCmd *pCommand) {
    volatile AT91PS_MCI pMciHw = pMci->pMciHw;
    uint32_t mciIer = 0, mciMr = 0;

    // Command is now being executed
    pMci->pCommand = pCommand;
    pCommand->status = MCI_STATUS_PENDING;

    AT91_PMC &pmc = AT91::PMC();

    // Enable Peripheral clock
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCER, (1 << pMci->mciId));

    //Disable MCI clock for new transfer
    MCI_Enable(pMci, false);

    mciMr = READ_MCI(pMciHw, MCI_MR) & (~(AT91C_MCI_WRPROOF | AT91C_MCI_RDPROOF));

    // Command with DATA stage
    if (pCommand->blockSize > 0) {
        // Enable PDC mode and set block size
        WRITE_MCI(pMciHw, MCI_MR, mciMr | AT91C_MCI_RDPROOF | AT91C_MCI_WRPROOF);
        MCI_PreConfig(pMci, pCommand);

        if (pCommand->nbBlock == 0)
            pCommand->nbBlock = 1;

        // Config DMA
        if (pCommand->isRead)
            DMA_Config(P2M, pCommand->pData);
        else
            DMA_Config(M2P, pCommand->pData);
    }
    else   // No data transfer: stop at the end of the command
    {
        WRITE_MCI(pMciHw, MCI_MR, mciMr);
        WRITE_MCI(pMciHw, MCI_BLKR, (512 << 16));
    }

    mciIer = AT91C_MCI_CMDRDY | STATUS_ERRORS;

    // Config Interrupt
    mciIer &= ~(AT91C_MCI_UNRE | AT91C_MCI_OVRE | AT91C_MCI_DTOE | AT91C_MCI_DCRCE | AT91C_MCI_RCRCE);

    // Enable MCI clock - start transfer
    MCI_Enable(pMci, true);
    // Send the command
    WRITE_MCI(pMciHw, MCI_ARGR, pCommand->arg);
    WRITE_MCI(pMciHw, MCI_CMDR, pCommand->cmd);

    // Interrupt enable shall be done after PDC TXTEN and RXTEN
    WRITE_MCI(pMciHw, MCI_IER, mciIer);
}

//------------------------------------------------------------------------------
/// Check NOTBUSY and DTIP bits of status register on the given MCI driver.
/// Return value, false for bus ready, true for bus busy
/// \param pMci  Pointer to a MCI driver instance.
//------------------------------------------------------------------------------
bool MCI_CheckBusy(Mci *pMci) {
    AT91S_MCI *pMciHw = pMci->pMciHw;
    uint32_t status;
    // Enable MCI clock
    MCI_Enable(pMci, true);

    status = READ_MCI(pMciHw, MCI_SR);

    if (((status & AT91C_MCI_NOTBUSY) != 0)
        && ((status & AT91C_MCI_DTIP) == 0)) {

        // Disable MCI clock
        MCI_Enable(pMci, false);

        return false;
    }
    else {
        return true;
    }
}

//------------------------------------------------------------------------------
/// Check BLKE bit of status register on the given MCI driver.
/// \param pMci  Pointer to a MCI driver instance.
//------------------------------------------------------------------------------
bool MCI_CheckBlke(Mci *pMci) {
    AT91S_MCI *pMciHw = pMci->pMciHw;
    uint32_t status;

    status = READ_MCI(pMciHw, MCI_SR);

    if ((status & AT91C_MCI_BLKE) != 0) {
        return false;
    }
    else {
        return true;
    }
}

//------------------------------------------------------------------------------
/// Processes pending events on the given MCI driver.
/// \param pMci  Pointer to a MCI driver instance.
//------------------------------------------------------------------------------
void MCI_Handler(Mci *pMci) {

    AT91S_MCI *pMciHw = pMci->pMciHw;
    MciCmd *pCommand = pMci->pCommand;
    uint32_t status;
    uint8_t i;
    uint32_t data;
#if defined(at91rm9200)
    uint32_t mciCr, mciSdcr, mciMr, mciDtor;
#endif

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Read the status register
    status = READ_MCI(pMciHw, MCI_SR) & READ_MCI(pMciHw, MCI_IMR);

    // Check if an error has occured
    if ((status & STATUS_ERRORS) != 0) {

        // Check error code
        if ((status & STATUS_ERRORS) == AT91C_MCI_RTOE) {

            pCommand->status = MCI_STATUS_NORESPONSE;
        }
        // if the command is SEND_OP_COND the CRC error flag is always present
        // (cf : R3 response)
        else if (((status & STATUS_ERRORS) != AT91C_MCI_RCRCE)
            || ((pCommand->cmd != SDCARD_APP_OP_COND_CMD)
                && (pCommand->cmd != MMC_SEND_OP_COND_CMD))) {

            pCommand->status = MCI_STATUS_ERROR;
            if ((status & AT91C_MCI_RDIRE) != 0) {
                WRITE_MCI(pMciHw, MCI_IDR, AT91C_MCI_RDIRE); // disable AT91C_MCI_RDIRE
            }
        }
    }

    // Check if a transfer has been completed
    if (((status & AT91C_MCI_CMDRDY) != 0)
        || ((status & AT91C_MCI_BLKE) != 0)
        || ((status & AT91C_MCI_XFRDONE) != 0) // transfer is done
        || ((status & AT91C_MCI_DMADONE) != 0) // transfer is done
        || ((status & AT91C_MCI_BLKOVRE) != 0) // transfer is done
        || ((status & AT91C_MCI_RTOE) != 0)
        ) {

        // If no error occured, the transfer is successful
        if (pCommand->status == MCI_STATUS_PENDING) {
            pCommand->status = 0;
        }

        // Store the card response in the provided buffer
        if (pCommand->pResp) {
            uint8_t resSize;

            switch (pCommand->resType) {
            case 1:
                resSize = 1;
                break;

            case 2:
                resSize = 4;
                break;

            case 3:
                resSize = 1;
                break;

            case 4:
                resSize = 1;
                break;

            case 5:
                resSize = 1;
                break;

            case 6:
                resSize = 1;
                break;

            case 7:
                resSize = 1;
                break;

            default:
                resSize = 0;
                break;
            }
            for (i = 0; i < resSize; i++) {
                pCommand->pResp[i] = READ_MCI(pMciHw, MCI_RSPR[0]);
            }
        }

        if ((status & AT91C_MCI_CMDRDY)) {
            WRITE_MCI(pMciHw, MCI_IDR, AT91C_MCI_CMDRDY); // disable all
        }

        if ((status & AT91C_MCI_RTOE)) {
            WRITE_MCI(pMciHw, MCI_IDR, AT91C_MCI_RTOE); // disable all
        }

        if ((status & AT91C_MCI_DMADONE)) {
            WRITE_MCI(pMciHw, MCI_IDR, AT91C_MCI_DMADONE); // disable all
        }

        if ((status & AT91C_MCI_XFRDONE)) {
            WRITE_MCI(pMciHw, MCI_IDR, AT91C_MCI_XFRDONE); // disable all

        }

        if ((status & AT91C_MCI_BLKE)) {
            WRITE_MCI(pMciHw, MCI_IDR, AT91C_MCI_BLKE); // disable all
        }

        if ((status & AT91C_MCI_BLKOVRE)) {
            WRITE_MCI(pMciHw, MCI_IDR, AT91C_MCI_BLKOVRE); // disable all
        }

        // Invoke the callback associated with the current command (if any)
        if (pCommand->callback) {
            (pCommand->callback)(pCommand->status, pCommand);
        }
    }
}


//------------------------------------------------------------------------------
/// Returns 1 if the given MCI transfer is complete; otherwise returns 0.
/// \param pCommand  Pointer to a MciCmd instance.
//------------------------------------------------------------------------------
bool MCI_IsTxComplete(MciCmd *pCommand) {
    return (pCommand->status != MCI_STATUS_PENDING) ? true : false;
}

// sdmmc

/// There was no error with the SD driver.
#define SD_ERROR_NO_ERROR        0
/// There was an error with the SD driver.
#define SD_ERROR_DRIVER          1
/// The SD card did not answer the command.
#define SD_ERROR_NORESPONSE      2
/// The SD card did not answer the command.
#define SD_ERROR_NOT_INITIALIZED 3

/// SD card block size in bytes.
#define SD_BLOCK_SIZE           512
/// SD card block size binary shift value
#define SD_BLOCK_SIZE_BIT     9

//------------------------------------------------------------------------------
//         Macros
//------------------------------------------------------------------------------

// CSD register access macros.
#define SD_CSD(pSd, bitfield, bits)   ((((pSd)->csd)[3-(bitfield)/32] >> ((bitfield)%32)) & ((1 << (bits)) - 1))
#define SD_CSD_STRUCTURE(pSd)          SD_CSD(pSd, 126, 2) ///< CSD structure 00b  Version 1.0 01b version 2.0 High Cap
#define SD_CSD_TAAC(pSd)               SD_CSD(pSd, 112, 8) ///< Data read-access-time-1
#define SD_CSD_NSAC(pSd)               SD_CSD(pSd, 104, 8) ///< Data read access-time-2 in CLK cycles
#define SD_CSD_TRAN_SPEED(pSd)         SD_CSD(pSd, 96,  8) ///< Max. data transfer rate
#define SD_CSD_READ_BL_LEN(pSd)        SD_CSD(pSd, 80,  4) ///< Max. read data block length
#define SD_CSD_READ_BL_PARTIAL(pSd)    SD_CSD(pSd, 79,  1) ///< Bartial blocks for read allowed
#define SD_CSD_WRITE_BLK_MISALIGN(pSd) SD_CSD(pSd, 78,  1) ///< Write block misalignment
#define SD_CSD_READ_BLK_MISALIGN(pSd)  SD_CSD(pSd, 77,  1) ///< Read block misalignment
#define SD_CSD_DSR_IMP(pSd)            SD_CSD(pSd, 76,  1) ///< DSP implemented
#define SD_CSD_C_SIZE(pSd)             ((SD_CSD(pSd, 72,  2) << 10) + \
                                        (SD_CSD(pSd, 64,  8) << 2)  + \
                                        SD_CSD(pSd, 62,  2)) ///< Device size
#define SD_CSD_C_SIZE_HC(pSd)          ((SD_CSD(pSd, 64,  6) << 16) + \
                                        (SD_CSD(pSd, 56,  8) << 8)  + \
                                        SD_CSD(pSd, 48,  8)) ///< Device size v2.0 High Capacity
#define SD_CSD_VDD_R_CURR_MIN(pSd)     SD_CSD(pSd, 59,  3) ///< Max. read current @VDD min
#define SD_CSD_VDD_R_CURR_MAX(pSd)     SD_CSD(pSd, 56,  3) ///< Max. read current @VDD max
#define SD_CSD_VDD_W_CURR_MIN(pSd)     SD_CSD(pSd, 53,  3) ///< Max. write current @VDD min
#define SD_CSD_VDD_W_CURR_MAX(pSd)     SD_CSD(pSd, 50,  3) ///< Max. write current @VDD max
#define SD_CSD_C_SIZE_MULT(pSd)        SD_CSD(pSd, 47,  3) ///< Device size multiplier
#define SD_CSD_ERASE_BLK_EN(pSd)       SD_CSD(pSd, 46,  1) ///< Erase single block enable
#define SD_CSD_SECTOR_SIZE(pSd)        ((SD_CSD(pSd, 40,  6) << 1) + SD_CSD(pSd, 39,  1)) ///< Erase sector size
#define SD_CSD_WP_GRP_SIZE(pSd)        SD_CSD(pSd, 32,  7) ///< Write protect group size
#define SD_CSD_WP_GRP_ENABLE(pSd)      SD_CSD(pSd, 31,  1) ///< write protect group enable
#define SD_CSD_R2W_FACTOR(pSd)         SD_CSD(pSd, 26,  3) ///< Write speed factor
#define SD_CSD_WRITE_BL_LEN(pSd)       ((SD_CSD(pSd, 24,  2) << 2) + SD_CSD(pSd, 22,  2)) ///< Max write block length
#define SD_CSD_WRITE_BL_PARTIAL(pSd)   SD_CSD(pSd, 21,  1) ///< Partial blocks for write allowed
#define SD_CSD_FILE_FORMAT_GRP(pSd)    SD_CSD(pSd, 15,  1) ///< File format group
#define SD_CSD_COPY(pSd)               SD_CSD(pSd, 14,  1) ///< Copy flag (OTP)
#define SD_CSD_PERM_WRITE_PROTECT(pSd) SD_CSD(pSd, 13,  1) ///< Permanent write protect
#define SD_CSD_TMP_WRITE_PROTECT(pSd)  SD_CSD(pSd, 12,  1) ///< Temporary write protection
#define SD_CSD_FILE_FORMAT(pSd)        SD_CSD(pSd, 11,  2) ///< File format
#define SD_CSD_CRC(pSd)                SD_CSD(pSd,  1,  7) ///< CRC
#define SD_CSD_MULT(pSd)               (1 << (SD_CSD_C_SIZE_MULT(pSd) + 2))
#define SD_CSD_BLOCKNR(pSd)            ((SD_CSD_C_SIZE(pSd) + 1) * SD_CSD_MULT(pSd))
#define SD_CSD_BLOCKNR_HC(pSd)         ((SD_CSD_C_SIZE_HC(pSd) + 1) * 1024)
#define SD_CSD_BLOCK_LEN(pSd)          (1 << SD_CSD_READ_BL_LEN(pSd))
#define SD_CSD_TOTAL_SIZE(pSd)         (SD_CSD_BLOCKNR(pSd) * SD_CSD_BLOCK_LEN(pSd))
#define SD_CSD_TOTAL_SIZE_HC(pSd)      ((SD_CSD_C_SIZE_HC(pSd) + 1) * 512* 1024)
#define SD_TOTAL_SIZE(pSd)             ((pSd)->totalSize)
#define SD_TOTAL_BLOCK(pSd)            ((pSd)->blockNr)

// SCR register access macros.
#define SD_SCR_BUS_WIDTHS(pScr)        ((pScr[1] >> 16) & 0xF) ///< Describes all the DAT bus that are supported by this card
#define SD_SCR_BUS_WIDTH_4BITS         (1 << 1) ///< 4bit Bus Width is supported
#define SD_SCR_BUS_WIDTH_1BIT          (1 << 0) ///< 1bit Bus Width is supported

//------------------------------------------------------------------------------
//         Types
//------------------------------------------------------------------------------

/// SD end-of-transfer callback function.
typedef void(*SdCallback)(uint8_t status, void *pCommand);

//------------------------------------------------------------------------------
/// SD Transfer Request prepared by the application upper layer. This structure
/// is sent to the SD_SendCommand function to start the transfer. At the end of
/// the transfer, the callback is invoked by the interrupt handler.
//------------------------------------------------------------------------------
typedef struct _SdCmd {

    /// Command status.
    volatile int8_t status;
    /// Command code.
    uint32_t cmd;
    /// Command argument.
    uint32_t arg;
    /// Data buffer.
    uint8_t *pData;
    /// Size of data buffer in bytes.
    uint16_t blockSize;
    /// Number of blocks to be transfered
    uint16_t nbBlock;
    /// Indicate if continue to transfer data
    uint8_t conTrans;
    /// Indicates if the command is a read operation.
    uint8_t isRead;
    /// Response buffer.
    uint32_t  *pResp;
    /// SD card response type.
    uint8_t  resType;
    /// Optional user-provided callback function.
    SdCallback callback;
    /// Optional argument to the callback function.
    void *pArg;

} SdCmd;

//------------------------------------------------------------------------------
/// SD driver structure. Holds the internal state of the SD driver and
/// prevents parallel access to a SPI peripheral.
//------------------------------------------------------------------------------
typedef struct {
    /// Pointer to a SPI peripheral.
    AT91S_MCI *pSdHw;
    /// SPI peripheral identifier.
    uint8_t spiId;
    /// Pointer to currently executing command.
    SdCmd *pCommand;
    /// Mutex.
    volatile int8_t semaphore;

} SdDriver;

//------------------------------------------------------------------------------
/// Sdcard driver structure. It holds the current command being processed and
/// the SD card address.
//------------------------------------------------------------------------------
typedef struct _SdCard {

    /// Pointer to the underlying MCI driver.
    SdDriver *pSdDriver;
    /// Current MCI command being processed.
    SdCmd command;
    /// SD card current address.
    uint16_t cardAddress;
    /// Card-specific data.
    uint32_t csd[4];
    /// Previous access block number.
    uint32_t preBlock;
    /// State after sd command complete
    uint8_t state;
    /// Card type
    uint8_t cardType;
    /// Card total size
    uint32_t totalSize;
    /// Card block number
    uint32_t blockNr;
    /// Card access mode
    uint8_t mode;

} SdCard;

// SD card operation states
#define SD_STATE_STBY     0
#define SD_STATE_DATA     1
#define SD_STATE_RCV      2

// Card type
#define UNKNOWN_CARD      0
#define CARD_SD           1
#define CARD_SDHC         2
#define CARD_MMC          3

// Delay between sending MMC commands
#define MMC_DELAY     0x4FF

#define SD_ADDRESS(pSd, address) (((pSd)->cardType == CARD_SDHC) ? \
                                 (address):((address) << SD_BLOCK_SIZE_BIT))

//-----------------------------------------------------------------------------
/// MMC/SD in SPI mode reports R1 status always, and R2 for SEND_STATUS
/// R1 is the low order byte; R2 is the next highest byte, when present.
//-----------------------------------------------------------------------------
#define R1_SPI_IDLE             (1 << 0)
#define R1_SPI_ERASE_RESET      (1 << 1)
#define R1_SPI_ILLEGAL_COMMAND  (1 << 2)
#define R1_SPI_COM_CRC          (1 << 3)
#define R1_SPI_ERASE_SEQ        (1 << 4)
#define R1_SPI_ADDRESS          (1 << 5)
#define R1_SPI_PARAMETER        (1 << 6)
// R1 bit 7 is always zero
#define R2_SPI_CARD_LOCKED      (1 << 0)
#define R2_SPI_WP_ERASE_SKIP    (1 << 1)
#define R2_SPI_LOCK_UNLOCK_FAIL R2_SPI_WP_ERASE_SKIP
#define R2_SPI_ERROR            (1 << 2)
#define R2_SPI_CC_ERROR         (1 << 3)
#define R2_SPI_CARD_ECC_ERROR   (1 << 4)
#define R2_SPI_WP_VIOLATION     (1 << 5)
#define R2_SPI_ERASE_PARAM      (1 << 6)
#define R2_SPI_OUT_OF_RANGE     (1 << 7)
#define R2_SPI_CSD_OVERWRITE    R2_SPI_OUT_OF_RANGE

// Status register constants
#define STATUS_READY_FOR_DATA   (1 << 8)
#define STATUS_IDLE             (0 << 9)
#define STATUS_READY            (1 << 9)
#define STATUS_IDENT            (2 << 9)
#define STATUS_STBY             (3 << 9)
#define STATUS_TRAN             (4 << 9)
#define STATUS_DATA             (5 << 9)
#define STATUS_RCV              (6 << 9)
#define STATUS_PRG              (7 << 9)
#define STATUS_DIS              (8 << 9)
#define STATUS_STATE          (0xF << 9)

//-----------------------------------------------------------------------------
/// OCR Register
//-----------------------------------------------------------------------------
#define AT91C_VDD_16_17          (1 << 4)
#define AT91C_VDD_17_18          (1 << 5)
#define AT91C_VDD_18_19          (1 << 6)
#define AT91C_VDD_19_20          (1 << 7)
#define AT91C_VDD_20_21          (1 << 8)
#define AT91C_VDD_21_22          (1 << 9)
#define AT91C_VDD_22_23          (1 << 10)
#define AT91C_VDD_23_24          (1 << 11)
#define AT91C_VDD_24_25          (1 << 12)
#define AT91C_VDD_25_26          (1 << 13)
#define AT91C_VDD_26_27          (1 << 14)
#define AT91C_VDD_27_28          (1 << 15)
#define AT91C_VDD_28_29          (1 << 16)
#define AT91C_VDD_29_30          (1 << 17)
#define AT91C_VDD_30_31          (1 << 18)
#define AT91C_VDD_31_32          (1 << 19)
#define AT91C_VDD_32_33          (1 << 20)
#define AT91C_VDD_33_34          (1 << 21)
#define AT91C_VDD_34_35          (1 << 22)
#define AT91C_VDD_35_36          (1 << 23)
#define AT91C_CARD_POWER_UP_BUSY (1 << 31)

#define AT91C_MMC_HOST_VOLTAGE_RANGE     (AT91C_VDD_27_28 +\
                                          AT91C_VDD_28_29 +\
                                          AT91C_VDD_29_30 +\
                                          AT91C_VDD_30_31 +\
                                          AT91C_VDD_31_32 +\
                                          AT91C_VDD_32_33)
#define AT91C_CCS    (1 << 30)

// MCI_CMD Register Value
#define AT91C_POWER_ON_INIT         (0  | AT91C_MCI_TRCMD_NO    | AT91C_MCI_SPCMD_INIT | AT91C_MCI_OPDCMD)

//-----------------------------------------------------------------------------
// Command Classes
//-----------------------------------------------------------------------------
//
// Class 0, 2, 4, 5, 7 and 8 are mandatory and shall be supported by all SD Memory Cards.
// Basic Commands (class 0)
//
// Cmd0 MCI + SPI
#define   AT91C_GO_IDLE_STATE_CMD     (0 | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE )
// Cmd1 SPI
#define   AT91C_MMC_SEND_OP_COND_CMD  (1 | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48 | AT91C_MCI_OPDCMD)
// Cmd2 MCI
#define   AT91C_ALL_SEND_CID_CMD      (2 | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_136 )
// Cmd3 MCI
#define   AT91C_SET_RELATIVE_ADDR_CMD (3 | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48 | AT91C_MCI_MAXLAT )
// Cmd4 MCI
//#define AT91C_SET_DSR_CMD             (4 | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_NO | AT91C_MCI_MAXLAT )
// cmd7 MCI
#define   AT91C_SEL_DESEL_CARD_CMD    (7  | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48 | AT91C_MCI_MAXLAT )
// Cmd8 MCI + SPI
#define   AT91C_SEND_IF_COND          (8  | AT91C_MCI_TRCMD_NO    | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48  | AT91C_MCI_MAXLAT )
// Cmd9 MCI + SPI
#define   AT91C_SEND_CSD_CMD          (9  | AT91C_MCI_TRCMD_NO    | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_136 | AT91C_MCI_MAXLAT )
// Cmd10 MCI + SPI
#define   AT91C_SEND_CID_CMD          (10 | AT91C_MCI_TRCMD_NO    | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_136 | AT91C_MCI_MAXLAT )
// Cmd12 MCI + SPI
#define   AT91C_STOP_TRANSMISSION_CMD (12 | AT91C_MCI_TRCMD_STOP  | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48  | AT91C_MCI_MAXLAT )
// Cmd13 MCI + SPI
#define   AT91C_SEND_STATUS_CMD       (13 | AT91C_MCI_TRCMD_NO    | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48  | AT91C_MCI_MAXLAT )
// Cmd15 MCI
//#define AT91C_GO_INACTIVE_STATE_CMD   (15 | AT91C_MCI_RSPTYP_NO )
// Cmd58 SPI
#define   AT91C_READ_OCR_CMD          (58 | AT91C_MCI_RSPTYP_48   | AT91C_MCI_SPCMD_NONE | AT91C_MCI_MAXLAT )
// Cmd59 SPI
#define   AT91C_CRC_ON_OFF_CMD        (59 | AT91C_MCI_RSPTYP_48   | AT91C_MCI_SPCMD_NONE | AT91C_MCI_MAXLAT )
//#define AT91C_MMC_ALL_SEND_CID_CMD      (2 | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_136| AT91C_MCI_OPDCMD)
//#define AT91C_MMC_SET_RELATIVE_ADDR_CMD (3 | AT91C_MCI_TRCMD_NO | AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48 | AT91C_MCI_MAXLAT | AT91C_MCI_OPDCMD)
//#define AT91C_MMC_READ_DAT_UNTIL_STOP_CMD (11 | AT91C_MCI_TRTYP_STREAM| AT91C_MCI_SPCMD_NONE | AT91C_MCI_RSPTYP_48  | AT91C_MCI_TRDIR  | AT91C_MCI_TRCMD_START | AT91C_MCI_MAXLAT )
//#define AT91C_STOP_TRANSMISSION_SYNC_CMD  (12 | AT91C_MCI_TRCMD_STOP  | AT91C_MCI_SPCMD_SYNC | AT91C_MCI_RSPTYP_48  | AT91C_MCI_MAXLAT )

//*------------------------------------------------
//* Class 2 commands: Block oriented Read commands
//*------------------------------------------------
// Cmd16
#define AT91C_SET_BLOCKLEN_CMD          (16 | AT91C_MCI_TRCMD_NO    | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_MAXLAT )
// Cmd17
#define AT91C_READ_SINGLE_BLOCK_CMD     (17 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_BLOCK | AT91C_MCI_TRDIR   | AT91C_MCI_MAXLAT)
// Cmd18
#define AT91C_READ_MULTIPLE_BLOCK_CMD   (18 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_MULTIPLE  | AT91C_MCI_TRDIR   | AT91C_MCI_MAXLAT)

//*------------------------------------------------
//* Class 4 commands: Block oriented write commands
//*------------------------------------------------
// Cmd24
#define AT91C_WRITE_BLOCK_CMD           (24 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_START | (AT91C_MCI_TRTYP_BLOCK    &  ~(AT91C_MCI_TRDIR))  | AT91C_MCI_MAXLAT)
// Cmd25
#define AT91C_WRITE_MULTIPLE_BLOCK_CMD  (25 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_START | (AT91C_MCI_TRTYP_MULTIPLE &  ~(AT91C_MCI_TRDIR))  | AT91C_MCI_MAXLAT)
// Cmd27
//#define AT91C_PROGRAM_CSD_CMD           (27 | AT91C_MCI_RSPTYP_48 )

//*----------------------------------------
//* Class 5 commands: Erase commands
//*----------------------------------------
// Cmd32
#define AT91C_TAG_SECTOR_START_CMD          (32 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT)
// Cmd33
#define AT91C_TAG_SECTOR_END_CMD            (33 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT)
// Cmd38
#define AT91C_ERASE_CMD                     (38 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT )

//*----------------------------------------
//* Class 7 commands: Lock commands
//*----------------------------------------
// Cmd42
//#define AT91C_LOCK_UNLOCK           (42 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT) // not tested

//*-----------------------------------------------
// Class 8 commands: Application specific commands
//*-----------------------------------------------
// Cmd55
#define AT91C_APP_CMD               (55 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO | AT91C_MCI_MAXLAT)
// cmd 56
//#define AT91C_GEN_CMD               (56 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO | AT91C_MCI_MAXLAT)    // not tested
// ACMD6
#define AT91C_SDCARD_SET_BUS_WIDTH_CMD          (6  | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT)
// ACMD13
//#define AT91C_SDCARD_STATUS_CMD                 (13 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_BLOCK | AT91C_MCI_TRDIR | AT91C_MCI_MAXLAT)
// ACMD22
//#define AT91C_SDCARD_SEND_NUM_WR_BLOCKS_CMD     (22 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT)
// ACMD23
//#define AT91C_SDCARD_SET_WR_BLK_ERASE_COUNT_CMD (23 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT)
// ACMD41
#define AT91C_SDCARD_APP_OP_COND_CMD            (41 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO )
// ACMD42
//#define AT91C_SDCARD_SET_CLR_CARD_DETECT_CMD    (42 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT)
// ACMD51
#define AT91C_SDCARD_SEND_SCR_CMD               (51 | AT91C_MCI_SPCMD_NONE  | AT91C_MCI_RSPTYP_48   | AT91C_MCI_TRCMD_NO    | AT91C_MCI_MAXLAT)

//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Sends the current SD card driver command to the card.
/// Returns 0 if successful; Otherwise, returns the transfer status code or
/// SD_ERROR_DRIVER if there was a problem with the SD transfer.
/// \param pSd  Pointer to a SdCard driver instance.
//------------------------------------------------------------------------------

static uint8_t SendCommand(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    SdDriver *pSdDriver = pSd->pSdDriver;
    uint8_t error;
    uint32_t i;

    // Send command
    MCI_SendCommand((Mci *)pSdDriver, (MciCmd *)pCommand);

    int32_t timeout = TRANSFER_CMD_TIMEOUT;
    // Wait for command to complete
    while (MCI_IsTxComplete((MciCmd *)pCommand) == false) {
        timeout--;
        if (timeout == 0) break;
    }

    if (pCommand->cmd == AT91C_STOP_TRANSMISSION_CMD) {
        while (MCI_CheckBusy((Mci *)pSdDriver) == true);
    }

    // Delay between sending commands, only for MMC card test.
    if ((pSd->cardType == CARD_MMC)
        || (pSd->cardType == UNKNOWN_CARD)
        || (pSd->cardType == CARD_SD)) {

        for (i = 0; i < MMC_DELAY; i++);
    }

    return pCommand->status;
}

//------------------------------------------------------------------------------
/// Initialization delay: The maximum of 1 msec, 74 clock cycles and supply ramp
/// up time.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SdCard driver instance.
//------------------------------------------------------------------------------
static uint8_t Pon(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_POWER_ON_INIT;
    pCommand->pResp = &response;

    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Resets all cards to idle state
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SdCard driver instance.
//------------------------------------------------------------------------------
static uint8_t Cmd0(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_GO_IDLE_STATE_CMD;
    pCommand->pResp = &response;

    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// MMC send operation condition command.
/// Sends host capacity support information and activates the card's
/// initialization process.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SdCard driver instance.
//------------------------------------------------------------------------------
static uint8_t Cmd1(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    uint8_t error;
    uint32_t response;


    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_MMC_SEND_OP_COND_CMD;
    pCommand->arg = AT91C_MMC_HOST_VOLTAGE_RANGE;
    pCommand->resType = 3;
    pCommand->pResp = &response;

    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // send command
    error = SendCommand(pSd);
    if (error) {
        return error;
    }
    if ((response & AT91C_CARD_POWER_UP_BUSY) == AT91C_CARD_POWER_UP_BUSY) {
        return SD_ERROR_NO_ERROR;
    }
    else {
        return SD_ERROR_DRIVER;
    }
}

//------------------------------------------------------------------------------
/// Asks any card to send the CID numbers
/// on the CMD line (any card that is
/// connected to the host will respond)
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
/// \param pCid  Buffer for storing the CID numbers.
//------------------------------------------------------------------------------
static uint8_t Cmd2(SdCard *pSd, uint32_t *pCid) {
    SdCmd *pCommand = &(pSd->command);

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill the command information
    pCommand->cmd = AT91C_ALL_SEND_CID_CMD;
    pCommand->resType = 2;
    pCommand->pResp = pCid;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send the command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Ask the card to publish a new relative address (RCA)
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
//------------------------------------------------------------------------------
static uint8_t Cmd3(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t cardAddress;
    uint8_t error;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_SET_RELATIVE_ADDR_CMD;
    // Assign relative address to MMC card
    if (pSd->cardType == CARD_MMC) {
        pCommand->arg = (0x1 << 16);
    }
    pCommand->resType = 1;
    pCommand->pResp = &cardAddress;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    error = SendCommand(pSd);
    if (error) {
        return error;
    }

    // Save card address in driver
    if (pSd->cardType != CARD_MMC) {
        pSd->cardAddress = (cardAddress >> 16) & 0xFFFF;
    }
    else {
        // Default MMC RCA is 0x0001
        pSd->cardAddress = 1;
    }

    return SD_ERROR_NO_ERROR;
}

//------------------------------------------------------------------------------
/// Command toggles a card between the
/// stand-by and transfer states or between
/// the programming and disconnect states.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
/// \param address  Relative Card Address (0 deselects all).
//------------------------------------------------------------------------------
static uint8_t Cmd7(SdCard *pSd, uint32_t address) {
    SdCmd *pCommand = &(pSd->command);

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_SEL_DESEL_CARD_CMD;
    pCommand->arg = address << 16;
    pCommand->resType = 1;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Sends SD Memory Card interface
/// condition, which includes host supply
/// voltage information and asks the card
/// whether card supports voltage.
/// Returns 0 if successful; otherwise returns SD_ERROR_NORESPONSE if the card did
/// not answer the command, or SD_ERROR_DRIVER.
/// \param pSd  Pointer to a SD card driver instance.
/// \param supplyVoltage  Expected supply voltage.
//------------------------------------------------------------------------------
static uint8_t Cmd8(SdCard *pSd, uint8_t supplyVoltage) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response[2];
    uint8_t error;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_SEND_IF_COND;
    pCommand->arg = (supplyVoltage << 8) | (0xAA);
    pCommand->resType = 7;
    pCommand->pResp = &response[0];
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    error = SendCommand(pSd);

    // Check result
    if (error == MCI_STATUS_NORESPONSE) {
        return SD_ERROR_NORESPONSE;
    }
    // SD_R7
    // Bit 0 - 7: check pattern
    // Bit 8 -11: voltage accepted
    else if (!error && ((response[0] & 0x00000FFF) == ((supplyVoltage << 8) | 0xAA))) {
        return SD_ERROR_NO_ERROR;
    }
    else {
        return SD_ERROR_DRIVER;
    }
}

//------------------------------------------------------------------------------
/// Addressed card sends its card-specific
/// data (CSD) on the CMD line.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
//------------------------------------------------------------------------------
static uint8_t Cmd9(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_SEND_CSD_CMD;
    pCommand->arg = pSd->cardAddress << 16;
    pCommand->resType = 2;
    pCommand->pResp = pSd->csd;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Forces the card to stop transmission
/// \param pSd  Pointer to a SD card driver instance.
/// \param pStatus  Pointer to a status variable.
//------------------------------------------------------------------------------
static uint8_t Cmd12(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_STOP_TRANSMISSION_CMD;
    pCommand->conTrans = MCI_NEW_TRANSFER;
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Addressed card sends its status register.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
/// \param pStatus  Pointer to a status variable.
//------------------------------------------------------------------------------
static uint8_t Cmd13(SdCard *pSd, uint32_t *pStatus) {
    SdCmd *pCommand = &(pSd->command);

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_SEND_STATUS_CMD;
    pCommand->arg = pSd->cardAddress << 16;
    pCommand->resType = 1;
    pCommand->pResp = pStatus;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// In the case of a Standard Capacity SD Memory Card, this command sets the
/// block length (in bytes) for all following block commands (read, write, lock).
/// Default block length is fixed to 512 Bytes.
/// Set length is valid for memory access commands only if partial block read
/// operation are allowed in CSD.
/// In the case of a High Capacity SD Memory Card, block length set by CMD16
/// command does not affect the memory read and write commands. Always 512
/// Bytes fixed block length is used. This command is effective for LOCK_UNLOCK command.
/// In both cases, if block length is set larger than 512Bytes, the card sets the
/// BLOCK_LEN_ERROR bit.
/// \param pSd  Pointer to a SD card driver instance.
/// \param blockLength  Block length in bytes.
//------------------------------------------------------------------------------
static uint8_t Cmd16(SdCard *pSd, uint16_t blockLength) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_SET_BLOCKLEN_CMD;
    pCommand->arg = blockLength;
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

// erase block start
uint8_t Cmd32(SdCard *pSd, uint16_t startsector) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;


    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_TAG_SECTOR_START_CMD;
    pCommand->arg = startsector;
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

// erase block end
uint8_t Cmd33(SdCard *pSd, uint16_t endsector) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_TAG_SECTOR_END_CMD;
    pCommand->arg = endsector;
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

// erase
uint8_t Cmd38(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_ERASE_CMD;
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}
//------------------------------------------------------------------------------
/// Continously transfers datablocks from card to host until interrupted by a
/// STOP_TRANSMISSION command.
/// \param pSd  Pointer to a SD card driver instance.
/// \param blockSize  Block size (shall be set to 512 in case of high capacity).
/// \param pData  Pointer to the application buffer to be filled.
/// \param address  SD card address.
//------------------------------------------------------------------------------
static uint8_t Cmd17(SdCard *pSd, uint16_t nbBlock, uint8_t *pData, uint32_t address) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_READ_SINGLE_BLOCK_CMD;
    pCommand->arg = address;
    pCommand->blockSize = SD_BLOCK_SIZE;
    pCommand->nbBlock = nbBlock;
    pCommand->pData = pData;
    pCommand->isRead = 1;
    pCommand->conTrans = MCI_NEW_TRANSFER;
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_DATA;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Continously transfers datablocks from card to host until interrupted by a
/// STOP_TRANSMISSION command.
/// \param pSd  Pointer to a SD card driver instance.
/// \param blockSize  Block size (shall be set to 512 in case of high capacity).
/// \param pData  Pointer to the application buffer to be filled.
/// \param address  SD card address.
//------------------------------------------------------------------------------
static uint8_t Cmd18(SdCard *pSd, uint16_t nbBlock, uint8_t *pData, uint32_t address) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_READ_MULTIPLE_BLOCK_CMD;
    pCommand->arg = address;
    pCommand->blockSize = SD_BLOCK_SIZE;
    pCommand->nbBlock = nbBlock;
    pCommand->pData = pData;
    pCommand->isRead = 1;
    pCommand->conTrans = MCI_NEW_TRANSFER;
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_DATA;

    // Send command
    return SendCommand(pSd);
}
//------------------------------------------------------------------------------
/// Write block command
/// \param pSd  Pointer to a SD card driver instance.
/// \param blockSize  Block size (shall be set to 512 in case of high capacity).
/// \param pData  Pointer to the application buffer to be filled.
/// \param address  SD card address.
//------------------------------------------------------------------------------
static uint8_t Cmd24(SdCard *pSd, uint16_t nbBlock, uint8_t *pData, uint32_t address) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;


    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_WRITE_BLOCK_CMD;
    pCommand->arg = address;
    pCommand->blockSize = SD_BLOCK_SIZE;
    pCommand->nbBlock = nbBlock;
    pCommand->pData = (uint8_t *)pData;
    pCommand->conTrans = MCI_NEW_TRANSFER;
    pCommand->resType = 1;
    pCommand->pResp = &response;

    // Set SD command state
    pSd->state = SD_STATE_RCV;

    // Send command
    return SendCommand(pSd);
}
//------------------------------------------------------------------------------
/// Write block command
/// \param pSd  Pointer to a SD card driver instance.
/// \param blockSize  Block size (shall be set to 512 in case of high capacity).
/// \param pData  Pointer to the application buffer to be filled.
/// \param address  SD card address.
//------------------------------------------------------------------------------
static uint8_t Cmd25(SdCard *pSd, uint16_t nbBlock, uint8_t *pData, uint32_t address) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;


    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_WRITE_MULTIPLE_BLOCK_CMD;
    pCommand->arg = address;
    pCommand->blockSize = SD_BLOCK_SIZE;
    pCommand->nbBlock = nbBlock;
    pCommand->pData = (uint8_t *)pData;
    pCommand->conTrans = MCI_NEW_TRANSFER;
    pCommand->resType = 1;
    pCommand->pResp = &response;

    // Set SD command state
    pSd->state = SD_STATE_RCV;

    // Send command
    return SendCommand(pSd);
}


//------------------------------------------------------------------------------
/// Initialization delay: The maximum of 1 msec, 74 clock cycles and supply
/// ramp up time.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
//------------------------------------------------------------------------------
static uint8_t Cmd55(SdCard *pSd) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_APP_CMD;
    pCommand->arg = (pSd->cardAddress << 16);
    pCommand->resType = 1;
    pCommand->pResp = &response;
    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// SPI Mode, Reads the OCR register of a card
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
/// \param pOcr   OCR value of the card
//------------------------------------------------------------------------------
static uint8_t Cmd58(SdCard *pSd, uint32_t *pOcr) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response[2];


    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_READ_OCR_CMD;
    pCommand->resType = 3;
    pCommand->pResp = &response[0];

    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// SPI Mode, Set CRC option of a card
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
/// \param option  CRC option, 1 to turn on, 0 to trun off
//------------------------------------------------------------------------------
static uint8_t Cmd59(SdCard *pSd, uint8_t option) {
    SdCmd *pCommand = &(pSd->command);
    uint32_t response;

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_CRC_ON_OFF_CMD;
    pCommand->arg = (option & 0x1);
    pCommand->resType = 1;
    pCommand->pResp = &response;

    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Defines the data bus width (’00’=1bit or ’10’=4 bits bus) to be used for data transfer.
/// The allowed data bus widths are given in SCR register.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
/// \param busWidth  Bus width in bits.
//------------------------------------------------------------------------------
static uint8_t Acmd6(SdCard *pSd, uint8_t busWidth) {
    SdCmd *pCommand = &(pSd->command);
    uint8_t error;
    uint32_t response;

    // Delay
    error = Cmd55(pSd);

    if (error) {
        return error;
    }

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->cmd = AT91C_SDCARD_SET_BUS_WIDTH_CMD;
    if (busWidth == 4) {
        pCommand->arg = SD_SCR_BUS_WIDTH_4BITS;
    }
    else {
        pCommand->arg = SD_SCR_BUS_WIDTH_1BIT;
    }
    pCommand->resType = 1;
    pCommand->pResp = &response;

    // Set SD command state
    pSd->state = SD_STATE_STBY;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Asks to all cards to send their operations conditions.
/// Returns the command transfer result (see SendCommand).
/// \param pSd  Pointer to a SD card driver instance.
/// \param hcs  Shall be true if Host support High capacity.
/// \param pCCS  Set the pointed flag to 1 if hcs != 0 and SD OCR CCS flag is set.
//------------------------------------------------------------------------------
static uint8_t Acmd41(SdCard *pSd, uint8_t hcs, uint8_t *pCCS) {
    SdCmd *pCommand = &(pSd->command);
    uint8_t error;
    uint32_t response;

    do {
        error = Cmd55(pSd);
        if (error) {
            return error;
        }

        memset(pCommand, 0, sizeof(SdCmd));
        // Fill command information
        pCommand->cmd = AT91C_SDCARD_APP_OP_COND_CMD;
        pCommand->arg = AT91C_MMC_HOST_VOLTAGE_RANGE;
        if (hcs) {
            pCommand->arg |= AT91C_CCS;
        }

        pCommand->resType = 3;
        pCommand->pResp = &response;

        // Set SD command state
        pSd->state = SD_STATE_STBY;

        // Send command

        error = SendCommand(pSd);
        if (error) {
            return error;
        }
        *pCCS = ((response & AT91C_CCS) != 0);
    } while ((response & AT91C_CARD_POWER_UP_BUSY) != AT91C_CARD_POWER_UP_BUSY);

    return SD_ERROR_NO_ERROR;
}

//------------------------------------------------------------------------------
/// Continue to transfer datablocks from card to host until interrupted by a
/// STOP_TRANSMISSION command.
/// \param pSd  Pointer to a SD card driver instance.
/// \param blockSize  Block size (shall be set to 512 in case of high capacity).
/// \param pData  Pointer to the application buffer to be filled.
/// \param address  SD card address.
//------------------------------------------------------------------------------
static uint8_t ContinuousRead(SdCard *pSd, uint16_t nbBlock, uint8_t *pData, uint32_t address) {
    SdCmd *pCommand = &(pSd->command);

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->blockSize = SD_BLOCK_SIZE;
    pCommand->nbBlock = nbBlock;
    pCommand->pData = pData;
    pCommand->isRead = 1;
    pCommand->conTrans = MCI_CONTINUE_TRANSFER;
    // Set SD command state
    pSd->state = SD_STATE_DATA;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Continue to transfer datablocks from host to card until interrupted by a
/// STOP_TRANSMISSION command.
/// \param pSd  Pointer to a SD card driver instance.
/// \param blockSize  Block size (shall be set to 512 in case of high capacity).
/// \param pData  Pointer to the application buffer to be filled.
/// \param address  SD card address.
//------------------------------------------------------------------------------
static uint8_t ContinuousWrite(SdCard *pSd, uint16_t nbBlock, const uint8_t *pData, uint32_t address) {
    SdCmd *pCommand = &(pSd->command);

    memset(pCommand, 0, sizeof(SdCmd));
    // Fill command information
    pCommand->blockSize = SD_BLOCK_SIZE;
    pCommand->nbBlock = nbBlock;
    pCommand->pData = (uint8_t *)pData;
    pCommand->isRead = 0;
    pCommand->conTrans = MCI_CONTINUE_TRANSFER;
    // Set SD command state
    pSd->state = SD_STATE_RCV;

    // Send command
    return SendCommand(pSd);
}

//------------------------------------------------------------------------------
/// Move SD card to transfer state. The buffer size must be at
/// least 512 byte long. This function checks the SD card status register and
/// address the card if required before sending the transfer command.
/// Returns 0 if successful; otherwise returns an code describing the error.
/// \param pSd  Pointer to a SD card driver instance.
/// \param address  Address of the block to transfer.
/// \param nbBlocks Number of blocks to be transfer.
/// \param pData  Data buffer whose size is at least the block size.
/// \param isRead 1 for read data and 0 for write data.
//------------------------------------------------------------------------------


static uint8_t MoveToTransferState(SdCard *pSd, uint32_t address, uint16_t nbBlocks, uint8_t *pData, uint8_t isRead, int32_t timeout) {
    uint32_t status;
    uint8_t error;
    int32_t trycnt = timeout;

    if ((pSd->state == SD_STATE_DATA)
        || (pSd->state == SD_STATE_RCV)) {

        error = Cmd12(pSd);
        if (error) {
            return error;
        }
    }

    pSd->preBlock = address + (nbBlocks - 1);

    if (isRead) {

        // Wait for card to be ready for data transfers
        do {

            error = Cmd13(pSd, &status);

            AT91_Time_Delay(nullptr, 1);

            trycnt--;

            if (error && trycnt == 0) {
                return error;
            }
        }

        while (((status & STATUS_READY_FOR_DATA) == 0) ||
            ((status & STATUS_STATE) != STATUS_TRAN));

        // cmd17 read single block
        return Cmd17(pSd, nbBlocks, pData, SD_ADDRESS(pSd, address));
    }
    else {

        // Wait for card to be ready for data transfers
        do {
            error = Cmd13(pSd, &status);

            AT91_Time_Delay(nullptr, 1);

            trycnt--;

            if (error && trycnt == 0) {
                return error;
            }

        } while ((status & STATUS_READY_FOR_DATA) == 0);

        // Move to Sending data state
        return Cmd24(pSd, nbBlocks, pData, SD_ADDRESS(pSd, address));
    }

    return error;
}

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Read Block of data in a buffer pointed by pData. The buffer size must be at
/// least 512 byte long. This function checks the SD card status register and
/// address the card if required before sending the read command.
/// Returns 0 if successful; otherwise returns an code describing the error.
/// \param pSd  Pointer to a SD card driver instance.
/// \param address  Address of the block to read.
/// \param nbBlocks Number of blocks to be read.
/// \param pData  Data buffer whose size is at least the block size.
//------------------------------------------------------------------------------
uint8_t SD_ReadBlock(SdCard *pSd, uint32_t address, uint16_t nbBlocks, uint8_t *pData, int32_t timeout) {
    return MoveToTransferState(pSd, address, nbBlocks, pData, 1, timeout);;
}

//------------------------------------------------------------------------------
/// Write Block of data pointed by pData. The buffer size must be at
/// least 512 byte long. This function checks the SD card status register and
/// address the card if required before sending the read command.
/// Returns 0 if successful; otherwise returns an SD_ERROR code.
/// \param pSd  Pointer to a SD card driver instance.
/// \param address  Address of block to write.
/// \param nbBlocks Number of blocks to be read
/// \param pData  Pointer to a 512 bytes buffer to be transfered
//------------------------------------------------------------------------------
uint8_t SD_WriteBlock(SdCard *pSd, uint32_t address, uint16_t nbBlocks, const uint8_t *pData, int32_t timeout) {
    return MoveToTransferState(pSd, address, nbBlocks,
        (uint8_t *)pData, 0, timeout);
}

//------------------------------------------------------------------------------
/// Run the SDcard SD Mode initialization sequence. This function runs the
/// initialisation procedure and the identification process, then it sets the
/// SD card in transfer state to set the block length and the bus width.
/// Returns 0 if successful; otherwise returns an SD_ERROR code.
/// \param pSd  Pointer to a SD card driver instance.
/// \param pSdDriver  Pointer to SD driver already initialized
//------------------------------------------------------------------------------

bool SD_ReadyToTransfer(SdCard *pSd, int32_t timeout) {
    uint32_t status;

    while (Cmd13(pSd, &status) && timeout--) {
        AT91_Time_Delay(nullptr, 1);
    }

    return timeout > 0;
}
uint8_t SD_MCI_Init(SdCard *pSd, SdDriver *pSdDriver) {
    uint32_t sdCid[4];
    uint8_t isCCSet;
    uint8_t error;
    uint32_t status;
    uint8_t cmd8Retries = 2;
    uint8_t cmd1Retries = 100;

    // The command GO_IDLE_STATE (CMD0) is the software reset command and sets card into Idle State
    // regardless of the current card state.
    error = Cmd0(pSd);
    if (error) {
        return error;
    }

    // CMD8 is newly added in the Physical Layer Specification Version 2.00 to support multiple voltage
    // ranges and used to check whether the card supports supplied voltage. The version 2.00 host shall
    // issue CMD8 and verify voltage before card initialization.
    // The host that does not support CMD8 shall supply high voltage range...

    do {
        error = Cmd8(pSd, 1);
    } while ((error == SD_ERROR_NORESPONSE) && (cmd8Retries-- > 0));

    if (error == SD_ERROR_NORESPONSE) {
        // No response : Ver2.00 or later SD Memory Card(voltage mismatch)
        // or Ver1.X SD Memory Card
        // or not SD Memory Card

        // ACMD41 is a synchronization command used to negotiate the operation voltage range and to poll the
        // cards until they are out of their power-up sequence.
        error = Acmd41(pSd, 0, &isCCSet);
        if (error) {
            // Acmd41 failed : MMC card or unknown card
            error = Cmd0(pSd);
            if (error) {
                return error;
            }
            do {
                error = Cmd1(pSd);
            } while ((error) && (cmd1Retries-- > 0));
            if (error) {
                return error;
            }
            else {

                pSd->cardType = CARD_MMC;
            }
        }
        else {
            if (isCCSet == 0) {

                pSd->cardType = CARD_SD;
            }
        }
    }
    else if (!error) {

        // Valid response : Ver2.00 or later SD Memory Card
        error = Acmd41(pSd, 1, &isCCSet);
        if (error) {
            return error;
        }
        if (isCCSet) {

            pSd->cardType = CARD_SDHC;
        }
        else {

            pSd->cardType = CARD_SD;
        }
    }
    else {
        return error;
    }

    // The host then issues the command ALL_SEND_CID (CMD2) to the card to get its unique card identification (CID) number.
    // Card that is unidentified (i.e. which is in Ready State) sends its CID number as the response (on the CMD line).
    error = Cmd2(pSd, sdCid);
    if (error) {
        return error;
    }

    // Thereafter, the host issues CMD3 (SEND_RELATIVE_ADDR) asks the
    // card to publish a new relative card address (RCA), which is shorter than CID and which is used to
    // address the card in the future data transfer mode. Once the RCA is received the card state changes to
    // the Stand-by State. At this point, if the host wants to assign another RCA number, it can ask the card to
    // publish a new number by sending another CMD3 command to the card. The last published RCA is the
    // actual RCA number of the card.
    error = Cmd3(pSd);
    if (error) {
        return error;
    }

    // The host issues SEND_CSD (CMD9) to obtain the Card Specific Data (CSD register),
    // e.g. block length, card storage capacity, etc...
    error = Cmd9(pSd);
    if (error) {
        return error;
    }

    // At this stage the Initialization and identification process is achieved
    // The SD card is supposed to be in Stand-by State
    do {
        error = Cmd13(pSd, &status);
        if (error) {
            return error;
        }
    } while ((status & STATUS_READY_FOR_DATA) == 0);

    // If the 4 bit bus transfer is supported switch to this mode
    // Select the current SD, goto transfer state
    error = Cmd7(pSd, pSd->cardAddress);
    if (error) {
        return error;
    }

    if (pSd->cardType != CARD_MMC) {
        // Switch to 4 bits bus width (All SD Card shall support 1-bit, 4 bitswidth
        error = Acmd6(pSd, 4); //1); //4); //                                                    <--- To force 1 bit bus?
        if (error) {
            return error;
        }
    }
    else {
        MCI_SetBusWidth((Mci *)pSdDriver, MCI_SDCBUS_1BIT);
    }
    return SD_ERROR_NO_ERROR;
}

//------------------------------------------------------------------------------
/// Run the SDcard SPI Mode initialization sequence. This function runs the
/// initialisation procedure and the identification process, then it sets the SD
/// card in transfer state to set the block length.
/// Returns 0 if successful; otherwise returns an SD_ERROR code.
/// \param pSd  Pointer to a SD card driver instance.
/// \param pSdDriver  Pointer to SD driver already initialized.
//------------------------------------------------------------------------------
uint8_t SD_SPI_Init(SdCard *pSd, SdDriver *pSpi) {
    uint8_t isCCSet;
    uint8_t error;
    uint8_t cmd8Retries = 2;
    uint8_t cmd1Retries = 1;
    uint32_t pOCR;

    // The command GO_IDLE_STATE (CMD0) is the software reset command and sets card into Idle State
    // regardless of the current card state.
    error = Cmd0(pSd);
    if (error) {
        return error;
    }

    // CMD8 is newly added in the Physical Layer Specification Version 2.00 to support multiple voltage
    // ranges and used to check whether the card supports supplied voltage. The version 2.00 host shall
    // issue CMD8 and verify voltage before card initialization.
    // The host that does not support CMD8 shall supply high voltage range...
    do {
        error = Cmd8(pSd, 1);
    } while ((error == SD_ERROR_NORESPONSE) && (cmd8Retries-- > 0));

    if (error == SD_ERROR_NORESPONSE) {
        // No response : Ver2.00 or later SD Memory Card(voltage mismatch)
        // or Ver1.X SD Memory Card
        // or not SD Memory Card

        error = Cmd58(pSd, &pOCR);
        if (error) {
            return error;
        }

        // ACMD41 is a synchronization command used to negotiate the operation voltage range and to poll the
        // cards until they are out of their power-up sequence.
        error = Acmd41(pSd, 0, &isCCSet);

        if (error) {
            // Acmd41 failed : MMC card or unknown card
            error = Cmd0(pSd);
            if (error) {
                return error;
            }
            do {
                error = Cmd1(pSd);
            } while ((error) && (cmd1Retries-- > 0));

            if (error) {
                return error;
            }
            else {
                pSd->cardType = CARD_MMC;
            }
        }
        else {
            if (isCCSet == 0) {

                pSd->cardType = CARD_SD;
            }
        }
    }
    else if (!error) {
        error = Cmd58(pSd, &pOCR);
        if (error) {
            return error;
        }

        // Valid response : Ver2.00 or later SD Memory Card
        error = Acmd41(pSd, 1, &isCCSet);
        if (error) {
            return error;
        }
        error = Cmd58(pSd, &pOCR);
        if (error) {
            return error;
        }
        if (isCCSet) {

            pSd->cardType = CARD_SDHC;
        }
        else {

            pSd->cardType = CARD_SD;
        }
    }
    else {
        return error;
    }

    if (pSd->cardType != CARD_MMC) {
        // The host issues CRC_ON_OFF (CMD59) to set data CRC on/off
        // The host can turn the CRC option on and off using the CRC_ON_OFF command (CMD59).
        // Host should enable CRC verification before issuing ACMD41.
        error = Cmd59(pSd, 0);  // turn crc option OFF

        if (error) {
            return error;
        }
    }

    // The host issues SEND_CSD (CMD9) to obtain the Card Specific Data (CSD register),
    // e.g. block length, card storage capacity, etc...
    error = Cmd9(pSd);

    if (error) {
        return error;
    }

    return SD_ERROR_NO_ERROR;
}
//

//------------------------------------------------------------------------------
/// Run the SDcard initialization sequence. This function runs the initialisation
/// procedure and the identification process, then it sets the SD card in transfer
/// state to set the block length and the bus width.
/// Returns 0 if successful; otherwise returns an SD_ERROR code.
/// \param pSd  Pointer to a SD card driver instance.
/// \param pSdDriver  Pointer to SD driver already initialized.
//------------------------------------------------------------------------------
uint8_t SD_Init(SdCard *pSd, SdDriver *pSdDriver) {
    uint8_t error;

    // Initialize SdCard structure
    pSd->pSdDriver = pSdDriver;
    pSd->cardAddress = 0;
    pSd->preBlock = 0xffffffff;
    pSd->state = SD_STATE_STBY;
    pSd->cardType = UNKNOWN_CARD;
    memset(&(pSd->command), 0, sizeof(SdCmd));

    // Initialization delay: The maximum of 1 msec, 74 clock cycles and supply ramp up time
    // ‘Supply ramp up time’ provides the time that the power is built up to the operating level (the bus
    // master supply voltage) and the time to wait until the SD card can accept the first command

    // Power On Init Special Command

    error = Pon(pSd);

    if (error) {
        return error;
    }

    // After power-on or CMD0, all cards’ CMD lines are in input mode, waiting for start bit of the next command.
    // The cards are initialized with a default relative card address (RCA=0x0000) and with a default
    // driver stage register setting (lowest speed, highest driving current capability).

    error = SD_MCI_Init(pSd, pSdDriver);

    if (error) {
        return error;
    }

    // In the case of a Standard Capacity SD Memory Card, this command sets the
    // block length (in bytes) for all following block commands (read, write, lock).
    // Default block length is fixed to 512 Bytes.
    // Set length is valid for memory access commands only if partial block read
    // operation are allowed in CSD.
    // In the case of a High Capacity SD Memory Card, block length set by CMD16
    // command does not affect the memory read and write commands. Always 512
    // Bytes fixed block length is used. This command is effective for LOCK_UNLOCK command.
    // In both cases, if block length is set larger than 512Bytes, the card sets the
    // BLOCK_LEN_ERROR bit.
    if (pSd->cardType == CARD_SD) {
        error = Cmd16(pSd, SD_BLOCK_SIZE);
        if (error) {
            return error;
        }
    }

    // If SD CSD v2.0
    if ((pSd->cardType != CARD_MMC) && (SD_CSD_STRUCTURE(pSd) == 1)) {
        pSd->totalSize = SD_CSD_TOTAL_SIZE_HC(pSd);
        pSd->blockNr = SD_CSD_BLOCKNR_HC(pSd);
    }
    else {
        pSd->totalSize = SD_CSD_TOTAL_SIZE(pSd);
        pSd->blockNr = SD_CSD_BLOCKNR(pSd);
    }

    if (pSd->cardType == UNKNOWN_CARD) {
        return SD_ERROR_NOT_INITIALIZED;
    }
    else {
        return SD_ERROR_NO_ERROR;
    }
}

//------------------------------------------------------------------------------
/// Stop the SDcard. This function stops all SD operations.
/// Returns 0 if successful; otherwise returns an SD_ERROR code.
/// \param pSd  Pointer to a SD card driver instance.
/// \param pSdDriver  Pointer to MCI driver already initialized.
//------------------------------------------------------------------------------
uint8_t SD_Stop(SdCard *pSd, SdDriver *pSdDriver) {
    uint8_t error;
    SdCmd *pCommand = &(pSd->command);

    if (pCommand->conTrans == MCI_CONTINUE_TRANSFER) {
        error = Cmd12(pSd);
        if (error) {
            return error;
        }
    }

    MCI_Close((Mci *)pSdDriver);

    return SD_ERROR_NO_ERROR;
}

//AT91
static TinyCLR_SdCard_Controller sdCardProvider;
static TinyCLR_Api_Info sdApi;

#define AT91_SD_SECTOR_SIZE 512
#define AT91_SD_TIMEOUT 5000000

struct SdController {
    int32_t controller;
    size_t  sectorCount;

    size_t  *sectorSizes;
    uint8_t *pBuffer;
    uint8_t *pBufferAligned;
};

static const AT91_Gpio_Pin g_AT91_SdCard_Data0_Pins[] = AT91_SD_DATA0_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Data1_Pins[] = AT91_SD_DATA1_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Data2_Pins[] = AT91_SD_DATA2_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Data3_Pins[] = AT91_SD_DATA3_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Clk_Pins[] = AT91_SD_CLK_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Cmd_Pins[] = AT91_SD_CMD_PINS;

SdController sdController[1];

/// MCI driver instance.
static Mci mciDrv;

/// SDCard driver instance.
static SdCard sdDrv;

const TinyCLR_Api_Info* AT91_SdCard_GetApi() {
    sdCardProvider.ApiInfo = &sdApi;

    sdCardProvider.Acquire = &AT91_SdCard_Acquire;
    sdCardProvider.Release = &AT91_SdCard_Release;
    sdCardProvider.GetControllerCount = &AT91_SdCard_GetControllerCount;

    sdCardProvider.WriteSectors = &AT91_SdCard_WriteSector;
    sdCardProvider.ReadSectors = &AT91_SdCard_ReadSector;
    sdCardProvider.EraseSectors = &AT91_SdCard_EraseSector;
    sdCardProvider.IsSectorErased = &AT91_SdCard_IsSectorErased;
    sdCardProvider.GetSectorMap = &AT91_SdCard_GetSectorMap;

    sdApi.Author = "GHI Electronics, LLC";
    sdApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.SdCardProvider";
    sdApi.Type = TinyCLR_Api_Type::SdCardProvider;
    sdApi.Version = 0;
    sdApi.Implementation = &sdCardProvider;

    return &sdApi;
}

TinyCLR_Result AT91_SdCard_Acquire(const TinyCLR_SdCard_Controller* self) {
    sdController[controller].controller = controller;

    auto d0 = g_AT91_SdCard_Data0_Pins[controller];
    auto d1 = g_AT91_SdCard_Data1_Pins[controller];
    auto d2 = g_AT91_SdCard_Data2_Pins[controller];
    auto d3 = g_AT91_SdCard_Data3_Pins[controller];
    auto clk = g_AT91_SdCard_Clk_Pins[controller];
    auto cmd = g_AT91_SdCard_Cmd_Pins[controller];

    if (!AT91_Gpio_OpenPin(d0.number)
        || !AT91_Gpio_OpenPin(d1.number)
        || !AT91_Gpio_OpenPin(d2.number)
        || !AT91_Gpio_OpenPin(d3.number)
        || !AT91_Gpio_OpenPin(clk.number)
        || !AT91_Gpio_OpenPin(cmd.number)
        )
        return TinyCLR_Result::SharingViolation;

    AT91_Gpio_ConfigurePin(d0.number, AT91_Gpio_Direction::Input, d0.peripheralSelection, AT91_Gpio_ResistorMode::PullUp);
    AT91_Gpio_ConfigurePin(d1.number, AT91_Gpio_Direction::Input, d1.peripheralSelection, AT91_Gpio_ResistorMode::PullUp);
    AT91_Gpio_ConfigurePin(d2.number, AT91_Gpio_Direction::Input, d2.peripheralSelection, AT91_Gpio_ResistorMode::PullUp);
    AT91_Gpio_ConfigurePin(d3.number, AT91_Gpio_Direction::Input, d3.peripheralSelection, AT91_Gpio_ResistorMode::PullUp);
    AT91_Gpio_ConfigurePin(clk.number, AT91_Gpio_Direction::Input, clk.peripheralSelection, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(cmd.number, AT91_Gpio_Direction::Input, cmd.peripheralSelection, AT91_Gpio_ResistorMode::PullUp);

    AT91_PMC &pmc = AT91::PMC();
    pmc.EnablePeriphClock(AT91C_ID_HSMCI0);

    DMA_Init();

    MCI_Init(&mciDrv, (AT91PS_MCI)AT91C_BASE_MCI, AT91C_ID_HSMCI0, MCI_SD_SLOTA);

    AT91_Interrupt_Activate(AT91C_ID_HSMCI0, (uint32_t*)&MCI_Handler, (void*)&mciDrv);

    if (SD_Init(&sdDrv, (SdDriver *)&mciDrv) != SD_ERROR_NO_ERROR) {

        return TinyCLR_Result::InvalidOperation;
    }

    MCI_SetSpeed(&mciDrv, 8000000);

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

    sdController[controller].pBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, AT91_SD_SECTOR_SIZE + 4);

    uint32_t alignAddress = (uint32_t)sdController[controller].pBuffer;

    while (alignAddress % 4 > 0) {
        alignAddress++;
    }

    sdController[controller].pBufferAligned = (uint8_t*)alignAddress;

    sdController[controller].sectorSizes = (size_t*)memoryProvider->Allocate(memoryProvider, sizeof(size_t));

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_Release(const TinyCLR_SdCard_Controller* self) {
    auto d0 = g_AT91_SdCard_Data0_Pins[controller];
    auto d1 = g_AT91_SdCard_Data1_Pins[controller];
    auto d2 = g_AT91_SdCard_Data2_Pins[controller];
    auto d3 = g_AT91_SdCard_Data3_Pins[controller];
    auto clk = g_AT91_SdCard_Clk_Pins[controller];
    auto cmd = g_AT91_SdCard_Cmd_Pins[controller];

    AT91_PMC &pmc = AT91::PMC();

    pmc.DisablePeriphClock(AT91C_ID_HSMCI0); /* Disable clock to the Mci block */
    pmc.DisablePeriphClock(AT91C_ID_DMAC0); /* Disable clock to the Dma block */

    AT91_Interrupt_Deactivate(AT91C_ID_HSMCI0); /* Disable Interrupt */

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

    memoryProvider->Free(memoryProvider, sdController[controller].pBuffer);
    memoryProvider->Free(memoryProvider, sdController[controller].sectorSizes);

    AT91_Gpio_ClosePin(d0.number);
    AT91_Gpio_ClosePin(d1.number);
    AT91_Gpio_ClosePin(d2.number);
    AT91_Gpio_ClosePin(d3.number);
    AT91_Gpio_ClosePin(clk.number);
    AT91_Gpio_ClosePin(cmd.number);

    return TinyCLR_Result::Success;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_GetControllerCount(const TinyCLR_SdCard_Controller* self, int32_t& count) {
    count = SIZEOF_ARRAY(g_AT91_SdCard_Data0_Pins);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_WriteSector(const TinyCLR_SdCard_Controller* self, uint64_t sector, size_t& count, const uint8_t* data, int32_t timeout) {
    int32_t to;

    int32_t sectorCount = count;

    int32_t sectorNum = sector;

    SdCard *pSd = &sdDrv;

    uint8_t* pData = (uint8_t*)data;

    Mci *pMci = &mciDrv;

    AT91S_MCI *pMciHw = pMci->pMciHw;

    volatile uint32_t error = 0;

    volatile uint32_t status;

    while (sectorCount > 0) {
        memcpy(sdController[controller].pBufferAligned, pData, AT91_SD_SECTOR_SIZE);

        AT91_Cache_DisableCaches();

        if (SD_ReadyToTransfer(pSd, timeout) == false) {
            return TinyCLR_Result::InvalidOperation;
        }

        to = timeout;

        if ((error = SD_WriteBlock(&sdDrv, sectorNum, 1, sdController[controller].pBufferAligned, timeout)) == SD_ERROR_NO_ERROR) {
            to = timeout;

            while (to > 0 && (((status & AT91C_MCI_DMADONE) != AT91C_MCI_DMADONE) || ((status & AT91C_MCI_XFRDONE) != AT91C_MCI_XFRDONE) || ((status & AT91C_MCI_BLKE) != AT91C_MCI_BLKE))) {
                AT91_Time_Delay(nullptr, 1);
                to--;
                status |= READ_MCI(pMciHw, MCI_SR);
            }
        }

        AT91_Cache_EnableCaches();

        if (error) {
            return TinyCLR_Result::InvalidOperation;
        }

        if (!to) {
            return TinyCLR_Result::TimedOut;
        }

        pData += AT91_SD_SECTOR_SIZE;
        sectorNum++;
        sectorCount--;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_ReadSector(const TinyCLR_SdCard_Controller* self, uint64_t sector, size_t& count, uint8_t* data, int32_t timeout) {
    int32_t to;

    int32_t sectorCount = count;

    int32_t sectorNum = sector;

    SdCard *pSd = &sdDrv;

    Mci *pMci = &mciDrv;

    AT91S_MCI *pMciHw = pMci->pMciHw;

    volatile uint32_t error = 0;

    volatile  uint32_t status;

    uint8_t* pData = (uint8_t*)data;

    while (sectorCount > 0) {
        memset(sdController[controller].pBufferAligned, 0, AT91_SD_SECTOR_SIZE);

        AT91_Cache_DisableCaches();

        if (SD_ReadyToTransfer(pSd, timeout) == false) {
            return TinyCLR_Result::InvalidOperation;
        }

        status = 0;
        to = timeout;

        if ((error = SD_ReadBlock(&sdDrv, sectorNum, 1, sdController[controller].pBufferAligned, timeout)) == SD_ERROR_NO_ERROR) {
            to = timeout;

            while (to > 0 && (((status & AT91C_MCI_DMADONE) != AT91C_MCI_DMADONE) || ((status & AT91C_MCI_XFRDONE) != AT91C_MCI_XFRDONE))) {
                AT91_Time_Delay(nullptr, 1);
                to--;
                status |= READ_MCI(pMciHw, MCI_SR);
            }
        }

        AT91_Cache_EnableCaches();

        if (error) {
            return TinyCLR_Result::InvalidOperation;
        }

        if (!to) {
            return TinyCLR_Result::TimedOut;
        }

        memcpy(pData, sdController[controller].pBufferAligned, AT91_SD_SECTOR_SIZE);

        pData += AT91_SD_SECTOR_SIZE;
        sectorNum++;
        sectorCount--;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_IsSectorErased(const TinyCLR_SdCard_Controller* self, uint64_t sector, bool& erased) {
    erased = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_EraseSector(const TinyCLR_SdCard_Controller* self, uint64_t sector, size_t& count, int32_t timeout) {
    uint32_t addressStart = sector * AT91_SD_SECTOR_SIZE;

    uint32_t addressEnd = addressStart + (count * AT91_SD_SECTOR_SIZE);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_GetSectorMap(const TinyCLR_SdCard_Controller* self, const size_t*& sizes, size_t& count, bool& isUniform) {
    sdController[controller].sectorSizes[0] = AT91_SD_SECTOR_SIZE;

    uint8_t C_SIZE_MULT = 0;

    uint8_t TAAC, NSAC, MAX_RAN_SPEED, READ_BL_LEN, SECTOR_SIZE;

    bool ERASE_BL_EN;

    uint32_t C_SIZE;

    uint64_t MemCapacity = 0; //total memory size, in unit of byte

    uint32_t Max_Trans_Speed = 0;

    TAAC = SD_CSD_TAAC(&sdDrv);
    NSAC = SD_CSD_NSAC(&sdDrv);
    MAX_RAN_SPEED = SD_CSD_TRAN_SPEED(&sdDrv);
    READ_BL_LEN = SD_CSD_READ_BL_LEN(&sdDrv);

    // Checks to see if the SD card is Version 1.0: Standard or Version 2.0: High Capacity
    if (SD_CSD_STRUCTURE(&sdDrv) == 0x00) {
        C_SIZE = SD_CSD_C_SIZE(&sdDrv);

        C_SIZE_MULT = SD_CSD_C_SIZE_MULT(&sdDrv);

        ERASE_BL_EN = (SD_CSD_ERASE_BLK_EN(&sdDrv) == 0x00) ? false : true;

        SECTOR_SIZE = SD_CSD_SECTOR_SIZE(&sdDrv);

        MemCapacity = (C_SIZE + 1) * (0x1 << (C_SIZE_MULT + 2)) * (0x1 << READ_BL_LEN);
    }
    else {
        C_SIZE = SD_CSD_C_SIZE_HC(&sdDrv);

        ERASE_BL_EN = (SD_CSD_ERASE_BLK_EN(&sdDrv) == 0x00) ? false : true;

        SECTOR_SIZE = SD_CSD_SECTOR_SIZE(&sdDrv);

        MemCapacity = (uint64_t)(C_SIZE + 1) * 512 * 1024;
    }

    sizes = sdController[controller].sectorSizes;
    count = MemCapacity / AT91_SD_SECTOR_SIZE;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_SdCard_Reset() {
    return TinyCLR_Result::Success;
}
#endif // INCLUDE_SD