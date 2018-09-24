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

#include <algorithm>
#include <string.h>
#include "AT91.h"

///////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN

#if defined(__GNUC__)
// GCC ARM linker does not link to some variable below if optimize mode.
#pragma GCC optimize 0
#endif

static const uint32_t canDefaultBuffersSize[] = AT91_CAN_BUFFER_DEFAULT_SIZE;

#define CAN_TRANSFER_TIMEOUT 0xFFFFFF

#define CANMB_NUMBER 8
#define CAN_NUM_MAILBOX     8

// CAN mail box
typedef struct {
    uint32_t  CAN_MMR;        /**< \brief (CanMb Offset: 0x0) Mailbox Mode Register */
    uint32_t  CAN_MAM;        /**< \brief (CanMb Offset: 0x4) Mailbox Acceptance Mask Register */
    uint32_t  CAN_MID;        /**< \brief (CanMb Offset: 0x8) Mailbox ID Register */
    uint32_t  CAN_MFID;       /**< \brief (CanMb Offset: 0xC) Mailbox Family ID Register */
    uint32_t  CAN_MSR;        /**< \brief (CanMb Offset: 0x10) Mailbox Status Register */
    uint32_t  CAN_MDL;        /**< \brief (CanMb Offset: 0x14) Mailbox Data Low Register */
    uint32_t  CAN_MDH;        /**< \brief (CanMb Offset: 0x18) Mailbox Data High Register */
    uint32_t  CAN_MCR;        /**< \brief (CanMb Offset: 0x1C) Mailbox Control Register */
} CanMb;

// CAN register

typedef struct {
    uint32_t  CAN_MR;         /**< \brief (Can Offset: 0x0000) Mode Register */
    uint32_t  CAN_IER;        /**< \brief (Can Offset: 0x0004) Interrupt Enable Register */
    uint32_t  CAN_IDR;        /**< \brief (Can Offset: 0x0008) Interrupt Disable Register */
    uint32_t  CAN_IMR;        /**< \brief (Can Offset: 0x000C) Interrupt Mask Register */
    uint32_t  CAN_SR;         /**< \brief (Can Offset: 0x0010) Status Register */
    uint32_t  CAN_BR;         /**< \brief (Can Offset: 0x0014) Baudrate Register */
    uint32_t  CAN_TIM;        /**< \brief (Can Offset: 0x0018) Timer Register */
    uint32_t  CAN_TIMESTP;    /**< \brief (Can Offset: 0x001C) Timestamp Register */
    uint32_t  CAN_ECR;        /**< \brief (Can Offset: 0x0020) Error Counter Register */
    uint32_t  CAN_TCR;        /**< \brief (Can Offset: 0x0024) Transfer Command Register */
    uint32_t  CAN_ACR;        /**< \brief (Can Offset: 0x0028) Abort Command Register */
    uint32_t  Reserved1[117];
    CanMb  CAN_MB[CANMB_NUMBER]; /**< \brief (Can Offset: 0x200) MB = 0 .. 7 */
} Can;

/** CAN Driver Transfer Parameters */
typedef struct _CandTransfer {
    uint32_t dwMsgID;       /**< Message ID _MIDx */
    uint32_t msgData[2];    /**< Message data */
    uint8_t bMailbox;       /**< Mailbox used */
    uint8_t bMsgLen;        /**< Message length */
    uint8_t bState;         /**< Transfer state */
    uint8_t bRC;            /**< Transfer return code */
} sCandTransfer;

// CAN Driver instance struct.
typedef struct _Cand {
    Can* pHw;                   /**< Pointer to HW register base */

    sCandTransfer *pMbs[CAN_NUM_MAILBOX];   /**< Pointer list to mailboxes */

    uint32_t dwMck;             /**< MCK for baudrate calculating */
    uint16_t wBaudrate;         /**< Current working baudrate */

    uint8_t bID;                /**< Peripheral ID */
    uint8_t bState;             /**< CAN states */

} sCand;

/** CAN Driver Mailbox settings */
typedef struct _CandMbCfg {
    uint32_t dwMsgMask;     /**< Message ID Mask _MAMx */
    uint8_t  bMsgType;      /**< Message type */
    uint8_t  bTxPriority;   /**< Priority for TX */
} sCandMbCfg;


#define AT91_CAN0     ((Can *)(0xF8000000U)) /**< \brief (AT91_CAN0    ) Base Address */
#define AT91_CAN1     ((Can *)(0xF8004000U)) /**< \brief (AT91_CAN0    ) Base Address */

/* -------- CAN_MR : (CAN Offset: 0x0000) Mode Register -------- */
#define CAN_MR_CANEN (0x1u << 0) /**< \brief (CAN_MR) CAN Controller Enable */
#define CAN_MR_LPM (0x1u << 1) /**< \brief (CAN_MR) Disable/Enable Low Power Mode */
#define CAN_MR_ABM (0x1u << 2) /**< \brief (CAN_MR) Disable/Enable Autobaud/Listen mode */
#define CAN_MR_OVL (0x1u << 3) /**< \brief (CAN_MR) Disable/Enable Overload Frame */
#define CAN_MR_TEOF (0x1u << 4) /**< \brief (CAN_MR) Timestamp messages at each end of Frame */
#define CAN_MR_TTM (0x1u << 5) /**< \brief (CAN_MR) Disable/Enable Time Triggered Mode */
#define CAN_MR_TIMFRZ (0x1u << 6) /**< \brief (CAN_MR) Enable Timer Freeze */
#define CAN_MR_DRPT (0x1u << 7) /**< \brief (CAN_MR) Disable Repeat */
/* -------- CAN_IER : (CAN Offset: 0x0004) Interrupt Enable Register -------- */
#define CAN_IER_MB0 (0x1u << 0) /**< \brief (CAN_IER) Mailbox 0 Interrupt Enable */
#define CAN_IER_MB1 (0x1u << 1) /**< \brief (CAN_IER) Mailbox 1 Interrupt Enable */
#define CAN_IER_MB2 (0x1u << 2) /**< \brief (CAN_IER) Mailbox 2 Interrupt Enable */
#define CAN_IER_MB3 (0x1u << 3) /**< \brief (CAN_IER) Mailbox 3 Interrupt Enable */
#define CAN_IER_MB4 (0x1u << 4) /**< \brief (CAN_IER) Mailbox 4 Interrupt Enable */
#define CAN_IER_MB5 (0x1u << 5) /**< \brief (CAN_IER) Mailbox 5 Interrupt Enable */
#define CAN_IER_MB6 (0x1u << 6) /**< \brief (CAN_IER) Mailbox 6 Interrupt Enable */
#define CAN_IER_MB7 (0x1u << 7) /**< \brief (CAN_IER) Mailbox 7 Interrupt Enable */
#define CAN_IER_ERRA (0x1u << 16) /**< \brief (CAN_IER) Error Active Mode Interrupt Enable */
#define CAN_IER_WARN (0x1u << 17) /**< \brief (CAN_IER) Warning Limit Interrupt Enable */
#define CAN_IER_ERRP (0x1u << 18) /**< \brief (CAN_IER) Error Passive Mode Interrupt Enable */
#define CAN_IER_BOFF (0x1u << 19) /**< \brief (CAN_IER) Bus Off Mode Interrupt Enable */
#define CAN_IER_SLEEP (0x1u << 20) /**< \brief (CAN_IER) Sleep Interrupt Enable */
#define CAN_IER_WAKEUP (0x1u << 21) /**< \brief (CAN_IER) Wakeup Interrupt Enable */
#define CAN_IER_TOVF (0x1u << 22) /**< \brief (CAN_IER) Timer Overflow Interrupt Enable */
#define CAN_IER_TSTP (0x1u << 23) /**< \brief (CAN_IER) TimeStamp Interrupt Enable */
#define CAN_IER_CERR (0x1u << 24) /**< \brief (CAN_IER) CRC Error Interrupt Enable */
#define CAN_IER_SERR (0x1u << 25) /**< \brief (CAN_IER) Stuffing Error Interrupt Enable */
#define CAN_IER_AERR (0x1u << 26) /**< \brief (CAN_IER) Acknowledgment Error Interrupt Enable */
#define CAN_IER_FERR (0x1u << 27) /**< \brief (CAN_IER) Form Error Interrupt Enable */
#define CAN_IER_BERR (0x1u << 28) /**< \brief (CAN_IER) Bit Error Interrupt Enable */
/* -------- CAN_IDR : (CAN Offset: 0x0008) Interrupt Disable Register -------- */
#define CAN_IDR_MB0 (0x1u << 0) /**< \brief (CAN_IDR) Mailbox 0 Interrupt Disable */
#define CAN_IDR_MB1 (0x1u << 1) /**< \brief (CAN_IDR) Mailbox 1 Interrupt Disable */
#define CAN_IDR_MB2 (0x1u << 2) /**< \brief (CAN_IDR) Mailbox 2 Interrupt Disable */
#define CAN_IDR_MB3 (0x1u << 3) /**< \brief (CAN_IDR) Mailbox 3 Interrupt Disable */
#define CAN_IDR_MB4 (0x1u << 4) /**< \brief (CAN_IDR) Mailbox 4 Interrupt Disable */
#define CAN_IDR_MB5 (0x1u << 5) /**< \brief (CAN_IDR) Mailbox 5 Interrupt Disable */
#define CAN_IDR_MB6 (0x1u << 6) /**< \brief (CAN_IDR) Mailbox 6 Interrupt Disable */
#define CAN_IDR_MB7 (0x1u << 7) /**< \brief (CAN_IDR) Mailbox 7 Interrupt Disable */
#define CAN_IDR_ERRA (0x1u << 16) /**< \brief (CAN_IDR) Error Active Mode Interrupt Disable */
#define CAN_IDR_WARN (0x1u << 17) /**< \brief (CAN_IDR) Warning Limit Interrupt Disable */
#define CAN_IDR_ERRP (0x1u << 18) /**< \brief (CAN_IDR) Error Passive Mode Interrupt Disable */
#define CAN_IDR_BOFF (0x1u << 19) /**< \brief (CAN_IDR) Bus Off Mode Interrupt Disable */
#define CAN_IDR_SLEEP (0x1u << 20) /**< \brief (CAN_IDR) Sleep Interrupt Disable */
#define CAN_IDR_WAKEUP (0x1u << 21) /**< \brief (CAN_IDR) Wakeup Interrupt Disable */
#define CAN_IDR_TOVF (0x1u << 22) /**< \brief (CAN_IDR) Timer Overflow Interrupt */
#define CAN_IDR_TSTP (0x1u << 23) /**< \brief (CAN_IDR) TimeStamp Interrupt Disable */
#define CAN_IDR_CERR (0x1u << 24) /**< \brief (CAN_IDR) CRC Error Interrupt Disable */
#define CAN_IDR_SERR (0x1u << 25) /**< \brief (CAN_IDR) Stuffing Error Interrupt Disable */
#define CAN_IDR_AERR (0x1u << 26) /**< \brief (CAN_IDR) Acknowledgment Error Interrupt Disable */
#define CAN_IDR_FERR (0x1u << 27) /**< \brief (CAN_IDR) Form Error Interrupt Disable */
#define CAN_IDR_BERR (0x1u << 28) /**< \brief (CAN_IDR) Bit Error Interrupt Disable */
/* -------- CAN_IMR : (CAN Offset: 0x000C) Interrupt Mask Register -------- */
#define CAN_IMR_MB0 (0x1u << 0) /**< \brief (CAN_IMR) Mailbox 0 Interrupt Mask */
#define CAN_IMR_MB1 (0x1u << 1) /**< \brief (CAN_IMR) Mailbox 1 Interrupt Mask */
#define CAN_IMR_MB2 (0x1u << 2) /**< \brief (CAN_IMR) Mailbox 2 Interrupt Mask */
#define CAN_IMR_MB3 (0x1u << 3) /**< \brief (CAN_IMR) Mailbox 3 Interrupt Mask */
#define CAN_IMR_MB4 (0x1u << 4) /**< \brief (CAN_IMR) Mailbox 4 Interrupt Mask */
#define CAN_IMR_MB5 (0x1u << 5) /**< \brief (CAN_IMR) Mailbox 5 Interrupt Mask */
#define CAN_IMR_MB6 (0x1u << 6) /**< \brief (CAN_IMR) Mailbox 6 Interrupt Mask */
#define CAN_IMR_MB7 (0x1u << 7) /**< \brief (CAN_IMR) Mailbox 7 Interrupt Mask */
#define CAN_IMR_ERRA (0x1u << 16) /**< \brief (CAN_IMR) Error Active Mode Interrupt Mask */
#define CAN_IMR_WARN (0x1u << 17) /**< \brief (CAN_IMR) Warning Limit Interrupt Mask */
#define CAN_IMR_ERRP (0x1u << 18) /**< \brief (CAN_IMR) Error Passive Mode Interrupt Mask */
#define CAN_IMR_BOFF (0x1u << 19) /**< \brief (CAN_IMR) Bus Off Mode Interrupt Mask */
#define CAN_IMR_SLEEP (0x1u << 20) /**< \brief (CAN_IMR) Sleep Interrupt Mask */
#define CAN_IMR_WAKEUP (0x1u << 21) /**< \brief (CAN_IMR) Wakeup Interrupt Mask */
#define CAN_IMR_TOVF (0x1u << 22) /**< \brief (CAN_IMR) Timer Overflow Interrupt Mask */
#define CAN_IMR_TSTP (0x1u << 23) /**< \brief (CAN_IMR) Timestamp Interrupt Mask */
#define CAN_IMR_CERR (0x1u << 24) /**< \brief (CAN_IMR) CRC Error Interrupt Mask */
#define CAN_IMR_SERR (0x1u << 25) /**< \brief (CAN_IMR) Stuffing Error Interrupt Mask */
#define CAN_IMR_AERR (0x1u << 26) /**< \brief (CAN_IMR) Acknowledgment Error Interrupt Mask */
#define CAN_IMR_FERR (0x1u << 27) /**< \brief (CAN_IMR) Form Error Interrupt Mask */
#define CAN_IMR_BERR (0x1u << 28) /**< \brief (CAN_IMR) Bit Error Interrupt Mask */
/* -------- CAN_SR : (CAN Offset: 0x0010) Status Register -------- */
#define CAN_SR_MB0 (0x1u << 0) /**< \brief (CAN_SR) Mailbox 0 Event */
#define CAN_SR_MB1 (0x1u << 1) /**< \brief (CAN_SR) Mailbox 1 Event */
#define CAN_SR_MB2 (0x1u << 2) /**< \brief (CAN_SR) Mailbox 2 Event */
#define CAN_SR_MB3 (0x1u << 3) /**< \brief (CAN_SR) Mailbox 3 Event */
#define CAN_SR_MB4 (0x1u << 4) /**< \brief (CAN_SR) Mailbox 4 Event */
#define CAN_SR_MB5 (0x1u << 5) /**< \brief (CAN_SR) Mailbox 5 Event */
#define CAN_SR_MB6 (0x1u << 6) /**< \brief (CAN_SR) Mailbox 6 Event */
#define CAN_SR_MB7 (0x1u << 7) /**< \brief (CAN_SR) Mailbox 7 Event */
#define CAN_SR_ERRA (0x1u << 16) /**< \brief (CAN_SR) Error Active Mode */
#define CAN_SR_WARN (0x1u << 17) /**< \brief (CAN_SR) Warning Limit */
#define CAN_SR_ERRP (0x1u << 18) /**< \brief (CAN_SR) Error Passive Mode */
#define CAN_SR_BOFF (0x1u << 19) /**< \brief (CAN_SR) Bus Off Mode */
#define CAN_SR_SLEEP (0x1u << 20) /**< \brief (CAN_SR) CAN controllerIndex in Low power Mode */
#define CAN_SR_WAKEUP (0x1u << 21) /**< \brief (CAN_SR) CAN controllerIndex is not in Low power Mode */
#define CAN_SR_TOVF (0x1u << 22) /**< \brief (CAN_SR) Timer Overflow */
#define CAN_SR_TSTP (0x1u << 23) /**< \brief (CAN_SR)  */
#define CAN_SR_CERR (0x1u << 24) /**< \brief (CAN_SR) Mailbox CRC Error */
#define CAN_SR_SERR (0x1u << 25) /**< \brief (CAN_SR) Mailbox Stuffing Error */
#define CAN_SR_AERR (0x1u << 26) /**< \brief (CAN_SR) Acknowledgment Error */
#define CAN_SR_FERR (0x1u << 27) /**< \brief (CAN_SR) Form Error */
#define CAN_SR_BERR (0x1u << 28) /**< \brief (CAN_SR) Bit Error */
#define CAN_SR_RBSY (0x1u << 29) /**< \brief (CAN_SR) Receiver busy */
#define CAN_SR_TBSY (0x1u << 30) /**< \brief (CAN_SR) Transmitter busy */
#define CAN_SR_OVLSY (0x1u << 31) /**< \brief (CAN_SR) Overload busy */
/* -------- CAN_BR : (CAN Offset: 0x0014) Baudrate Register -------- */
#define CAN_BR_PHASE2_Pos 0
#define CAN_BR_PHASE2_Msk (0x7u << CAN_BR_PHASE2_Pos) /**< \brief (CAN_BR) Phase 2 segment */
#define CAN_BR_PHASE2(value) ((CAN_BR_PHASE2_Msk & ((value) << CAN_BR_PHASE2_Pos)))
#define CAN_BR_PHASE1_Pos 4
#define CAN_BR_PHASE1_Msk (0x7u << CAN_BR_PHASE1_Pos) /**< \brief (CAN_BR) Phase 1 segment */
#define CAN_BR_PHASE1(value) ((CAN_BR_PHASE1_Msk & ((value) << CAN_BR_PHASE1_Pos)))
#define CAN_BR_PROPAG_Pos 8
#define CAN_BR_PROPAG_Msk (0x7u << CAN_BR_PROPAG_Pos) /**< \brief (CAN_BR) Programming time segment */
#define CAN_BR_PROPAG(value) ((CAN_BR_PROPAG_Msk & ((value) << CAN_BR_PROPAG_Pos)))
#define CAN_BR_SJW_Pos 12
#define CAN_BR_SJW_Msk (0x3u << CAN_BR_SJW_Pos) /**< \brief (CAN_BR) Re-synchronization jump width */
#define CAN_BR_SJW(value) ((CAN_BR_SJW_Msk & ((value) << CAN_BR_SJW_Pos)))
#define CAN_BR_BRP_Pos 16
#define CAN_BR_BRP_Msk (0x7fu << CAN_BR_BRP_Pos) /**< \brief (CAN_BR) Baudrate Prescaler. */
#define CAN_BR_BRP(value) ((CAN_BR_BRP_Msk & ((value) << CAN_BR_BRP_Pos)))
#define CAN_BR_SMP (0x1u << 24) /**< \brief (CAN_BR) Sampling Mode */
#define   CAN_BR_SMP_ONCE (0x0u << 24) /**< \brief (CAN_BR) The incoming bit stream is sampled once at sample point. */
#define   CAN_BR_SMP_THREE (0x1u << 24) /**< \brief (CAN_BR) The incoming bit stream is sampled three times with a period of a MCK clock period, centered on sample point. */
/* -------- CAN_TIM : (CAN Offset: 0x0018) Timer Register -------- */
#define CAN_TIM_TIMER0 (0x1u << 0) /**< \brief (CAN_TIM) Timer */
#define CAN_TIM_TIMER1 (0x1u << 1) /**< \brief (CAN_TIM) Timer */
#define CAN_TIM_TIMER2 (0x1u << 2) /**< \brief (CAN_TIM) Timer */
#define CAN_TIM_TIMER3 (0x1u << 3) /**< \brief (CAN_TIM) Timer */
#define CAN_TIM_TIMER4 (0x1u << 4) /**< \brief (CAN_TIM) Timer */
#define CAN_TIM_TIMER5 (0x1u << 5) /**< \brief (CAN_TIM) Timer */
#define CAN_TIM_TIMER6 (0x1u << 6) /**< \brief (CAN_TIM) Timer */
#define CAN_TIM_TIMER7 (0x1u << 7) /**< \brief (CAN_TIM) Timer */
/* -------- CAN_TIMESTP : (CAN Offset: 0x001C) Timestamp Register -------- */
#define CAN_TIMESTP_MTIMESTAMP0 (0x1u << 0) /**< \brief (CAN_TIMESTP) Timestamp */
#define CAN_TIMESTP_MTIMESTAMP1 (0x1u << 1) /**< \brief (CAN_TIMESTP) Timestamp */
#define CAN_TIMESTP_MTIMESTAMP2 (0x1u << 2) /**< \brief (CAN_TIMESTP) Timestamp */
#define CAN_TIMESTP_MTIMESTAMP3 (0x1u << 3) /**< \brief (CAN_TIMESTP) Timestamp */
#define CAN_TIMESTP_MTIMESTAMP4 (0x1u << 4) /**< \brief (CAN_TIMESTP) Timestamp */
#define CAN_TIMESTP_MTIMESTAMP5 (0x1u << 5) /**< \brief (CAN_TIMESTP) Timestamp */
#define CAN_TIMESTP_MTIMESTAMP6 (0x1u << 6) /**< \brief (CAN_TIMESTP) Timestamp */
#define CAN_TIMESTP_MTIMESTAMP7 (0x1u << 7) /**< \brief (CAN_TIMESTP) Timestamp */
/* -------- CAN_ECR : (CAN Offset: 0x0020) Error Counter Register -------- */
#define CAN_ECR_REC_Pos 0
#define CAN_ECR_REC_Msk (0xffu << CAN_ECR_REC_Pos) /**< \brief (CAN_ECR) Receive Error Counter */
#define CAN_ECR_TEC_Pos 16
#define CAN_ECR_TEC_Msk (0x1ffu << CAN_ECR_TEC_Pos) /**< \brief (CAN_ECR) Transmit Error Counter */
/* -------- CAN_TCR : (CAN Offset: 0x0024) Transfer Command Register -------- */
#define CAN_TCR_MB0 (0x1u << 0) /**< \brief (CAN_TCR) Transfer Request for Mailbox 0 */
#define CAN_TCR_MB1 (0x1u << 1) /**< \brief (CAN_TCR) Transfer Request for Mailbox 1 */
#define CAN_TCR_MB2 (0x1u << 2) /**< \brief (CAN_TCR) Transfer Request for Mailbox 2 */
#define CAN_TCR_MB3 (0x1u << 3) /**< \brief (CAN_TCR) Transfer Request for Mailbox 3 */
#define CAN_TCR_MB4 (0x1u << 4) /**< \brief (CAN_TCR) Transfer Request for Mailbox 4 */
#define CAN_TCR_MB5 (0x1u << 5) /**< \brief (CAN_TCR) Transfer Request for Mailbox 5 */
#define CAN_TCR_MB6 (0x1u << 6) /**< \brief (CAN_TCR) Transfer Request for Mailbox 6 */
#define CAN_TCR_MB7 (0x1u << 7) /**< \brief (CAN_TCR) Transfer Request for Mailbox 7 */
#define CAN_TCR_TIMRST (0x1u << 31) /**< \brief (CAN_TCR) Timer Reset */
/* -------- CAN_ACR : (CAN Offset: 0x0028) Abort Command Register -------- */
#define CAN_ACR_MB0 (0x1u << 0) /**< \brief (CAN_ACR) Abort Request for Mailbox 0 */
#define CAN_ACR_MB1 (0x1u << 1) /**< \brief (CAN_ACR) Abort Request for Mailbox 1 */
#define CAN_ACR_MB2 (0x1u << 2) /**< \brief (CAN_ACR) Abort Request for Mailbox 2 */
#define CAN_ACR_MB3 (0x1u << 3) /**< \brief (CAN_ACR) Abort Request for Mailbox 3 */
#define CAN_ACR_MB4 (0x1u << 4) /**< \brief (CAN_ACR) Abort Request for Mailbox 4 */
#define CAN_ACR_MB5 (0x1u << 5) /**< \brief (CAN_ACR) Abort Request for Mailbox 5 */
#define CAN_ACR_MB6 (0x1u << 6) /**< \brief (CAN_ACR) Abort Request for Mailbox 6 */
#define CAN_ACR_MB7 (0x1u << 7) /**< \brief (CAN_ACR) Abort Request for Mailbox 7 */
/* -------- CAN_MMR : (CAN Offset: N/A) Mailbox Mode Register -------- */
#define CAN_MMR_MTIMEMARK0 (0x1u << 0) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK1 (0x1u << 1) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK2 (0x1u << 2) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK3 (0x1u << 3) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK4 (0x1u << 4) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK5 (0x1u << 5) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK6 (0x1u << 6) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK7 (0x1u << 7) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_PRIOR_Pos 16
#define CAN_MMR_PRIOR_Msk (0xfu << CAN_MMR_PRIOR_Pos) /**< \brief (CAN_MMR) Mailbox Priority */
#define CAN_MMR_PRIOR(value) ((CAN_MMR_PRIOR_Msk & ((value) << CAN_MMR_PRIOR_Pos)))
#define CAN_MMR_MOT_Pos 24
#define CAN_MMR_MOT_Msk (0x7u << CAN_MMR_MOT_Pos) /**< \brief (CAN_MMR) Mailbox Object Type */
#define   CAN_MMR_MOT_MB_DISABLED (0x0u << 24) /**< \brief (CAN_MMR) Mailbox is disabled. This prevents receiving or transmitting any messages with this mailbox. */
#define   CAN_MMR_MOT_MB_RX (0x1u << 24) /**< \brief (CAN_MMR) Reception Mailbox. Mailbox is configured for reception. If a message is received while the mailbox data register is full, it is discarded. */
#define   CAN_MMR_MOT_MB_RX_OVERWRITE (0x2u << 24) /**< \brief (CAN_MMR) Reception mailbox with overwrite. Mailbox is configured for reception. If a message is received while the mailbox is full, it overwrites the previous message. */
#define   CAN_MMR_MOT_MB_TX (0x3u << 24) /**< \brief (CAN_MMR) Transmit mailbox. Mailbox is configured for transmission. */
#define   CAN_MMR_MOT_MB_CONSUMER (0x4u << 24) /**< \brief (CAN_MMR) Consumer Mailbox. Mailbox is configured in reception but behaves as a Transmit Mailbox, i.e., it sends a remote frame and waits for an answer. */
#define   CAN_MMR_MOT_MB_PRODUCER (0x5u << 24) /**< \brief (CAN_MMR) Producer Mailbox. Mailbox is configured in transmission but also behaves like a reception mailbox, i.e., it waits to receive a Remote Frame before sending its contents. */
/* -------- CAN_MAM : (CAN Offset: N/A) Mailbox Acceptance Mask Register -------- */
#define CAN_MAM_MIDvB_Pos 0
#define CAN_MAM_MIDvB_Msk (0x3ffffu << CAN_MAM_MIDvB_Pos) /**< \brief (CAN_MAM) Complementary bits for identifier in extended frame mode */
#define CAN_MAM_MIDvB(value) ((CAN_MAM_MIDvB_Msk & ((value) << CAN_MAM_MIDvB_Pos)))
#define CAN_MAM_MIDvA_Pos 18
#define CAN_MAM_MIDvA_Msk (0x7ffu << CAN_MAM_MIDvA_Pos) /**< \brief (CAN_MAM) Identifier for standard frame mode */
#define CAN_MAM_MIDvA(value) ((CAN_MAM_MIDvA_Msk & ((value) << CAN_MAM_MIDvA_Pos)))
#define CAN_MAM_MIDE (0x1u << 29) /**< \brief (CAN_MAM) Identifier Version */
/* -------- CAN_MID : (CAN Offset: N/A) Mailbox ID Register -------- */
#define CAN_MID_MIDvB_Pos 0
#define CAN_MID_MIDvB_Msk (0x3ffffu << CAN_MID_MIDvB_Pos) /**< \brief (CAN_MID) Complementary bits for identifier in extended frame mode */
#define CAN_MID_MIDvB(value) ((CAN_MID_MIDvB_Msk & ((value) << CAN_MID_MIDvB_Pos)))
#define CAN_MID_MIDvA_Pos 18
#define CAN_MID_MIDvA_Msk (0x7ffu << CAN_MID_MIDvA_Pos) /**< \brief (CAN_MID) Identifier for standard frame mode */
#define CAN_MID_MIDvA(value) ((CAN_MID_MIDvA_Msk & ((value) << CAN_MID_MIDvA_Pos)))
#define CAN_MID_MIDE (0x1u << 29) /**< \brief (CAN_MID) Identifier Version */
/* -------- CAN_MFID : (CAN Offset: N/A) Mailbox Family ID Register -------- */
#define CAN_MFID_MFID_Pos 0
#define CAN_MFID_MFID_Msk (0x1fffffffu << CAN_MFID_MFID_Pos) /**< \brief (CAN_MFID) Family ID */
/* -------- CAN_MSR : (CAN Offset: N/A) Mailbox Status Register -------- */
#define CAN_MSR_MTIMESTAMP0 (0x1u << 0) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MTIMESTAMP1 (0x1u << 1) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MTIMESTAMP2 (0x1u << 2) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MTIMESTAMP3 (0x1u << 3) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MTIMESTAMP4 (0x1u << 4) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MTIMESTAMP5 (0x1u << 5) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MTIMESTAMP6 (0x1u << 6) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MTIMESTAMP7 (0x1u << 7) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MDLC_Pos 16
#define CAN_MSR_MDLC_Msk (0xfu << CAN_MSR_MDLC_Pos) /**< \brief (CAN_MSR) Mailbox Data Length Code */
#define CAN_MSR_MRTR (0x1u << 20) /**< \brief (CAN_MSR) Mailbox Remote Transmission Request */
#define CAN_MSR_MABT (0x1u << 22) /**< \brief (CAN_MSR) Mailbox Message Abort */
#define CAN_MSR_MRDY (0x1u << 23) /**< \brief (CAN_MSR) Mailbox Ready */
#define CAN_MSR_MMI (0x1u << 24) /**< \brief (CAN_MSR) Mailbox Message Ignored */
/* -------- CAN_MDL : (CAN Offset: N/A) Mailbox Data Low Register -------- */
#define CAN_MDL_MDL_Pos 0
#define CAN_MDL_MDL_Msk (0xffffffffu << CAN_MDL_MDL_Pos) /**< \brief (CAN_MDL) Message Data Low Value */
#define CAN_MDL_MDL(value) ((CAN_MDL_MDL_Msk & ((value) << CAN_MDL_MDL_Pos)))
/* -------- CAN_MDH : (CAN Offset: N/A) Mailbox Data High Register -------- */
#define CAN_MDH_MDH_Pos 0
#define CAN_MDH_MDH_Msk (0xffffffffu << CAN_MDH_MDH_Pos) /**< \brief (CAN_MDH) Message Data High Value */
#define CAN_MDH_MDH(value) ((CAN_MDH_MDH_Msk & ((value) << CAN_MDH_MDH_Pos)))
/* -------- CAN_MCR : (CAN Offset: N/A) Mailbox Control Register -------- */
#define CAN_MCR_MDLC_Pos 16
#define CAN_MCR_MDLC_Msk (0xfu << CAN_MCR_MDLC_Pos) /**< \brief (CAN_MCR) Mailbox Data Length Code */
#define CAN_MCR_MDLC(value) ((CAN_MCR_MDLC_Msk & ((value) << CAN_MCR_MDLC_Pos)))
#define CAN_MCR_MRTR (0x1u << 20) /**< \brief (CAN_MCR) Mailbox Remote Transmission Request */
#define CAN_MCR_MACR (0x1u << 22) /**< \brief (CAN_MCR) Abort Request for Mailbox x */
#define CAN_MCR_MTCR (0x1u << 23) /**< \brief (CAN_MCR) Mailbox Transfer Command */


#define CAND_STATE_DISABLED     0 /**< Power-up reset, controllerIndex is disabled */
#define CAND_STATE_INIT         1 /**< Initializing */
#define CAND_STATE_SLEEP        2 /**< Low-power mode */
#define CAND_STATE_SYNC         3 /**< Synchronizating */
#define CAND_STATE_ERROR        4 /**< Error halt */
#define CAND_STATE_ACTIVATED    5 /**< Bus synchronization is done */
#define CAND_STATE_XFR          6 /**< Transfer in progress */

#define CAND_XFR_DISABLED       0 /**< Transfer not used */
#define CAND_XFR_HALTED         1 /**< Error halt */
#define CAND_XFR_IDLE           2 /**< No transfer */
#define CAND_XFR_TX             3 /**< Transferring data */

/** CAN mailbox event statuses bits */
#define CAN_MB_EVENTS       0xFF
/** CAN errors statuses bits */
#define CAN_ERRS    (0 \
                    /*|CAN_SR_ERRA*/ \
                    /*|CAN_SR_WARN*/ \
                    |CAN_SR_ERRP \
                    |CAN_SR_BOFF \
                    /*|CAN_SR_SLEEP*/ \
                    /*|CAN_SR_WAKEUP*/ \
                    /*|CAN_SR_TOVF*/ \
                    /*|CAN_SR_TSTP*/ \
                    |CAN_SR_CERR \
                    |CAN_SR_SERR \
                    |CAN_SR_AERR \
                    |CAN_SR_FERR \
                    |CAN_SR_BERR \
                    /*|CAN_SR_RBSY*/ \
                    /*|CAN_SR_TBSY*/ \
                    /*|CAN_SR_OVLSY*/ \
                    )
/** CAN mailbox ID mask */
#define CAN_ID_MASK (CAN_MID_MIDE | CAN_MID_MIDvA_Msk | CAN_MID_MIDvB_Msk)

#define CAN_MMR_MOT(x) (((x)<<CAN_MMR_MOT_Pos)&CAN_MMR_MOT_Msk)

/** Operation success */
#define CAND_OK             0
/** The state/mailbox is busy */
#define CAND_BUSY           1
/** General error */
#define CAND_ERROR          0x10
/** Bad operation because of wrong state */
#define CAND_ERR_STATE      0x11
/** Bad operation for parameter error */
#define CAND_ERR_PARAM      0xFE

#define MAILBOX_TO_SEND_INDEX 0
#define MAILBOX_TO_RECEIVE_INDEX 1
#define MAILBOX_TO_RECEIVE_RTC_INDEX 2

/* ----------- CAN_MR Operations --------------- */
/**
 * \brief Set CAN Mode Register (CAN_MR)
 * \param pCan Pointer to Can instance.
 * \param dwMr Mode register settings.
 */
void CAN_ConfigureMode(Can *pCan, uint32_t dwMr) {
    pCan->CAN_MR = dwMr;
}

/**
 * \brief CAN Controller Enable/Disable
 * \param pCan   Pointer to Can instance.
 * \param bEnDis 1 to enable and 0 to disable.
 */
void CAN_Enable(Can *pCan, uint8_t bEnDis) {
    if (bEnDis) pCan->CAN_MR |= CAN_MR_CANEN;
    else        pCan->CAN_MR &= ~CAN_MR_CANEN;
}

/**
 * \brief CAN Low Power Mode Enable/Disable
 * \param pCan   Pointer to Can instance.
 * \param bEnDis 1 to enable and 0 to disable.
 */
void CAN_EnableLowPower(Can *pCan, uint8_t bEnDis) {
    if (bEnDis) pCan->CAN_MR |= CAN_MR_LPM;
    else        pCan->CAN_MR &= ~CAN_MR_LPM;
}

/* ---------- Interrupt settings ------------- */

/**
 * \brief CAN Interrupts Enable
 * \param pCan      Pointer to Can instance.
 * \param dwSources Interrupt sources bits.
 */
void CAN_EnableIt(Can *pCan, uint32_t dwSources) {
    pCan->CAN_IER = dwSources;
}

/**
 * \brief CAN Interrupts Disable
 * \param pCan      Pointer to Can instance.
 * \param dwSources Interrupt sources bits.
 */
void CAN_DisableIt(Can *pCan, uint32_t dwSources) {
    pCan->CAN_IDR = dwSources;
}

/**
 * \brief Return CAN Interrupts Masks
 * \param pCan      Pointer to Can instance.
 */
uint32_t CAN_GetItMask(Can *pCan) {
    return pCan->CAN_IMR;
}

/**
 * \brief Return CAN Statuses
 * \param pCan      Pointer to Can instance.
 */
uint32_t CAN_GetStatus(Can *pCan) {
    return pCan->CAN_SR;
}

/**
 * \brief Set CAN baudrate register
 * \param pCan      Pointer to Can instance.
 * \param dwBr      Setting value for CAN_BR.
 */
void CAN_ConfigureBaudrate(Can *pCan, uint32_t dwBr) {
    pCan->CAN_BR = dwBr;
}

/**
 * \brief Return Receive Error Count
 * \param pCan      Pointer to Can instance.
 */
uint32_t CAN_GetRxErrorCount(Can *pCan) {
    return (pCan->CAN_ECR & CAN_ECR_REC_Msk) >> CAN_ECR_REC_Pos;
}

/**
 * \brief Return Transmit Error Count
 * \param pCan      Pointer to Can instance.
 */
uint32_t CAN_GetTxErrorCount(Can *pCan) {
    return (pCan->CAN_ECR & CAN_ECR_TEC_Msk) >> CAN_ECR_TEC_Pos;
}

/**
 * \brief Set Transfer Command Register to initialize transfer requests.
 * \param pCan       Pointer to Can instance.
 * \param dwRequests Transfer Command Requests.
 */
void CAN_Command(Can *pCan, uint32_t dwRequests) {
    pCan->CAN_TCR = dwRequests;
}

/**
 * \brief Configure CAN Message Mode (_MMRx)
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 * \param dwMr       Mode settings.
 */
void CAN_ConfigureMessageMode(Can *pCan, uint8_t bMb, uint32_t dwMr) {
    pCan->CAN_MB[bMb].CAN_MMR = dwMr;
}

/**
 * \brief Return CAN Message Mode (_MMRx)
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 */
uint32_t CAN_GetMessageMode(Can *pCan, uint8_t bMb) {
    return pCan->CAN_MB[bMb].CAN_MMR;
}

/**
 * \brief Set Mailbox Priority.
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 * \param bPriority  Mailbox Priority.
 */
void CAN_SetPriority(Can *pCan, uint8_t bMb, uint8_t bPriority) {
    uint32_t dwMmr = (pCan->CAN_MB[bMb].CAN_MMR & ~CAN_MMR_PRIOR_Msk);
    pCan->CAN_MB[bMb].CAN_MMR = dwMmr | CAN_MMR_PRIOR(bPriority);
}

/**
 * \brief Configure CAN Message Acceptance Mask (_MAMx)
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 * \param dwMam      The setting value for _MAMx.
 */
void CAN_ConfigureMessageAcceptanceMask(Can *pCan, uint8_t bMb, uint32_t dwMAM) {
    pCan->CAN_MB[bMb].CAN_MAM = dwMAM;
}

/**
 * \brief Configure CAN Message ID (_MIDx)
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 * \param dwMID      The setting value for _MIDx.
 */
void CAN_ConfigureMessageID(Can *pCan, uint8_t bMb, uint32_t dwMID) {
    pCan->CAN_MB[bMb].CAN_MID = dwMID;
}

/**
 * \brief Return CAN Message ID (_MIDx)
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 */
uint32_t CAN_GetMessageID(Can *pCan, uint8_t bMb) {
    return pCan->CAN_MB[bMb].CAN_MID;
}

/**
 * \brief Return CAN Message Status
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 */
uint32_t CAN_GetMessageStatus(Can *pCan, uint8_t bMb) {
    return pCan->CAN_MB[bMb].CAN_MSR;
}

/**
 * \brief Copy DW array to CAN Message Data.
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 * \param pDwData    Pointer to a buffer for data.
 */
void CAN_SetMessage(Can *pCan, uint8_t bMb, uint32_t *pDwData) {
    pCan->CAN_MB[bMb].CAN_MDL = pDwData[0];
    pCan->CAN_MB[bMb].CAN_MDH = pDwData[1];
}

/**
 * \brief Copy CAN Message Data to DW array.
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 * \param pDwData    Pointer to a buffer for data.
 */
void CAN_GetMessage(Can *pCan, uint8_t bMb, uint32_t *pDwData) {
    pDwData[0] = pCan->CAN_MB[bMb].CAN_MDL;
    pDwData[1] = pCan->CAN_MB[bMb].CAN_MDH;
}

/**
 * \brief Set CAN Message Control Register (_MCRx).
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 * \param dwCtrl     Control value.
 */
void CAN_MessageControl(Can *pCan, uint8_t bMb, uint32_t dwCtrl) {
    pCan->CAN_MB[bMb].CAN_MCR = dwCtrl;
}

/**
 * \brief Start remote frame.
 * \param pCan       Pointer to Can instance.
 * \param bMb        Mailbox number.
 */
void CAN_MessageRemote(Can *pCan, uint8_t bMb) {
    pCan->CAN_MB[bMb].CAN_MCR = CAN_MCR_MRTR;
}

/**
 * Check if the mailbox is ready to transfer
 * \param pMb The current mailbox transfer parameters.
 */
uint8_t CAND_IsMbReady(sCandTransfer *pMb) {
    /* MB has no transfer, OK */
    if (pMb == NULL) return 1;
    /* MB has transfer, NOK */
    if (pMb->bState == CAND_XFR_TX) return 0;
    /* MB in a state that acceps modification */
    return 1;
}

/**
 * Reset mailbox with specified configuration value.
 * \param pCand Pointer to CAN Driver instance.
 * \param pCfg  Pointer to list of configurations.
 */
void CAND_ResetMb(sCand *pCand, uint8_t bMb, sCandMbCfg *pCfg) {
    Can *pCan = pCand->pHw;
    CAN_DisableIt(pCan, (1 << bMb) & CAN_MB_EVENTS);
    CAN_MessageControl(pCan, bMb, 0);
    CAN_ConfigureMessageMode(pCan, bMb, 0);
    if (pCand->pMbs[bMb]) {
        pCand->pMbs[bMb] = CAND_XFR_DISABLED;
        pCand->pMbs[bMb] = NULL;
    }
    if (pCfg) {
        CAN_ConfigureMessageAcceptanceMask(pCan, bMb, pCfg->dwMsgMask);
        CAN_ConfigureMessageMode(pCan, bMb, CAN_MMR_MOT(pCfg->bMsgType)
            | CAN_MMR_PRIOR(pCfg->bTxPriority));
    }
}

/**
 * Initialize transfer on specific Mailbox.
 * \param pCand  Pointer to CAN Driver instance.
 * \param pXfr   Pointer to CAN Transfer instance.
 * \param bStart Start transfer immediately.
 */
void CAND_InitXfr(sCand *pCand, sCandTransfer *pXfr, uint8_t bStart) {
    Can *pCan = pCand->pHw;
    uint8_t bMb = pXfr->bMailbox;
    uint32_t dwMmr = CAN_GetMessageMode(pCan, bMb);

    if (pXfr == NULL)
        return;
    /* Log tranfser */
    pCand->pMbs[bMb] = pXfr;
    /* Set transfer state */
    if (bStart) {
        pXfr->bState = CAND_XFR_TX;
        pCand->bState = CAND_STATE_XFR;
    }
    /* Reset transfer state */
    else
        pXfr->bState = CAND_XFR_IDLE;
    /* Fill ID */
    CAN_ConfigureMessageID(pCan, bMb, pXfr->dwMsgID);
    /* Fill data registers */
    CAN_SetMessage(pCan, bMb, pXfr->msgData);
    /* Start TX if not RX */
    if ((dwMmr & CAN_MMR_MOT_Msk) > CAN_MMR_MOT_MB_RX_OVERWRITE) {
        CAN_MessageControl(pCan, bMb,
            CAN_MCR_MDLC(pXfr->bMsgLen)
            | (bStart ? CAN_MCR_MTCR : 0));
    }
}

void CAND_InitXfr_RTRMode(sCand *pCand, sCandTransfer *pXfr, uint8_t bStart) {
    Can *pCan = pCand->pHw;
    uint8_t bMb = pXfr->bMailbox;
    uint32_t dwMmr = CAN_GetMessageMode(pCan, bMb);

    if (pXfr == NULL)
        return;
    /* Log tranfser */
    pCand->pMbs[bMb] = pXfr;
    /* Set transfer state */
    if (bStart) {
        pXfr->bState = CAND_XFR_TX;
        pCand->bState = CAND_STATE_XFR;
    }
    /* Reset transfer state */
    else
        pXfr->bState = CAND_XFR_IDLE;
    /* Fill ID */
    CAN_ConfigureMessageID(pCan, bMb, pXfr->dwMsgID);
    /* Fill data registers */
    CAN_SetMessage(pCan, bMb, pXfr->msgData);
    /* Start TX if not RX */
    if ((dwMmr & CAN_MMR_MOT_Msk) > CAN_MMR_MOT_MB_RX_OVERWRITE) {
        CAN_MessageControl(pCan, bMb,
            CAN_MCR_MDLC(pXfr->bMsgLen)
            | (bStart ? CAN_MCR_MTCR : 0) | (CAN_MSR_MRTR));
    }
}

/**
 * Finish transfer on specific Mailbox.
 * \param pCand  Pointer to CAN Driver instance.
 * \param pXfr   Pointer to CAN Transfer instance.
 * \param bSC    Status code.
 */
void CAND_EndXfr(sCand *pCand, sCandTransfer *pXfr, uint8_t bSC) {
    if (!pCand) return;
    /* Return status */
    pXfr->bRC = bSC;
    if (bSC >= CAND_ERROR)
        pXfr->bState = CAND_XFR_HALTED;
    else if (pXfr->bState == CAND_XFR_TX)
        pXfr->bState = CAND_XFR_IDLE;
    /* Invoke callbacks */
}

/**
 * Disable all mailboxes
 */
void CAND_ResetMailboxes(sCand *pCand) {
    uint32_t i;
    /* Reset all mailboxes */
    for (i = 0; i < CAN_NUM_MAILBOX; i++) {
        CAND_ResetMb(pCand, i, NULL);
    }
    pCand->bState = CAND_STATE_INIT;
}

/**
 * Handler for messages
 * \param pCand Pointer to CAN Driver instance.
 */
void CAND_MessageHandler(sCand *pCand) {
    Can *pCan = pCand->pHw;
    sCandTransfer *pXfr;
    uint8_t bMb;
    uint32_t dwMsr;
    for (bMb = 0; bMb < CAN_NUM_MAILBOX; bMb++) {
        /* Mailbox used ? */
        pXfr = pCand->pMbs[bMb];
        if (pXfr == NULL)
            continue;
        /* Mailbox ready ? */
        dwMsr = CAN_GetMessageStatus(pCan, bMb);
        if ((dwMsr & CAN_MSR_MRDY) != CAN_MSR_MRDY)
            continue;
        /* Handle data */
        switch (CAN_GetMessageMode(pCan, bMb) & CAN_MMR_MOT_Msk) {
        case CAN_MMR_MOT_MB_RX_OVERWRITE: /** Next data overwrite current */
            /*pXfr->bState = CAND_XFR_RX_ONE;*/
        case CAN_MMR_MOT_MB_RX:
        case CAN_MMR_MOT_MB_CONSUMER:   /** TX then RX message */
            pXfr->bMsgLen = (dwMsr & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
            CAN_GetMessage(pCan, bMb, pXfr->msgData);
            CAND_EndXfr(pCand, pXfr, CAND_OK);
            break;

        case CAN_MMR_MOT_MB_TX:
        case CAN_MMR_MOT_MB_PRODUCER:   /** RX then TX message */
            CAND_EndXfr(pCand, pXfr, CAND_OK);
            break;

        default:
            CAND_EndXfr(pCand, pXfr, CAND_ERROR);
            break;
        }
        /*if (pXfr->bState != CAND_XFR_RX_ONE)*/
        {
            /* Disable mailbox interrupt */
            CAN_DisableIt(pCan, 1 << bMb);
            /* Unlink transfer */
            pCand->pMbs[bMb] = NULL;
        }
    }
    /* All transfer finished ? */
    if ((CAN_GetItMask(pCan)&CAN_MB_EVENTS) == 0)
        pCand->bState = CAND_STATE_ACTIVATED;
}

/**
 * Activate CAN.
 * \param pCand Pointer to CAN Driver instance.
 */
void CAND_Activate(sCand *pCand) {
    Can *pCan = pCand->pHw;
    if (pCand->bState > CAND_STATE_SYNC)
        return;
    /* Disable low-power mode */
    CAN_EnableLowPower(pCan, 0);
    /* Start sync state */
    pCand->bState = CAND_STATE_SYNC;
    /* Enable CAN and wait interrupt */
    CAN_EnableIt(pCan, CAN_IER_WAKEUP);
    CAN_Enable(pCan, 1);
}

/**
 * Put into sleep mode
 * \param pCand Pointer to CAN Driver instance.
 */
void CAND_Sleep(sCand *pCand) {
    Can *pCan = pCand->pHw;
    CAN_EnableIt(pCan, CAN_IER_SLEEP);
    CAN_EnableLowPower(pCan, 1);
}

/**
 * Check if CAN is ready to transfer messages.
 * \param pCand Pointer to CAN Driver instance.
 */
uint8_t CAND_IsReady(sCand *pCand) {
    return (pCand->bState >= CAND_STATE_ACTIVATED);
}

/**
 * Interrupt handler for CAN Driver.
 * \param pCand Pointer to CAN Driver instance.
 */
void CAND_Handler(sCand *pCand) {
    Can *pHw = pCand->pHw;
    uint32_t dwSr = CAN_GetStatus(pHw);
    //uint32_t dwSm = CAN_GetItMask(pHw);
    //TRACE_INFO("%d:%8x\n\r", (pHw==AT91_CAN0)?0:1, dwSr);
    /* Errors */
    if (dwSr & CAN_ERRS) {
        pCand->bState = CAND_STATE_ERROR;
        CAN_DisableIt(pHw, dwSr & CAN_ERRS);
    }
    else {
        /* Wakeup and bus synchronization done */
        if (pCand->bState > CAND_STATE_ACTIVATED) {
            /* Mailbox events */
            if (dwSr & CAN_MB_EVENTS) {
                CAND_MessageHandler(pCand);
            }
        }
        else if (dwSr & CAN_SR_WAKEUP) {
            CAN_DisableIt(pHw, CAN_IDR_WAKEUP);
            pCand->bState = CAND_STATE_ACTIVATED;
        }
    }
    /* Low-power Mode enabled */
    if (dwSr & CAN_SR_SLEEP) {
        CAN_DisableIt(pHw, CAN_IDR_SLEEP);
        pCand->bState = CAND_STATE_SLEEP;
    }
    /* Timestamp */
    if (dwSr & CAN_SR_TSTP) {
    }
    /* Timer overflow */
    if (dwSr & CAN_SR_TOVF) {
    }
}

/**
 * Check if the mailbox is ready to configure or transfer.
 * \param pCand Pointer to CAN Driver instance.
 * \param bMb   Mailbox number.
 * \return 1 if mailbox is free.
 */
uint8_t CAND_IsMailboxReady(sCand *pCand, uint8_t bMb) {
    return (CAND_IsMbReady(pCand->pMbs[bMb]));
}

/**
 * Reset the CAN Mailbox (with configuration).
 * \param pCand Pointer to CAN Driver instance.
 * \param bMb   Mailbox number.
 * \param pCfg  Pointer to Mailbox configuration instance.
 *              NULL to reset and disable the mailbox.
 */
void CAND_ResetMailbox(sCand *pCand, uint8_t bMb, sCandMbCfg *pCfg) {
    CAND_ResetMb(pCand, bMb, pCfg);
}

/**
 * Configure the CAN Mailbox for message transfer.
 * \param pCand Pointer to CAN Driver instance.
 * \param pCfg  Pointer to Mailbox configuration instance.
 *              NULL to use old configuration.
 * \param pXfr  Pointer to transfer configuration instance.
 */
uint8_t CAND_ConfigureTransfer(sCand *pCand,
    sCandMbCfg *pCfg,
    sCandTransfer *pXfr) {
    uint8_t bMb = pXfr->bMailbox;
    sCandTransfer *pTx = pCand->pMbs[bMb];

    if (!CAND_IsMbReady(pTx))
        return CAND_BUSY;
    if (pCfg)
        CAND_ResetMb(pCand, bMb, pCfg);
    CAND_InitXfr(pCand, pXfr, 0);
    return CAND_OK;
}

/**
 * Transfer CAN message through a configured mailbox.
 * The transfer will not start until it's started by CAND_StartTransfers().
 * \note For data receiving, if there is previous pending message in
 *       mailbox, the RX operation will return this message data.
 * \param pCand  Pointer to CAN Driver instance.
 * \param pXfr   Pointer to transfer configuration instance.
 * \param bStart 1 to start the transfer immediately.
 */
uint8_t CAND_Transfer(sCand *pCand, sCandTransfer *pXfr) {
    Can *pCan = pCand->pHw;
    sCandTransfer *pTx;
    uint8_t bMb = pXfr->bMailbox;

    pTx = pCand->pMbs[bMb];
    if (!CAND_IsMbReady(pTx)) return CAND_BUSY;
    if (0 == CAN_GetMessageMode(pCan, bMb))
        return CAND_ERR_STATE;
    /* Configure and start transfer */
    CAND_InitXfr(pCand, pXfr, 1);
    /* Enable interrupts statuses */
    CAN_EnableIt(pCan, (CAN_ID_MASK & (1 << bMb)) | CAN_ERRS);
    return CAND_OK;
}

uint8_t CAND_Transfer_RTRMode(sCand *pCand, sCandTransfer *pXfr) {
    Can *pCan = pCand->pHw;
    sCandTransfer *pTx;
    uint8_t bMb = pXfr->bMailbox;

    pTx = pCand->pMbs[bMb];
    if (!CAND_IsMbReady(pTx)) return CAND_BUSY;
    if (0 == CAN_GetMessageMode(pCan, bMb))
        return CAND_ERR_STATE;
    /* Configure and start transfer */
    CAND_InitXfr_RTRMode(pCand, pXfr, 1);
    /* Enable interrupts statuses */
    CAN_EnableIt(pCan, (CAN_ID_MASK & (1 << bMb)) | CAN_ERRS);
    return CAND_OK;
}

/**
 * Start configured transfers (by CAND_ConfigureTransfer()).
 * \note For data receiving, if there is previous pending message in
 *       mailbox, the RX operation will return this message data.
 * \param pCand Pointer to CAN Driver instance.
 * \param bmMbs Mailbox bitmap.
 */
void CAND_StartTransfers(sCand *pCand, uint32_t bmMbs) {
    Can *pCan = pCand->pHw;
    sCandTransfer *pTx;
    uint8_t bMb;
    uint32_t bmTx = 0;
    uint32_t  bmRx = 0;
    uint32_t dwMMR;
    /* Scan mailboxes that not started */
    for (bMb = 0; bMb < CAN_NUM_MAILBOX; bMb++) {
        if ((bmMbs & (1 << bMb)) == 0)
            continue;
        /* Check if the mailbox is ready to transfer */
        pTx = pCand->pMbs[bMb];
        if (pTx == NULL) {
            /* Ignore the mailbox */
            bmMbs &= ~(1 << bMb);
            continue;
        }
        if (pTx->bState > CAND_XFR_IDLE) {
            /* Ignore the mailbox */
            bmMbs &= ~(1 << bMb);
            continue;
        }
        dwMMR = CAN_GetMessageMode(pCan, bMb);
        /* Disabled ? */
        if (0 == dwMMR) {
            /* Ignore the mailbox */
            bmMbs &= ~(1 << bMb);
            continue;
        }
        /* RX ? */
        else if ((dwMMR & CAN_MMR_MOT_Msk) <= CAN_MMR_MOT_MB_RX_OVERWRITE) {
            bmRx |= 1 << bMb;
        }
        /* TX ! */
        else {
            bmTx |= 1 << bMb;
        }

        /* Change transfer state */
        pTx->bState = CAND_XFR_TX;

        /* Nothing to start */
        if (bmMbs == 0)
            return;
    }
    /* Change CAN state */
    pCand->bState = CAND_STATE_XFR;
    /* Start transfers */
    CAN_Command(pCan, bmTx);
    /* Enable interrupts */
    CAN_EnableIt(pCan, bmMbs | CAN_ERRS);
}

/**
 * Check if the transfer is finished.
 * \return 1 if it's ready to transfer data.
 */
uint8_t CAND_IsTransferDone(sCandTransfer *pXfr) {
    return CAND_IsMbReady(pXfr);
}

struct AT91_Can_Filter {
    uint32_t *matchFilters;
    uint32_t matchFiltersSize;

    uint32_t *lowerBoundFilters;
    uint32_t *upperBoundFilters;
    uint32_t groupFiltersSize;

};

typedef struct {
    uint32_t timeStampL;
    uint32_t timeStampH;

    uint32_t msgId;

    bool extendedId;
    bool remoteTransmissionRequest;

    uint32_t dataA;
    uint32_t dataB;

    int32_t length;

} AT91_Can_Message;

struct CanState {
    int32_t controllerIndex;

    const TinyCLR_Can_Controller* controller;
    AT91_Can_Message *canRxMessagesFifo;

    TinyCLR_Can_ErrorReceivedHandler   errorEventHandler;
    TinyCLR_Can_MessageReceivedHandler    messageReceivedEventHandler;

    int32_t can_rx_count;
    int32_t can_rx_in;
    int32_t can_rx_out;

    size_t can_rxBufferSize;
    size_t can_txBufferSize;

    uint32_t baudrate;

    sCand cand;

    sCandTransfer can_tx;
    sCandTransfer can_rx;

    AT91_Can_Filter canDataFilter;

    uint16_t initializeCount;

    bool enable;
};

static const AT91_Gpio_Pin canTxPins[] = AT91_CAN_TX_PINS;
static const AT91_Gpio_Pin canRxPins[] = AT91_CAN_RX_PINS;

static CanState canStates[TOTAL_CAN_CONTROLLERS];

static TinyCLR_Can_Controller canControllers[TOTAL_CAN_CONTROLLERS];
static TinyCLR_Api_Info canApi[TOTAL_CAN_CONTROLLERS];

void CAN_DisableExplicitFilters(int32_t controllerIndex) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto state = &canStates[controllerIndex];

    if (state->canDataFilter.matchFiltersSize && state->canDataFilter.matchFilters != nullptr) {
        memoryProvider->Free(memoryProvider, state->canDataFilter.matchFilters);

        state->canDataFilter.matchFiltersSize = 0;
    }
}

void CAN_DisableGroupFilters(int32_t controllerIndex) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto state = &canStates[controllerIndex];

    if (state->canDataFilter.groupFiltersSize) {
        if (state->canDataFilter.lowerBoundFilters != nullptr)
            memoryProvider->Free(memoryProvider, state->canDataFilter.lowerBoundFilters);

        if (state->canDataFilter.upperBoundFilters != nullptr)
            memoryProvider->Free(memoryProvider, state->canDataFilter.upperBoundFilters);

        state->canDataFilter.groupFiltersSize = 0;
    }
}

/******************************************************************************
** Function name:        CAN_SetACCF_Lookup
**
** Descriptions:        Initialize CAN, install CAN interrupt handler
**
** parameters:            bitrate
** Returned value:        true or false, false if initialization failed.
**
******************************************************************************/
void CAN_SetACCF_Lookup(void) {

    return;
}

/******************************************************************************
** Function name:        CAN_SetACCF
**
** Descriptions:        Set acceptance filter and SRAM associated with
**
** parameters:            ACMF mode
** Returned value:        None
**
**
******************************************************************************/
void CAN_SetACCF(uint32_t ACCFMode) {

    return;
}

bool InsertionSort2CheckOverlap(uint32_t *lowerBounds, uint32_t *upperBounds, int32_t length) {

    uint32_t i, j, tmp, tmp2;

    for (i = 1; i < length; i++) {
        j = i;

        while (j > 0 && lowerBounds[j - 1] > lowerBounds[j]) {
            tmp = lowerBounds[j]; tmp2 = upperBounds[j];
            lowerBounds[j] = lowerBounds[j - 1];    upperBounds[j] = upperBounds[j - 1];
            lowerBounds[j - 1] = tmp;    upperBounds[j - 1] = tmp2;

            j--;
        }
    }

    // check for overlap
    if (lowerBounds[0] > upperBounds[0])
        return false;

    for (i = 1; i < length; i++) {
        if (lowerBounds[i] > upperBounds[i])
            return false;

        if (lowerBounds[i] <= upperBounds[i - 1])
            return false;
    }

    return true;
}

int32_t BinarySearch(uint32_t *sortedArray, int32_t first, int32_t last, uint32_t key) {
    int32_t mid;
    while (first <= last) {
        mid = (first + last) / 2;  // compute mid point.
        if (key > sortedArray[mid])
            first = mid + 1;  // repeat search in top half.
        else if (key < sortedArray[mid])
            last = mid - 1; // repeat search in bottom half.
        else
            return mid;     // found it. return position /////
    }

    return -1;    // failed to find key
}

int32_t BinarySearch2(uint32_t *lowerBounds, uint32_t *upperBounds, int32_t first, int32_t last, uint32_t key) {

    int32_t mid;

    while (first <= last) {
        mid = (first + last) / 2;  // compute mid point.

        if (key > upperBounds[mid])
            first = mid + 1;  // repeat search in top half.
        else if (key < lowerBounds[mid])
            last = mid - 1; // repeat search in bottom half.
        else
            return mid;     // found it. return position /////
    }

    return -1;    // failed to find key
}

const char* canApiNames[] = {
#if TOTAL_CAN_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.AT91.CanController\\0",
#if TOTAL_CAN_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.AT91.CanController\\1"
#endif
#endif
};

void AT91_Can_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_CAN_CONTROLLERS; i++) {
        canControllers[i].ApiInfo = &canApi[i];
        canControllers[i].Acquire = &AT91_Can_Acquire;
        canControllers[i].Release = &AT91_Can_Release;
        canControllers[i].Enable = &AT91_Can_Enable;
        canControllers[i].Disable = &AT91_Can_Disable;
        canControllers[i].CanWriteMessage = &AT91_Can_CanWriteMessage;
        canControllers[i].CanReadMessage = &AT91_Can_CanReadMessage;
        canControllers[i].WriteMessage = &AT91_Can_WriteMessage;
        canControllers[i].ReadMessage = &AT91_Can_ReadMessage;
        canControllers[i].SetBitTiming = &AT91_Can_SetBitTiming;
        canControllers[i].GetMessagesToRead = &AT91_Can_GetMessagesToRead;
        canControllers[i].SetMessageReceivedHandler = &AT91_Can_SetMessageReceivedHandler;
        canControllers[i].SetErrorReceivedHandler = &AT91_Can_SetErrorReceivedHandler;
        canControllers[i].SetExplicitFilters = &AT91_Can_SetExplicitFilters;
        canControllers[i].SetGroupFilters = &AT91_Can_SetGroupFilters;
        canControllers[i].ClearReadBuffer = &AT91_Can_ClearReadBuffer;
        canControllers[i].GetWriteErrorCount = &AT91_Can_GetWriteErrorCount;
        canControllers[i].GetReadErrorCount = &AT91_Can_GetReadErrorCount;
        canControllers[i].GetSourceClock = &AT91_Can_GetSourceClock;
        canControllers[i].GetReadBufferSize = &AT91_Can_GetReadBufferSize;
        canControllers[i].SetReadBufferSize = &AT91_Can_SetReadBufferSize;
        canControllers[i].GetWriteBufferSize = &AT91_Can_GetWriteBufferSize;
        canControllers[i].SetWriteBufferSize = &AT91_Can_SetWriteBufferSize;

        canApi[i].Author = "GHI Electronics, LLC";
        canApi[i].Name = canApiNames[i];
        canApi[i].Type = TinyCLR_Api_Type::CanController;
        canApi[i].Version = 0;
        canApi[i].Implementation = &canControllers[i];
        canApi[i].State = &canStates[i];

        canStates[i].controllerIndex = i;

        apiManager->Add(apiManager, &canApi[i]);
    }
}

bool CAN_RxInitialize(int8_t controllerIndex) {
    auto state = &canStates[controllerIndex];

    sCand *pCand = &state->cand;

    sCandMbCfg candCfg;

    candCfg.bMsgType = CAN_MMR_MOT_MB_RX >> CAN_MMR_MOT_Pos;
    candCfg.bTxPriority = 0;

    state->can_rx.bMailbox = MAILBOX_TO_RECEIVE_INDEX;
    state->can_rx.dwMsgID = 1 << 29;

    CAND_ConfigureTransfer(pCand, &candCfg, &state->can_rx);

    pCand->pHw->CAN_MB[MAILBOX_TO_RECEIVE_INDEX].CAN_MID = 1 << 29;
    pCand->pHw->CAN_MB[MAILBOX_TO_RECEIVE_INDEX].CAN_MAM = 0;

    CAND_StartTransfers(pCand, CAN_SR_MB1);
}

void CopyMessageFromMailBoxToBuffer(uint8_t controllerIndex, uint32_t dwMsr) {
    auto state = &canStates[controllerIndex];

    sCand *pCand = &state->cand;

    uint32_t msgid = 0;
    bool extendMode = 0;
    char passed = 0;

    msgid = pCand->pHw->CAN_MB[MAILBOX_TO_RECEIVE_INDEX].CAN_MID;

    if ((msgid & CAN_MID_MIDE) == CAN_MID_MIDE) {
        msgid = msgid & 0x1FFFFFFF; //CAN_MID_MIDvA_Msk & CAN_MID_MIDvB_Msk;
        extendMode = true; // last bit in frame is extend mode flag 0: 11 bit, 1: 29 bit id
    }
    else {
        msgid = (msgid & CAN_MID_MIDvA_Msk) >> 18;
        extendMode = false; // last bit in frame is extend mode flag 0: 11 bit, 1: 29 bit id
    }

    // filter
    if (state->canDataFilter.groupFiltersSize || state->canDataFilter.matchFiltersSize) {
        //Added filter for AT91_CAN0
        if (state->canDataFilter.groupFiltersSize || state->canDataFilter.matchFiltersSize) {
            if (state->canDataFilter.groupFiltersSize) {
                if (BinarySearch2(state->canDataFilter.lowerBoundFilters, state->canDataFilter.upperBoundFilters, 0, state->canDataFilter.groupFiltersSize - 1, msgid) >= 0)
                    passed = 1;
            }

            if (!passed && state->canDataFilter.matchFiltersSize) {
                if (BinarySearch(state->canDataFilter.matchFilters, 0, state->canDataFilter.matchFiltersSize - 1, msgid) >= 0)
                    passed = 1;
            }

            if (!passed) {
                return;
            }
        }
    }

    if (state->can_rx_count > (state->can_rxBufferSize - 3)) {
        state->errorEventHandler(state->controller, TinyCLR_Can_Error::BufferFull, AT91_Time_GetCurrentProcessorTime());
    }

    // initialize destination pointer
    AT91_Can_Message *can_msg = &state->canRxMessagesFifo[state->can_rx_in];

    // timestamp
    uint64_t t = AT91_Time_GetCurrentProcessorTime();

    can_msg->timeStampL = t & 0xFFFFFFFF;
    can_msg->timeStampH = t >> 32;

    can_msg->length = (state->can_rx.bMsgLen) & 0x0F;

    can_msg->extendedId = extendMode;

    can_msg->remoteTransmissionRequest = ((dwMsr >> 20) & 0x01) ? true : false;

    can_msg->msgId = msgid; // ID

    can_msg->dataA = state->can_rx.msgData[0]; // Data A

    can_msg->dataB = state->can_rx.msgData[1]; // Data B

    state->can_rx_count++;
    state->can_rx_in++;

    if (state->can_rx_in == state->can_rxBufferSize) {
        state->can_rx_in = 0;
    }

    state->messageReceivedEventHandler(state->controller, state->can_rx_count, t);
}

void CAN_ProccessMailbox(uint8_t controllerIndex) {
    auto state = &canStates[controllerIndex];

    sCand *pCand = &state->cand;
    Can *pCan = pCand->pHw;
    sCandTransfer *pXfr;
    uint8_t bMb;
    uint32_t dwMsr;
    for (bMb = 0; bMb < CAN_NUM_MAILBOX; bMb++) {
        /* Mailbox used ? */
        pXfr = pCand->pMbs[bMb];
        if (pXfr == NULL)
            continue;
        /* Mailbox ready ? */
        dwMsr = CAN_GetMessageStatus(pCan, bMb);
        if ((dwMsr & CAN_MSR_MRDY) != CAN_MSR_MRDY)
            continue;
        /* Handle data */
        switch (CAN_GetMessageMode(pCan, bMb) & CAN_MMR_MOT_Msk) {
        case CAN_MMR_MOT_MB_RX_OVERWRITE: /** Next data overwrite current */
            /*pXfr->bState = CAND_XFR_RX_ONE;*/
        case CAN_MMR_MOT_MB_RX:
        case CAN_MMR_MOT_MB_CONSUMER:   /** TX then RX message */

            pXfr->bMsgLen = (dwMsr & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
            CAN_GetMessage(pCan, bMb, pXfr->msgData);
            CopyMessageFromMailBoxToBuffer(controllerIndex, dwMsr);
            CAND_EndXfr(pCand, pXfr, CAND_OK);

            break;

        case CAN_MMR_MOT_MB_TX:
        case CAN_MMR_MOT_MB_PRODUCER:   /** RX then TX message */
            CAND_EndXfr(pCand, pXfr, CAND_OK);
            break;

        default:
            CAND_EndXfr(pCand, pXfr, CAND_ERROR);
            break;
        }
        /* Disable mailbox interrupt */
        CAN_DisableIt(pCan, 1 << bMb);
        /* Unlink transfer */
        pCand->pMbs[bMb] = NULL;
    }
    /* All transfer finished ? */
    if ((CAN_GetItMask(pCan)&CAN_MB_EVENTS) == 0)
        pCand->bState = CAND_STATE_ACTIVATED;
}

void CAN_ErrorHandler(sCand *pCand, uint32_t dwErrS, int32_t controllerIndex) {
    auto state = &canStates[controllerIndex];

    if (dwErrS & CAN_SR_BOFF) // BusOff is higher priority
    {
        state->errorEventHandler(state->controller, TinyCLR_Can_Error::BusOff, AT91_Time_GetCurrentProcessorTime());

    }
    else if (dwErrS & CAN_SR_ERRP) {
        state->errorEventHandler(state->controller, TinyCLR_Can_Error::Passive, AT91_Time_GetCurrentProcessorTime());
    }
}

/******************************************************************************
** Function name:        AT91_Can_RxInterruptHandler
**
** Descriptions:        CAN Rx1 interrupt handler
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/

void AT91_Can_RxInterruptHandler(void *param) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t controllerIndex = *reinterpret_cast<uint32_t*>(param);

    auto state = &canStates[controllerIndex];

    sCand *pCand = &state->cand;
    Can *pHw = pCand->pHw;
    uint32_t dwSr = (CAN_GetStatus(pHw) & CAN_GetItMask(pHw));
    if (dwSr & CAN_ERRS) {
        CAN_DisableIt(pHw, (dwSr & CAN_ERRS));
        if (pCand->bState != CAND_STATE_DISABLED) {
            pCand->bState = CAND_STATE_ERROR;
            CAN_ErrorHandler(pCand, (dwSr & CAN_ERRS), controllerIndex);
        }
    }
    else {
        /* Wakeup and bus synchronization done */
        if (pCand->bState > CAND_STATE_ACTIVATED) {
            /* Mailbox events */
            if (dwSr & CAN_MB_EVENTS) {
                CAN_ProccessMailbox(controllerIndex);
            }
        }
        else if (dwSr & CAN_SR_WAKEUP) {
            CAN_DisableIt(pHw, CAN_IDR_WAKEUP);
            pCand->bState = CAND_STATE_ACTIVATED;
        }
        CAN_RxInitialize(controllerIndex);
    }
    /* Low-power Mode enabled */
    if (dwSr & CAN_SR_SLEEP) {
        CAN_DisableIt(pHw, CAN_IDR_SLEEP);
        pCand->bState = CAND_STATE_SLEEP;
    }
    /* Timestamp */
    if (dwSr & CAN_SR_TSTP) {
    }
    /* Timer overflow */
    if (dwSr & CAN_SR_TOVF) {
        state->errorEventHandler(state->controller, TinyCLR_Can_Error::Overrun, AT91_Time_GetCurrentProcessorTime());
    }
}

TinyCLR_Result AT91_Can_Acquire(const TinyCLR_Can_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        if (!AT91_Gpio_OpenPin(canTxPins[controllerIndex].number))
            return TinyCLR_Result::SharingViolation;

        if (!AT91_Gpio_OpenPin(canRxPins[controllerIndex].number))
            return TinyCLR_Result::SharingViolation;


        AT91_Gpio_ConfigurePin(canTxPins[controllerIndex].number, AT91_Gpio_Direction::Input, canTxPins[controllerIndex].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);
        AT91_Gpio_ConfigurePin(canRxPins[controllerIndex].number, AT91_Gpio_Direction::Input, canRxPins[controllerIndex].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);

        state->can_rx_count = 0;
        state->can_rx_in = 0;
        state->can_rx_out = 0;
        state->baudrate = 0;
        state->can_rxBufferSize = canDefaultBuffersSize[controllerIndex];
        state->controller = self;
        state->enable = false;

        state->canDataFilter.matchFiltersSize = 0;
        state->canDataFilter.groupFiltersSize = 0;

        state->cand.pHw = (controllerIndex == 0 ? AT91_CAN0 : AT91_CAN1);
        state->cand.bID = (controllerIndex == 0 ? AT91C_ID_CAN0 : AT91C_ID_CAN1);

        AT91_PMC &pmc = AT91::PMC();
        pmc.EnablePeriphClock((controllerIndex == 0) ? AT91C_ID_CAN0 : AT91C_ID_CAN1);

        CAN_DisableIt(state->cand.pHw, 0xFFFFFFFF);

        state->controllerIndex = controllerIndex;
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_Release(const TinyCLR_Can_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

        CAN_DisableIt(state->cand.pHw, 0xFFFFFFFF);

        AT91_PMC &pmc = AT91::PMC();
        pmc.DisablePeriphClock((controllerIndex == 0) ? AT91C_ID_CAN0 : AT91C_ID_CAN1);


        AT91_Gpio_ClosePin(canTxPins[controllerIndex].number);
        AT91_Gpio_ClosePin(canRxPins[controllerIndex].number);


        if (state->canRxMessagesFifo != nullptr) {
            memoryProvider->Free(memoryProvider, state->canRxMessagesFifo);

            state->canRxMessagesFifo = nullptr;
        }

        CAN_DisableExplicitFilters(controllerIndex);
        CAN_DisableGroupFilters(controllerIndex);

    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SoftReset(const TinyCLR_Can_Controller* self) {
    volatile int32_t i;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    state->can_rx_count = 0;
    state->can_rx_in = 0;
    state->can_rx_out = 0;

    sCand *pCand = &state->cand;

    // Disable
    CAN_Enable(state->cand.pHw, 0);
    pCand->bState = CAND_STATE_DISABLED;

    for (i = 0xFF; i > 0; i--);

    state->cand.wBaudrate = state->baudrate;

    CAN_ConfigureBaudrate(state->cand.pHw, state->baudrate);

    CAN_ConfigureMode(state->cand.pHw, 0);

    /* Reset all mailboxes */
    CAND_ResetMailboxes(&state->cand);

    /* Enable the interrupts for error cases */
    CAN_EnableIt(state->cand.pHw, CAN_ERRS);

    CAND_Activate(&state->cand);

    int32_t timeout = CAN_TRANSFER_TIMEOUT;

    while (timeout > 0) {
        if (CAND_IsReady(&state->cand)) {
            return TinyCLR_Result::Success;
        }
        timeout--;
    }

    pCand->bState = CAND_STATE_ACTIVATED;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_WriteMessage(const TinyCLR_Can_Controller* self, uint32_t arbitrationId, bool isExtendedId, bool isRemoteTransmissionRequest, const uint8_t* data, size_t length) {

    uint32_t *data32 = (uint32_t*)data;

    bool readyToSend = false;

    sCandMbCfg candCfg;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    sCand *pCand = &state->cand;

    candCfg.bTxPriority = 1;

    uint32_t timeout = CAN_TRANSFER_TIMEOUT;

    while (readyToSend == false && timeout-- > 0) {
        AT91_Can_IsWritingAllowed(self, readyToSend);
        AT91_Time_Delay(nullptr, 1);
    }

    if (timeout == 0)
        return TinyCLR_Result::Busy;

    candCfg.bMsgType = CAN_MMR_MOT_MB_TX >> CAN_MMR_MOT_Pos;
    state->can_tx.bMailbox = MAILBOX_TO_SEND_INDEX;

    CAND_ResetMailbox(pCand, state->can_tx.bMailbox, &candCfg);

    candCfg.bTxPriority = 0;

    if (isExtendedId) {
        state->can_tx.dwMsgID = (arbitrationId & 0x1FFFFFFF);
        state->can_tx.dwMsgID |= 1 << 29;
    }
    else {
        state->can_tx.dwMsgID = CAN_MID_MIDvA(arbitrationId);
    }

    if (isRemoteTransmissionRequest) {
        state->can_tx.bMsgLen = 0;
        state->can_tx.msgData[0] = 0;
        state->can_tx.msgData[1] = 0;

        CAND_Transfer_RTRMode(pCand, &state->can_tx);
    }
    else {
        state->can_tx.bMsgLen = length;
        state->can_tx.msgData[0] = data32[0];
        state->can_tx.msgData[1] = data32[1];
        CAND_Transfer(pCand, &state->can_tx);
    }

    timeout = CAN_TRANSFER_TIMEOUT;

    while (timeout > 0) {
        if (CAND_IsTransferDone(&state->can_tx))
            return TinyCLR_Result::Success;

        AT91_Time_Delay(nullptr, 1);
        timeout--;
    }

    return TinyCLR_Result::Busy;
}

TinyCLR_Result AT91_Can_ReadMessage(const TinyCLR_Can_Controller* self, uint32_t& arbitrationId, bool& isExtendedId, bool& isRemoteTransmissionRequest, uint8_t* data, size_t& length, uint64_t& timestamp) {
    AT91_Can_Message *can_msg;

    uint32_t *data32 = (uint32_t*)data;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    if (state->can_rx_count) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        can_msg = &state->canRxMessagesFifo[state->can_rx_out];
        state->can_rx_out++;

        if (state->can_rx_out == state->can_rxBufferSize)
            state->can_rx_out = 0;

        state->can_rx_count--;

        arbitrationId = can_msg->msgId;
        isExtendedId = can_msg->extendedId;
        isRemoteTransmissionRequest = can_msg->remoteTransmissionRequest;

        data32[0] = can_msg->dataA;
        data32[1] = can_msg->dataB;

        length = can_msg->length;

        timestamp = ((uint64_t)can_msg->timeStampL) | ((uint64_t)can_msg->timeStampH << 32);
    }

    return TinyCLR_Result::Success;

}

TinyCLR_Result AT91_Can_SetBitTiming(const TinyCLR_Can_Controller* self, uint32_t propagation, uint32_t phase1, uint32_t phase2, uint32_t baudratePrescaler, uint32_t synchronizationJumpWidth, bool useMultiBitSampling) {

    uint32_t sourceClk;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    state->cand.dwMck = AT91_SYSTEM_PERIPHERAL_CLOCK_HZ;

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    if (state->canRxMessagesFifo == nullptr)
        state->canRxMessagesFifo = (AT91_Can_Message*)memoryProvider->Allocate(memoryProvider, state->can_rxBufferSize * sizeof(AT91_Can_Message));

    if (state->canRxMessagesFifo == nullptr) {
        return TinyCLR_Result::OutOfMemory;
    }

    state->baudrate = CAN_BR_PHASE2(phase2) | CAN_BR_PHASE1(phase1) | CAN_BR_PROPAG(propagation) | CAN_BR_SJW(synchronizationJumpWidth) | CAN_BR_BRP(baudratePrescaler) | (useMultiBitSampling ? CAN_BR_SMP_THREE : CAN_BR_SMP_ONCE);

    state->cand.wBaudrate = state->baudrate;

    CAN_ConfigureBaudrate(state->cand.pHw, state->baudrate);

    CAN_ConfigureMode(state->cand.pHw, 0);

    /* Reset all mailboxes */
    CAND_ResetMailboxes(&state->cand);

    /* Enable the interrupts for error cases */
    CAN_EnableIt(state->cand.pHw, CAN_ERRS);

    AT91_Interrupt_Activate(controllerIndex == 0 ? AT91C_ID_CAN0 : AT91C_ID_CAN1, (uint32_t*)&AT91_Can_RxInterruptHandler, (void*)&state->controllerIndex);

    CAND_Activate(&state->cand);

    int32_t timeout = CAN_TRANSFER_TIMEOUT;

    while (timeout > 0) {
        if (CAND_IsReady(&state->cand)) {
            return TinyCLR_Result::Success;
        }
        timeout--;
    }

    return TinyCLR_Result::InvalidOperation;
}

size_t AT91_Can_GetMessagesToRead(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    return state->can_rx_count;
}

TinyCLR_Result AT91_Can_SetMessageReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_MessageReceivedHandler handler) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);
    state->messageReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SetErrorReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_ErrorReceivedHandler handler) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);
    state->errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SetExplicitFilters(const TinyCLR_Can_Controller* self, const uint32_t* filters, size_t count) {
    uint32_t *_matchFilters;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    _matchFilters = (uint32_t*)memoryProvider->Allocate(memoryProvider, count * sizeof(uint32_t));

    if (!_matchFilters)
        return TinyCLR_Result::OutOfMemory;

    memcpy(_matchFilters, filters, count * sizeof(uint32_t));

    std::sort(_matchFilters, _matchFilters + count);

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        CAN_DisableExplicitFilters(controllerIndex);

        state->canDataFilter.matchFiltersSize = count;
        state->canDataFilter.matchFilters = _matchFilters;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SetGroupFilters(const TinyCLR_Can_Controller* self, const uint32_t* lowerBounds, const uint32_t* upperBounds, size_t count) {
    uint32_t *_lowerBoundFilters, *_upperBoundFilters;

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    _lowerBoundFilters = (uint32_t*)memoryProvider->Allocate(memoryProvider, count * sizeof(uint32_t));
    _upperBoundFilters = (uint32_t*)memoryProvider->Allocate(memoryProvider, count * sizeof(uint32_t));

    if (!_lowerBoundFilters || !_upperBoundFilters) {
        memoryProvider->Free(memoryProvider, _lowerBoundFilters);
        memoryProvider->Free(memoryProvider, _upperBoundFilters);

        return  TinyCLR_Result::OutOfMemory;
    }

    memcpy(_lowerBoundFilters, lowerBounds, count * sizeof(uint32_t));
    memcpy(_upperBoundFilters, upperBounds, count * sizeof(uint32_t));

    bool success = InsertionSort2CheckOverlap(_lowerBoundFilters, _upperBoundFilters, count);

    if (!success) {
        memoryProvider->Free(memoryProvider, _lowerBoundFilters);
        memoryProvider->Free(memoryProvider, _upperBoundFilters);

        return TinyCLR_Result::ArgumentInvalid;
    }

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        CAN_DisableGroupFilters(controllerIndex);

        state->canDataFilter.groupFiltersSize = count;
        state->canDataFilter.lowerBoundFilters = _lowerBoundFilters;
        state->canDataFilter.upperBoundFilters = _upperBoundFilters;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_ClearReadBuffer(const TinyCLR_Can_Controller* self) {

    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    state->can_rx_count = 0;
    state->can_rx_in = 0;
    state->can_rx_out = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_IsWritingAllowed(const TinyCLR_Can_Controller* self, bool& allowed) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    sCand *pCand = &state->cand;

    uint32_t status = CAN_GetStatus(pCand->pHw);

    allowed = true;

    if (((status >> 29) & 0x1) != 0 // receive Busy
        || ((status >> 30) & 0x1) != 0 // transfer Busy
        )
        allowed = false;

    return TinyCLR_Result::Success;
}

size_t AT91_Can_GetReadErrorCount(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    return CAN_GetRxErrorCount(state->cand.pHw);
}

size_t AT91_Can_GetWriteErrorCount(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    return CAN_GetTxErrorCount(state->cand.pHw);
}

uint32_t AT91_Can_GetSourceClock(const TinyCLR_Can_Controller* self) {
    return AT91_SYSTEM_PERIPHERAL_CLOCK_HZ;
}

size_t AT91_Can_GetReadBufferSize(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    return state->can_rxBufferSize == 0 ? canDefaultBuffersSize[controllerIndex] : state->can_rxBufferSize;
}

TinyCLR_Result AT91_Can_SetReadBufferSize(const TinyCLR_Can_Controller* self, size_t size) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (size > 3) {
        state->can_rxBufferSize = size;
        return TinyCLR_Result::Success;
    }
    else {
        state->can_rxBufferSize = canDefaultBuffersSize[controllerIndex];
        return TinyCLR_Result::ArgumentInvalid;;
    }
}

size_t AT91_Can_GetWriteBufferSize(const TinyCLR_Can_Controller* self) {
    return 1;
}

TinyCLR_Result AT91_Can_SetWriteBufferSize(const TinyCLR_Can_Controller* self, size_t size) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    state->can_txBufferSize = 1;

    return size == 1 ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

void AT91_Can_Reset() {
    for (int i = 0; i < TOTAL_CAN_CONTROLLERS; i++) {
        canStates[i].canRxMessagesFifo = nullptr;

        AT91_Can_Release(&canControllers[i]);

        canStates[i].initializeCount = 0;
    }

}

TinyCLR_Result AT91_Can_Enable(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);
    state->enable = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_Disable(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);
    state->enable = false;

    return TinyCLR_Result::Success;
}

bool AT91_Can_CanWriteMessage(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);
    bool canWrite;

    AT91_Can_IsWritingAllowed(self, canWrite);
    return (state->enable && canWrite);
}

bool AT91_Can_CanReadMessage(const TinyCLR_Can_Controller* self) {
    auto state = reinterpret_cast<CanState*>(self->ApiInfo->State);

    return (state->enable);
}

#endif // INCLUDE_CAN
