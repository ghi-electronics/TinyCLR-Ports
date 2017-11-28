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
#include "STM32F4.h"

///////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN

#define CAN_MESSAGES_MAX 128

#define CAN_Mode_Normal             ((uint8_t)0x00)  /*!< normal mode */
#define CAN_Mode_LoopBack           ((uint8_t)0x01)  /*!< loopback mode */
#define CAN_Mode_Silent             ((uint8_t)0x02)  /*!< silent mode */
#define CAN_Mode_Silent_LoopBack    ((uint8_t)0x03)  /*!< loopback combined with silent mode */


/** @defgroup CAN_InitStatus
  * @{
  */

#define CAN_InitStatus_Failed              ((uint8_t)0x00) /*!< CAN initialization failed */
#define CAN_InitStatus_Success             ((uint8_t)0x01) /*!< CAN initialization OK */

  /** @defgroup CAN_filter_mode
    * @{
    */
#define CAN_FilterMode_IdMask       ((uint8_t)0x00)  /*!< identifier/mask mode */
#define CAN_FilterMode_IdList       ((uint8_t)0x01)  /*!< identifier list mode */

    /** @defgroup CAN_filter_scale
      * @{
      */
#define CAN_FilterScale_16bit       ((uint8_t)0x00) /*!< Two 16-bit filters */
#define CAN_FilterScale_32bit       ((uint8_t)0x01) /*!< One 32-bit filter */

      /** @defgroup CAN_filter_FIFO
        * @{
        */
#define CAN_Filter_FIFO0             ((uint8_t)0x00)  /*!< Filter FIFO 0 assignment for filter x */
#define CAN_Filter_FIFO1             ((uint8_t)0x01)  /*!< Filter FIFO 1 assignment for filter x *

/* CAN Master Control Register bits */
#define MCR_DBF           ((uint32_t)0x00010000) /* software master reset */

/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ       ((uint32_t)0x00000001) /* Transmit mailbox request */

/* CAN Filter Master Register bits */
#define FMR_FINIT         ((uint32_t)0x00000001) /* Filter init mode */

/* Time out for INAK bit */
#define INAK_TIMEOUT      ((uint32_t)0x0000FFFF)
/* Time out for SLAK bit */
#define SLAK_TIMEOUT      ((uint32_t)0x0000FFFF)

/* Flags in TSR register */
#define CAN_FLAGS_TSR     ((uint32_t)0x08000000)
/* Flags in RF1R register */
#define CAN_FLAGS_RF1R    ((uint32_t)0x04000000)
/* Flags in RF0R register */
#define CAN_FLAGS_RF0R    ((uint32_t)0x02000000)
/* Flags in MSR register */
#define CAN_FLAGS_MSR     ((uint32_t)0x01000000)
/* Flags in ESR register */
#define CAN_FLAGS_ESR     ((uint32_t)0x00F00000)

/* Mailboxes definition */
#define CAN_TXMAILBOX_0   ((uint8_t)0x00)
#define CAN_TXMAILBOX_1   ((uint8_t)0x01)
#define CAN_TXMAILBOX_2   ((uint8_t)0x02)

#define CAN_MODE_MASK     ((uint32_t) 0x00000003)

/**
  * @brief  CAN init structure definition
  */
typedef struct {
    uint16_t CAN_Prescaler;   /*!< Specifies the length of a time quantum.
                                   It ranges from 1 to 1024. */

    uint8_t CAN_Mode;         /*!< Specifies the CAN operating mode.
                                   This parameter can be a value of @ref CAN_operating_mode */

    uint8_t CAN_SJW;          /*!< Specifies the maximum number of time quanta
                                   the CAN hardware is allowed to lengthen or
                                   shorten a bit to perform resynchronization.
                                   This parameter can be a value of @ref CAN_synchronisation_jump_width */

    uint8_t CAN_BS1;          /*!< Specifies the number of time quanta in Bit
                                   Segment 1. This parameter can be a value of
                                   @ref CAN_time_quantum_in_bit_segment_1 */

    uint8_t CAN_BS2;          /*!< Specifies the number of time quanta in Bit Segment 2.
                                   This parameter can be a value of @ref CAN_time_quantum_in_bit_segment_2 */

    FunctionalState CAN_TTCM; /*!< Enable or disable the time triggered communication mode.
                                  This parameter can be set either to ENABLE or DISABLE. */

    FunctionalState CAN_ABOM;  /*!< Enable or disable the automatic bus-off management.
                                    This parameter can be set either to ENABLE or DISABLE. */

    FunctionalState CAN_AWUM;  /*!< Enable or disable the automatic wake-up mode.
                                    This parameter can be set either to ENABLE or DISABLE. */

    FunctionalState CAN_NART;  /*!< Enable or disable the non-automatic retransmission mode.
                                    This parameter can be set either to ENABLE or DISABLE. */

    FunctionalState CAN_RFLM;  /*!< Enable or disable the Receive FIFO Locked mode.
                                    This parameter can be set either to ENABLE or DISABLE. */

    FunctionalState CAN_TXFP;  /*!< Enable or disable the transmit FIFO priority.
                                    This parameter can be set either to ENABLE or DISABLE. */
} STM32F4_Can_InitTypeDef;

/**
  * @brief  CAN filter init structure definition
  */
typedef struct {
    uint16_t CAN_FilterIdHigh;         /*!< Specifies the filter identification number (MSBs for a 32-bit
                                                configuration, first one for a 16-bit configuration).
                                                This parameter can be a value between 0x0000 and 0xFFFF */

    uint16_t CAN_FilterIdLow;          /*!< Specifies the filter identification number (LSBs for a 32-bit
                                                configuration, second one for a 16-bit configuration).
                                                This parameter can be a value between 0x0000 and 0xFFFF */

    uint16_t CAN_FilterMaskIdHigh;     /*!< Specifies the filter mask number or identification number,
                                                according to the mode (MSBs for a 32-bit configuration,
                                                first one for a 16-bit configuration).
                                                This parameter can be a value between 0x0000 and 0xFFFF */

    uint16_t CAN_FilterMaskIdLow;      /*!< Specifies the filter mask number or identification number,
                                                according to the mode (LSBs for a 32-bit configuration,
                                                second one for a 16-bit configuration).
                                                This parameter can be a value between 0x0000 and 0xFFFF */

    uint16_t CAN_FilterFIFOAssignment; /*!< Specifies the FIFO (0 or 1) which will be assigned to the filter.
                                                This parameter can be a value of @ref CAN_filter_FIFO */

    uint8_t CAN_FilterNumber;          /*!< Specifies the filter which will be initialized. It ranges from 0 to 13. */

    uint8_t CAN_FilterMode;            /*!< Specifies the filter mode to be initialized.
                                                This parameter can be a value of @ref CAN_filter_mode */

    uint8_t CAN_FilterScale;           /*!< Specifies the filter scale.
                                                This parameter can be a value of @ref CAN_filter_scale */

    FunctionalState CAN_FilterActivation; /*!< Enable or disable the filter.
                                                This parameter can be set either to ENABLE or DISABLE. */
} CAN_FilterInitTypeDef;

/**
  * @brief  CAN Tx message structure definition
  */
typedef struct {
    uint32_t StdId;  /*!< Specifies the standard identifier.
                          This parameter can be a value between 0 to 0x7FF. */

    uint32_t ExtId;  /*!< Specifies the extended identifier.
                          This parameter can be a value between 0 to 0x1FFFFFFF. */

    uint8_t IDE;     /*!< Specifies the type of identifier for the message that
                          will be transmitted. This parameter can be a value
                          of @ref CAN_identifier_type */

    uint8_t RTR;     /*!< Specifies the type of frame for the message that will
                          be transmitted. This parameter can be a value of
                          @ref CAN_remote_transmission_request */

    uint8_t DLC;     /*!< Specifies the length of the frame that will be
                          transmitted. This parameter can be a value between
                          0 to 8 */

    uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0
                          to 0xFF. */
} CanTxMsg;

/**
  * @brief  CAN Rx message structure definition
  */
typedef struct {
    uint32_t StdId;  /*!< Specifies the standard identifier.
                          This parameter can be a value between 0 to 0x7FF. */

    uint32_t ExtId;  /*!< Specifies the extended identifier.
                          This parameter can be a value between 0 to 0x1FFFFFFF. */

    uint8_t IDE;     /*!< Specifies the type of identifier for the message that
                          will be received. This parameter can be a value of
                          @ref CAN_identifier_type */

    uint8_t RTR;     /*!< Specifies the type of frame for the received message.
                          This parameter can be a value of
                          @ref CAN_remote_transmission_request */

    uint8_t DLC;     /*!< Specifies the length of the frame that will be received.
                          This parameter can be a value between 0 to 8 */

    uint8_t Data[8]; /*!< Contains the data to be received. It ranges from 0 to
                          0xFF. */

    uint8_t FMI;     /*!< Specifies the index of the filter the message stored in
                          the mailbox passes through. This parameter can be a
                          value between 0 to 0xFF */
} CanRxMsg;


struct CANData_T {
    uint32_t *matchFilters;
    uint32_t matchFiltersSize;

    uint32_t *lowerBoundFilters;
    uint32_t *upperBoundFilters;
    uint32_t groupFiltersSize;

}CANData[2];


typedef struct {
    uint32_t TimeStampL;
    uint32_t TimeStampH;

    uint32_t Frame; // Bits 16..19: DLC - Data Length Counter
                    // Bit 30: Set if this is a RTR message
                    // Bit 31: Set if this is a 29-bit ID message
    uint32_t MsgID;	// CAN Message ID (11-bit or 29-bit)
    uint32_t DataA;	// CAN Message Data Bytes 0-3
    uint32_t DataB;	// CAN Message Data Bytes 4-7

} STM32F4_Can_Message;

enum class STM32F4_Can_Error : uint32_t {
    OverRun,
    RxOver,
    BusOff,
    ErrorPassive,
    dataReveived = 0xFF,
};

struct STM32F4_Can_Controller {
    STM32F4_Can_Message canMessage[CAN_MESSAGES_MAX];

    int32_t can_rx_count;
    int32_t can_rx_in;
    int32_t can_rx_out;

    int32_t can_send_errorEvent;

    uint32_t baudrate;

    CanTxMsg txMessage;
    CanRxMsg rxMessage;

    uint8_t sendEvent;

    STM32F4_Can_InitTypeDef initTypeDef;
    CAN_FilterInitTypeDef filterInitTypeDef;

};

static TinyCLR_Memory_Provider canLocateMemoryProvider;

void InsertionSort(uint32_t *arr, int32_t length) {
    uint32_t i, j, tmp;

    for (i = 1; i < length; i++) {
        j = i;

        while (j > 0 && arr[j - 1] > arr[j]) {
            tmp = arr[j];
            arr[j] = arr[j - 1];
            arr[j - 1] = tmp;

            j--;
        }
    }
}

bool InsertionSort2CheckOverlap(uint32_t *lowerBounds, uint32_t *upperBounds, int32_t length) {

    uint32_t i, j, tmp, tmp2;

    for (i = 1; i < length; i++) {
        j = i;

        while (j > 0 && lowerBounds[j - 1] > lowerBounds[j]) {
            tmp = lowerBounds[j]; tmp2 = upperBounds[j];
            lowerBounds[j] = lowerBounds[j - 1];	upperBounds[j] = upperBounds[j - 1];
            lowerBounds[j - 1] = tmp;	upperBounds[j - 1] = tmp2;

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

TinyCLR_Result GHAL_CAN_SetExplicitFilters(int channel, uint32_t *filters, uint32_t length) {
    uint32_t *_matchFilters;

    _matchFilters = (uint32_t*)canLocateMemoryProvider.Allocate(&canLocateMemoryProvider, length * sizeof(uint32_t));

    if (!_matchFilters)
        return TinyCLR_Result::OutOfMemory;

    memcpy(_matchFilters, filters, length * sizeof(uint32_t));

    InsertionSort(_matchFilters, length);

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        CANData[channel - 1].matchFiltersSize = length;
        CANData[channel - 1].matchFilters = _matchFilters;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result GHAL_CAN_SetGroupFilters(int channel, uint32_t *lowerBounds, uint32_t *upperBounds, uint32_t length) {
    uint32_t *_lowerBoundFilters, *_upperBoundFilters;

    _lowerBoundFilters = (uint32_t*)canLocateMemoryProvider.Allocate(&canLocateMemoryProvider, length * sizeof(uint32_t));
    _upperBoundFilters = (uint32_t*)canLocateMemoryProvider.Allocate(&canLocateMemoryProvider, length * sizeof(uint32_t));

    if (!_lowerBoundFilters || !_upperBoundFilters) {
        canLocateMemoryProvider.Free(&canLocateMemoryProvider, _lowerBoundFilters);
        canLocateMemoryProvider.Free(&canLocateMemoryProvider, _upperBoundFilters);

        return  TinyCLR_Result::OutOfMemory;
    }

    memcpy(_lowerBoundFilters, lowerBounds, length * sizeof(uint32_t));
    memcpy(_upperBoundFilters, upperBounds, length * sizeof(uint32_t));

    bool success = InsertionSort2CheckOverlap(_lowerBoundFilters, _upperBoundFilters, length);

    if (!success) {
        canLocateMemoryProvider.Free(&canLocateMemoryProvider, _lowerBoundFilters);
        canLocateMemoryProvider.Free(&canLocateMemoryProvider, _upperBoundFilters);

        return TinyCLR_Result::ArgumentInvalid;
    }

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        CANData[channel - 1].groupFiltersSize = length;
        CANData[channel - 1].lowerBoundFilters = _lowerBoundFilters;
        CANData[channel - 1].upperBoundFilters = _upperBoundFilters;
    }

    return TinyCLR_Result::Success;;
}

/**
  * @brief  Initializes the CAN peripheral according to the specified
  *         parameters in the CAN_InitStruct.
  * @param  CANx: where x can be 1 or 2 to select the CAN peripheral.
  * @param  CAN_InitStruct: pointer to a STM32F4_Can_InitTypeDef structure that contains
  *         the configuration information for the CAN peripheral.
  * @retval Constant indicates initialization succeed which will be
  *         CAN_InitStatus_Failed or CAN_InitStatus_Success.
  */
uint8_t STM32F4_Can_Initialize(CAN_TypeDef* CANx, STM32F4_Can_InitTypeDef* CAN_InitStruct) {
    uint8_t InitStatus = CAN_InitStatus_Failed;
    uint32_t wait_ack = 0x00000000;

    /* Exit from sleep mode */
    CANx->MCR &= (~(uint32_t)CAN_MCR_SLEEP);

    /* Request initialisation */
    CANx->MCR |= CAN_MCR_INRQ;

    /* Wait the acknowledge */
    while (((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT)) {
        wait_ack++;
    }

    /* Check acknowledge */
    if ((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) {
        InitStatus = CAN_InitStatus_Failed;
    }
    else {
        /* Set the time triggered communication mode */
        if (CAN_InitStruct->CAN_TTCM == ENABLE) {
            CANx->MCR |= CAN_MCR_TTCM;
        }
        else {
            CANx->MCR &= ~(uint32_t)CAN_MCR_TTCM;
        }

        /* Set the automatic bus-off management */
        if (CAN_InitStruct->CAN_ABOM == ENABLE) {
            CANx->MCR |= CAN_MCR_ABOM;
        }
        else {
            CANx->MCR &= ~(uint32_t)CAN_MCR_ABOM;
        }

        /* Set the automatic wake-up mode */
        if (CAN_InitStruct->CAN_AWUM == ENABLE) {
            CANx->MCR |= CAN_MCR_AWUM;
        }
        else {
            CANx->MCR &= ~(uint32_t)CAN_MCR_AWUM;
        }

        /* Set the no automatic retransmission */
        if (CAN_InitStruct->CAN_NART == ENABLE) {
            CANx->MCR |= CAN_MCR_NART;
        }
        else {
            CANx->MCR &= ~(uint32_t)CAN_MCR_NART;
        }

        /* Set the receive FIFO locked mode */
        if (CAN_InitStruct->CAN_RFLM == ENABLE) {
            CANx->MCR |= CAN_MCR_RFLM;
        }
        else {
            CANx->MCR &= ~(uint32_t)CAN_MCR_RFLM;
        }

        /* Set the transmit FIFO priority */
        if (CAN_InitStruct->CAN_TXFP == ENABLE) {
            CANx->MCR |= CAN_MCR_TXFP;
        }
        else {
            CANx->MCR &= ~(uint32_t)CAN_MCR_TXFP;
        }

        /* Set the bit timing register */
        CANx->BTR = (uint32_t)((uint32_t)CAN_InitStruct->CAN_Mode << 30) | \
            ((uint32_t)CAN_InitStruct->CAN_SJW << 24) | \
            ((uint32_t)CAN_InitStruct->CAN_BS1 << 16) | \
            ((uint32_t)CAN_InitStruct->CAN_BS2 << 20) | \
            ((uint32_t)CAN_InitStruct->CAN_Prescaler - 1);

        /* Request leave initialisation */
        CANx->MCR &= ~(uint32_t)CAN_MCR_INRQ;

        /* Wait the acknowledge */
        wait_ack = 0;

        while (((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT)) {
            wait_ack++;
        }

        /* ...and check acknowledged */
        if ((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {
            InitStatus = CAN_InitStatus_Failed;
        }
        else {
            InitStatus = CAN_InitStatus_Success;
        }
    }

    /* At this step, return the status of initialization */
    return InitStatus;
}


/**
  * @brief  Configures the CAN reception filter according to the specified
  *         parameters in the CAN_FilterInitStruct.
  * @param  CAN_FilterInitStruct: pointer to a CAN_FilterInitTypeDef structure that
  *         contains the configuration information.
  * @retval None
  */
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct) {
    uint32_t filter_number_bit_pos = 0;
    /* Check the parameters */
    // TQD assert_param(IS_CAN_FILTER_NUMBER(CAN_FilterInitStruct->CAN_FilterNumber));
    // TQD assert_param(IS_CAN_FILTER_MODE(CAN_FilterInitStruct->CAN_FilterMode));
    // TQD assert_param(IS_CAN_FILTER_SCALE(CAN_FilterInitStruct->CAN_FilterScale));
    // TQD assert_param(IS_CAN_FILTER_FIFO(CAN_FilterInitStruct->CAN_FilterFIFOAssignment));
    // TQD assert_param(IS_FUNCTIONAL_STATE(CAN_FilterInitStruct->CAN_FilterActivation));

    filter_number_bit_pos = ((uint32_t)1) << CAN_FilterInitStruct->CAN_FilterNumber;

    /* Initialisation mode for the filter */
    CAN1->FMR |= FMR_FINIT;

    /* Filter Deactivation */
    CAN1->FA1R &= ~(uint32_t)filter_number_bit_pos;

    /* Filter Scale */
    if (CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_16bit) {
        /* 16-bit scale for the filter */
        CAN1->FS1R &= ~(uint32_t)filter_number_bit_pos;

        /* First 16-bit identifier and First 16-bit mask */
        /* Or First 16-bit identifier and Second 16-bit identifier */
        CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 =
            ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow) << 16) |
            (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdLow);

        /* Second 16-bit identifier and Second 16-bit mask */
        /* Or Third 16-bit identifier and Fourth 16-bit identifier */
        CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 =
            ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
            (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdHigh);
    }

    if (CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_32bit) {
        /* 32-bit scale for the filter */
        CAN1->FS1R |= filter_number_bit_pos;
        /* 32-bit identifier or First 32-bit identifier */
        CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 =
            ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdHigh) << 16) |
            (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdLow);
        /* 32-bit mask or Second 32-bit identifier */
        CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 =
            ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
            (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow);
    }

    /* Filter Mode */
    if (CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdMask) {
        /*Id/Mask mode for the filter*/
        CAN1->FM1R &= ~(uint32_t)filter_number_bit_pos;
    }
    else /* CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdList */
    {
        /*Identifier list mode for the filter*/
        CAN1->FM1R |= (uint32_t)filter_number_bit_pos;
    }

    /* Filter FIFO assignment */
    if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_Filter_FIFO0) {
        /* FIFO 0 assignation for the filter */
        CAN1->FFA1R &= ~(uint32_t)filter_number_bit_pos;
    }

    if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_Filter_FIFO1) {
        /* FIFO 1 assignation for the filter */
        CAN1->FFA1R |= (uint32_t)filter_number_bit_pos;
    }

    /* Filter activation */
    if (CAN_FilterInitStruct->CAN_FilterActivation == ENABLE) {
        CAN1->FA1R |= filter_number_bit_pos;
    }

    /* Leave the initialisation mode for the filter */
    CAN1->FMR &= ~FMR_FINIT;
}

uint32_t STM32F4_Can_CalculateBaudrate(int32_t propagation, int32_t phase1, int32_t phase2, int32_t brp, int32_t synchronizationJumpWidth, int8_t useMultiBitSampling) {
    uint32_t baudrate = (((synchronizationJumpWidth - 1) << 24) | ((phase2 - 1) << 20) | ((phase1 - 1) << 16) | brp);
    return baudrate;

}

static TinyCLR_Can_Provider canProvider;
static TinyCLR_Api_Info canApi;

static const STM32F4_Gpio_Pin g_STM32F4_Can_Tx_Pins[] = STM32F4_CAN_TX_PINS;
static const STM32F4_Gpio_Pin g_STM32F4_Can_Rx_Pins[] = STM32F4_CAN_RX_PINS;

static const int TOTAL_CAN_CONTROLLERS = SIZEOF_ARRAY(g_STM32F4_Can_Tx_Pins);

static STM32F4_Can_Controller canController[TOTAL_CAN_CONTROLLERS];

const TinyCLR_Api_Info* STM32F4_Can_GetApi() {

    return nullptr;
}

TinyCLR_Result STM32F4_Can_Acquire(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t channel = self->Index;

    if (!STM32F4_GpioInternal_OpenPin(g_STM32F4_Can_Tx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;

    if (!STM32F4_GpioInternal_OpenPin(g_STM32F4_Can_Rx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;
    // set pin as analog
    STM32F4_GpioInternal_ConfigurePin(g_STM32F4_Can_Tx_Pins[channel].number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, g_STM32F4_Can_Tx_Pins[channel].alternateFunction);
    STM32F4_GpioInternal_ConfigurePin(g_STM32F4_Can_Rx_Pins[channel].number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, g_STM32F4_Can_Rx_Pins[channel].alternateFunction);

    RCC->APB1ENR |= ((channel == 0) ? RCC_APB1ENR_CAN1EN : RCC_APB1ENR_CAN2EN);

    /* CAN cell init */
    canController[channel].initTypeDef.CAN_TTCM = DISABLE;
    canController[channel].initTypeDef.CAN_ABOM = DISABLE;
    canController[channel].initTypeDef.CAN_AWUM = DISABLE;
    canController[channel].initTypeDef.CAN_NART = DISABLE;
    canController[channel].initTypeDef.CAN_RFLM = DISABLE;
    canController[channel].initTypeDef.CAN_TXFP = DISABLE;
    canController[channel].initTypeDef.CAN_Mode = CAN_Mode_Normal;
    // canController[channel].initTypeDef.CAN_SJW = CAN_SJW_1tq;

    // CAN Baudrate = 1 MBps (CAN clocked at 30 MHz) */
    // canController[channel].initTypeDef.CAN_BS1 = CAN_BS1_12tq;
    // canController[channel].initTypeDef.CAN_BS2 = CAN_BS2_8tq;
    // canController[channel].initTypeDef.CAN_Prescaler = 4; // 5-1 = 4

    canController[channel].initTypeDef.CAN_SJW = ((canController[channel].baudrate >> 24) & 0x03);
    canController[channel].initTypeDef.CAN_BS2 = ((canController[channel].baudrate >> 20) & 0x07);
    canController[channel].initTypeDef.CAN_BS1 = ((canController[channel].baudrate >> 16) & 0x0F);
    canController[channel].initTypeDef.CAN_Prescaler = ((canController[channel].baudrate >> 0) & 0x3FF);

    STM32F4_Can_Initialize(channel == 0 ? CAN1 : CAN2, &canController[channel].initTypeDef);



    canController[channel].filterInitTypeDef.CAN_FilterNumber = (channel == 1 ? 0 : 14);

    canController[channel].filterInitTypeDef.CAN_FilterMode = CAN_FilterMode_IdMask;
    canController[channel].filterInitTypeDef.CAN_FilterScale = CAN_FilterScale_32bit;
    canController[channel].filterInitTypeDef.CAN_FilterIdHigh = 0x0000;
    canController[channel].filterInitTypeDef.CAN_FilterIdLow = 0x0000;
    canController[channel].filterInitTypeDef.CAN_FilterMaskIdHigh = 0x0000;
    canController[channel].filterInitTypeDef.CAN_FilterMaskIdLow = 0x0000;
    canController[channel].filterInitTypeDef.CAN_FilterFIFOAssignment = 0;
    canController[channel].filterInitTypeDef.CAN_FilterActivation = ENABLE;

    CAN_FilterInit(&canController[channel].filterInitTypeDef);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Can_Release(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t channel = self->Index;

    TinyCLR_Result releasePin = STM32F4_Gpio_ReleasePin(nullptr, g_STM32F4_Can_Tx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    releasePin = STM32F4_Gpio_ReleasePin(nullptr, g_STM32F4_Can_Rx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    // free pin
    STM32F4_GpioInternal_ClosePin(g_STM32F4_Can_Tx_Pins[channel].number);
    STM32F4_GpioInternal_ClosePin(g_STM32F4_Can_Rx_Pins[channel].number);

    RCC->APB1ENR &= ((channel == 0) ? ~RCC_APB1ENR_CAN1EN : ~RCC_APB1ENR_CAN2EN);






    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Can_Reset(const TinyCLR_Can_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Can_PostMessage(const TinyCLR_Can_Provider* self, uint32_t arbID, uint32_t flags, uint8_t *data) {

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Can_GetMessage(const TinyCLR_Can_Provider* self, uint32_t * arbID, uint32_t *flags, uint64_t *ts, uint8_t *data) {

    return TinyCLR_Result::Success;
}
#endif // INCLUDE_CAN
