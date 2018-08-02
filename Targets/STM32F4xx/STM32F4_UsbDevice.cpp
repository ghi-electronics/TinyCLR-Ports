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

#include <string.h>
#include "STM32F4.h"
#include "../../Drivers/USBClient/USBClient.h"

#define OTG_FS_BASE           (0x50000000)
#define OTG_FS                ((OTG_TypeDef *) OTG_FS_BASE)

#define OTG_GUSBCFG_PHYSEL    (1<<6)
#define OTG_GUSBCFG_PHYLPCS   (1<<15)
#define OTG_GUSBCFG_FDMOD     (1<<30)

#define OTG_GCCFG_PWRDWN      (1<<16)
#define OTG_GCCFG_VBUSASEN    (1<<18)
#define OTG_GCCFG_VBUSBSEN    (1<<19)
#define OTG_GCCFG_SOFOUTEN    (1<<20)
#define OTG_GCCFG_NOVBUSSENS  (1<<21)

#define OTG_GAHBCFG_GINTMSK   (1<<0)
#define OTG_GAHBCFG_TXFELVL   (1<<7)

#define OTG_GINTSTS_MMIS      (1<<1)
#define OTG_GINTSTS_OTGINT    (1<<2)
#define OTG_GINTSTS_RXFLVL    (1<<4)
#define OTG_GINTSTS_USBSUSP   (1<<11)
#define OTG_GINTSTS_USBRST    (1<<12)
#define OTG_GINTSTS_ENUMDNE   (1<<13)
#define OTG_GINTSTS_IEPINT    (1<<18)
#define OTG_GINTSTS_OEPINT    (1<<19)
#define OTG_GINTSTS_SRQINT    (1<<30)
#define OTG_GINTSTS_WKUPINT   (1U<<31)

#define OTG_GINTMSK_MMISM     (1<<1)
#define OTG_GINTMSK_OTGINT    (1<<2)
#define OTG_GINTMSK_RXFLVLM   (1<<4)
#define OTG_GINTMSK_USBSUSPM  (1<<11)
#define OTG_GINTMSK_USBRST    (1<<12)
#define OTG_GINTMSK_ENUMDNEM  (1<<13)
#define OTG_GINTMSK_IEPINT    (1<<18)
#define OTG_GINTMSK_OEPINT    (1<<19)
#define OTG_GINTMSK_SRQIM     (1<<30)
#define OTG_GINTMSK_WUIM      (1U<<31)

#define OTG_GRSTCTL_CSRST     (1<<0)
#define OTG_GRSTCTL_RXFFLSH   (1<<4)
#define OTG_GRSTCTL_TXFFLSH   (1<<5)
#define OTG_GRSTCTL_TXFNUM    (0x1F<<6)
#define OTG_GRSTCTL_TXF_ALL   (0x10<<6)

#define OTG_DCFG_DSPD         (0x3<<0)
#define OTG_DCFG_DAD          (0x7F<<4)

#define OTG_DCTL_RWUSIG       (1<<0)
#define OTG_DCTL_SDIS         (1<<1)
#define OTG_DCTL_POPRGDNE     (1<<11)

#define OTG_GRXSTSP_EPNUM     (0x0F<<0)
#define OTG_GRXSTSP_CHNUM     (0x0F<<0)
#define OTG_GRXSTSP_BCNT      (0x7FF<<4)
#define OTG_GRXSTSP_DPID      (0x03<<15)
#define OTG_GRXSTSP_PKTSTS    (0x0F<<17)
#define OTG_GRXSTSP_PKTSTS_GN (0x01<<17) // global OUT NAK
#define OTG_GRXSTSP_PKTSTS_PR (0x02<<17) // packet received
#define OTG_GRXSTSP_PKTSTS_DC (0x03<<17) // data transaction completed
#define OTG_GRXSTSP_PKTSTS_SC (0x04<<17) // setup stage completed
#define OTG_GRXSTSP_PKTSTS_TE (0x05<<17) // toggle error
#define OTG_GRXSTSP_PKTSTS_SR (0x06<<17) // setup data received
#define OTG_GRXSTSP_PKTSTS_CH (0x07<<17) // channel haltet
#define OTG_GRXSTSP_FRMNUM    (0x0F<<21)

#define OTG_DIEPMSK_XFRCM     (1<<0) // transfer completed
#define OTG_DIEPMSK_TOM       (1<<3) // timeout

#define OTG_DOEPMSK_XFRCM     (1<<0) // transfer completed
#define OTG_DOEPMSK_STUPM     (1<<3) // setup phase done

#define OTG_DIEPINT_XFRC      (1<<0) // transfer completed
#define OTG_DIEPINT_TOC       (1<<3) // timeout

#define OTG_DOEPINT_XFRC      (1<<0) // transfer completed
#define OTG_DOEPINT_STUP      (1<<3) // setup phase done

#define OTG_DIEPCTL_USBAEP    (1<<15)
#define OTG_DIEPCTL_STALL     (1<<21)
#define OTG_DIEPCTL_CNAK      (1<<26)
#define OTG_DIEPCTL_SNAK      (1<<27)
#define OTG_DIEPCTL_SD0PID    (1<<28)
#define OTG_DIEPCTL_EPDIS     (1<<30)
#define OTG_DIEPCTL_EPENA     (1U<<31)

#define OTG_DOEPCTL_USBAEP    (1<<15)
#define OTG_DOEPCTL_STALL     (1<<21)
#define OTG_DOEPCTL_CNAK      (1<<26)
#define OTG_DOEPCTL_SNAK      (1<<27)
#define OTG_DOEPCTL_SD0PID    (1<<28)
#define OTG_DOEPCTL_EPDIS     (1<<30)
#define OTG_DOEPCTL_EPENA     (1U<<31)

#define OTG_DIEPTSIZ_PKTCNT   (3<<19)
#define OTG_DIEPTSIZ_PKTCNT_1 (1<<19)

#define OTG_DOEPTSIZ_PKTCNT   (3<<19)
#define OTG_DOEPTSIZ_PKTCNT_1 (1<<19)
#define OTG_DOEPTSIZ_STUPCNT  (3<<29)

#define STM32F4_USB_FS_USE_ID_PIN 0
#define STM32F4_USB_FS_USE_VB_PIN 0

// use OTG Full Speed
#define STM32F4_USB_FS_ID 0

#define STM32F4_USB_USE_ID_PIN(c) STM32F4_USB_FS_USE_ID_PIN
#define STM32F4_USB_USE_VB_PIN(c) STM32F4_USB_FS_USE_VB_PIN

#define USB_MAX_BUFFERS (USB_FS_MAX_EP_COUNT - 1)

// FIFO sizes (in 32 bit words)
#define USB_RXFIFO_SIZE  64
#define USB_TX0FIFO_SIZE 64
#define USB_TXnFIFO_SIZE 64

// PHY turnaround time
// (4 AHB clocks + 1 Phy clock in Phy clocks)
#define STM32F4_USB_TRDT ((4 * 48000000 - 1) / STM32F4_AHB_CLOCK_HZ + 2)

#define USB_OTG_NUM_FIFOS                8
#define USB_OTG_NUM_CHANNELS            16

typedef struct {
    // global registers
    __IO uint32_t GOTGCTL;
    __IO uint32_t GOTGINT;
    __IO uint32_t GAHBCFG;
    __IO uint32_t GUSBCFG;
    __IO uint32_t GRSTCTL;
    __IO uint32_t GINTSTS;
    __IO uint32_t GINTMSK;
    __IO uint32_t GRXSTSR;
    __IO uint32_t GRXSTSP;
    __IO uint32_t GRXFSIZ;
    union {
        __IO uint32_t HNPTXFSIZ;
        __IO uint32_t DIEPTXF0;
    };
    __IO uint32_t HNPTXSTS;
    uint32_t Res1[2];
    __IO uint32_t GCCFG;
    __IO uint32_t CID;
    uint32_t Res2[48];
    union {
        __IO uint32_t HPTXFSIZ;
        __IO uint32_t DIEPTXF[USB_OTG_NUM_FIFOS];
    };
    uint32_t Res3[184];
    // host mode registers
    __IO uint32_t HCFG;
    __IO uint32_t HFIR;
    __IO uint32_t HFNUM;
    uint32_t Res4;
    __IO uint32_t HPTXSTS;
    __IO uint32_t HAINT;
    __IO uint32_t HAINTMSK;
    uint32_t Res5[9];
    __IO uint32_t HPRT;
    uint32_t Res6[47];
    struct {
        __IO uint32_t CHAR;
        uint32_t Res7;
        __IO uint32_t INT;
        __IO uint32_t INTMSK;
        __IO uint32_t TSIZ;
        uint32_t Res8[3];
    } HC[USB_OTG_NUM_CHANNELS];
    uint32_t Res9[64];
    // device mode registers
    __IO uint32_t DCFG;
    __IO uint32_t DCTL;
    __IO uint32_t DSTS;
    uint32_t Res10;
    __IO uint32_t DIEPMSK;
    __IO uint32_t DOEPMSK;
    __IO uint32_t DAINT;
    __IO uint32_t DAINTMSK;
    uint32_t Res11[2];
    __IO uint32_t DVBUSDIS;
    __IO uint32_t DVBUSPULSE;
    uint32_t Res12;
    __IO uint32_t DIEPEMPMSK;
    uint32_t Res13[50];
    struct {
        __IO uint32_t CTL;
        uint32_t Res14;
        __IO uint32_t INT;
        uint32_t Res15;
        __IO uint32_t TSIZ;
        uint32_t Res16;
        __IO uint32_t TXFSTS;
        uint32_t Res17;
    } DIEP[USB_OTG_NUM_CHANNELS];
    struct {
        __IO uint32_t CTL;
        uint32_t Res18;
        __IO uint32_t INT;
        uint32_t Res19;
        __IO uint32_t TSIZ;
        uint32_t Res20[3];
    } DOEP[USB_OTG_NUM_CHANNELS];
    uint32_t Res21[64];
    // power and clock gating
    __IO uint32_t PCGCCTL;
    uint32_t Res22[127];
    // FIFO regions
    __IO uint32_t DFIFO[USB_OTG_NUM_FIFOS][1024];
}
OTG_TypeDef;

typedef struct {
    UsClientState* usClientState;

    uint8_t     previousDeviceState;
    uint16_t    endpointType;

} UsbDeviceController;

static const STM32F4_Gpio_Pin usbDeviceDmPins[] = STM32F4_USB_DM_PINS;
static const STM32F4_Gpio_Pin usbDeviceDpPins[] = STM32F4_USB_DP_PINS;
static const STM32F4_Gpio_Pin usbDeviceVbusPins[] = STM32F4_USB_VB_PINS;
static const STM32F4_Gpio_Pin usbDeviceIdPins[] = STM32F4_USB_ID_PINS;

void STM32F4_UsbDevice_ProtectPins(int32_t controller, bool On);
void STM32F4_UsbDevice_Interrupt(void* param);

/* usClientState variables for the controllers */
static UsbDeviceController usbDeviceControllers[STM32F4_TOTAL_USB_CONTROLLERS];

void STM32F4_UsbDevice_AddApi(const TinyCLR_Api_Manager* apiManager) {
    TinyCLR_UsbClient_AddApi(apiManager);

}
const TinyCLR_Api_Info* STM32F4_UsbDevice_GetRequiredApi() {
    return TinyCLR_UsbClient_GetRequiredApi();
}
void STM32F4_UsbDevice_Reset() {
    return TinyCLR_UsbClient_Reset(STM32F4_USB_FS_ID);
}

void STM32F4_UsbDevice_InitializeConfiguration(UsClientState *usClientState) {
    int32_t controller = STM32F4_USB_FS_ID;

    if (usClientState != nullptr) {
        usClientState->controllerIndex = controller;

        usClientState->maxFifoPacketCount = STM32F4_USB_PACKET_FIFO_COUNT;
        usClientState->totalEndpointsCount = STM32F4_USB_ENDPOINT_COUNT;
        usClientState->totalPipesCount = STM32F4_USB_PIPE_COUNT;

        usbDeviceControllers[controller].usClientState = usClientState;

        usbDeviceControllers[controller].endpointType = 0;
        for (auto i = 0; i < usbDeviceControllers[controller].usClientState->deviceDescriptor.Configurations->Interfaces->EndpointCount; i++) {
            TinyCLR_UsbClient_EndpointDescriptor  *ep = (TinyCLR_UsbClient_EndpointDescriptor*)&usbDeviceControllers[controller].usClientState->deviceDescriptor.Configurations->Interfaces->Endpoints[i];

            auto idx = ep->Address & 0x0F;

            usbDeviceControllers[controller].endpointType |= (ep->Attributes & 3) << (idx * 2);
        }
    }
}

bool STM32F4_UsbDevice_Initialize(UsClientState* usClientState) {

    if (usClientState == nullptr)
        return false;

    int32_t controller = usClientState->controllerIndex;

    auto& dp = usbDeviceDpPins[controller];
    auto& dm = usbDeviceDmPins[controller];
    auto& id = usbDeviceIdPins[controller];

    if (!STM32F4_GpioInternal_OpenPin(dp.number) || !STM32F4_GpioInternal_OpenPin(dm.number))
        return false;

    if (STM32F4_USB_USE_ID_PIN(controller) && !STM32F4_GpioInternal_OpenPin(id.number))
        return false;

    // Enable USB clock
    // FS on AHB2
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Detach usb port for a while to enforce re-initialization
    OTG->DCTL = OTG_DCTL_SDIS; // soft disconnect

    OTG->GAHBCFG = OTG_GAHBCFG_TXFELVL;     // int32_t on TxFifo completely empty, int32_t off
    OTG->GUSBCFG = OTG_GUSBCFG_FDMOD        // force device mode
        | STM32F4_USB_TRDT << 10   // turnaround time
        | OTG_GUSBCFG_PHYSEL;      // internal PHY

    OTG->GCCFG = OTG_GCCFG_VBUSBSEN       // B device Vbus sensing
        | OTG_GCCFG_PWRDWN;        // transceiver enabled

    OTG->DCFG |= OTG_DCFG_DSPD;           // device speed = HS

    if (STM32F4_USB_USE_VB_PIN(controller) == 0) { // no Vbus pin
        OTG->GCCFG |= OTG_GCCFG_NOVBUSSENS; // disable vbus sense
    }

    STM32F4_Time_Delay(nullptr, 1000); // asure host recognizes reattach

    // setup hardware
    STM32F4_UsbDevice_ProtectPins(controller, true);

    STM32F4_InterruptInternal_Activate(OTG_FS_IRQn, (uint32_t*)&STM32F4_UsbDevice_Interrupt, 0);
    STM32F4_InterruptInternal_Activate(OTG_FS_WKUP_IRQn, (uint32_t*)&STM32F4_UsbDevice_Interrupt, 0);

    // allow interrupts
    OTG->GINTSTS = 0xFFFFFFFF;           // clear all interrupts
    OTG->GINTMSK = OTG_GINTMSK_USBRST;   // enable reset only
    OTG->DIEPEMPMSK = 0;                 // disable Tx FIFO empty interrupts
    OTG->GAHBCFG |= OTG_GAHBCFG_GINTMSK; // gloabl interrupt enable

#if defined(STM32F413xx)
    OTG->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;
#endif

    return true;
}

bool STM32F4_UsbDevice_Uninitialize(UsClientState* usClientState) {
    STM32F4_InterruptInternal_Deactivate(OTG_FS_WKUP_IRQn);
    STM32F4_InterruptInternal_Deactivate(OTG_FS_IRQn);

    RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;

    if (usClientState != nullptr) {
        STM32F4_UsbDevice_ProtectPins(usClientState->controllerIndex, false);
        usClientState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
    }

    return true;
}

void STM32F4_UsbDevice_ResetEvent(OTG_TypeDef* OTG, UsClientState* usClientState) {
    // reset interrupts and FIFOs
    OTG->GINTSTS = 0xFFFFFFFF; // clear global interrupts
    OTG->GRXFSIZ = USB_RXFIFO_SIZE; // Rx Fifo
    OTG->DIEPTXF0 = (USB_TX0FIFO_SIZE << 16) | USB_RXFIFO_SIZE; // Tx Fifo 0
    uint32_t addr = USB_RXFIFO_SIZE + USB_TX0FIFO_SIZE;
    for (auto i = 0; i < usClientState->totalEndpointsCount; i++) {
        OTG->DIEPTXF[i] = (USB_TXnFIFO_SIZE << 16) | addr; // Tx Fifo i
        addr += USB_TXnFIFO_SIZE;
        OTG->DIEP[i].INT = 0xFF; // clear endpoint interrupts
        OTG->DOEP[i].INT = 0xFF;
        OTG->DIEP[i].CTL = OTG_DIEPCTL_EPDIS; // deactivate endpoint
        OTG->DOEP[i].CTL = OTG_DOEPCTL_EPDIS;
    }

    // flush FIFOs
    OTG->GRSTCTL = OTG_GRSTCTL_RXFFLSH | OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_TXF_ALL;

    // configure control endpoint
    OTG->DIEP[0].CTL = OTG_DIEPCTL_USBAEP; // Tx FIFO num = 0, max packet size = 64
    OTG->DOEP[0].CTL = OTG_DOEPCTL_USBAEP;
    OTG->DIEP[0].TSIZ = 0;
    OTG->DOEP[0].TSIZ = OTG_DOEPTSIZ_STUPCNT; // up to 3 setup packets

    // configure data endpoints
    uint32_t intMask = 0x00010001; // ep0 interrupts;
    uint32_t eptype = usbDeviceControllers[usClientState->controllerIndex].endpointType >> 2; // endpoint types (2 bits / endpoint)
    uint32_t i = 1, bit = 2;
    while (eptype) {
        uint32_t type = eptype & 3;
        if (type != 0) { // data endpoint
            uint32_t ctrl = OTG_DIEPCTL_SD0PID | OTG_DIEPCTL_USBAEP;
            ctrl |= type << 18; // endpoint type
            ctrl |= usClientState->maxEndpointsPacketSize[i]; // packet size
            if (usClientState->isTxQueue[i]) { // Tx (in) endpoint
                ctrl |= OTG_DIEPCTL_SNAK; // disable tx endpoint
                ctrl |= i << 22; // Tx FIFO number
                OTG->DIEP[i].CTL = ctrl; // configure in endpoint
                intMask |= bit; // enable in interrupt
            }
            else { // Rx (out) endpoint
                // Rx endpoints must be enabled here
                // Enabling after Set_Configuration does not work correctly
                OTG->DOEP[i].TSIZ = OTG_DOEPTSIZ_PKTCNT_1 | usClientState->maxEndpointsPacketSize[i];
                ctrl |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK; // enable rx endpoint
                OTG->DOEP[i].CTL = ctrl; // configure out endpoint
                intMask |= bit << 16; // enable out interrupt
            }
        }
        i++;
        eptype >>= 2;
        bit <<= 1;
    }

    // enable interrupts
    OTG->DIEPMSK = OTG_DIEPMSK_XFRCM; // transfer complete
    OTG->DOEPMSK = OTG_DOEPMSK_XFRCM | OTG_DOEPMSK_STUPM; // setup stage done
    OTG->DAINTMSK = intMask;   // enable ep interrupts
    OTG->GINTMSK = OTG_GINTMSK_OEPINT | OTG_GINTMSK_IEPINT | OTG_GINTMSK_RXFLVLM
        | OTG_GINTMSK_USBRST | OTG_GINTMSK_USBSUSPM | OTG_GINTMSK_WUIM;

    OTG->DCFG &= ~OTG_DCFG_DAD; // reset device address

    /* clear all flags */
    TinyCLR_UsbClient_ClearEvent(usClientState, 0xFFFFFFFF); // clear all events on all endpoints

    usClientState->firstGetDescriptor = true;

    usClientState->deviceState = USB_DEVICE_STATE_DEFAULT;
    usClientState->address = 0;
    TinyCLR_UsbClient_StateCallback(usClientState);
}

void STM32F4_UsbDevice_EndpointRxInterrupt(OTG_TypeDef* OTG, UsClientState* usClientState, uint32_t ep, uint32_t count) {
    uint32_t* pd;

    bool disableRx = false;

    if (ep == 0) { // control endpoint
        pd = (uint32_t*)usClientState->controlEndpointBuffer;
        usClientState->ptrData = (uint8_t*)pd;
        usClientState->dataSize = count;
    }
    else { // data endpoint
        USB_PACKET64* Packet64 = TinyCLR_UsbClient_RxEnqueue(usClientState, ep, disableRx);

        if (disableRx) return;

        pd = (uint32_t*)Packet64->Buffer;
        Packet64->Size = count;
    }

    // read data
    uint32_t volatile* ps = OTG->DFIFO[ep];
    for (int32_t c = count; c > 0; c -= 4) {
        *pd++ = *ps;
    }
}

void STM32F4_UsbDevice_EndpointInInterrupt(OTG_TypeDef* OTG, UsClientState* usClientState, uint32_t ep) {
    uint32_t bits = OTG->DIEP[ep].INT;
    if (bits & OTG_DIEPINT_XFRC) { // transfer completed
        OTG->DIEP[ep].INT = OTG_DIEPINT_XFRC; // clear interrupt
    }

    if (!(OTG->DIEP[ep].CTL & OTG_DIEPCTL_EPENA)) { // Tx idle
        uint32_t* ps = 0;
        uint32_t count;

        if (ep == 0) { // control endpoint
            if (usClientState->dataCallback) { // data to send
                usClientState->dataCallback(usClientState);  // this call can't fail
                ps = (uint32_t*)usClientState->ptrData;
                count = usClientState->dataSize;
            }
        }
        else if (usClientState->queues[ep] != 0 && usClientState->isTxQueue[ep]) { // Tx data endpoint

            USB_PACKET64* Packet64 = TinyCLR_UsbClient_TxDequeue(usClientState, ep);

            if (Packet64) {  // data to send
                ps = (uint32_t*)Packet64->Buffer;
                count = Packet64->Size;
            }
        }

        if (ps) { // data to send
            // enable endpoint
            OTG->DIEP[ep].TSIZ = OTG_DIEPTSIZ_PKTCNT_1 | count;
            OTG->DIEP[ep].CTL |= OTG_DIEPCTL_EPENA | OTG_DIEPCTL_CNAK;

            // write data
            uint32_t volatile* pd = OTG->DFIFO[ep];
            for (int32_t c = count; c > 0; c -= 4) {
                *pd = *ps++;
            }
        }
        else { // no data
            // disable endpoint
            OTG->DIEP[ep].CTL |= OTG_DIEPCTL_SNAK;
        }
    }
}

void STM32F4_UsbDevice_HandleSetup(OTG_TypeDef* OTG, UsClientState* usClientState) {
    /* send last setup packet to the upper layer */
    uint8_t result = TinyCLR_UsbClient_ControlCallback(usClientState);

    switch (result) {

    case USB_STATE_ADDRESS:
        /* upper layer needs us to change the address */
        OTG->DCFG |= usClientState->address << 4; // set device address
        break;

    case USB_STATE_DONE:
        usClientState->dataCallback = 0;
        break;

    case USB_STATE_STALL:
        // setup packet failed to process successfully
        // set stall condition on the control endpoint
        OTG->DIEP[0].CTL |= OTG_DIEPCTL_STALL;
        OTG->DOEP[0].CTL |= OTG_DOEPCTL_STALL;

        // ********** skip rest of function **********
        return;
    }

    // check ep0 for replies
    STM32F4_UsbDevice_EndpointInInterrupt(OTG, usClientState, 0);

    // check all Tx endpoints after configuration setup
    if (result == USB_STATE_CONFIGURATION) {
        for (int32_t ep = 1; ep < usClientState->totalEndpointsCount; ep++) {
            if (usClientState->queues[ep] && usClientState->isTxQueue[ep]) {
                STM32F4_UsbDevice_EndpointInInterrupt(OTG, usClientState, ep);
            }
        }
    }
}

void STM32F4_UsbDevice_EndpointOutInterrupt(OTG_TypeDef* OTG, UsClientState* usClientState, uint32_t ep) {
    uint32_t bits = OTG->DOEP[ep].INT;
    if (bits & OTG_DOEPINT_XFRC) { // transfer completed
        OTG->DOEP[ep].INT = OTG_DOEPINT_XFRC; // clear interrupt
    }

    if (bits & OTG_DOEPINT_STUP) { // setup phase done
        OTG->DOEP[ep].INT = OTG_DOEPINT_STUP; // clear interrupt
    }

    if (ep == 0) { // control endpoint
        // enable endpoint
        OTG->DOEP[0].TSIZ = OTG_DOEPTSIZ_STUPCNT | OTG_DOEPTSIZ_PKTCNT_1 | usClientState->maxEndpointsPacketSize[0];
        OTG->DOEP[0].CTL |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK;
        // Handle Setup data in upper layer
        STM32F4_UsbDevice_HandleSetup(OTG, usClientState);
    }
    else if (TinyCLR_UsbClient_CanReceivePackage(usClientState, ep)) {
        // enable endpoint
        OTG->DOEP[ep].TSIZ = OTG_DOEPTSIZ_PKTCNT_1 | usClientState->maxEndpointsPacketSize[ep];
        OTG->DOEP[ep].CTL |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK;
    }
    else {
        // disable endpoint
        OTG->DOEP[ep].CTL |= OTG_DOEPCTL_SNAK;
    }
}

void STM32F4_UsbDevice_Interrupt(void* param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    OTG_TypeDef* OTG = OTG_FS;

    int32_t controller = STM32F4_USB_FS_ID;

    UsClientState* usClientState = usbDeviceControllers[controller].usClientState;

    uint32_t intPend = OTG->GINTSTS; // get pending bits

    while (intPend & OTG_GINTSTS_RXFLVL) { // RxFifo non empty
        uint32_t status = OTG->GRXSTSP; // read and pop status word from fifo
        int32_t ep = status & OTG_GRXSTSP_EPNUM;
        int32_t count = (status & OTG_GRXSTSP_BCNT) >> 4;
        status &= OTG_GRXSTSP_PKTSTS;
        if (status == OTG_GRXSTSP_PKTSTS_PR // data received
            || status == OTG_GRXSTSP_PKTSTS_SR // setup received
            ) {
            STM32F4_UsbDevice_EndpointRxInterrupt(OTG, usClientState, ep, count);
        }
        else {
            // others: nothing to do
        }
        intPend = OTG->GINTSTS; // update pending bits
    }

    if (intPend & OTG_GINTSTS_IEPINT) { // IN endpoint
        uint32_t bits = OTG->DAINT & 0xFFFF; // pending IN endpoints
        int32_t ep = 0;
        while (bits) {
            if (bits & 1)
                STM32F4_UsbDevice_EndpointInInterrupt(OTG, usClientState, ep);
            ep++;
            bits >>= 1;
        }
    }

    if (intPend & OTG_GINTSTS_OEPINT) { // OUT endpoint
        uint32_t bits = OTG->DAINT >> 16; // pending OUT endpoints
        int32_t ep = 0;
        while (bits) {
            if (bits & 1)
                STM32F4_UsbDevice_EndpointOutInterrupt(OTG, usClientState, ep);

            ep++;
            bits >>= 1;
        }
    }

    if (intPend & OTG_GINTSTS_USBRST) { // reset
        STM32F4_UsbDevice_ResetEvent(OTG, usClientState);
        OTG->GINTSTS = OTG_GINTSTS_USBRST; // clear interrupt
    }
    else {
        if (intPend & OTG_GINTSTS_USBSUSP) { // suspend
            usbDeviceControllers[controller].previousDeviceState = usClientState->deviceState;

            usClientState->deviceState = USB_DEVICE_STATE_SUSPENDED;

            TinyCLR_UsbClient_StateCallback(usClientState);

            OTG->GINTSTS = OTG_GINTSTS_USBSUSP; // clear interrupt
        }

        if (intPend & OTG_GINTSTS_WKUPINT) { // wakeup
            OTG->DCTL &= ~OTG_DCTL_RWUSIG; // remove remote wakeup signaling

            usClientState->deviceState = usbDeviceControllers[controller].previousDeviceState;

            TinyCLR_UsbClient_StateCallback(usClientState);

            OTG->GINTSTS = OTG_GINTSTS_WKUPINT; // clear interrupt
        }
    }
}

bool STM32F4_UsbDevice_StartOutput(UsClientState* usClientState, int32_t ep) {
    if (usClientState == 0 || ep >= usClientState->totalEndpointsCount)
        return false;

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // If endpoint is not an output
    if (usClientState->queues[ep] == 0 || !usClientState->isTxQueue[ep])
        return false;

    /* if the halt feature for this endpoint is set, then just clear all the characters */
    if (usClientState->endpointStatus[ep] & USB_STATUS_ENDPOINT_HALT) {
        TinyCLR_UsbClient_ClearEndpoints(usClientState, ep);

        return true;
    }

    if (irq.IsDisabled()) { // check all endpoints for pending actions
        STM32F4_UsbDevice_Interrupt((void *)OTG);
    }
    // write first packet if not done yet
    STM32F4_UsbDevice_EndpointInInterrupt(OTG, usClientState, ep);

    return true;
}

bool STM32F4_UsbDevice_RxEnable(UsClientState* usClientState, int32_t ep) {
    // If this is not a legal Rx queue
    if (usClientState == 0 || usClientState->queues[ep] == 0 || usClientState->isTxQueue[ep])
        return false;

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // enable Rx
    if (!(OTG->DOEP[ep].CTL & OTG_DOEPCTL_EPENA)) {
        OTG->DOEP[ep].TSIZ = OTG_DOEPTSIZ_PKTCNT_1 | usClientState->maxEndpointsPacketSize[ep];
        OTG->DOEP[ep].CTL |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK; // enable endpoint
    }

    return true;
}

void STM32F4_UsbDevice_ProtectPins(int32_t controller, bool on) {
    UsClientState *usClientState = usbDeviceControllers[controller].usClientState;

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto& dp = usbDeviceDpPins[controller];
    auto& dm = usbDeviceDmPins[controller];
    auto& id = usbDeviceIdPins[controller];

    if (on) {

        STM32F4_GpioInternal_ConfigurePin(dp.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, dp.alternateFunction);
        STM32F4_GpioInternal_ConfigurePin(dm.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, dm.alternateFunction);

        if (STM32F4_USB_USE_ID_PIN(controller)) {
            STM32F4_GpioInternal_ConfigurePin(id.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, id.alternateFunction);
        }

        // attach usb port
        OTG->DCTL &= ~OTG_DCTL_SDIS; // remove soft disconnect
    }
    else {
        // detach usb port
        OTG->DCTL |= OTG_DCTL_SDIS; // soft disconnect

        // clear USB Txbuffer
        for (int32_t ep = 1; ep < usClientState->totalEndpointsCount; ep++) {
            if (usClientState->queues[ep] && usClientState->isTxQueue[ep]) {
                TinyCLR_UsbClient_ClearEndpoints(usClientState, ep);
            }
        }

        STM32F4_GpioInternal_ClosePin(dp.number);
        STM32F4_GpioInternal_ClosePin(dm.number);

        if (STM32F4_USB_USE_ID_PIN(controller))
            STM32F4_GpioInternal_ClosePin(id.number);
    }

    usClientState->deviceState = on ? USB_DEVICE_STATE_ATTACHED : USB_DEVICE_STATE_DETACHED;

    TinyCLR_UsbClient_StateCallback(usClientState);
}

bool TinyCLR_UsbClient_Initialize(UsClientState* usClientState) {
    return STM32F4_UsbDevice_Initialize(usClientState);
}

bool TinyCLR_UsbClient_Uninitialize(UsClientState* usClientState) {
    return STM32F4_UsbDevice_Uninitialize(usClientState);
}

bool TinyCLR_UsbClient_StartOutput(UsClientState* usClientState, int32_t endpoint) {
    return STM32F4_UsbDevice_StartOutput(usClientState, endpoint);
}

bool TinyCLR_UsbClient_RxEnable(UsClientState* usClientState, int32_t endpoint) {
    return STM32F4_UsbDevice_RxEnable(usClientState, endpoint);
}

void TinyCLR_UsbClient_Delay(uint64_t microseconds) {
    STM32F4_Time_Delay(nullptr, microseconds);
}

void TinyCLR_UsbClient_InitializeConfiguration(UsClientState *usClientState) {
    STM32F4_UsbDevice_InitializeConfiguration(usClientState);
}

uint32_t TinyCLR_UsbClient_GetEndpointSize(int32_t endpoint) {
    return endpoint == 0 ? STM32F4_USB_ENDPOINT0_SIZE : STM32F4_USB_ENDPOINT_SIZE;
}