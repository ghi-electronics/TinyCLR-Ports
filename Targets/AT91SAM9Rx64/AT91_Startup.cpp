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

#if defined(__GNUC__)
// GCC ARM linker does not link to some variable below if optimize mode.
#pragma GCC optimize 0
#endif

#include "AT91.h"

void AT91_Startup_OnSoftReset(const TinyCLR_Api_Provider* apiProvider, const TinyCLR_Interop_Provider* interopProvider) {
#ifdef INCLUDE_ADC
    AT91_Adc_Reset();
#endif
#ifdef INCLUDE_CAN
    AT91_Can_Reset();
#endif
#ifdef INCLUDE_DAC
    AT91_Dac_Reset();
#endif
#ifdef INCLUDE_DISPLAY
    AT91_Display_Reset();
#endif
#ifdef INCLUDE_GPIO
    AT91_Gpio_Reset();
#endif
#ifdef INCLUDE_I2C
    AT91_I2c_Reset();
#endif
#ifdef INCLUDE_PWM
    AT91_Pwm_Reset();
#endif
#ifdef INCLUDE_SD
    AT91_SdCard_Reset();
#endif
#ifdef INCLUDE_SPI
    AT91_Spi_Reset();
#endif
#ifdef INCLUDE_UART
    AT91_Uart_Reset();
#endif
#ifdef INCLUDE_USBCLIENT
    AT91_UsbClient_Reset();
#endif
}

extern "C" {
    extern int HeapBegin;
    extern int HeapEnd;

    extern uint32_t Load$$ER_RAM_RW$$Base;
    extern uint32_t Image$$ER_RAM_RW$$Base;
    extern uint32_t Image$$ER_RAM_RW$$Length;

    extern uint32_t Image$$ER_RAM_RW$$ZI$$Base;
    extern uint32_t Image$$ER_RAM_RW$$ZI$$Length;

    extern uint32_t Load$$ER_RAM_RO$$Base;
    extern uint32_t Image$$ER_RAM_RO$$Base;
    extern uint32_t Image$$ER_RAM_RO$$Length;

    extern uint32_t Load$$ER_FLASH$$Base;

    extern uint32_t ARM_Vectors;

}
void AT91_SAM_ClockInit(void);
#pragma arm section code = "SectionForBootstrapOperations"

extern "C" {
    void __section("SectionForBootstrapOperations") SystemInit() {

        AT91_SAM_ClockInit();

        AT91_CPU_BootstrapCode();

        AT91_MMU_Initialize();

        AT91_Cache_EnableCaches();

        AT91_WATCHDOG &g_WDT = AT91::WTDG();

        g_WDT.WTDG_MR |= 1 << 15;  // Disable watchdog

        return;

    }
}

#pragma arm section code = "SectionForBootstrapOperations"

static void __section("SectionForBootstrapOperations") Prepare_Copy(uint32_t* src, uint32_t* dst, uint32_t len) {
    if (dst != src) {
        int32_t extraLen = len & 0x00000003;
        len = len & 0xFFFFFFFC;

        while (len != 0) {
            *dst++ = *src++;

            len -= 4;
        }

        // thumb2 code can be multiples of 2...

        uint8_t *dst8 = (uint8_t*)dst, *src8 = (uint8_t*)src;

        while (extraLen > 0) {
            *dst8++ = *src8++;

            extraLen--;
        }
    }
}

static void __section("SectionForBootstrapOperations") Prepare_Zero(uint32_t* dst, uint32_t len) {
    int32_t extraLen = len & 0x00000003;
    len = len & 0xFFFFFFFC;

    while (len != 0) {
        *dst++ = 0;

        len -= 4;
    }

    // thumb2 code can be multiples of 2...

    uint8_t *dst8 = (uint8_t*)dst;

    while (extraLen > 0) {
        *dst8++ = 0;

        extraLen--;
    }
}

void AT91_Startup_GetHeap(uint8_t*& start, size_t& length) {
    start = (uint8_t*)&HeapBegin;
    length = (size_t)(((int)&HeapEnd) - ((int)&HeapBegin));
}

void AT91_Startup_Initialize() {
    //
    // Copy RAM RO regions into proper location.
    //
    {
        uint32_t* src = (uint32_t*)((uint32_t)&Load$$ER_RAM_RO$$Base);
        uint32_t* dst = (uint32_t*)((uint32_t)&Image$$ER_RAM_RO$$Base);
        uint32_t  len = (uint32_t)((uint32_t)&Image$$ER_RAM_RO$$Length);

        if (len != 0) // only copy if len is not 0
            Prepare_Copy(src, dst, len);
    }

    //
    // Copy RAM RW regions into proper location.
    //
    {
        uint32_t* src = (uint32_t*)((uint32_t)&Load$$ER_RAM_RW$$Base);
        uint32_t* dst = (uint32_t*)((uint32_t)&Image$$ER_RAM_RW$$Base);
        uint32_t  len = (uint32_t)((uint32_t)&Image$$ER_RAM_RW$$Length);

        if (len != 0) // only copy if len is not zero or not compress RW data
            Prepare_Copy(src, dst, len);
    }

    //
    // Initialize RAM ZI regions.
    //
    {
        uint32_t* dst = (uint32_t*)((uint32_t)&Image$$ER_RAM_RW$$ZI$$Base);
        uint32_t  len = (uint32_t)((uint32_t)&Image$$ER_RAM_RW$$ZI$$Length);

        Prepare_Zero(dst, len);
    }

    //
    // Copy Vector.
    //
    {

        uint32_t* src = (uint32_t*)((uint32_t)&ARM_Vectors);
        uint32_t* dst = (uint32_t*)0x0000000;
        uint32_t  len = 44;

        if ((dst != src) && (*src != 0)) {
            while (len) {
                *dst++ = *src++;
                len -= 4;
            }
        }
    }

}

const TinyCLR_Startup_UsbDebuggerConfiguration AT91_Startup_UsbDebuggerConfiguration = {
    USB_DEBUGGER_INDEX,
    USB_DEBUGGER_VENDOR_ID,
    USB_DEBUGGER_PRODUCT_ID,
    CONCAT(L,DEVICE_MANUFACTURER),
    CONCAT(L,DEVICE_NAME),
    0
};

const TinyCLR_Startup_UartDebuggerConfiguration AT91_Startup_UartDebuggerConfiguration = {
    UART_DEBUGGER_INDEX
};

void AT91_Startup_GetDebuggerTransportProvider(const TinyCLR_Api_Info*& api, const void*& configuration) {
#if defined(DEBUGGER_SELECTOR_PIN)
    TinyCLR_Gpio_PinValue value;
    auto provider = static_cast<const TinyCLR_Gpio_Controller*>(AT91_Gpio_GetApi()->Implementation);
    auto gpioController = 0; //TODO Temporary set to 0

    provider->AcquirePin(provider, gpioController, DEBUGGER_SELECTOR_PIN);
    provider->SetDriveMode(provider, gpioController, DEBUGGER_SELECTOR_PIN, DEBUGGER_SELECTOR_PULL);
    provider->Read(provider, gpioController, DEBUGGER_SELECTOR_PIN, value);
    provider->ReleasePin(provider, gpioController, DEBUGGER_SELECTOR_PIN);

    if (value == DEBUGGER_SELECTOR_USB_STATE) {
        api = AT91_UsbClient_GetApi();        
        configuration = (const void*)&AT91_Startup_UsbDebuggerConfiguration;
    }
    else {
        api = AT91_Uart_GetApi();
        configuration = (const void*)&AT91_Startup_UartDebuggerConfiguration;
    }
#elif defined(DEBUGGER_FORCE_API) && defined(DEBUGGER_FORCE_INDEX)
    api = DEBUGGER_FORCE_API;
    index = DEBUGGER_FORCE_INDEX;
#else
#error You must specify a debugger mode pin or specify the API explicitly.
#endif
}

void AT91_Startup_GetRunApp(bool& runApp) {
#if defined(RUN_APP_PIN)
    TinyCLR_Gpio_PinValue value;
    auto provider = static_cast<const TinyCLR_Gpio_Controller*>(AT91_Gpio_GetApi()->Implementation);
    auto gpioController = 0; //TODO Temporary set to 0

    provider->AcquirePin(provider, gpioController, RUN_APP_PIN);
    provider->SetDriveMode(provider, gpioController, RUN_APP_PIN, RUN_APP_PULL);
    provider->Read(provider, gpioController, RUN_APP_PIN, value);
    provider->ReleasePin(provider, gpioController, RUN_APP_PIN);

    runApp = value == RUN_APP_STATE;
#elif defined(RUN_APP_FORCE_STATE)
    runApp = RUN_APP_FORCE_STATE;
#else
    runApp = true;
#endif
}

#define BOARD_OSCOUNT           (AT91_PMC::CKGR_OSCOUNT & (64 << 8))
#define BOARD_CKGR_PLLA         ( (0x1 << 29) | AT91_PMC::CKGR_OUT_2)
#define BOARD_PLLACOUNT         (63 << 8)
#define BOARD_MULA              (AT91_PMC::CKGR_MUL & (199 << 16))
#define BOARD_DIVA              (AT91_PMC::CKGR_DIV & 12)
#define BOARD_PRESCALER         AT91_PMC::PMC_MDIV_2

#define BOARD_USBDIV            AT91C_CKGR_USBDIV_2
#define BOARD_CKGR_PLLB         AT91C_CKGR_OUT_0
#define BOARD_PLLBCOUNT         BOARD_PLLACOUNT
#define BOARD_MULB              (124 << 16)
#define BOARD_DIVB              12

#define BOARD_USBEN				(1<<16)
#define BOARD_USBPLLCOUNT		(0x0F<<20)
#define BOARD_BIASEN			(1<<24)
#define BOARD_BIASCOUNT			(0x0F<<28)

#define READ(peripheral, register)          (peripheral.register)
#define WRITE(peripheral, register, value)  (peripheral.register = value)

/*
* Setup PLL & SDRAM
*/
void AT91_SAM_ClockInit(void) {
    // Power Management Controller
    AT91_PMC &pmc = AT91::PMC();

    // Initialize main oscillator
    pmc.PMC_CKGR_MOR = BOARD_OSCOUNT | AT91_PMC::CKGR_MOSCEN;
    while (!(pmc.PMC_SR & AT91_PMC::PMC_MOSCS));

    // Initialize PLLA at 200MHz
    pmc.PMC_CKGR_PLLAR = BOARD_CKGR_PLLA
        | BOARD_PLLACOUNT
        | BOARD_MULA
        | BOARD_DIVA;
    while (!(pmc.PMC_SR & AT91_PMC::PMC_LOCKA));

    // Initialize UTMI for USB usage
    pmc.PMC_CKGR_UCKR = BOARD_USBEN | BOARD_USBPLLCOUNT | BOARD_BIASEN | BOARD_BIASCOUNT;
    while (!(pmc.PMC_SR & AT91_PMC::PMC_LOCKU));


    // Wait for the master clock if it was already initialized
    while (!(pmc.PMC_SR & AT91_PMC::PMC_MCKRDY));

    // Switch to fast clock
    // Switch to main oscillator + prescaler
    pmc.PMC_MCKR = BOARD_PRESCALER;
    while (!(pmc.PMC_SR & AT91_PMC::PMC_MCKRDY));

    // Switch to PLL + prescaler
    pmc.PMC_MCKR |= AT91_PMC::PMC_CSS_PLLA_CLK;
    while (!(pmc.PMC_SR & AT91_PMC::PMC_MCKRDY));
}


