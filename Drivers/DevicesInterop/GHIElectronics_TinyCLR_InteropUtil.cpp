#include "GHIElectronics_TinyCLR_InteropUtil.h"

#include"./Adc/GHIElectronics_TinyCLR_Devices_Adc.h"
#include"./Can/GHIElectronics_TinyCLR_Devices_Can.h"
#include"./Dac/GHIElectronics_TinyCLR_Devices_Dac.h"
#include"./Display/GHIElectronics_TinyCLR_Devices_Display.h"
#include"./Gpio/GHIElectronics_TinyCLR_Devices_Gpio.h"
#include"./I2c/GHIElectronics_TinyCLR_Devices_I2c.h"
#include"./Pwm/GHIElectronics_TinyCLR_Devices_Pwm.h"
#include"./Rtc/GHIElectronics_TinyCLR_Devices_Rtc.h"
#include"./Signals/GHIElectronics_TinyCLR_Devices_Signals.h"
#include"./Spi/GHIElectronics_TinyCLR_Devices_Spi.h"
#include"./Storage/GHIElectronics_TinyCLR_Devices_Storage.h"
#include"./Uart/GHIElectronics_TinyCLR_Devices_Uart.h"

#include <Device.h>

const void* TinyCLR_Interop_GetApi(const TinyCLR_Interop_MethodData md, size_t fieldId) {
    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, fieldId, fld);

    return reinterpret_cast<const void*>(fld.Data.Numeric->I);
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetFieldSelf(const TinyCLR_Interop_MethodData md, size_t fieldId) {
    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, fieldId, fld);

    return fld;
}

void DevicesInterop_Add(const TinyCLR_Interop_Manager* interopManager) {
#ifdef INCLUDE_ADC
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Adc);
#endif
#ifdef INCLUDE_CAN
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Can);
#endif
#ifdef INCLUDE_DAC
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Dac);
#endif
#ifdef INCLUDE_DISPLAY
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Display);
#endif
#ifdef INCLUDE_GPIO
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Gpio);
#endif
#ifdef INCLUDE_I2C
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_I2c);
#endif
#ifdef INCLUDE_PWM
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Pwm);
#endif
#ifdef INCLUDE_RTC
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Rtc);
#endif
#ifdef INCLUDE_SIGNALS
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Signals);
#endif
#ifdef INCLUDE_SPI
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Spi);
#endif
#ifdef INCLUDE_STORAGE
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Storage);
#endif
#ifdef INCLUDE_UART
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices_Uart);
#endif
}