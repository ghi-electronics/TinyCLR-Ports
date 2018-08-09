#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback {
    static const size_t FIELD___mode___GHIElectronicsTinyCLRDevicesSignalsPulseFeedbackMode = 1;
    static const size_t FIELD___gpioApi___I = 2;
    static const size_t FIELD___pulsePinNumber___I4 = 3;
    static const size_t FIELD___echoPinNumber___I4 = 4;
    static const size_t FIELD___pulsePin___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 5;
    static const size_t FIELD___echoPin___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 6;
    static const size_t FIELD___DisableInterrupts__BackingField___BOOLEAN = 7;
    static const size_t FIELD___Timeout__BackingField___mscorlibSystemTimeSpan = 8;
    static const size_t FIELD___PulseLength__BackingField___mscorlibSystemTimeSpan = 9;
    static const size_t FIELD___PulsePinValue__BackingField___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue = 10;
    static const size_t FIELD___EchoPinValue__BackingField___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue = 11;
    static const size_t FIELD___EchoPinDriveMode__BackingField___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode = 12;

    static TinyCLR_Result GeneratePulse___I8(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture {
    static const size_t FIELD___gpioApi___I = 1;
    static const size_t FIELD___pinNumber___I4 = 2;
    static const size_t FIELD___pin___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___DisableInterrupts__BackingField___BOOLEAN = 4;
    static const size_t FIELD___Timeout__BackingField___mscorlibSystemTimeSpan = 5;

    static TinyCLR_Result Read___I4__BYREF_GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue__SZARRAY_I8__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue__SZARRAY_I8__I4__I4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator {
    static const size_t FIELD___gpioApi___I = 1;
    static const size_t FIELD___pinNumber___I4 = 2;
    static const size_t FIELD___pin___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___DisableInterrupts__BackingField___BOOLEAN = 4;
    static const size_t FIELD___GenerateCarrierFrequency__BackingField___BOOLEAN = 5;
    static const size_t FIELD___CarrierFrequency__BackingField___I8 = 6;

    static TinyCLR_Result Write___VOID__SZARRAY_I8__I4__I4(const TinyCLR_Interop_MethodData md);
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Signals;
