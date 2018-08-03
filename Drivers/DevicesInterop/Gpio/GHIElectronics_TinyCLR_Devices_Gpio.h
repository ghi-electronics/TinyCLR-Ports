#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_GpioController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesGpioProviderIGpioControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_GpioPin {
    static const size_t FIELD___callbacks___GHIElectronicsTinyCLRDevicesGpioGpioPinValueChangedEventHandler = 1;
    static const size_t FIELD___valueChangedEdge___GHIElectronicsTinyCLRDevicesGpioGpioPinEdge = 2;
    static const size_t FIELD___PinNumber__BackingField___I4 = 3;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesGpioGpioController = 4;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_GpioPinValueChangedEventArgs {
    static const size_t FIELD___Edge__BackingField___GHIElectronicsTinyCLRDevicesGpioGpioPinEdge = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___pinMap___mscorlibSystemCollectionsIDictionary = 2;
    static const size_t FIELD___dispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 3;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 4;

    static TinyCLR_Result get_PinCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenPin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClosePin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result GetDebounceTimeout___mscorlibSystemTimeSpan__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDebounceTimeout___VOID__I4__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result GetDriveMode___GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDriveMode___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___GHIElectronicsTinyCLRDevicesGpioGpioPinValue__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinValue(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result IsDriveModeSupported___BOOLEAN__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetPinChangedEdge___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinEdge(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearPinChangedEdge___VOID__I4(const TinyCLR_Interop_MethodData md);
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Gpio;
