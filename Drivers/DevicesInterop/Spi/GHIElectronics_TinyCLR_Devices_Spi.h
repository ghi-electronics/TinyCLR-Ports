#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChipSelectLineCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinClockFrequency___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxClockFrequency___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SupportedDataBitLengths___SZARRAY_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetActiveSettings___VOID__GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteRead___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerSoftwareProvider {
    static const size_t FIELD___chipSelects___mscorlibSystemCollectionsIDictionary = 1;
    static const size_t FIELD___gpioController___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioController = 2;
    static const size_t FIELD___mosi___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___miso___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 4;
    static const size_t FIELD___sck___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 5;
    static const size_t FIELD___cs___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 6;
    static const size_t FIELD___captureOnRisingEdge___BOOLEAN = 7;
    static const size_t FIELD___clockIdleState___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue = 8;
    static const size_t FIELD___clockActiveState___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue = 9;
    static const size_t FIELD___chipSelectSetupTime___mscorlibSystemTimeSpan = 10;
    static const size_t FIELD___chipSelectHoldTime___mscorlibSystemTimeSpan = 11;
    static const size_t FIELD___chipSelectActiveState___BOOLEAN = 12;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings {
    static const size_t FIELD___ChipSelectType__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiChipSelectType = 1;
    static const size_t FIELD___ChipSelectLine__BackingField___I4 = 2;
    static const size_t FIELD___ClockFrequency__BackingField___I4 = 3;
    static const size_t FIELD___DataBitLength__BackingField___I4 = 4;
    static const size_t FIELD___Mode__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiMode = 5;
    static const size_t FIELD___ChipSelectSetupTime__BackingField___mscorlibSystemTimeSpan = 6;
    static const size_t FIELD___ChipSelectHoldTime__BackingField___mscorlibSystemTimeSpan = 7;
    static const size_t FIELD___ChipSelectActiveState__BackingField___BOOLEAN = 8;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesSpiProviderISpiControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiDevice {
    static const size_t FIELD___ConnectionSettings__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiController = 2;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Spi;
