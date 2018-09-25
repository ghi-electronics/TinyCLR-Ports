#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesDisplayProviderIDisplayControllerProvider = 1;
    static const size_t FIELD___ActiveConfiguration__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayControllerSettings = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings {
    static const size_t FIELD___Width__BackingField___I4 = 1;
    static const size_t FIELD___Height__BackingField___I4 = 2;
    static const size_t FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_I2cDisplayControllerSettings {
    static const size_t FIELD___ApiName__BackingField___STRING = 4;
    static const size_t FIELD___Settings__BackingField___GHIElectronicsTinyCLRDevicesI2cGHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings {
    static const size_t FIELD___DataEnableIsFixed__BackingField___BOOLEAN = 4;
    static const size_t FIELD___DataEnablePolarity__BackingField___BOOLEAN = 5;
    static const size_t FIELD___PixelPolarity__BackingField___BOOLEAN = 6;
    static const size_t FIELD___PixelClockRate__BackingField___I4 = 7;
    static const size_t FIELD___HorizontalSyncPolarity__BackingField___BOOLEAN = 8;
    static const size_t FIELD___HorizontalSyncPulseWidth__BackingField___I4 = 9;
    static const size_t FIELD___HorizontalFrontPorch__BackingField___I4 = 10;
    static const size_t FIELD___HorizontalBackPorch__BackingField___I4 = 11;
    static const size_t FIELD___VerticalSyncPolarity__BackingField___BOOLEAN = 12;
    static const size_t FIELD___VerticalSyncPulseWidth__BackingField___I4 = 13;
    static const size_t FIELD___VerticalFrontPorch__BackingField___I4 = 14;
    static const size_t FIELD___VerticalBackPorch__BackingField___I4 = 15;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result Enable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Disable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DrawBuffer___VOID__I4__I4__I4__I4__SZARRAY_U1__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DrawPixel___VOID__I4__I4__I8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DrawString___VOID__STRING(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_Interface___GHIElectronicsTinyCLRDevicesDisplayDisplayInterface(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SupportedDataFormats___SZARRAY_GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetConfiguration___VOID__GHIElectronicsTinyCLRDevicesDisplayParallelDisplayControllerSettings(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetConfiguration___VOID__GHIElectronicsTinyCLRDevicesDisplaySpiDisplayControllerSettings(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetConfiguration___VOID__GHIElectronicsTinyCLRDevicesDisplayI2cDisplayControllerSettings(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_SpiDisplayControllerSettings {
    static const size_t FIELD___ApiName__BackingField___STRING = 4;
    static const size_t FIELD___Settings__BackingField___GHIElectronicsTinyCLRDevicesSpiGHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings = 5;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Display;
