#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings {
    static const size_t FIELD___SlaveAddress__BackingField___I4 = 1;
    static const size_t FIELD___AddressFormat__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cAddressFormat = 2;
    static const size_t FIELD___BusSpeed__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cController {
    static const size_t FIELD___active___GHIElectronicsTinyCLRDevicesI2cI2cDevice = 1;
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesI2cProviderII2cControllerProvider = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cDevice {
    static const size_t FIELD___ConnectionSettings__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cController = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cTransferResult {
    static const size_t FIELD___Status__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cTransferStatus = 1;
    static const size_t FIELD___BytesWritten__BackingField___I4 = 2;
    static const size_t FIELD___BytesRead__BackingField___I4 = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result SetActiveSettings___VOID__GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteRead___GHIElectronicsTinyCLRDevicesI2cI2cTransferStatus__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN__BOOLEAN__BYREF_I4__BYREF_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerSoftwareProvider {
    static const size_t FIELD___usePullups___BOOLEAN = 1;
    static const size_t FIELD___sda___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 2;
    static const size_t FIELD___scl___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___writeAddress___U1 = 4;
    static const size_t FIELD___readAddress___U1 = 5;
    static const size_t FIELD___start___BOOLEAN = 6;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_I2c;
