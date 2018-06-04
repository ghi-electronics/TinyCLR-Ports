#include "GHIElectronics_TinyCLR_Devices.h"


TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::ReadInternal___VOID__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_I2c_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);;

    uint8_t* data = nullptr;
    int32_t offset = 0;
    size_t length = 0;

    TinyCLR_I2c_TransferStatus result;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        auto agr1 = TinyCLR_Interop_GetArguments(md, 1);

        uint8_t* buffer = (uint8_t*)agr1.Data.SzArray.Data;

        auto clrValueSettingObject = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings);

        if (buffer == nullptr || clrValueSettingObject.Object == nullptr) {
            return TinyCLR_Result::NullReference;
        }

        auto clrValueSettingObjectSlaveAddress = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4);
        auto clrValueSettingObjectBusSpeed = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed);

        auto slaveAddress = clrValueSettingObjectSlaveAddress.Data.Numeric->I4;
        auto busSpeed = (TinyCLR_I2c_BusSpeed)clrValueSettingObjectBusSpeed.Data.Numeric->U4;

        data = buffer;

        auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);

        offset = arg2.Data.Numeric->I4;
        length = arg3.Data.Numeric->I4;

        auto fieldParent = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___parent___GHIElectronicsTinyCLRDevicesI2cProviderDefaultI2cControllerProvider);
        auto fieldParentObject = TinyCLR_Interop_GetFieldInObject(md, fieldParent.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___idx___I4);
        auto controller = fieldParentObject.Data.Numeric->I4;

        provider->SetActiveSettings(provider, controller, slaveAddress, busSpeed);

        if (provider->Read(provider, controller, data + offset, length, result) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
        else {
            auto arg4 = TinyCLR_Interop_GetArguments(md, 4);
            auto arg5 = TinyCLR_Interop_GetArguments(md, 5);

            arg4.Data.Numeric->I4 = length;
            arg5.Data.Numeric->I4 = (int32_t)result;

        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::WriteInternal___VOID__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_I2c_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* data = nullptr;
    int32_t offset = 0;
    size_t length = 0;

    TinyCLR_I2c_TransferStatus result;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        auto agr1 = TinyCLR_Interop_GetArguments(md, 1);

        uint8_t* buffer = (uint8_t*)agr1.Data.SzArray.Data;

        auto clrValueSettingObject = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings);

        if (buffer == nullptr || clrValueSettingObject.Object == nullptr) {
            return TinyCLR_Result::NullReference;
        }

        auto clrValueSettingObjectSlaveAddress = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4);
        auto clrValueSettingObjectBusSpeed = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed);

        auto slaveAddress = clrValueSettingObjectSlaveAddress.Data.Numeric->I4;
        auto busSpeed = (TinyCLR_I2c_BusSpeed)clrValueSettingObjectBusSpeed.Data.Numeric->U4;

        data = buffer;

        auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);

        offset = arg2.Data.Numeric->I4;
        length = arg3.Data.Numeric->I4;

        auto fieldParent = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___parent___GHIElectronicsTinyCLRDevicesI2cProviderDefaultI2cControllerProvider);
        auto fieldParentObject = TinyCLR_Interop_GetFieldInObject(md, fieldParent.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___idx___I4);
        auto controller = fieldParentObject.Data.Numeric->I4;

        provider->SetActiveSettings(provider, controller, slaveAddress, busSpeed);

        if (provider->Write(provider, controller, data + offset, length, result) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
        else {
            auto arg4 = TinyCLR_Interop_GetArguments(md, 4);
            auto arg5 = TinyCLR_Interop_GetArguments(md, 5);

            arg4.Data.Numeric->I4 = length;
            arg5.Data.Numeric->I4 = (int32_t)result;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::WriteReadInternal___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_I2c_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* writeData = nullptr;
    int32_t writeOffset = 0;
    size_t writeLength = 0;

    uint8_t* readData = nullptr;
    int32_t readOffset = 0;
    size_t readLength = 0;

    TinyCLR_I2c_TransferStatus result;
    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        auto agr1 = TinyCLR_Interop_GetArguments(md, 1);

        uint8_t* writeBuffer = (uint8_t*)agr1.Data.SzArray.Data;

        auto agr4 = TinyCLR_Interop_GetArguments(md, 4);

        uint8_t* readBuffer = (uint8_t*)agr4.Data.SzArray.Data;

        auto clrValueSettingObject = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings);

        if (writeBuffer == nullptr || readBuffer == nullptr || clrValueSettingObject.Object == nullptr) {
            return TinyCLR_Result::NullReference;
        }

        auto clrValueSettingObjectSlaveAddress = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4);
        auto clrValueSettingObjectBusSpeed = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed);

        auto slaveAddress = clrValueSettingObjectSlaveAddress.Data.Numeric->I4;
        auto busSpeed = (TinyCLR_I2c_BusSpeed)clrValueSettingObjectBusSpeed.Data.Numeric->U4;

        writeData = writeBuffer;
        auto agr2 = TinyCLR_Interop_GetArguments(md, 2);
        auto agr3 = TinyCLR_Interop_GetArguments(md, 3);

        writeOffset = agr2.Data.Numeric->I4;
        writeLength = agr3.Data.Numeric->I4;

        readData = readBuffer;

        auto agr5 = TinyCLR_Interop_GetArguments(md, 5);
        auto agr6 = TinyCLR_Interop_GetArguments(md, 6);

        readOffset = agr5.Data.Numeric->I4;
        readLength = agr6.Data.Numeric->I4;

        auto fieldParent = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___parent___GHIElectronicsTinyCLRDevicesI2cProviderDefaultI2cControllerProvider);
        auto fieldParentObject = TinyCLR_Interop_GetFieldInObject(md, fieldParent.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___idx___I4);
        auto controller = fieldParentObject.Data.Numeric->I4;

        provider->SetActiveSettings(provider, controller, slaveAddress, busSpeed);

        if (provider->WriteRead(provider, controller, writeData + writeOffset, writeLength, readData + readOffset, readLength, result) != TinyCLR_Result::Success) {
            return TinyCLR_Result::NullReference;
        }
        else {
            auto arg7 = TinyCLR_Interop_GetArguments(md, 7);
            auto arg8 = TinyCLR_Interop_GetArguments(md, 8);

            arg7.Data.Numeric->I4 = writeLength + readLength;
            arg8.Data.Numeric->I4 = (int32_t)result;
        }
    }

    return TinyCLR_Result::Success;
}
