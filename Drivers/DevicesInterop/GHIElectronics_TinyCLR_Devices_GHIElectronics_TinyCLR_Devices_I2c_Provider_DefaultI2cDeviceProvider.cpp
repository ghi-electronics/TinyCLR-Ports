#include "GHIElectronics_TinyCLR_Devices.h"


TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::ReadInternal___VOID__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
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
        auto channelArg = TinyCLR_Interop_GetArguments(md, 1);
        auto agr2 = TinyCLR_Interop_GetArguments(md, 2);

        uint8_t* buffer = (uint8_t*)agr2.Data.SzArray.Data;

        auto clrValueSettingObject = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings);

        if (buffer == nullptr || clrValueSettingObject.Object == nullptr) {
            return TinyCLR_Result::NullReference;
        }

        auto clrValueSettingObjectSlaveAddress = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4);
        auto clrValueSettingObjectBusSpeed = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed);

        auto slaveAddress = clrValueSettingObjectSlaveAddress.Data.Numeric->I4;
        auto busSpeed = (TinyCLR_I2c_BusSpeed)clrValueSettingObjectBusSpeed.Data.Numeric->U4;

        data = buffer;

        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);
        auto arg4 = TinyCLR_Interop_GetArguments(md, 4);

        offset = arg3.Data.Numeric->I4;
        length = arg4.Data.Numeric->I4;

        int32_t channel = channelArg.Data.Numeric->I4;

        provider->SetActiveSettings(provider, channel, slaveAddress, busSpeed);

        if (provider->Read(provider, channel, data + offset, length, result) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
        else {
            auto arg5 = TinyCLR_Interop_GetArguments(md, 5);
            auto arg6 = TinyCLR_Interop_GetArguments(md, 6);

            arg5.Data.Numeric->I4 = length;
            arg6.Data.Numeric->I4 = (int32_t)result;

        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::WriteInternal___VOID__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
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
        auto channelArg = TinyCLR_Interop_GetArguments(md, 1);
        auto agr2 = TinyCLR_Interop_GetArguments(md, 2);

        uint8_t* buffer = (uint8_t*)agr2.Data.SzArray.Data;

        auto clrValueSettingObject = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings);

        if (buffer == nullptr || clrValueSettingObject.Object == nullptr) {
            return TinyCLR_Result::NullReference;
        }

        auto clrValueSettingObjectSlaveAddress = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4);
        auto clrValueSettingObjectBusSpeed = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed);

        auto slaveAddress = clrValueSettingObjectSlaveAddress.Data.Numeric->I4;
        auto busSpeed = (TinyCLR_I2c_BusSpeed)clrValueSettingObjectBusSpeed.Data.Numeric->U4;

        data = buffer;

        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);
        auto arg4 = TinyCLR_Interop_GetArguments(md, 4);

        offset = arg3.Data.Numeric->I4;
        length = arg4.Data.Numeric->I4;

        int32_t channel = channelArg.Data.Numeric->I4;

        provider->SetActiveSettings(provider, channel, slaveAddress, busSpeed);

        if (provider->Write(provider, channel, data + offset, length, result) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
        else {
            auto arg5 = TinyCLR_Interop_GetArguments(md, 5);
            auto arg6 = TinyCLR_Interop_GetArguments(md, 6);

            arg5.Data.Numeric->I4 = length;
            arg6.Data.Numeric->I4 = (int32_t)result;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::WriteReadInternal___VOID__I4__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
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
        auto channelArg = TinyCLR_Interop_GetArguments(md, 1);
        auto agr2 = TinyCLR_Interop_GetArguments(md, 2);

        uint8_t* writeBuffer = (uint8_t*)agr2.Data.SzArray.Data;

        auto agr5 = TinyCLR_Interop_GetArguments(md, 5);

        uint8_t* readBuffer = (uint8_t*)agr5.Data.SzArray.Data;

        auto clrValueSettingObject = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings);

        if (writeBuffer == nullptr || readBuffer == nullptr || clrValueSettingObject.Object == nullptr) {
            return TinyCLR_Result::NullReference;
        }

        auto clrValueSettingObjectSlaveAddress = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4);
        auto clrValueSettingObjectBusSpeed = TinyCLR_Interop_GetFieldInObject(md, clrValueSettingObject.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed);

        auto slaveAddress = clrValueSettingObjectSlaveAddress.Data.Numeric->I4;
        auto busSpeed = (TinyCLR_I2c_BusSpeed)clrValueSettingObjectBusSpeed.Data.Numeric->U4;

        writeData = writeBuffer;
        auto agr3 = TinyCLR_Interop_GetArguments(md, 3);
        auto agr4 = TinyCLR_Interop_GetArguments(md, 4);

        writeOffset = agr3.Data.Numeric->I4;
        writeLength = agr4.Data.Numeric->I4;

        readData = readBuffer;

        auto agr6 = TinyCLR_Interop_GetArguments(md, 6);
        auto agr7 = TinyCLR_Interop_GetArguments(md, 7);

        readOffset = agr6.Data.Numeric->I4;
        readLength = agr7.Data.Numeric->I4;

        int32_t channel = channelArg.Data.Numeric->I4;

        provider->SetActiveSettings(provider, channel, slaveAddress, busSpeed);

        if (provider->WriteRead(provider, channel, writeData + writeOffset, writeLength, readData + readOffset, readLength, result) != TinyCLR_Result::Success) {
            return TinyCLR_Result::NullReference;
        }
        else {

            auto arg8 = TinyCLR_Interop_GetArguments(md, 8);
            auto arg9 = TinyCLR_Interop_GetArguments(md, 9);

            arg8.Data.Numeric->I4 = writeLength + readLength;
            arg9.Data.Numeric->I4 = (int32_t)result;

        }
    }

    return TinyCLR_Result::Success;
}
