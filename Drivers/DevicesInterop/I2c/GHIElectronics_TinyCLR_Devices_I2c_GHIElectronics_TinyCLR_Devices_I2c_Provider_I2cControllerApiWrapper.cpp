#include "GHIElectronics_TinyCLR_Devices_I2c.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper::WriteRead___GHIElectronicsTinyCLRDevicesI2cI2cTransferStatus__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN__BOOLEAN__BYREF_I4__BYREF_I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_I2c_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    uint8_t* writeData = nullptr;
    int32_t writeOffset = 0;
    size_t writeLength = 0;

    uint8_t* readData = nullptr;
    int32_t readOffset = 0;
    size_t readLength = 0;

    TinyCLR_Interop_ClrValue args[10];

    for (auto i = 0; i < sizeof(args) / sizeof(TinyCLR_Interop_ClrValue); i++) {
        md.InteropManager->GetArgument(md.InteropManager, md.Stack, i, args[i]);
    }

    uint8_t* writeBuffer = (uint8_t*)args[0].Data.SzArray.Data;

    uint8_t* readBuffer = (uint8_t*)args[3].Data.SzArray.Data;

    writeData = writeBuffer;

    writeOffset = args[1].Data.Numeric->I4;
    writeLength = args[2].Data.Numeric->I4;

    readData = readBuffer;

    readOffset = args[4].Data.Numeric->I4;
    readLength = args[5].Data.Numeric->I4;

    bool sendStartCondition = args[6].Data.Numeric->Boolean;
    bool sendStopCondition = args[7].Data.Numeric->Boolean;

    TinyCLR_I2c_TransferStatus error;

    auto result = api->WriteRead(api, writeData + writeOffset, writeLength, readData + readOffset, readLength, sendStartCondition, sendStopCondition, error);

    args[8].Data.Numeric->I4 = writeLength;
    args[9].Data.Numeric->I4 = readLength;

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = static_cast<uint32_t>(error);

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_I2c_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_I2c_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper::SetActiveSettings___VOID__GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_I2c_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue obj;
    TinyCLR_Interop_ClrValue args[3];

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, obj);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___SlaveAddress__BackingField___I4, args[0]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___AddressFormat__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cAddressFormat, args[1]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___BusSpeed__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed, args[2]);

    auto slaveAddress = args[0].Data.Numeric->I4;
    auto addressFormat = static_cast<TinyCLR_I2c_AddressFormat>(args[1].Data.Numeric->I4);
    auto busSpeed = static_cast<TinyCLR_I2c_BusSpeed>(args[2].Data.Numeric->I4);

    return api->SetActiveSettings(api, slaveAddress, addressFormat, busSpeed);
}
