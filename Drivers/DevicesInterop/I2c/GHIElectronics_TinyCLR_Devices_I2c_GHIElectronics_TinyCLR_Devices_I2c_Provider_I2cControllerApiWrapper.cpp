#include "GHIElectronics_TinyCLR_Devices.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper::WriteRead___GHIElectronicsTinyCLRDevicesI2cI2cTransferStatus__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN__BOOLEAN__BYREF_I4__BYREF_I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_I2c_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    uint8_t* writeData = nullptr;
    int32_t writeOffset = 0;
    size_t writeLength = 0;

    uint8_t* readData = nullptr;
    int32_t readOffset = 0;
    size_t readLength = 0;

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 5, arg5);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 6, arg6);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 7, arg7);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 8, arg8);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 9, arg9);

    uint8_t* writeBuffer = (uint8_t*)arg0.Data.SzArray.Data;

    uint8_t* readBuffer = (uint8_t*)arg3.Data.SzArray.Data;

    writeData = writeBuffer;

    writeOffset = arg1.Data.Numeric->I4;
    writeLength = arg2.Data.Numeric->I4;

    readData = readBuffer;

    readOffset = arg4.Data.Numeric->I4;
    readLength = arg5.Data.Numeric->I4;

    bool sendStartCondition = arg6.Data.Numeric->Boolean;
    bool sendStopCondition = arg7.Data.Numeric->Boolean;

    TinyCLR_I2c_TransferStatus error;

    auto result = api->WriteRead(api, writeData + writeOffset, writeLength, readData + readOffset, readLength, sendStartCondition, sendStopCondition, error);

    arg8.Data.Numeric->I4 = writeLength;
    arg9.Data.Numeric->I4 = readLength;

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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper::SetActiveSettings___VOID__I4__GHIElectronicsTinyCLRDevicesI2cI2cAddressFormat__GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_I2c_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto slaveAddress = arg0.Data.Numeric->I4;
    auto addressFormat = static_cast<TinyCLR_I2c_AddressFormat>(arg1.Data.Numeric->I4);
    auto busSpeed = static_cast<TinyCLR_I2c_BusSpeed>(arg2.Data.Numeric->I4);

    return api->SetActiveSettings(api, slaveAddress, addressFormat, busSpeed);
}
