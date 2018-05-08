#include "GHIElectronics_TinyCLR_Devices.h"


TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::ReadInternal___VOID__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
    // auto provider = (const TinyCLR_I2c_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);;
    
    // uint8_t* data = NULL;
    // int32_t offset = 0;
    // size_t length = 0;

    // TinyCLR_I2c_TransferStatus result;
    
    // TinyCLR_Interop_ClrValue fld;

    // fld = TinyCLR_Interop_GetField(md, FIELD___m_disposed___BOOLEAN);
    
    // if (fld.Data.Numeric->I1 != 0) {
        // return TinyCLR_Result::Disposed;
    // }

    // {
        // auto agr1 = TinyCLR_Interop_GetArguments(md, 1);
        
        // uint8_t* buffer = (uint8_t*)agr1.Data.SzArray.Data;
        
        
        
        // CLR_RT_HeapBlock* settings = pThis[FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings].Dereference();

        // auto slaveAddress = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4].NumericByRef().s4;
        // auto busSpeed = (TinyCLR_I2c_BusSpeed)settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed].NumericByRef().u4;

        // if (buffer == NULL || settings == NULL) {
            // LIB_TC_SET_AND_LEAVE(CLR_E_INVALID_OPERATION);
        // }

        // data = buffer->GetFirstElement();
        // offset = md.Stack.Arg2().NumericByRef().s4;
        // length = md.Stack.Arg3().NumericByRef().s4;

        // provider->SetActiveSettings(provider, slaveAddress, busSpeed);

        // if (provider->Read(provider, data + offset, length, result) != TinyCLR_Result::Success) {
            // LIB_TC_SET_AND_LEAVE(CLR_E_FAIL);
        // }
        // else {
            // CLR_RT_HeapBlock hbHdc;
            // hbHdc.SetInteger((INT32)length);
            // LIB_TC_CHECK_HRESULT(hbHdc.StoreToReference(md.Stack.ArgN(4), 0));
            // hbHdc.SetInteger((INT32)result);
            // LIB_TC_CHECK_HRESULT(hbHdc.StoreToReference(md.Stack.ArgN(5), 0));
        // }
    // }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::WriteInternal___VOID__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {    
    // auto provider = (const TinyCLR_I2c_Provider*)md.Stack.This()[FIELD___nativeProvider___I].NumericByRefConst().s4;

    // uint8_t* data = NULL;
    // int32_t offset = 0;
    // size_t length = 0;

    // TinyCLR_I2c_TransferStatus result;
    // CLR_RT_HeapBlock* pThis = md.Stack.This(); LIB_TC_FAULT_ON_NULL(pThis);

    // if (pThis[FIELD___m_disposed___BOOLEAN].NumericByRef().u1) {
        // LIB_TC_SET_AND_LEAVE(CLR_E_OBJECT_DISPOSED);
    // }

    // {
        // CLR_RT_HeapBlock_Array* buffer = md.Stack.Arg1().DereferenceArray();
        // CLR_RT_HeapBlock* settings = pThis[FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings].Dereference();

        // auto slaveAddress = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4].NumericByRef().s4;
        // auto busSpeed = (TinyCLR_I2c_BusSpeed)settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed].NumericByRef().u4;

        // if (buffer == NULL || settings == NULL) {
            // LIB_TC_SET_AND_LEAVE(CLR_E_INVALID_OPERATION);
        // }

        // data = buffer->GetFirstElement();
        // offset = md.Stack.Arg2().NumericByRef().s4;
        // length = md.Stack.Arg3().NumericByRef().s4;

        // provider->SetActiveSettings(provider, slaveAddress, busSpeed);

        // if (provider->Write(provider, data + offset, length, result) != TinyCLR_Result::Success) {
            // LIB_TC_SET_AND_LEAVE(CLR_E_FAIL);
        // }
        // else {
            // CLR_RT_HeapBlock hbHdc;
            // hbHdc.SetInteger((INT32)length);
            // LIB_TC_CHECK_HRESULT(hbHdc.StoreToReference(md.Stack.ArgN(4), 0));
            // hbHdc.SetInteger((INT32)result);
            // LIB_TC_CHECK_HRESULT(hbHdc.StoreToReference(md.Stack.ArgN(5), 0));
        // }
    // }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider::WriteReadInternal___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md) {
    // auto provider = (const TinyCLR_I2c_Provider*)md.Stack.This()[FIELD___nativeProvider___I].NumericByRefConst().s4;

    // uint8_t* writeData = NULL;
    // int32_t writeOffset = 0;
    // size_t writeLength = 0;

    // uint8_t* readData = NULL;
    // int32_t readOffset = 0;
    // size_t readLength = 0;

    // TinyCLR_I2c_TransferStatus result;
    // CLR_RT_HeapBlock* pThis = md.Stack.This(); LIB_TC_FAULT_ON_NULL(pThis);

    // if (pThis[FIELD___m_disposed___BOOLEAN].NumericByRef().u1) {
        // LIB_TC_SET_AND_LEAVE(CLR_E_OBJECT_DISPOSED);
    // }

    // {
        // CLR_RT_HeapBlock_Array* writeBuffer = md.Stack.Arg1().DereferenceArray();
        // CLR_RT_HeapBlock_Array* readBuffer = md.Stack.Arg4().DereferenceArray();
        // CLR_RT_HeapBlock* settings = pThis[FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings].Dereference();

        // auto slaveAddress = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_slaveAddress___I4].NumericByRef().s4;
        // auto busSpeed = (TinyCLR_I2c_BusSpeed)settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed].NumericByRef().u4;

        // if (writeBuffer == NULL || readBuffer == NULL || settings == NULL) {
            // LIB_TC_SET_AND_LEAVE(CLR_E_INVALID_OPERATION);
        // }

        // writeData = writeBuffer->GetFirstElement();
        // writeOffset = md.Stack.Arg2().NumericByRef().s4;
        // writeLength = md.Stack.Arg3().NumericByRef().s4;

        // readData = readBuffer->GetFirstElement();
        // readOffset = md.Stack.Arg5().NumericByRef().s4;
        // readLength = md.Stack.Arg6().NumericByRef().s4;

        // provider->SetActiveSettings(provider, slaveAddress, busSpeed);

        // if (provider->WriteRead(provider, writeData + writeOffset, writeLength, readData + readOffset, readLength, result) != TinyCLR_Result::Success) {
            // LIB_TC_SET_AND_LEAVE(CLR_E_FAIL);
        // }
        // else {
            // CLR_RT_HeapBlock hbHdc;
            // hbHdc.SetInteger((INT32)(writeLength + readLength));
            // LIB_TC_CHECK_HRESULT(hbHdc.StoreToReference(md.Stack.ArgN(7), 0));
            // hbHdc.SetInteger((INT32)result);
            // LIB_TC_CHECK_HRESULT(hbHdc.StoreToReference(md.Stack.ArgN(8), 0));
        // }
    // }

    return TinyCLR_Result::Success;
}
