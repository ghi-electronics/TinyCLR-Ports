#include "GHIElectronics_TinyCLR_Devices.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::TransferFullDuplexInternal___VOID__SZARRAY_U1__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {


    auto provider = (const TinyCLR_Spi_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* writeData = NULL;
    int32_t writeOffset = 0;
    size_t writeLength = 0;

    uint8_t* readData = NULL;
    int32_t readOffset = 0;
    size_t readLength = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        while (1); // TODO
        // CLR_RT_HeapBlock_Array* writeBuffer = md.Stack.Arg1().DereferenceArray();
        // CLR_RT_HeapBlock_Array* readBuffer = md.Stack.Arg3().DereferenceArray();
        // CLR_RT_HeapBlock* settings = pThis[FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings].Dereference();

        // auto chipSelectLine = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4].NumericByRef().s4;
        // auto clockFrequency = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4].NumericByRef().s4;
        // auto dataBitLength = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4].NumericByRef().s4;
        // auto mode = (TinyCLR_Spi_Mode)settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode].NumericByRef().u4;

        // if (writeBuffer == NULL || readBuffer == NULL || settings == NULL) {
            // return TinyCLR_Result::ArgumentNull;
        // }

        // writeData = writeBuffer->GetFirstElement();
        // writeOffset = md.Stack.Arg2().NumericByRef().s4;

        // readData = readBuffer->GetFirstElement();
        // readOffset = md.Stack.Arg4().NumericByRef().s4;

        // writeLength = readLength = md.Stack.Arg5().NumericByRef().s4;

        // provider->SetActiveSettings(provider, chipSelectLine, clockFrequency, dataBitLength, mode);

        // if (provider->TransferFullDuplex(provider, writeData + writeOffset, writeLength, readData + readOffset, readLength) != TinyCLR_Result::Success) {
             // return TinyCLR_Result::InvalidOperation;
        // }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::TransferSequentialInternal___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {


    auto provider = (const TinyCLR_Spi_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* writeData = NULL;
    int32_t writeOffset = 0;
    size_t writeLength = 0;

    uint8_t* readData = NULL;
    int32_t readOffset = 0;
    size_t readLength = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        while (1); // TODO
        // CLR_RT_HeapBlock_Array* writeBuffer = md.Stack.Arg1().DereferenceArray();
        // CLR_RT_HeapBlock_Array* readBuffer = md.Stack.Arg4().DereferenceArray();
        // CLR_RT_HeapBlock* settings = pThis[FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings].Dereference();

        // auto chipSelectLine = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4].NumericByRef().s4;
        // auto clockFrequency = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4].NumericByRef().s4;
        // auto dataBitLength = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4].NumericByRef().s4;
        // auto mode = (TinyCLR_Spi_Mode)settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode].NumericByRef().u4;

        // if (writeBuffer == NULL || readBuffer == NULL || settings == NULL) {
            // return TinyCLR_Result::ArgumentNull;
        // }

        // writeData = writeBuffer->GetFirstElement();
        // writeOffset = md.Stack.Arg2().NumericByRef().s4;
        // writeLength = md.Stack.Arg3().NumericByRef().s4;

        // readData = readBuffer->GetFirstElement();
        // readOffset = md.Stack.Arg5().NumericByRef().s4;
        // readLength = md.Stack.Arg6().NumericByRef().s4;

        // provider->SetActiveSettings(provider, chipSelectLine, clockFrequency, dataBitLength, mode);

        // if (provider->TransferSequential(provider, writeData + writeOffset, writeLength, readData + readOffset, readLength) != TinyCLR_Result::Success) {
            // return TinyCLR_Result::InvalidOperation;
        // }
    }


    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::WriteInternal___VOID__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {


    auto provider = (const TinyCLR_Spi_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* data = NULL;
    int32_t offset = 0;
    size_t length = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        while (1); // TODO
        // CLR_RT_HeapBlock_Array* buffer = md.Stack.Arg1().DereferenceArray();
        // CLR_RT_HeapBlock* settings = pThis[FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings].Dereference();

        // auto chipSelectLine = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4].NumericByRef().s4;
        // auto clockFrequency = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4].NumericByRef().s4;
        // auto dataBitLength = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4].NumericByRef().s4;
        // auto mode = (TinyCLR_Spi_Mode)settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode].NumericByRef().u4;

        // if (buffer == NULL || settings == NULL) {
            // return TinyCLR_Result::ArgumentNull;
        // }

        // data = buffer->GetFirstElement();
        // offset = md.Stack.Arg2().NumericByRef().s4;
        // length = md.Stack.Arg3().NumericByRef().s4;

        // provider->SetActiveSettings(provider, chipSelectLine, clockFrequency, dataBitLength, mode);

        // if (provider->Write(provider, data + offset, length) != TinyCLR_Result::Success) {
            // return TinyCLR_Result::InvalidOperation;
        // }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::ReadInternal___VOID__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {


    auto provider = (const TinyCLR_Spi_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* data = NULL;
    int32_t offset = 0;
    size_t length = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        while (1); // TODO
        // CLR_RT_HeapBlock_Array* buffer = md.Stack.Arg1().DereferenceArray();
        // CLR_RT_HeapBlock* settings = pThis[FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings].Dereference();

        // auto chipSelectLine = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4].NumericByRef().s4;
        // auto clockFrequency = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4].NumericByRef().s4;
        // auto dataBitLength = settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4].NumericByRef().s4;
        // auto mode = (TinyCLR_Spi_Mode)settings[Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode].NumericByRef().u4;

        // if (buffer == NULL || settings == NULL) {
            // return TinyCLR_Result::ArgumentNull;
        // }

        // data = buffer->GetFirstElement();
        // offset = md.Stack.Arg2().NumericByRef().s4;
        // length = md.Stack.Arg3().NumericByRef().s4;

        // provider->SetActiveSettings(provider, chipSelectLine, clockFrequency, dataBitLength, mode);

        // if (provider->Read(provider, data + offset, length) != TinyCLR_Result::Success) {
            // return TinyCLR_Result::InvalidOperation;
        // }
    }

    return TinyCLR_Result::Success;
}
