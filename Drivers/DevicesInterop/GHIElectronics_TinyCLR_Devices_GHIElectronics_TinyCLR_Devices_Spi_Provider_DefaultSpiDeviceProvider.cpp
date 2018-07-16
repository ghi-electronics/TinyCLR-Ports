#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::TransferFullDuplexInternal___VOID__SZARRAY_U1__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Spi_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* writeData = nullptr;
    int32_t writeOffset = 0;
    size_t writeLength = 0;

    uint8_t* readData = nullptr;
    int32_t readOffset = 0;
    size_t readLength = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        auto arg1 = TinyCLR_Interop_GetArguments(md, 1);
        auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);
        auto arg4 = TinyCLR_Interop_GetArguments(md, 4);
        auto arg5 = TinyCLR_Interop_GetArguments(md, 5);

        uint8_t* writeBuffer = (uint8_t*)arg1.Data.SzArray.Data;
        uint8_t* readBuffer = (uint8_t*)arg3.Data.SzArray.Data;;

        auto settings = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings);

        auto chipSelectLineObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4);
        auto clockFrequencyObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4);
        auto dataBitLengthObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4);
        auto modeObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode);

        auto chipSelectLine = chipSelectLineObject.Data.Numeric->I4;
        auto clockFrequency = clockFrequencyObject.Data.Numeric->I4;
        auto dataBitLength = dataBitLengthObject.Data.Numeric->I4;
        auto mode = (TinyCLR_Spi_Mode)modeObject.Data.Numeric->U4;

        if (writeBuffer == nullptr || readBuffer == nullptr || settings.Object == nullptr) {
            return TinyCLR_Result::ArgumentNull;
        }

        writeData = writeBuffer;

        writeOffset = arg2.Data.Numeric->I4;

        readData = readBuffer;
        readOffset = arg4.Data.Numeric->I4;

        writeLength = readLength = arg5.Data.Numeric->I4;

        auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___parent___GHIElectronicsTinyCLRDevicesSpiProviderDefaultSpiControllerProvider);

        auto controllerObject = TinyCLR_Interop_GetFieldInObject(md, controllerProvider.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___idx___I4);

        auto controller = controllerObject.Data.Numeric->I4;

        provider->SetActiveSettings(provider, controller, chipSelectLine, clockFrequency, dataBitLength, mode);

        if (provider->TransferFullDuplex(provider, controller, writeData + writeOffset, writeLength, readData + readOffset, readLength) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::TransferSequentialInternal___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Spi_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* writeData = nullptr;
    int32_t writeOffset = 0;
    size_t writeLength = 0;

    uint8_t* readData = nullptr;
    int32_t readOffset = 0;
    size_t readLength = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        auto arg1 = TinyCLR_Interop_GetArguments(md, 1);
        auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);
        auto arg4 = TinyCLR_Interop_GetArguments(md, 4);
        auto arg5 = TinyCLR_Interop_GetArguments(md, 5);
        auto arg6 = TinyCLR_Interop_GetArguments(md, 6);

        uint8_t* writeBuffer = (uint8_t*)arg1.Data.SzArray.Data;
        uint8_t* readBuffer = (uint8_t*)arg4.Data.SzArray.Data;;

        auto settings = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings);

        auto chipSelectLineObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4);
        auto clockFrequencyObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4);
        auto dataBitLengthObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4);
        auto modeObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode);

        auto chipSelectLine = chipSelectLineObject.Data.Numeric->I4;
        auto clockFrequency = clockFrequencyObject.Data.Numeric->I4;
        auto dataBitLength = dataBitLengthObject.Data.Numeric->I4;
        auto mode = (TinyCLR_Spi_Mode)modeObject.Data.Numeric->U4;

        if (writeBuffer == nullptr || readBuffer == nullptr || settings.Object == nullptr) {
            return TinyCLR_Result::ArgumentNull;
        }

        writeData = writeBuffer;
        writeOffset = arg2.Data.Numeric->I4;
        writeLength = arg3.Data.Numeric->I4;

        readData = readBuffer;
        readOffset = arg5.Data.Numeric->I4;
        readLength = arg6.Data.Numeric->I4;

        auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___parent___GHIElectronicsTinyCLRDevicesSpiProviderDefaultSpiControllerProvider);

        auto controllerObject = TinyCLR_Interop_GetFieldInObject(md, controllerProvider.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___idx___I4);

        auto controller = controllerObject.Data.Numeric->I4;

        provider->SetActiveSettings(provider, controller, chipSelectLine, clockFrequency, dataBitLength, mode);

        if (provider->TransferSequential(provider, controller, writeData + writeOffset, writeLength, readData + readOffset, readLength) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::WriteInternal___VOID__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Spi_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* data = nullptr;
    int32_t offset = 0;
    size_t length = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        auto arg1 = TinyCLR_Interop_GetArguments(md, 1);
        auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);

        uint8_t* buffer = (uint8_t*)arg1.Data.SzArray.Data;

        auto settings = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings);

        auto chipSelectLineObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4);
        auto clockFrequencyObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4);
        auto dataBitLengthObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4);
        auto modeObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode);

        auto chipSelectLine = chipSelectLineObject.Data.Numeric->I4;
        auto clockFrequency = clockFrequencyObject.Data.Numeric->I4;
        auto dataBitLength = dataBitLengthObject.Data.Numeric->I4;
        auto mode = (TinyCLR_Spi_Mode)modeObject.Data.Numeric->U4;

        if (buffer == nullptr || settings.Object == nullptr) {
            return TinyCLR_Result::ArgumentNull;
        }

        data = buffer;

        offset = arg2.Data.Numeric->I4;
        length = arg3.Data.Numeric->I4;

        auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___parent___GHIElectronicsTinyCLRDevicesSpiProviderDefaultSpiControllerProvider);

        auto controllerObject = TinyCLR_Interop_GetFieldInObject(md, controllerProvider.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___idx___I4);

        auto controller = controllerObject.Data.Numeric->I4;

        provider->SetActiveSettings(provider, controller, chipSelectLine, clockFrequency, dataBitLength, mode);

        if (provider->Write(provider, controller, data + offset, length) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider::ReadInternal___VOID__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Spi_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    uint8_t* data = nullptr;
    int32_t offset = 0;
    size_t length = 0;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto fld = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    {
        auto arg1 = TinyCLR_Interop_GetArguments(md, 1);
        auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
        auto arg3 = TinyCLR_Interop_GetArguments(md, 3);

        uint8_t* buffer = (uint8_t*)arg1.Data.SzArray.Data;

        auto settings = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings);

        auto chipSelectLineObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_chipSelectionLine___I4);
        auto clockFrequencyObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_clockFrequency___I4);
        auto dataBitLengthObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_dataBitLength___I4);
        auto modeObject = TinyCLR_Interop_GetFieldInObject(md, settings.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode);

        auto chipSelectLine = chipSelectLineObject.Data.Numeric->I4;
        auto clockFrequency = clockFrequencyObject.Data.Numeric->I4;
        auto dataBitLength = dataBitLengthObject.Data.Numeric->I4;
        auto mode = (TinyCLR_Spi_Mode)modeObject.Data.Numeric->U4;

        if (buffer == nullptr || settings.Object == nullptr) {
            return TinyCLR_Result::ArgumentNull;
        }

        data = buffer;

        offset = arg2.Data.Numeric->I4;
        length = arg3.Data.Numeric->I4;

        auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___parent___GHIElectronicsTinyCLRDevicesSpiProviderDefaultSpiControllerProvider);

        auto controllerObject = TinyCLR_Interop_GetFieldInObject(md, controllerProvider.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___idx___I4);

        auto controller = controllerObject.Data.Numeric->I4;

        provider->SetActiveSettings(provider, controller, chipSelectLine, clockFrequency, dataBitLength, mode);

        if (provider->Read(provider, controller, data + offset, length) != TinyCLR_Result::Success) {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    return TinyCLR_Result::Success;
}
