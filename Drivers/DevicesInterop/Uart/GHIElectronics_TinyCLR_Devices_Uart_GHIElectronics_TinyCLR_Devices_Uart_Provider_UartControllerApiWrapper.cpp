#include "GHIElectronics_TinyCLR_Devices_Uart.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

void TinyCLR_Uart_DataReceivedIsr(const TinyCLR_Uart_Controller* self, size_t count, uint64_t timestamp) {
    extern const TinyCLR_Api_Manager* apiManager;
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    int32_t controllerIndex = *(reinterpret_cast<int32_t*>(self->ApiInfo->State));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Uart.DataReceived", self->ApiInfo->Name, (uint64_t)count, 0, 0, 0, timestamp);
}

void TinyCLR_Uart_ErrorReceivedIsr(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_Error error, uint64_t timestamp) {
    extern const TinyCLR_Api_Manager* apiManager;
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    int32_t controllerIndex = *(reinterpret_cast<int32_t*>(self->ApiInfo->State));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Uart.ErrorReceived", self->ApiInfo->Name, (uint64_t)error, 0, 0, 0, timestamp);
}

void TinyCLR_Uart_ClearToSendIsr(const TinyCLR_Uart_Controller* self, bool state, uint64_t timestamp) {
    extern const TinyCLR_Api_Manager* apiManager;
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    int32_t controllerIndex = *(reinterpret_cast<int32_t*>(self->ApiInfo->State));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Uart.ClearToSendChanged", self->ApiInfo->Name, (uint64_t)state, 0, 0, 0, timestamp);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetWriteBufferSize(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    return api->SetWriteBufferSize(api, arg0.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetReadBufferSize(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    return api->SetReadBufferSize(api, arg0.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_BytesToWrite___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetBytesToWrite(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_BytesToRead___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetBytesToRead(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_IsRequestToSendEnabled___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    return api->GetIsRequestToSendEnabled(api, ret.Data.Numeric->Boolean);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::set_IsRequestToSendEnabled___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    return api->SetIsRequestToSendEnabled(api, arg0.Data.Numeric->Boolean);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_ClearToSendState___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    return api->GetClearToSendState(api, ret.Data.Numeric->Boolean);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Enable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Enable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Disable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Disable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::SetActiveSettings___VOID__I4__I4__GHIElectronicsTinyCLRDevicesUartUartParity__GHIElectronicsTinyCLRDevicesUartUartStopBitCount__GHIElectronicsTinyCLRDevicesUartUartHandshake(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue args[5];
    TinyCLR_Uart_Settings settings;

    for (auto i = 0; i < sizeof(args) / sizeof(TinyCLR_Interop_ClrValue); i++) {
        md.InteropManager->GetArgument(md.InteropManager, md.Stack, i, args[i]);
    }

    settings.BaudRate = args[0].Data.Numeric->I4;
    settings.DataBits = args[1].Data.Numeric->I4;
    settings.Parity = static_cast<TinyCLR_Uart_Parity>(args[2].Data.Numeric->I4);
    settings.StopBits = static_cast<TinyCLR_Uart_StopBitCount>(args[3].Data.Numeric->I4);
    settings.Handshaking = static_cast<TinyCLR_Uart_Handshake>(args[4].Data.Numeric->I4);

    return api->SetActiveSettings(api, &settings);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Flush___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Flush(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Read___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue args[3];

    for (auto i = 0; i < sizeof(args) / sizeof(TinyCLR_Interop_ClrValue); i++) {
        md.InteropManager->GetArgument(md.InteropManager, md.Stack, i, args[i]);
    };

    auto buffer = reinterpret_cast<uint8_t*>(args[0].Data.SzArray.Data);
    auto offset = args[1].Data.Numeric->I4;
    auto count = static_cast<size_t>(args[2].Data.Numeric->I4);

    auto result = api->Read(api, buffer + offset, count);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Write___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue args[3];

    for (auto i = 0; i < sizeof(args) / sizeof(TinyCLR_Interop_ClrValue); i++) {
        md.InteropManager->GetArgument(md.InteropManager, md.Stack, i, args[i]);
    };

    auto buffer = reinterpret_cast<uint8_t*>(args[0].Data.SzArray.Data);
    auto offset = args[1].Data.Numeric->I4;
    auto count = static_cast<size_t>(args[2].Data.Numeric->I4);

    auto result = api->Write(api, reinterpret_cast<const uint8_t*>(buffer + offset), count);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->ClearWriteBuffer(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->ClearReadBuffer(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::SetClearToSendChangedEventEnabled___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue arg;

    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    auto enable = arg.Data.Numeric->Boolean;

    return api->SetClearToSendChangedHandler(api, enable ? TinyCLR_Uart_ClearToSendIsr : nullptr);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::SetDataReceivedEventEnabled___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue arg;

    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    auto enable = arg.Data.Numeric->Boolean;

    return api->SetDataReceivedHandler(api, enable ? TinyCLR_Uart_DataReceivedIsr : nullptr);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::SetErrorReceivedEventEnabled___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue arg;

    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    auto enable = arg.Data.Numeric->Boolean;

    return api->SetErrorReceivedHandler(api, enable ? TinyCLR_Uart_ErrorReceivedIsr : nullptr);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
