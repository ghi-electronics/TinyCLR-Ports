#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetWriteBufferSize(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    return api->SetWriteBufferSize(api, arg0.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetReadBufferSize(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    return api->SetReadBufferSize(api, arg0.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_BytesToWrite___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetBytesToWrite(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_BytesToRead___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetBytesToRead(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_IsRequestToSendEnabled___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    return api->GetIsRequestToSendEnabled(api, ret.Data.Numeric->Boolean);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::set_IsRequestToSendEnabled___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    return api->SetIsRequestToSendEnabled(api, arg0.Data.Numeric->Boolean);

    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::get_ClearToSendState___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    return api->GetClearToSendState(api, ret.Data.Numeric->Boolean);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Enable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Enable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Disable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Disable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::SetActiveSettings___VOID__I4__I4__GHIElectronicsTinyCLRDevicesUartUartParity__GHIElectronicsTinyCLRDevicesUartUartStopBitCount__GHIElectronicsTinyCLRDevicesUartUartHandshake(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);

    auto baudRate = arg0.Data.Numeric->I4;
    auto dataBits = arg1.Data.Numeric->I4;
    auto parity = static_cast<TinyCLR_Uart_Parity>(arg2.Data.Numeric->I4);
    auto stopBits = static_cast<TinyCLR_Uart_StopBitCount>(arg3.Data.Numeric->I4);
    auto handshaking = static_cast<TinyCLR_Uart_Handshake>(arg4.Data.Numeric->I4);

    return api->SetActiveSettings(api, baudRate, dataBits, parity, stopBits, handshaking);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Flush___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Flush(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Read___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto buffer = reinterpret_cast<uint8_t*>(arg0.Data.SzArray.Data);
    auto offset = arg1.Data.Numeric->I4;
    auto count = static_cast<size_t>(arg2.Data.Numeric->I4);

    auto result = api->Read(api, buffer + offset, count);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Write___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto buffer = reinterpret_cast<uint8_t*>(arg0.Data.SzArray.Data);
    auto offset = arg1.Data.Numeric->I4;
    auto count = static_cast<size_t>(arg2.Data.Numeric->I4);

    auto result = api->Write(api, reinterpret_cast<const uint8_t*>(buffer + offset), count);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->ClearWriteBuffer(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->ClearReadBuffer(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Uart_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
