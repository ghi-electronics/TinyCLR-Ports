#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

void PinChangedIsr(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_PinChange pinChange) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropManager));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Uart.PinChanged", self->ApiInfo->Name, controller, (uint64_t)pinChange, 0, 0);
}

void ErrorReceivedIsr(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_Error error) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropManager));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Uart.ErrorReceived", self->ApiInfo->Name, controller, (uint64_t)error, 0, 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeOpen___VOID__U4__U4__U4__U4__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);
    auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
    auto arg3 = TinyCLR_Interop_GetArguments(md, 3);
    auto arg4 = TinyCLR_Interop_GetArguments(md, 4);
    auto arg5 = TinyCLR_Interop_GetArguments(md, 5);

    uint32_t baudRate = arg1.Data.Numeric->U4;
    uint32_t parity = arg2.Data.Numeric->U4;
    uint32_t dataBits = arg3.Data.Numeric->U4;
    uint32_t stopBits = arg4.Data.Numeric->U4;
    uint32_t handshaking = arg5.Data.Numeric->U4;

    if (provider != nullptr) {
        if (provider->Acquire(provider, controller) == TinyCLR_Result::Success) {
            if (provider->SetActiveSettings(provider, controller, baudRate, dataBits, (TinyCLR_Uart_Parity)parity, (TinyCLR_Uart_StopBitCount)stopBits, (TinyCLR_Uart_Handshake)handshaking) == TinyCLR_Result::Success) {
                provider->SetPinChangedHandler(provider, controller, PinChangedIsr);
                provider->SetErrorReceivedHandler(provider, controller, ErrorReceivedIsr);

                return TinyCLR_Result::Success;
            }

            return TinyCLR_Result::ArgumentInvalid;
        }

        return TinyCLR_Result::SharingViolation;
    }

    return TinyCLR_Result::ArgumentNull;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeClose___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    return provider->Release(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeFlush___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    provider->Flush(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeRead___I4__SZARRAY_U1__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1); // buffer
    auto arg2 = TinyCLR_Interop_GetArguments(md, 2); // offset
    auto arg3 = TinyCLR_Interop_GetArguments(md, 3); // count
    auto arg4 = TinyCLR_Interop_GetArguments(md, 4); // Timeout

    size_t count = arg3.Data.Numeric->U4;

    uint8_t* ptr = (uint8_t*)arg1.Data.SzArray.Data + arg2.Data.Numeric->I4;

    bool result = true;

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    provider->Read(provider, controller, ptr, count);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)count;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeWrite___I4__SZARRAY_U1__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1); // buffer
    auto arg2 = TinyCLR_Interop_GetArguments(md, 2); // offset
    auto arg3 = TinyCLR_Interop_GetArguments(md, 3); // count
    auto arg4 = TinyCLR_Interop_GetArguments(md, 4); // Timeout

    size_t count = arg3.Data.Numeric->U4;

    uint8_t* ptr = (uint8_t*)arg1.Data.SzArray.Data + arg2.Data.Numeric->I4;

    bool result = true;

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    provider->Write(provider, controller, ptr, count);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)count;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_ReadBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    auto result = provider->GetReadBufferSize(provider, controller, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::set_ReadBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    return provider->SetReadBufferSize(provider, controller, (size_t)arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_WriteBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    auto result = provider->GetWriteBufferSize(provider, controller, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::set_WriteBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    return provider->SetWriteBufferSize(provider, controller, (size_t)arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_UnreadCount___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    auto result = provider->GetUnreadCount(provider, controller, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_UnwrittenCount___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    auto result = provider->GetUnwrittenCount(provider, controller, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    return provider->ClearReadBuffer(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Controller*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto controllerProvider = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___U4);
    auto controller = controllerProvider.Data.Numeric->U4;

    return provider->ClearWriteBuffer(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 0);

    auto ret = TinyCLR_Interop_GetReturn(md);

    auto provider = (const TinyCLR_Uart_Controller*)(arg.Data.Numeric->I4);

    int32_t count;

    if (provider->GetControllerCount(provider, count) == TinyCLR_Result::Success) {
        ret.Data.Numeric->I4 = count;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}
