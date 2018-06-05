#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

void PinChangedIsr(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChange pinChange) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Uart.PinChanged", self->Parent->Name, self->Index, (uint64_t)pinChange, 0, 0);
}

void ErrorReceivedIsr(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_Error error) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Uart.ErrorReceived", self->Parent->Name, self->Index, (uint64_t)error, 0, 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeOpen___VOID__U4__U4__U4__U4__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

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
        if (provider->Acquire(provider) == TinyCLR_Result::Success) {
            if (provider->SetActiveSettings(provider, baudRate, dataBits, (TinyCLR_Uart_Parity)parity, (TinyCLR_Uart_StopBitCount)stopBits, (TinyCLR_Uart_Handshake)handshaking) == TinyCLR_Result::Success) {
                provider->SetPinChangedHandler(provider, PinChangedIsr);
                provider->SetErrorReceivedHandler(provider, ErrorReceivedIsr);

                return TinyCLR_Result::Success;
            }

            return TinyCLR_Result::ArgumentInvalid;
        }

        return TinyCLR_Result::SharingViolation;
    }

    return TinyCLR_Result::ArgumentNull;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeClose___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    return provider->Release(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeFlush___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    provider->Flush(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeRead___I4__SZARRAY_U1__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1); // buffer
    auto arg2 = TinyCLR_Interop_GetArguments(md, 2); // offset
    auto arg3 = TinyCLR_Interop_GetArguments(md, 3); // count
    auto arg4 = TinyCLR_Interop_GetArguments(md, 4); // Timeout

    size_t count = arg3.Data.Numeric->U4;

    uint8_t* ptr = (uint8_t*)arg1.Data.SzArray.Data + arg2.Data.Numeric->I4;

    bool result = true;

    provider->Read(provider, ptr, count);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)count;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::NativeWrite___I4__SZARRAY_U1__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1); // buffer
    auto arg2 = TinyCLR_Interop_GetArguments(md, 2); // offset
    auto arg3 = TinyCLR_Interop_GetArguments(md, 3); // count
    auto arg4 = TinyCLR_Interop_GetArguments(md, 4); // Timeout

    size_t count = arg3.Data.Numeric->U4;

    uint8_t* ptr = (uint8_t*)arg1.Data.SzArray.Data + arg2.Data.Numeric->I4;

    bool result = true;

    provider->Write(provider, ptr, count);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)count;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_ReadBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto result = provider->GetReadBufferSize(provider, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::set_ReadBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    return provider->SetReadBufferSize(provider, (size_t)arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_WriteBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto result = provider->GetWriteBufferSize(provider, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::set_WriteBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    return provider->SetWriteBufferSize(provider, (size_t)arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_UnreadCount___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto result = provider->GetUnreadCount(provider, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::get_UnwrittenCount___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    size_t val;

    auto result = provider->GetUnwrittenCount(provider, val);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    return provider->ClearReadBuffer(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Uart_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    return provider->ClearWriteBuffer(provider);
}
