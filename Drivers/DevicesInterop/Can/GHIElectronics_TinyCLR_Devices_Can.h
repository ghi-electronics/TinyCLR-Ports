#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming {
    static const size_t FIELD___Propagation__BackingField___I4 = 1;
    static const size_t FIELD___Phase1__BackingField___I4 = 2;
    static const size_t FIELD___Phase2__BackingField___I4 = 3;
    static const size_t FIELD___BaudratePrescaler__BackingField___I4 = 4;
    static const size_t FIELD___SynchronizationJumpWidth__BackingField___I4 = 5;
    static const size_t FIELD___UseMultiBitSampling__BackingField___BOOLEAN = 6;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_CanController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesCanProviderICanControllerProvider = 1;
    static const size_t FIELD___MessageReceived___GHIElectronicsTinyCLRDevicesCanMessageReceivedEventHandler = 2;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesCanErrorReceivedEventHandler = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_CanMessage {
    static const size_t FIELD___data___SZARRAY_U1 = 1;
    static const size_t FIELD___ArbitrationId__BackingField___I4 = 2;
    static const size_t FIELD___IsExtendedId__BackingField___BOOLEAN = 3;
    static const size_t FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN = 4;
    static const size_t FIELD___Length__BackingField___I4 = 5;
    static const size_t FIELD___Timestamp__BackingField___mscorlibSystemDateTime = 6;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_ErrorReceivedEventArgs {
    static const size_t FIELD___Error__BackingField___GHIElectronicsTinyCLRDevicesCanCanError = 1;
    static const size_t FIELD___Timestamp__BackingField___mscorlibSystemDateTime = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_MessageReceivedEventArgs {
    static const size_t FIELD___Count__BackingField___I4 = 1;
    static const size_t FIELD___Timestamp__BackingField___mscorlibSystemDateTime = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___messageReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 2;
    static const size_t FIELD___errorReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 3;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 4;
    static const size_t FIELD___MessageReceived___GHIElectronicsTinyCLRDevicesCanMessageReceivedEventHandler = 5;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesCanErrorReceivedEventHandler = 6;

    static TinyCLR_Result get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MessagesToWrite___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MessagesToRead___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_CanWriteMessage___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_CanReadMessage___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_WriteErrorCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadErrorCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SourceClock___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Enable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Disable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReadMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetBitTiming___VOID__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetExplicitFilters___VOID__SZARRAY_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetGroupFilters___VOID__SZARRAY_I4__SZARRAY_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Can;
