#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_ClearToSendChangedEventArgs {
    static const size_t FIELD___State__BackingField___BOOLEAN = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_DataReceivedEventArgs {
    static const size_t FIELD___Count__BackingField___I4 = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_ErrorReceivedEventArgs {
    static const size_t FIELD___Error__BackingField___GHIElectronicsTinyCLRDevicesUartUartError = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___clearToSendChangedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 2;
    static const size_t FIELD___dataReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 3;
    static const size_t FIELD___errorReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 4;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 5;
    static const size_t FIELD___ClearToSendChanged___GHIElectronicsTinyCLRDevicesUartClearToSendChangedEventHandler = 6;
    static const size_t FIELD___DataReceived___GHIElectronicsTinyCLRDevicesUartDataReceivedEventHandler = 7;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesUartErrorReceivedEventHandler = 8;

    static TinyCLR_Result get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_BytesToWrite___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_BytesToRead___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_IsRequestToSendEnabled___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_IsRequestToSendEnabled___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ClearToSendState___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Enable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Disable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetActiveSettings___VOID__I4__I4__GHIElectronicsTinyCLRDevicesUartUartParity__GHIElectronicsTinyCLRDevicesUartUartStopBitCount__GHIElectronicsTinyCLRDevicesUartUartHandshake(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Flush___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Uart_GHIElectronics_TinyCLR_Devices_Uart_UartController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesUartProviderIUartControllerProvider = 1;
    static const size_t FIELD___ClearToSendChanged___GHIElectronicsTinyCLRDevicesUartClearToSendChangedEventHandler = 2;
    static const size_t FIELD___DataReceived___GHIElectronicsTinyCLRDevicesUartDataReceivedEventHandler = 3;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesUartErrorReceivedEventHandler = 4;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Uart;
