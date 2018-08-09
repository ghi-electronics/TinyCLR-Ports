#include "GHIElectronics_TinyCLR_Devices_Signals.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::Write___VOID__SZARRAY_I8__I4__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue arrArg, apiFld, pinFld;
    const TinyCLR_Interop_ClrObject* self;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arrArg);
    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___gpioApi___I, apiFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___pinNumber___I4, pinFld);

    auto time = reinterpret_cast<const TinyCLR_NativeTime_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::NativeTimeController));
    auto arr = reinterpret_cast<int64_t*>(arrArg.Data.SzArray.Data);
    auto len = arrArg.Data.SzArray.Length;
    auto gpio = reinterpret_cast<const TinyCLR_Gpio_Controller*>(apiFld.Data.Numeric->I);
    auto pin = static_cast<uint32_t>(pinFld.Data.Numeric->I4);
    auto next = TinyCLR_Gpio_PinValue::Low;

    gpio->Read(gpio, pin, next);

    for (auto i = 0; i < len; i++)
        arr[i] = time->ConvertSystemTimeToNativeTime(time, arr[i] * 10);

    for (auto i = 0; i < len; i++) {
        next = next == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;

        gpio->Write(gpio, pin, next);

        time->Wait(time, arr[i]);
    }

    for (auto i = 0; i < len; i++)
        arr[i] = time->ConvertNativeTimeToSystemTime(time, arr[i]) / 10;

    return TinyCLR_Result::Success;
}
