#include "GHIElectronics_TinyCLR_Devices_Signals.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::Read___I4__BYREF_GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue__SZARRAY_I8__I4__I4(const TinyCLR_Interop_MethodData md) {


    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::Read___I4__GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue__SZARRAY_I8__I4__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, initialArg, arrArg, offsetArg, countArg, apiFld, pinFld;
    const TinyCLR_Interop_ClrObject* self;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, initialArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arrArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, offsetArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, countArg);
    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___gpioApi___I, apiFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___pinNumber___I4, pinFld);

    auto initial = static_cast<TinyCLR_Gpio_PinValue>(initialArg.Data.Numeric->I4);
    auto time = reinterpret_cast<const TinyCLR_NativeTime_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::NativeTimeController));
    auto arr = reinterpret_cast<int64_t*>(arrArg.Data.SzArray.Data) + offsetArg.Data.Numeric->I4;
    auto len = countArg.Data.Numeric->I4;
    auto gpio = reinterpret_cast<const TinyCLR_Gpio_Controller*>(apiFld.Data.Numeric->I);
    auto pin = static_cast<uint32_t>(pinFld.Data.Numeric->I4);
    auto state = TinyCLR_Gpio_PinValue::Low;
    auto next = TinyCLR_Gpio_PinValue::Low;

    do {
        gpio->Read(gpio, pin, state);
    } while (state != initial);

    int32_t count = 0;
    auto last = time->GetNativeTime(time);

    next = state == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;

    while (count < len) {
        gpio->Read(gpio, pin, state);

        if (state != next) {
            auto now = time->GetNativeTime(time);

            arr[count++] = now - last;
            last = now;
            next = state;
        }
    }

    for (auto i = 0; i < len; i++)
        arr[i] = time->ConvertNativeTimeToSystemTime(time, arr[i]) / 10;

    ret.Data.Numeric->I4 = count;

    return TinyCLR_Result::Success;
}
