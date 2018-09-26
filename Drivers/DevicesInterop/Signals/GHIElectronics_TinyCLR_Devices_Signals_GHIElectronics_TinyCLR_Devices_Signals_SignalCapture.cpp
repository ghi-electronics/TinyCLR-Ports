#include "GHIElectronics_TinyCLR_Devices_Signals.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::Read___I4__BYREF_GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue__SZARRAY_mscorlibSystemTimeSpan__I4__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, initialArg, arrArg, offsetArg, countArg, apiFld, pinFld, disableFld, timeoutFld;
    const TinyCLR_Interop_ClrObject* self;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, initialArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arrArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, offsetArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, countArg);
    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___gpioApi___I, apiFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___pinNumber___I4, pinFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___DisableInterrupts__BackingField___BOOLEAN, disableFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___Timeout__BackingField___mscorlibSystemTimeSpan, timeoutFld);

    auto time = reinterpret_cast<const TinyCLR_NativeTime_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::NativeTimeController));
    auto memory = reinterpret_cast<const TinyCLR_Memory_Manager*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::MemoryManager));
    auto interrupt = reinterpret_cast<const TinyCLR_Interrupt_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InterruptController));

    auto arr = reinterpret_cast<uint64_t*>(arrArg.Data.SzArray.Data) + offsetArg.Data.Numeric->I4;
    auto len = countArg.Data.Numeric->I4;
    auto gpio = reinterpret_cast<const TinyCLR_Gpio_Controller*>(apiFld.Data.Numeric->I);
    auto pin = static_cast<uint32_t>(pinFld.Data.Numeric->I4);
    auto disableInterrupts = disableFld.Data.Numeric->Boolean;
    auto timeout = timeoutFld.Data.Numeric->U8;

    auto mem = reinterpret_cast<uint64_t*>(memory->Allocate(memory, len * sizeof(uint64_t)));
    auto currentState = TinyCLR_Gpio_PinValue::Low;
    auto nextState = TinyCLR_Gpio_PinValue::Low;

    if (mem == nullptr)
        return TinyCLR_Result::OutOfMemory;

    if (disableInterrupts)
        interrupt->Disable();

    gpio->Read(gpio, pin, currentState);

    initialArg.Data.Numeric->I4 = static_cast<int32_t>(currentState);

    nextState = currentState == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;

    int32_t count = 0;
    auto currentTime = time->GetNativeTime(time);
    auto lastTime = currentTime;
    auto endTime = currentTime + time->ConvertSystemTimeToNativeTime(time, timeout);

    while (count < len && currentTime < endTime) {
        currentTime = time->GetNativeTime(time);

        gpio->Read(gpio, pin, currentState);

        if (currentState == nextState) {
            mem[count++] = currentTime - lastTime;
            lastTime = currentTime;
            nextState = nextState == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;
        }
    }

    if (disableInterrupts)
        interrupt->Enable();

    ret.Data.Numeric->I4 = count;

    for (auto i = 0; i < count; i++)
        arr[i * 2 + 1] = time->ConvertNativeTimeToSystemTime(time, mem[i]);

    memory->Free(memory, mem);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::Read___I4__GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue__SZARRAY_mscorlibSystemTimeSpan__I4__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, initialArg, arrArg, offsetArg, countArg, apiFld, pinFld, disableFld, timeoutFld;
    const TinyCLR_Interop_ClrObject* self;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, initialArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arrArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, offsetArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, countArg);
    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___gpioApi___I, apiFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___pinNumber___I4, pinFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___DisableInterrupts__BackingField___BOOLEAN, disableFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture::FIELD___Timeout__BackingField___mscorlibSystemTimeSpan, timeoutFld);

    auto time = reinterpret_cast<const TinyCLR_NativeTime_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::NativeTimeController));
    auto memory = reinterpret_cast<const TinyCLR_Memory_Manager*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::MemoryManager));
    auto interrupt = reinterpret_cast<const TinyCLR_Interrupt_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InterruptController));

    auto arr = reinterpret_cast<uint64_t*>(arrArg.Data.SzArray.Data) + offsetArg.Data.Numeric->I4;
    auto len = countArg.Data.Numeric->I4;
    auto gpio = reinterpret_cast<const TinyCLR_Gpio_Controller*>(apiFld.Data.Numeric->I);
    auto pin = static_cast<uint32_t>(pinFld.Data.Numeric->I4);
    auto disableInterrupts = disableFld.Data.Numeric->Boolean;
    auto timeout = timeoutFld.Data.Numeric->U8;

    auto mem = reinterpret_cast<uint64_t*>(memory->Allocate(memory, len * sizeof(uint64_t)));
    auto currentState = TinyCLR_Gpio_PinValue::Low;
    auto nextState = static_cast<TinyCLR_Gpio_PinValue>(initialArg.Data.Numeric->I4);

    if (mem == nullptr)
        return TinyCLR_Result::OutOfMemory;

    int32_t count = 0;
    auto currentTime = time->GetNativeTime(time);
    auto lastTime = currentTime;
    auto endTime = currentTime + time->ConvertSystemTimeToNativeTime(time, timeout);

    if (disableInterrupts)
        interrupt->Disable();

    do {
        currentTime = time->GetNativeTime(time);
        gpio->Read(gpio, pin, currentState);
    } while (currentState != nextState && currentTime < endTime);

    lastTime = currentTime;
    nextState = currentState == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;

    while (count < len && currentTime < endTime) {
        currentTime = time->GetNativeTime(time);

        gpio->Read(gpio, pin, currentState);

        if (currentState == nextState) {
            mem[count++] = currentTime - lastTime;
            lastTime = currentTime;
            nextState = nextState == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;
        }
    }

    if (disableInterrupts)
        interrupt->Enable();

    ret.Data.Numeric->I4 = count;

    for (auto i = 0; i < count; i++)
        arr[i * 2 + 1] = time->ConvertNativeTimeToSystemTime(time, mem[i]);

    memory->Free(memory, mem);

    return TinyCLR_Result::Success;
}
