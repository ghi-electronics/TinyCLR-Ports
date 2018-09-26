#include "GHIElectronics_TinyCLR_Devices_Signals.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::Write___VOID__SZARRAY_mscorlibSystemTimeSpan__I4__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue arrArg, offsetArg, countArg, apiFld, pinFld, idleFld, disableFld, generateFld, freqFld;
    const TinyCLR_Interop_ClrObject* self;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arrArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, offsetArg);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, countArg);
    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___gpioApi___I, apiFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___pinNumber___I4, pinFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___idleValue___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue, idleFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___DisableInterrupts__BackingField___BOOLEAN, disableFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___GenerateCarrierFrequency__BackingField___BOOLEAN, generateFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator::FIELD___CarrierFrequency__BackingField___I8, freqFld);

    auto time = reinterpret_cast<const TinyCLR_NativeTime_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::NativeTimeController));
    auto memory = reinterpret_cast<const TinyCLR_Memory_Manager*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::MemoryManager));
    auto interrupt = reinterpret_cast<const TinyCLR_Interrupt_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InterruptController));

    auto arr = reinterpret_cast<uint64_t*>(arrArg.Data.SzArray.Data) + offsetArg.Data.Numeric->I4;
    auto len = countArg.Data.Numeric->I4;
    auto gpio = reinterpret_cast<const TinyCLR_Gpio_Controller*>(apiFld.Data.Numeric->I);
    auto pin = static_cast<uint32_t>(pinFld.Data.Numeric->I4);
    auto idleState = static_cast<TinyCLR_Gpio_PinValue>(idleFld.Data.Numeric->I4);
    auto disableInterrupts = disableFld.Data.Numeric->Boolean;
    auto generateCarrierFrequency = generateFld.Data.Numeric->Boolean;
    auto carrierFrequency = freqFld.Data.Numeric->U8;

    if (generateCarrierFrequency)
        return TinyCLR_Result::NotImplemented;

    auto mem = reinterpret_cast<uint64_t*>(memory->Allocate(memory, len * sizeof(uint64_t)));
    auto next = idleState;

    if (mem == nullptr)
        return TinyCLR_Result::OutOfMemory;

    for (auto i = 0; i < len; i++)
        mem[i] = time->ConvertSystemTimeToNativeTime(time, arr[i * 2 + 1]);

    gpio->Write(gpio, pin, idleState);

    if (disableInterrupts)
        interrupt->Disable();

    for (auto i = 0; i < len; i++) {
        next = next == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;

        gpio->Write(gpio, pin, next);

        time->Wait(time, mem[i]);
    }

    gpio->Write(gpio, pin, idleState);

    if (disableInterrupts)
        interrupt->Enable();

    memory->Free(memory, mem);

    return TinyCLR_Result::Success;
}
