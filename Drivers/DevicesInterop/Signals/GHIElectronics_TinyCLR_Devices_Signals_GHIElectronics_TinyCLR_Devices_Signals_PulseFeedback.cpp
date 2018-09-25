#include "GHIElectronics_TinyCLR_Devices_Signals.h"

enum class PulseFeedbackMode {
    DrainDuration,
    EchoDuration,
    DurationUntilEcho
};

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::GeneratePulse___mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, modeFld, gpioFld, pulsePinFld, echoPinFld, disableFld, timeoutFld, lengthFld, pulseValueFld, echoValueFld, pulseDriveFld, echoDriveFld;
    const TinyCLR_Interop_ClrObject* self;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);
    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___mode___GHIElectronicsTinyCLRDevicesSignalsPulseFeedbackMode, modeFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___gpioApi___I, gpioFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___pulsePinNumber___I4, pulsePinFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___echoPinNumber___I4, echoPinFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___DisableInterrupts__BackingField___BOOLEAN, disableFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___Timeout__BackingField___mscorlibSystemTimeSpan, timeoutFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___PulseLength__BackingField___mscorlibSystemTimeSpan, lengthFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___PulsePinValue__BackingField___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue, pulseValueFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___EchoPinValue__BackingField___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinValue, echoValueFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___PulsePinDriveMode__BackingField___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode, pulseDriveFld);
    md.InteropManager->GetField(md.InteropManager, self, Interop_GHIElectronics_TinyCLR_Devices_Signals_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback::FIELD___EchoPinDriveMode__BackingField___GHIElectronicsTinyCLRDevicesGpioGHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode, echoDriveFld);

    auto time = reinterpret_cast<const TinyCLR_NativeTime_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::NativeTimeController));
    auto interrupt = reinterpret_cast<const TinyCLR_Interrupt_Controller*>(md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InterruptController));

    auto feedbackMode = static_cast<PulseFeedbackMode>(modeFld.Data.Numeric->I4);
    auto gpio = reinterpret_cast<const TinyCLR_Gpio_Controller*>(gpioFld.Data.Numeric->I);
    auto pulsePin = static_cast<uint32_t>(pulsePinFld.Data.Numeric->I4);
    auto echoPin = static_cast<uint32_t>(echoPinFld.Data.Numeric->I4);
    auto disableInterrupts = disableFld.Data.Numeric->Boolean;
    auto timeout = timeoutFld.Data.Numeric->U8;
    auto pulseLength = lengthFld.Data.Numeric->U8;
    auto pulseValue = static_cast<TinyCLR_Gpio_PinValue>(pulseValueFld.Data.Numeric->I4);
    auto echoValue = static_cast<TinyCLR_Gpio_PinValue>(echoValueFld.Data.Numeric->I4);
    auto pulseDriveMode = static_cast<TinyCLR_Gpio_PinDriveMode>(pulseDriveFld.Data.Numeric->I4);
    auto echoDriveMode = static_cast<TinyCLR_Gpio_PinDriveMode>(echoDriveFld.Data.Numeric->I4);

    pulseLength = time->ConvertSystemTimeToNativeTime(time, pulseLength);
    timeout = time->ConvertSystemTimeToNativeTime(time, timeout);

    if (disableInterrupts)
        interrupt->Disable(true);

    if (feedbackMode == PulseFeedbackMode::DrainDuration) {
        int64_t endTime = 0, start = 0, now = 0;
        auto current = TinyCLR_Gpio_PinValue::Low;

        gpio->Write(gpio, pulsePin, pulseValue);
        gpio->SetDriveMode(gpio, pulsePin, TinyCLR_Gpio_PinDriveMode::Output);

        time->Wait(time, pulseLength);

        start = time->GetNativeTime(time);
        endTime = start + timeout;

        gpio->SetDriveMode(gpio, echoPin, echoDriveMode);

        while (true) {
            now = time->GetNativeTime(time);

            gpio->Read(gpio, echoPin, current);

            if (current != echoValue) {
                ret.Data.Numeric->I8 = now - start;

                break;
            }

            if (now > endTime) {
                ret.Data.Numeric->I8 = 0;

                break;
            }
        }
    }
    else {
        int64_t endTime = 0, start = 0;
        auto current = TinyCLR_Gpio_PinValue::Low;
        auto notPulseValue = pulseValue == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High;

        endTime = time->GetNativeTime(time) + timeout;

        gpio->Write(gpio, pulsePin, notPulseValue);
        gpio->SetDriveMode(gpio, pulsePin, TinyCLR_Gpio_PinDriveMode::Output);
        time->Wait(time, time->ConvertSystemTimeToNativeTime(time, 100));

        gpio->Write(gpio, pulsePin, pulseValue);
        time->Wait(time, pulseLength);

        gpio->Write(gpio, pulsePin, notPulseValue);

        gpio->SetDriveMode(gpio, echoPin, echoDriveMode);

        if (feedbackMode == PulseFeedbackMode::DurationUntilEcho)
            start = time->GetNativeTime(time);

        while (true) {
            gpio->Read(gpio, echoPin, current);

            if (current == echoValue)
                break;

            if (time->GetNativeTime(time) > endTime) {
                ret.Data.Numeric->I8 = 0;

                break;
            }
        }

        if (feedbackMode == PulseFeedbackMode::EchoDuration) {
            start = time->GetNativeTime(time);

            while (true) {
                gpio->Read(gpio, echoPin, current);

                if (current != echoValue) {
                    ret.Data.Numeric->I8 = time->GetNativeTime(time) - start;

                    break;
                }

                if (time->GetNativeTime(time) > endTime) {
                    ret.Data.Numeric->I8 = 0;

                    break;
                }
            }
        }
        else {
            ret.Data.Numeric->I8 = time->GetNativeTime(time) - start;
        }
    }

    if (disableInterrupts)
        interrupt->Enable(true);

    ret.Data.Numeric->I8 = time->ConvertNativeTimeToSystemTime(time, ret.Data.Numeric->I8);

    return TinyCLR_Result::Success;
}
