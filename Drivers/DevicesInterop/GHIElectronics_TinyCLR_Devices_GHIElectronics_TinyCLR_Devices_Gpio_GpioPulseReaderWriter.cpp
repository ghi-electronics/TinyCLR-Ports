#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

struct GpioPulseReaderWriter {
    static int8_t& Get_disposed(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___disposed___BOOLEAN, val); return val.Data.Numeric->I1; }
    static int32_t& Get_timeout(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___timeout___I4, val); return val.Data.Numeric->I4; }
    static int32_t& Get_pulseLength(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___pulseLength___I4, val); return val.Data.Numeric->I4; }
    static int8_t& Get_pulseState(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___pulseState___BOOLEAN, val); return val.Data.Numeric->I1; }
    static int8_t& Get_echoState(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___echoState___BOOLEAN, val); return val.Data.Numeric->I1; }
    static uint8_t& Get_pulsePin(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___pulsePin___I4, val); return val.Data.Numeric->U1; }
    static uint8_t& Get_echoPin(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___echoPin___I4, val); return val.Data.Numeric->U1; }
    static int32_t& Get_mode(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___mode___GHIElectronicsTinyCLRDevicesGpioGpioPulseReaderWriterMode, val); return val.Data.Numeric->I4; }
    static int32_t& Get_driveMode(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::FIELD___driveMode___I4, val); return val.Data.Numeric->I4; }

    static int64_t NativeReadDrainTime(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result);
    static int64_t NativeReadEcho(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param0, TinyCLR_Result& result);
    static void NativeFinalize(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result);
};

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::NativeReadDrainTime___I8(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;
            TinyCLR_Interop_ClrValue ret;

            provider->GetThisObject(provider, md.Stack, self);
            provider->GetReturn(provider, md.Stack, ret);

            ret.Data.Numeric->I8 = GpioPulseReaderWriter::NativeReadDrainTime(provider, self, result);


        }
    }
    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::NativeReadEcho___I8__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;
            TinyCLR_Interop_ClrValue param1;
            TinyCLR_Interop_ClrValue ret;

            provider->GetThisObject(provider, md.Stack, self);
            provider->GetArgument(provider, md.Stack, 1, param1);
            provider->GetReturn(provider, md.Stack, ret);

            ret.Data.Numeric->I8 = GpioPulseReaderWriter::NativeReadEcho(provider, self, param1.Data.Numeric->I1, result);
        }
    }

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter::NativeFinalize___VOID(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;

            provider->GetThisObject(provider, md.Stack, self);

            GpioPulseReaderWriter::NativeFinalize(provider, self, result);


        }
    }
    return result;
}

#define GetMicroSeconds() (TinyCLR_Interop_CurrentTime() / 10)

static bool ReadValue(uint8_t pin) {
    TinyCLR_Gpio_PinValue value;

    auto gpioProvider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider);

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->Read(gpioProvider, gpioController, (int32_t)pin, value);

    return value == TinyCLR_Gpio_PinValue::High;
}

int64_t GpioPulseReaderWriter::NativeReadDrainTime(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result) {
    uint8_t pulsePin = Get_pulsePin(provider, self);
    int64_t pulseLength = Get_pulseLength(provider, self);
    int8_t pulseState = Get_pulseState(provider, self);
    TinyCLR_Gpio_PinDriveMode resistorMode = (TinyCLR_Gpio_PinDriveMode)Get_driveMode(provider, self);
    int64_t timeout = Get_timeout(provider, self);
    int64_t endTime = 0, start = 0, now = 0;

    auto gpioProvider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider);

    if (gpioProvider == nullptr) {
        result = TinyCLR_Result::ArgumentNull;
        return 0;
    }

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)pulsePin, TinyCLR_Gpio_PinDriveMode::Output);

    TinyCLR_Interop_Delay(pulseLength);

    endTime = GetMicroSeconds() + timeout;
    start = GetMicroSeconds();

    gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)pulsePin, resistorMode);

    while (true) {
        now = GetMicroSeconds();

        if (ReadValue(pulsePin) != pulseState)
            return now - start;

        if (timeout != -1 && now > endTime)
            return -1;
    }
}

int64_t GpioPulseReaderWriter::NativeReadEcho(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t readUntil, TinyCLR_Result& result) {
    uint8_t pulsePin = Get_pulsePin(provider, self);
    int64_t pulseLength = Get_pulseLength(provider, self);
    int8_t pulseState = Get_pulseState(provider, self);
    uint8_t echoPin = Get_echoPin(provider, self);
    int8_t echoState = Get_echoState(provider, self);
    TinyCLR_Gpio_PinDriveMode resistorMode = (TinyCLR_Gpio_PinDriveMode)Get_driveMode(provider, self);
    int64_t timeout = Get_timeout(provider, self);
    int64_t endTime = GetMicroSeconds() + timeout;
    int64_t start = 0;

    auto gpioProvider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider);

    if (gpioProvider == nullptr) {
        result = TinyCLR_Result::ArgumentNull;
        return 0;
    }

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)pulsePin, TinyCLR_Gpio_PinDriveMode::Output);
    gpioProvider->Write(gpioProvider, gpioController, (int32_t)pulsePin, !pulseState ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);
    TinyCLR_Interop_Delay(10);

    gpioProvider->Write(gpioProvider, gpioController, (int32_t)pulsePin, pulseState ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);
    TinyCLR_Interop_Delay(pulseLength);

    gpioProvider->Write(gpioProvider, gpioController, (int32_t)pulsePin, !pulseState ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);

    gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)echoPin, resistorMode);

    if (readUntil)
        start = GetMicroSeconds();

    while (ReadValue(echoPin) != echoState)
        if (timeout != -1 && GetMicroSeconds() > endTime)
            return -1;

    if (!readUntil) {
        start = GetMicroSeconds();

        while (ReadValue(echoPin) == echoState)
            if (timeout != -1 && GetMicroSeconds() > endTime)
                return -1;
    }

    return GetMicroSeconds() - start;
}

void GpioPulseReaderWriter::NativeFinalize(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result) {
    uint8_t pulsePin = Get_pulsePin(provider, self);
    uint8_t echoPin = Get_echoPin(provider, self);

    auto gpioProvider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider);

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)pulsePin, TinyCLR_Gpio_PinDriveMode::InputPullUp);
    gpioProvider->ReleasePin(gpioProvider, gpioController, (int32_t)pulsePin);

    if (pulsePin != echoPin) {
        gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)echoPin, TinyCLR_Gpio_PinDriveMode::InputPullUp);
        gpioProvider->ReleasePin(gpioProvider, gpioController, (int32_t)echoPin);
    }
}
