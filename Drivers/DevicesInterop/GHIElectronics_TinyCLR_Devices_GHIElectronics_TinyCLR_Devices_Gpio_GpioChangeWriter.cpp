#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

#include <string.h>

struct GpioChangeWriter {
    // Helper Functions to access fields of managed object
    static uint32_t& Get_pin(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::FIELD___pin___U4, val); return val.Data.Numeric->U4; }
    static int8_t& Get_disposed(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::FIELD___disposed___BOOLEAN, val); return val.Data.Numeric->I1; }
    static uint32_t& Get_nativePointer(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self) { TinyCLR_Interop_ClrValue val; provider->GetField(provider, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::FIELD___nativePointer___U4, val); return val.Data.Numeric->U4; }

    // Declaration of stubs. These functions are implemented by Interop code developers
    static int8_t NativeConstructor(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param0, TinyCLR_Result& result);
    static void NativeDispose(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result);
    static int8_t NativeIsActive(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result);
    static void NativeSet(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param0, TinyCLR_Result& result);
    static int8_t NativeSet(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param0, TinyCLR_Interop_ClrValue::SzArrayType& param1, int32_t param2, int32_t param3, int8_t param4, TinyCLR_Result& result);
    static void NativeSet(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param0, TinyCLR_Interop_ClrValue::SzArrayType& param1, int32_t param2, int32_t param3, uint32_t param4, int8_t param5, uint32_t param6, TinyCLR_Result& result);
};

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::NativeConstructor___BOOLEAN__BOOLEAN(const TinyCLR_Interop_MethodData md) {
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

            ret.Data.Numeric->I1 = GpioChangeWriter::NativeConstructor(provider, self, param1.Data.Numeric->I1, result);

        }
    }
    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::NativeDispose___VOID(const TinyCLR_Interop_MethodData md) {

    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;

            provider->GetThisObject(provider, md.Stack, self);

            GpioChangeWriter::NativeDispose(provider, self, result);

        }
    }
    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::NativeIsActive___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;
            TinyCLR_Interop_ClrValue ret;

            provider->GetThisObject(provider, md.Stack, self);
            provider->GetReturn(provider, md.Stack, ret);

            ret.Data.Numeric->I1 = GpioChangeWriter::NativeIsActive(provider, self, result);

        }
    }
    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::NativeSet___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;
            TinyCLR_Interop_ClrValue param1;

            provider->GetThisObject(provider, md.Stack, self);
            provider->GetArgument(provider, md.Stack, 1, param1);

            GpioChangeWriter::NativeSet(provider, self, param1.Data.Numeric->I1, result);

        }
    }
    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::NativeSet___BOOLEAN__BOOLEAN__SZARRAY_U4__I4__I4__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;
            TinyCLR_Interop_ClrValue param1;
            TinyCLR_Interop_ClrValue param2;
            TinyCLR_Interop_ClrValue param3;
            TinyCLR_Interop_ClrValue param4;
            TinyCLR_Interop_ClrValue param5;
            TinyCLR_Interop_ClrValue ret;

            provider->GetThisObject(provider, md.Stack, self);
            provider->GetArgument(provider, md.Stack, 1, param1);
            provider->GetArgument(provider, md.Stack, 2, param2);
            provider->GetArgument(provider, md.Stack, 3, param3);
            provider->GetArgument(provider, md.Stack, 4, param4);
            provider->GetArgument(provider, md.Stack, 5, param5);
            provider->GetReturn(provider, md.Stack, ret);

            ret.Data.Numeric->I1 = GpioChangeWriter::NativeSet(provider, self, param1.Data.Numeric->I1, param2.Data.SzArray, param3.Data.Numeric->I4, param4.Data.Numeric->I4, param5.Data.Numeric->I1, result);
        }
    }
    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter::NativeSet___VOID__BOOLEAN__SZARRAY_U4__I4__I4__U4__BOOLEAN__U4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;
    {
        auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        if (provider != nullptr) {
            const TinyCLR_Interop_ClrObject* self;
            TinyCLR_Interop_ClrValue param1;
            TinyCLR_Interop_ClrValue param2;
            TinyCLR_Interop_ClrValue param3;
            TinyCLR_Interop_ClrValue param4;
            TinyCLR_Interop_ClrValue param5;
            TinyCLR_Interop_ClrValue param6;
            TinyCLR_Interop_ClrValue param7;

            provider->GetThisObject(provider, md.Stack, self);
            provider->GetArgument(provider, md.Stack, 1, param1);
            provider->GetArgument(provider, md.Stack, 2, param2);
            provider->GetArgument(provider, md.Stack, 3, param3);
            provider->GetArgument(provider, md.Stack, 4, param4);
            provider->GetArgument(provider, md.Stack, 5, param5);
            provider->GetArgument(provider, md.Stack, 6, param6);
            provider->GetArgument(provider, md.Stack, 7, param7);

            GpioChangeWriter::NativeSet(provider, self, param1.Data.Numeric->I1, param2.Data.SzArray, param3.Data.Numeric->I4, param4.Data.Numeric->I4, param5.Data.Numeric->U4, param6.Data.Numeric->I1, param7.Data.Numeric->U4, result);
        }
    }
    return result;
}


struct OC {
    uint32_t *buffer;
    uint32_t BUFFER_MAX_SIZE;
    uint32_t bufferIndex;
    uint32_t bufferSize;
    uint32_t pin;
    uint8_t repeat;
    uint8_t currentState;
    uint8_t isActive;
};

#define COMPLETION_LATENCY_SCHED_TIME	100
volatile int32_t completionLatency;
int32_t carrierFrequency_hz_latency = -1;
static const TinyCLR_Gpio_Controller* gpioProvider = nullptr;

void CompletionLatencyIsr(void* arg) {
    uint64_t end = TinyCLR_Interop_CurrentTime() / 10;
    uint64_t start;

    start = *((uint64_t*)arg);

    completionLatency = end - start;
}

void ComputeCompletionLatency() {

    // compute completion latency
    if (TinyCLR_Interop_GetStateInterrupt()) // must be on!
    {
        uint64_t start, end;
        uint32_t getTimeLatency;

        // Get time latency
        {
            //DISABLE_INTERRUPTS_SCOPED(irq);
            start = TinyCLR_Interop_CurrentTime() / 10;
            end = TinyCLR_Interop_CurrentTime() / 10;
            getTimeLatency = end - start;
        }


        completionLatency -= getTimeLatency + COMPLETION_LATENCY_SCHED_TIME;
        if (completionLatency < 0)
            completionLatency = 0;
    }
    else {
        completionLatency = 0;
    }

}

void OC_ISR(void* arg) {
    // TODO
}

void GpioChangeWriter::NativeDispose(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    OC *oc = (OC*)Get_nativePointer(provider, self);
    if (!oc)
        return;

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)oc->pin, TinyCLR_Gpio_PinDriveMode::InputPullUp);
    gpioProvider->ReleasePin(gpioProvider, gpioController, (int32_t)oc->pin);

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

    if (oc->buffer != nullptr)
        memoryProvider->Free(memoryProvider, oc->buffer);

    memoryProvider->Free(memoryProvider, oc);

    oc = nullptr;

}

int8_t GpioChangeWriter::NativeIsActive(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, TinyCLR_Result& result) {
    result = TinyCLR_Result::Success;

    if (Get_disposed(provider, self)) {
        result = TinyCLR_Result::Disposed;
        return false;
    }

    OC *oc = (OC*)Get_nativePointer(provider, self);

    if (!oc) {
        result = TinyCLR_Result::ArgumentNull;
        return false;
    }

    return oc->isActive;
}

int8_t GpioChangeWriter::NativeConstructor(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param1, TinyCLR_Result& result) {

    uint8_t param0 = Get_pin(provider, self);

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

    OC *oc = (OC*)memoryProvider->Allocate(memoryProvider, sizeof(OC));

    if (!oc) {
        result = TinyCLR_Result::OutOfMemory;
        return 0;
    }

    if (gpioProvider == nullptr && (gpioProvider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider)) == nullptr) {
        result = TinyCLR_Result::InvalidOperation;
        return 0;
    }

    ComputeCompletionLatency();

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->SetDriveMode(gpioProvider, gpioController, (int32_t)param0, TinyCLR_Gpio_PinDriveMode::Output);
    gpioProvider->Write(gpioProvider, gpioController, (int32_t)param0, param1 ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);

    oc->pin = param0;
    oc->buffer = nullptr;
    oc->bufferSize = 0;
    oc->isActive = false;

    Get_nativePointer(provider, self) = (uint32_t)oc;
    Get_disposed(provider, self) = false;

    // calculate carrierFrequency_hz_latency
    if (carrierFrequency_hz_latency == -1) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        uint64_t start, end, a;

        start = TinyCLR_Interop_CurrentTime();
        end = TinyCLR_Interop_CurrentTime();
        a = end - start;

        start = TinyCLR_Interop_CurrentTime();
        gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, param1 ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);
        TinyCLR_Interop_Delay(10);
        end = TinyCLR_Interop_CurrentTime();

        carrierFrequency_hz_latency = end - start - a - 100;
        if (carrierFrequency_hz_latency < 0)
            carrierFrequency_hz_latency = 0;

        // round
        if ((carrierFrequency_hz_latency % 10) >= 5)
            carrierFrequency_hz_latency = (carrierFrequency_hz_latency + 1) / 10;
        else
            carrierFrequency_hz_latency /= 10;
    }

    return 1;
}

void GpioChangeWriter::NativeSet(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param0, TinyCLR_Result& result) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (Get_disposed(provider, self)) {
        result = TinyCLR_Result::Disposed;
        return;
    }

    OC *oc = (OC*)Get_nativePointer(provider, self);

    if (!oc) {
        result = TinyCLR_Result::ArgumentNull;
        return;
    }

    oc->isActive = false;

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, param0 ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);

}

int8_t GpioChangeWriter::NativeSet(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t param0, TinyCLR_Interop_ClrValue::SzArrayType& param1, int32_t param2, int32_t param3, int8_t param4, TinyCLR_Result& result) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (Get_disposed(provider, self)) {
        result = TinyCLR_Result::Disposed;
        return 0;
    }

    if ((param2 < 0) || (param3 < 0) || ((param2 + param3) > param1.Length)) {
        result = TinyCLR_Result::ArgumentOutOfRange;
        return 0;
    }

    OC *oc = (OC*)Get_nativePointer(provider, self);

    if (!oc) {
        result = TinyCLR_Result::ArgumentNull;
        return 0;
    }

    oc->isActive = false;

    // is there data??
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

    if (param3) {
        if (oc->buffer != nullptr)
            memoryProvider->Free(memoryProvider, oc->buffer);

        oc->buffer = (uint32_t*)memoryProvider->Allocate(memoryProvider, sizeof(uint32_t) * param3);

        if (!oc->buffer)
            return 0;

        memcpy(oc->buffer, (uint32_t*)param1.Data + param2, sizeof(uint32_t) * param3);

        oc->bufferIndex = 0;
        oc->bufferSize = param3;
        oc->repeat = param4;

        int32_t latency = 0;
        int32_t minLatency;
        if (latency)
            minLatency = latency;
        else
            minLatency = 2;

        for (int i = 0; i < oc->bufferSize; i++) {
            if (oc->buffer[i] > latency)
                oc->buffer[i] -= latency;

            if (oc->buffer[i] < minLatency)
                oc->buffer[i] = minLatency;
        }

        oc->currentState = param0;

        auto gpioController = 0; //TODO Temporary set to 0

        gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, param0 ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);

        oc->isActive = true;
    }

    return 1;
}

void GpioChangeWriter::NativeSet(const TinyCLR_Interop_Manager* provider, const TinyCLR_Interop_ClrObject* self, int8_t initialValue, TinyCLR_Interop_ClrValue::SzArrayType& timingsBuffer_us, int32_t offset, int32_t count, uint32_t lastBitHoldTime_us, int8_t disableInterrupts, uint32_t carrierFrequency_hz, TinyCLR_Result& result) {

    if ((offset < 0) || (count < 0) || ((offset + count) > timingsBuffer_us.Length)) {
        result = TinyCLR_Result::ArgumentOutOfRange;
        return;
    }

    OC *oc = (OC*)Get_nativePointer(provider, self);
    if (!oc) {
        result = TinyCLR_Result::ArgumentNull;
        return;
    }

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        oc->isActive = false;
    }

    int i;
    int f, fCount;
    int wasInterrptEnabled;
    uint32_t *buffer = (uint32_t*)timingsBuffer_us.Data + offset;

    // carrierFrequency_hz
    if (carrierFrequency_hz) {
        carrierFrequency_hz = ((uint32_t)1000000 / 2 / (carrierFrequency_hz));

        // avoids dividing by 0 for little values
        if (carrierFrequency_hz <= carrierFrequency_hz_latency)
            carrierFrequency_hz = carrierFrequency_hz_latency + 1;
        else
            carrierFrequency_hz -= carrierFrequency_hz_latency;
    }

    if (disableInterrupts) {
        wasInterrptEnabled = TinyCLR_Interop_GetStateInterrupt();
        if (wasInterrptEnabled) {
            TinyCLR_Interop_DisableInterrupt();
        }
    }

    auto gpioController = 0; //TODO Temporary set to 0

    gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, initialValue ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);

    for (i = 0; i < count; i++) {
        if (!(initialValue && carrierFrequency_hz)) {
            TinyCLR_Interop_Delay(buffer[i]);
        }
        else {
            fCount = (buffer[i] / (carrierFrequency_hz + carrierFrequency_hz_latency));
            for (f = 0; f < fCount; f += 2) {
                gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, TinyCLR_Gpio_PinValue::High);
                TinyCLR_Interop_Delay(carrierFrequency_hz);

                gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, TinyCLR_Gpio_PinValue::Low);
                TinyCLR_Interop_Delay(carrierFrequency_hz);
            }
        }

        initialValue = !initialValue;
        gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, initialValue ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);
    }

    if (lastBitHoldTime_us) {
        if (!(initialValue && carrierFrequency_hz)) {
            TinyCLR_Interop_Delay(lastBitHoldTime_us);
        }
        else {
            fCount = (lastBitHoldTime_us / (carrierFrequency_hz + carrierFrequency_hz_latency));
            for (f = 0; f < fCount; f += 2) {
                gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, TinyCLR_Gpio_PinValue::High);
                TinyCLR_Interop_Delay(carrierFrequency_hz);

                gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, TinyCLR_Gpio_PinValue::Low);
                TinyCLR_Interop_Delay(carrierFrequency_hz);
            }

            gpioProvider->Write(gpioProvider, gpioController, (int32_t)oc->pin, TinyCLR_Gpio_PinValue::High);
        }
    }

    if (disableInterrupts) {
        if (wasInterrptEnabled) {
            TinyCLR_Interop_EnableInterrupt();
        }
    }

}
