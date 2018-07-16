#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

struct GpioChangeReader {
    static int32_t NativeRead(uint32_t param0, int8_t* param1, TinyCLR_Interop_ClrValue::SzArrayType& param2, int32_t param3, int32_t param4, int32_t param5, TinyCLR_Result& result);
    static int32_t NativeRead(uint32_t param0, int8_t param1, TinyCLR_Interop_ClrValue::SzArrayType& param2, int32_t param3, int32_t param4, int32_t param5, TinyCLR_Result& result);
};

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeReader::NativeRead___STATIC___I4__U4__BYREF_BOOLEAN__SZARRAY_U4__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    TinyCLR_Result result = TinyCLR_Result::Success;

    if (provider != nullptr) {
        TinyCLR_Interop_ClrValue param0;
        TinyCLR_Interop_ClrValue param1;
        TinyCLR_Interop_ClrValue param2;
        TinyCLR_Interop_ClrValue param3;
        TinyCLR_Interop_ClrValue param4;
        TinyCLR_Interop_ClrValue param5;
        TinyCLR_Interop_ClrValue ret;

        provider->GetArgument(provider, md.Stack, 0, param0);
        provider->GetArgument(provider, md.Stack, 1, param1);
        provider->GetArgument(provider, md.Stack, 2, param2);
        provider->GetArgument(provider, md.Stack, 3, param3);
        provider->GetArgument(provider, md.Stack, 4, param4);
        provider->GetArgument(provider, md.Stack, 5, param5);
        provider->GetReturn(provider, md.Stack, ret);

        TinyCLR_Result result;

        ret.Data.Numeric->I4 = GpioChangeReader::NativeRead(param0.Data.Numeric->U4, &param1.Data.Numeric->I1, param2.Data.SzArray, param3.Data.Numeric->I4, param4.Data.Numeric->I4, param5.Data.Numeric->I4, result);
    }

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeReader::NativeRead___STATIC___I4__U4__BOOLEAN__SZARRAY_U4__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    TinyCLR_Result result = TinyCLR_Result::Success;

    if (provider != nullptr) {
        TinyCLR_Interop_ClrValue param0;
        TinyCLR_Interop_ClrValue param1;
        TinyCLR_Interop_ClrValue param2;
        TinyCLR_Interop_ClrValue param3;
        TinyCLR_Interop_ClrValue param4;
        TinyCLR_Interop_ClrValue param5;
        TinyCLR_Interop_ClrValue ret;

        provider->GetArgument(provider, md.Stack, 0, param0);
        provider->GetArgument(provider, md.Stack, 1, param1);
        provider->GetArgument(provider, md.Stack, 2, param2);
        provider->GetArgument(provider, md.Stack, 3, param3);
        provider->GetArgument(provider, md.Stack, 4, param4);
        provider->GetArgument(provider, md.Stack, 5, param5);
        provider->GetReturn(provider, md.Stack, ret);

        ret.Data.Numeric->I4 = GpioChangeReader::NativeRead(param0.Data.Numeric->U4, param1.Data.Numeric->I1, param2.Data.SzArray, param3.Data.Numeric->I4, param4.Data.Numeric->I4, param5.Data.Numeric->I4, result);
    }

    return result;
}

static bool ReadValue(uint8_t pin) {

    auto provider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider);

    TinyCLR_Gpio_PinValue value;

    auto gpioController = 0; //TODO Temporary set to 0

    provider->Read(provider, gpioController, (int32_t)pin, value);

    return value == TinyCLR_Gpio_PinValue::High;
}

int32_t _Read(uint32_t pin, uint8_t currentState, uint32_t* buffer, int32_t count, int32_t timeout) {
    int32_t retVal = 0;

    uint64_t start = TinyCLR_Interop_CurrentTime() / 10000;
    uint64_t startPinTime = TinyCLR_Interop_CurrentTime() / 10;
    uint64_t endPinTime;

    while (retVal < count) {
        if (ReadValue(pin) != currentState) {
            endPinTime = TinyCLR_Interop_CurrentTime() / 10;

            buffer[retVal] = endPinTime - startPinTime;

            startPinTime = endPinTime;

            currentState = !currentState;

            retVal++;
        }

        if (timeout > -1) {
            if (((TinyCLR_Interop_CurrentTime() / 10000) - start) > timeout)
                break;
        }
    }

    return retVal;
}

int32_t GpioChangeReader::NativeRead(uint32_t pin, int8_t * initialState, TinyCLR_Interop_ClrValue::SzArrayType& buffer, int32_t offset, int32_t count, int32_t timeout, TinyCLR_Result& result) {
    if ((timeout < -1) || (offset < 0) || (count < 0) || ((offset + count) > buffer.Length)) {
        result = TinyCLR_Result::ArgumentOutOfRange;
        return 0;
    }

    auto provider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider);

    if (provider == nullptr) {
        result = TinyCLR_Result::ArgumentNull;
        return 0;
    }

    *initialState = ReadValue(pin);

    return _Read(pin, *initialState, (uint32_t*)buffer.Data + offset, count, timeout);
}

int32_t GpioChangeReader::NativeRead(uint32_t pin, int8_t waitForState, TinyCLR_Interop_ClrValue::SzArrayType& buffer, int32_t offset, int32_t count, int32_t timeout, TinyCLR_Result& result) {
    if ((timeout < -1) || (offset < 0) || (count < 0) || ((offset + count) > buffer.Length)) {
        result = TinyCLR_Result::ArgumentOutOfRange;
        return 0;
    }

    auto provider = (const TinyCLR_Gpio_Controller*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider);

    if (provider == nullptr) {
        result = TinyCLR_Result::ArgumentNull;
        return 0;
    }

    uint64_t start = TinyCLR_Interop_CurrentTime() / 10000;
    uint64_t end = start;

    while (true) {
        if (ReadValue(pin) == waitForState)
            break;

        if (timeout > -1) {
            end = TinyCLR_Interop_CurrentTime() / 10000;
            if ((end - start) > timeout)
                return 0;
        }
    }

    if (timeout > -1) {
        uint32_t t = end - start;
        timeout = (timeout > t) ? (timeout - t) : (0);
    }

    return _Read(pin, waitForState, (uint32_t*)buffer.Data + offset, count, timeout);
}
