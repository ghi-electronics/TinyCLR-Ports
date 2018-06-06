#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    return provider->Acquire(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    return provider->Release(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioControllerProvider::get_PinCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)provider->GetPinCount(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioControllerProvider::AcquireNative___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->AcquirePin(provider, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioControllerProvider::ReleaseNative___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->ReleasePin(provider, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 0);

    auto ret = TinyCLR_Interop_GetReturn(md);

    auto provider = (const TinyCLR_Gpio_Provider*)(arg.Data.Numeric->I4);

    int32_t count;

    if (provider->GetControllerCount(provider, count) == TinyCLR_Result::Success) {
        ret.Data.Numeric->I4 = count;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}
