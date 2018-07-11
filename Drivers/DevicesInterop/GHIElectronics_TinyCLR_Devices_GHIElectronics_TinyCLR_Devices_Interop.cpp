#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

const void* TinyCLR_Interop_GetProvider(const TinyCLR_Interop_MethodData md, int32_t fieldId) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    interop->GetThisObject(interop, md.Stack, self);

    if (self == nullptr)
        return (const void*)nullptr;

    interop->GetField(interop, self, fieldId, fld);

    return (const void*)fld.Data.Numeric->I;
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetFieldInMethodData(const TinyCLR_Interop_MethodData md, int32_t fieldId) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    interop->GetThisObject(interop, md.Stack, self);

    interop->GetField(interop, self, fieldId, fld);

    return fld;
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetFieldInObject(const TinyCLR_Interop_MethodData md, const TinyCLR_Interop_ClrObject* self, int32_t fieldId) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    TinyCLR_Interop_ClrValue fld;

    interop->GetField(interop, self, fieldId, fld);

    return fld;
}


TinyCLR_Interop_ClrValue TinyCLR_Interop_GetReturn(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    TinyCLR_Interop_ClrValue ret;

    const TinyCLR_Interop_ClrObject* self;

    interop->GetThisObject(interop, md.Stack, self);

    interop->GetReturn(interop, md.Stack, ret);

    return ret;

}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetArguments(const TinyCLR_Interop_MethodData md, int32_t argIndex) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    TinyCLR_Interop_ClrValue arg;

    const TinyCLR_Interop_ClrObject* self;

    interop->GetThisObject(interop, md.Stack, self);

    if (argIndex >= 0)
        interop->GetArgument(interop, md.Stack, argIndex, arg);

    return arg;
}

uint64_t TinyCLR_Interop_CurrentTime() {
    auto provider = (const TinyCLR_NativeTime_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::NativeTimeProvider);

    return provider->ConvertNativeTimeToSystemTime(provider, provider->GetNativeTime(provider));
}

void TinyCLR_Interop_Delay(uint32_t microseconds) {
    auto provider = (const TinyCLR_NativeTime_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::NativeTimeProvider);

    provider->Wait(provider, provider->ConvertSystemTimeToNativeTime(provider, microseconds));
}

void TinyCLR_Interop_EnableInterrupt() {
    auto provider = (const TinyCLR_Interrupt_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InterruptProvider);

    provider->Enable(true);
}

void TinyCLR_Interop_DisableInterrupt() {
    auto provider = (const TinyCLR_Interrupt_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InterruptProvider);

    provider->Disable(true);
}

bool TinyCLR_Interop_GetStateInterrupt() {
    auto provider = (const TinyCLR_Interrupt_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InterruptProvider);

    return !provider->IsDisabled();
}
