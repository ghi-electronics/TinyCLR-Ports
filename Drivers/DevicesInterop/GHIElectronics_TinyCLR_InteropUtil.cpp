#include "GHIElectronics_TinyCLR_InteropUtil.h"

const void* TinyCLR_Interop_GetArgument(const TinyCLR_Interop_MethodData md, size_t fieldId) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    interop->GetThisObject(interop, md.Stack, self);

    if (self == nullptr)
        return nullptr;

    interop->GetField(interop, self, fieldId, fld);

    return (const void*)fld.Data.Numeric->I;
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetFieldSelf(const TinyCLR_Interop_MethodData md, size_t fieldId) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, fieldId, fld);

    return fld;
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetField(const TinyCLR_Interop_MethodData md, const TinyCLR_Interop_ClrObject* self, size_t fieldId) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InteropManager);

    TinyCLR_Interop_ClrValue fld;

    interop->GetField(interop, self, fieldId, fld);

    return fld;
}


TinyCLR_Interop_ClrValue TinyCLR_Interop_GetReturn(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InteropManager);

    TinyCLR_Interop_ClrValue ret;

    const TinyCLR_Interop_ClrObject* self;

    interop->GetThisObject(interop, md.Stack, self);

    interop->GetReturn(interop, md.Stack, ret);

    return ret;
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetArguments(const TinyCLR_Interop_MethodData md, uint32_t argIndex) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager->FindDefault(md.ApiManager, TinyCLR_Api_Type::InteropManager);

    TinyCLR_Interop_ClrValue arg;

    const TinyCLR_Interop_ClrObject* self;

    interop->GetThisObject(interop, md.Stack, self);

    if (argIndex >= 0)
        interop->GetArgument(interop, md.Stack, argIndex, arg);

    return arg;
}
