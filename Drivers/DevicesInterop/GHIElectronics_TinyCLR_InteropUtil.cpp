#include "GHIElectronics_TinyCLR_InteropUtil.h"

const void* TinyCLR_Interop_GetApi(const TinyCLR_Interop_MethodData md, size_t fieldId) {
    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, fieldId, fld);

    return reinterpret_cast<const void*>(fld.Data.Numeric->I);
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetFieldSelf(const TinyCLR_Interop_MethodData md, size_t fieldId) {
    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    md.InteropManager->GetThisObject(md.InteropManager, md.Stack, self);
    md.InteropManager->GetField(md.InteropManager, self, fieldId, fld);

    return fld;
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetField(const TinyCLR_Interop_MethodData md, const TinyCLR_Interop_ClrObject* obj, size_t fieldId) {
    TinyCLR_Interop_ClrValue fld;

    md.InteropManager->GetField(md.InteropManager, obj, fieldId, fld);

    return fld;
}


TinyCLR_Interop_ClrValue TinyCLR_Interop_GetReturn(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    return ret;
}

TinyCLR_Interop_ClrValue TinyCLR_Interop_GetArgument(const TinyCLR_Interop_MethodData md, uint32_t argIndex) {
    TinyCLR_Interop_ClrValue arg;

	md.InteropManager->GetArgument(md.InteropManager, md.Stack, argIndex, arg);

    return arg;
}
