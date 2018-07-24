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
