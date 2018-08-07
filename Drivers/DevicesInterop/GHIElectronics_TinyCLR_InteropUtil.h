#pragma once

#include <TinyCLR.h>

const void* TinyCLR_Interop_GetApi(const TinyCLR_Interop_MethodData md, size_t fieldId);
TinyCLR_Interop_ClrValue TinyCLR_Interop_GetFieldSelf(const TinyCLR_Interop_MethodData md, size_t fieldId);

void DevicesInterop_Add(const TinyCLR_Interop_Manager* interopManager);