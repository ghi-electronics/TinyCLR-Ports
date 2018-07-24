#pragma once

#include <TinyCLR.h>
#include <Device.h>

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices;
extern const TinyCLR_Api_Manager* apiManager;

const void* TinyCLR_Interop_GetArgument(const TinyCLR_Interop_MethodData md, size_t fieldId);
TinyCLR_Interop_ClrValue TinyCLR_Interop_GetReturn(const TinyCLR_Interop_MethodData md);
TinyCLR_Interop_ClrValue TinyCLR_Interop_GetArguments(const TinyCLR_Interop_MethodData md, uint32_t argIndex);
TinyCLR_Interop_ClrValue TinyCLR_Interop_GetFieldSelf(const TinyCLR_Interop_MethodData md, size_t fieldId);
TinyCLR_Interop_ClrValue TinyCLR_Interop_GetField(const TinyCLR_Interop_MethodData md, const TinyCLR_Interop_ClrObject* self, size_t fieldId);

uint64_t TinyCLR_Interop_CurrentTime();
void TinyCLR_Interop_Delay(uint32_t microseconds);

void TinyCLR_Interop_EnableInterrupt();
void TinyCLR_Interop_DisableInterrupt();
bool TinyCLR_Interop_GetStateInterrupt();
