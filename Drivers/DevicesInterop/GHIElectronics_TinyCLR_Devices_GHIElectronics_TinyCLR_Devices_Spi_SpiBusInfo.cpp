#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiBusInfo::ctor___VOID__I__I4(const TinyCLR_Interop_MethodData md) {
    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    auto arg2 = TinyCLR_Interop_GetArguments(md, 2);

    auto provider = (const TinyCLR_Spi_Provider*)arg1.Data.Numeric->I4;

    auto controller = arg2.Data.Numeric->I4;

    auto thisObject = TinyCLR_Interop_GetFieldInMethodData(md, 0);

    if (thisObject.Object == nullptr)
        return TinyCLR_Result::NullReference;

    if (provider == nullptr)
        return TinyCLR_Result::ArgumentNull;

    {
        auto minClock = TinyCLR_Interop_GetFieldInObject(md, thisObject.Object, FIELD___MinClockFrequency__BackingField___I4);
        auto maxClock = TinyCLR_Interop_GetFieldInObject(md, thisObject.Object, FIELD___MaxClockFrequency__BackingField___I4);
        auto chipselectLineCount = TinyCLR_Interop_GetFieldInObject(md, thisObject.Object, FIELD___ChipSelectLineCount__BackingField___I4);

        minClock.Data.Numeric->I4 = provider->GetMinClockFrequency(provider, controller);
        maxClock.Data.Numeric->I4 = provider->GetMaxClockFrequency(provider, controller);
        chipselectLineCount.Data.Numeric->I4 = provider->GetChipSelectLineCount(provider, controller);

        int32_t ptr[10];
        size_t ptrLen = 10;

        provider->GetSupportedDataBitLengths(provider, controller, ptr, ptrLen);

        auto interop = (const TinyCLR_Interop_Provider*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

        const TinyCLR_Interop_ClrObject* self;

        TinyCLR_Interop_ClrValue arr;
        TinyCLR_Interop_ClrTypeId idx;

        interop->GetThisObject(interop, md.Stack, self);
        interop->FindType(interop, "mscorlib", "System", "Int32", idx);
        interop->GetField(interop, self, FIELD___SupportedDataBitLengths__BackingField___SZARRAY_I4, arr);

        interop->CreateArray(interop, ptrLen, idx, arr);

        auto p = reinterpret_cast<int32_t*>(arr.Data.SzArray.Data);
        for (auto i = 0; i < ptrLen; i++)
            p[i] = ptr[i];

    }

    return TinyCLR_Result::Success;
}
