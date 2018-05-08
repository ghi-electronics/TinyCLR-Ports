#include "GHIElectronics_TinyCLR_Devices.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiBusInfo::ctor___VOID__I(const TinyCLR_Interop_MethodData md) {

    //auto provider = (const TinyCLR_Spi_Provider*)md.Stack.Arg1().NumericByRefConst().s4;

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    auto provider = (const TinyCLR_Spi_Provider*)arg1.Data.Numeric->I4;

    while (1); // TODO

    // CLR_RT_HeapBlock* pThis = md.Stack.This();

    // LIB_TC_FAULT_ON_NULL(pThis);


    // if (provider == nullptr)
        // return TinyCLR_Result::ArgumentNull;

    // {
        // pThis[FIELD___MinClockFrequency__BackingField___I4].NumericByRef().s4 = provider->GetMinClockFrequency(provider);
        // pThis[FIELD___MaxClockFrequency__BackingField___I4].NumericByRef().s4 = provider->GetMaxClockFrequency(provider);
        // pThis[FIELD___ChipSelectLineCount__BackingField___I4].NumericByRef().s4 = provider->GetChipSelectLineCount(provider);

        // int32_t ptr[10];
        // size_t ptrLen = 10;

        // provider->GetSupportedDataBitLengths(provider, ptr, ptrLen);

        // auto& hb = pThis[FIELD___SupportedDataBitLengths__BackingField___SZARRAY_I4];

        // hb.SetObjectReference(nullptr);

        // LIB_TC_CHECK_HRESULT(CLR_RT_HeapBlock_Array::CreateInstance(hb, ptrLen, g_CLR_RT_WellKnownTypes.m_Int32));
        // {
            // auto p = reinterpret_cast<int32_t*>(hb.DereferenceArray()->GetFirstElement());

            // for (auto i = 0; i < ptrLen; i++)
                // p[i] = ptr[i];
        // }
    // }

    return TinyCLR_Result::Success;
}
