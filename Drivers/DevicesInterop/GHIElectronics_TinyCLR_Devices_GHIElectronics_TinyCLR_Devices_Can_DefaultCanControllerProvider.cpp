#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

#ifdef INCLUDE_CAN
void TinyCLR_Can_ErrorReceivedIsr(const TinyCLR_Can_Controller* self, TinyCLR_Can_Error error) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Can.ErrorReceived", self->ApiInfo->Name, controller, (uint64_t)error, 0, 0);
}

void TinyCLR_Can_MessageReceivedIsr(const TinyCLR_Can_Controller* self, size_t count) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Can.MessageReceived", self->ApiInfo->Name, controller, (uint64_t)count, 0, 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::NativeAcquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);

    if (provider != nullptr) {
        if (provider->Acquire(provider) == TinyCLR_Result::Success) {
            provider->SetMessageReceivedHandler(provider, TinyCLR_Can_MessageReceivedIsr);
            provider->SetErrorReceivedHandler(provider, TinyCLR_Can_ErrorReceivedIsr);

            return TinyCLR_Result::Success;
        }

        return TinyCLR_Result::SharingViolation;
    }

    return TinyCLR_Result::ArgumentNull;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::NativeRelease___VOID(const TinyCLR_Interop_MethodData md) {
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);

    if (provider != nullptr)
        provider->Release(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetBitTiming___VOID__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 1);
    auto arg1 = TinyCLR_Interop_GetFieldInObject(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Propagation__BackingField___I4);
    auto arg2 = TinyCLR_Interop_GetFieldInObject(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase1__BackingField___I4);
    auto arg3 = TinyCLR_Interop_GetFieldInObject(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase2__BackingField___I4);
    auto arg4 = TinyCLR_Interop_GetFieldInObject(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___BaudratePrescaler__BackingField___I4);
    auto arg5 = TinyCLR_Interop_GetFieldInObject(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___SynchronizationJumpWidth__BackingField___I4);
    auto arg6 = TinyCLR_Interop_GetFieldInObject(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___UseMultiBitSampling__BackingField___BOOLEAN);

    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    int32_t propagation = arg1.Data.Numeric->I4;
    int32_t phase1 = arg2.Data.Numeric->I4;
    int32_t phase2 = arg3.Data.Numeric->I4;

    int32_t brp = arg4.Data.Numeric->I4;
    int32_t synchronizationJumpWidth = arg5.Data.Numeric->I4;
    int32_t useMultiBitSampling = arg6.Data.Numeric->Boolean;

    if (provider != nullptr)
        provider->SetBitTiming(provider, propagation, phase1, phase2, brp, synchronizationJumpWidth, useMultiBitSampling);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ReadMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    uint8_t* data;

    uint32_t arbID;
    size_t length;

    bool extendedId;
    bool remoteTransmissionRequest;

    uint64_t ts;

    int32_t offset;
    int32_t count;
    int32_t sent = 0;

    const TinyCLR_Interop_ClrObject* msgObj;

    TinyCLR_Interop_ClrValue managedValueMessages, managedValueOffset, managedValueCount, ret;
    TinyCLR_Interop_ClrValue fldData, fldarbID, fldLen, fldRtr, fldEid, fldts;

    managedValueMessages = TinyCLR_Interop_GetArguments(md, 1);
    managedValueOffset = TinyCLR_Interop_GetArguments(md, 2);
    managedValueCount = TinyCLR_Interop_GetArguments(md, 3);
    ret = TinyCLR_Interop_GetReturn(md);

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = (TinyCLR_Interop_ClrObjectReference*)managedValueMessages.Data.SzArray.Data;

    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    size_t availableMsgCount;

    provider->GetUnreadMessageCount(provider, availableMsgCount);

    int32_t read = 0;

    if (availableMsgCount > count)
        availableMsgCount = count;

    int i = 0;

    while (i < offset) {
        i++;
        msgArray++;
    }

    for (i = 0; i < availableMsgCount; i++) {
        interop->ExtractObjectFromReference(interop, msgArray, msgObj);

        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___data___SZARRAY_U1, fldData);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___ArbitrationId__BackingField___U4, fldarbID);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___Length__BackingField___I4, fldLen);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN, fldRtr);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsExtendedId__BackingField___BOOLEAN, fldEid);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___TimeStamp__BackingField___mscorlibSystemDateTime, fldts);

        data = (uint8_t*)fldData.Data.SzArray.Data;


        if (provider->ReadMessage(provider, arbID, extendedId, remoteTransmissionRequest, ts, data, length) != TinyCLR_Result::Success)
            break;

        fldarbID.Data.Numeric->I4 = arbID;
        fldLen.Data.Numeric->I4 = length & 0xF;
        fldRtr.Data.Numeric->I4 = (remoteTransmissionRequest != false) ? 1 : 0;
        fldEid.Data.Numeric->I4 = (extendedId != false) ? 1 : 0;
        fldts.Data.Numeric->I8 = ts;

        read++;
        msgArray++;
    }

    ret.Data.Numeric->I4 = read;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::WriteMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    uint8_t* data;

    uint32_t arbID;

    bool extendedId;
    bool remoteTransmissionRequest;

    size_t length;

    int32_t offset;
    int32_t count;
    int32_t sent = 0;

    const TinyCLR_Interop_ClrObject* msgObj;

    TinyCLR_Interop_ClrValue managedValueMessages, managedValueOffset, managedValueCount, ret;
    TinyCLR_Interop_ClrValue fldData, fldarbID, fldLen, fldRtr, fldEid;

    managedValueMessages = TinyCLR_Interop_GetArguments(md, 1);
    managedValueOffset = TinyCLR_Interop_GetArguments(md, 2);
    managedValueCount = TinyCLR_Interop_GetArguments(md, 3);
    ret = TinyCLR_Interop_GetReturn(md);

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = (TinyCLR_Interop_ClrObjectReference*)managedValueMessages.Data.SzArray.Data;

    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    int i = 0;

    while (i < offset) {
        i++;
        msgArray++;
    }

    for (i = 0; i < count; i++) {
        interop->ExtractObjectFromReference(interop, msgArray, msgObj);

        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___data___SZARRAY_U1, fldData);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___ArbitrationId__BackingField___U4, fldarbID);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___Length__BackingField___I4, fldLen);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN, fldRtr);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsExtendedId__BackingField___BOOLEAN, fldEid);

        data = (uint8_t*)fldData.Data.SzArray.Data;
        arbID = fldarbID.Data.Numeric->I4;
        length = fldLen.Data.Numeric->I4 & 0xFF;

        remoteTransmissionRequest = (fldRtr.Data.Numeric->I4 != 0) ? true : false;
        extendedId = (fldEid.Data.Numeric->I4 != 0) ? true : false;

        if (provider->WriteMessage(provider, arbID, extendedId, remoteTransmissionRequest, data, length) != TinyCLR_Result::Success)
            break;

        msgArray++;
        sent++;
    }

    ret.Data.Numeric->I4 = sent;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_UnreadMessageCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    auto ret = TinyCLR_Interop_GetReturn(md);

    size_t availableMsgCount;

    provider->GetUnreadMessageCount(provider, availableMsgCount);

    ret.Data.Numeric->I4 = availableMsgCount;

    if (!availableMsgCount)
        return TinyCLR_Result::NoDataAvailable;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_UnwrittenMessageCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    auto ret = TinyCLR_Interop_GetReturn(md);

    size_t availableMsgCount;

    provider->GetUnwrittenMessageCount(provider, availableMsgCount);

    ret.Data.Numeric->I4 = availableMsgCount;

    if (!availableMsgCount)
        return TinyCLR_Result::NoDataAvailable;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::Reset___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    return provider->Reset(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetExplicitFilters___VOID__SZARRAY_U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    uint8_t* filterData = (uint8_t*)arg1.Data.SzArray.Data;

    int32_t length = arg1.Data.SzArray.Length;

    if (filterData == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return provider->SetExplicitFilters(provider, filterData, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetGroupFilters___VOID__SZARRAY_U4__SZARRAY_U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);
    auto arg2 = TinyCLR_Interop_GetArguments(md, 2);

    uint8_t* lowerBounds = (uint8_t*)arg1.Data.SzArray.Data;
    uint8_t* upperBounds = (uint8_t*)arg2.Data.SzArray.Data;

    size_t length = arg1.Data.SzArray.Length;

    if (lowerBounds == nullptr || upperBounds == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return provider->SetGroupFilters(provider, lowerBounds, upperBounds, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    return provider->ClearReadBuffer(provider);
}


TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;

    return provider->ClearWriteBuffer(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_IsWritingAllowed___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto ret = TinyCLR_Interop_GetReturn(md);

    bool val = true;

    provider->IsWritingAllowed(provider, val);

    ret.Data.Numeric->I4 = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_ReadErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto ret = TinyCLR_Interop_GetReturn(md);

    size_t val = 0;

    provider->GetReadErrorCount(provider, val);

    ret.Data.Numeric->I4 = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_WriteErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto ret = TinyCLR_Interop_GetReturn(md);

    size_t val = 0;

    provider->GetWriteErrorCount(provider, val);

    ret.Data.Numeric->I4 = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_SourceClock___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto ret = TinyCLR_Interop_GetReturn(md);

    uint32_t val = 0;

    provider->GetSourceClock(provider, val);

    ret.Data.Numeric->U4 = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_ReadBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto ret = TinyCLR_Interop_GetReturn(md);

    size_t val;
    auto result = provider->GetReadBufferSize(provider, val);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::set_ReadBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->SetReadBufferSize(provider, (size_t)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_WriteBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto ret = TinyCLR_Interop_GetReturn(md);

    size_t val;
    auto result = provider->GetWriteBufferSize(provider, val);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::set_WriteBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Can_Controller*)TinyCLR_Interop_GetManager(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___idx___I4).Data.Numeric->I4;
    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->SetWriteBufferSize(provider, (size_t)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {

    return TinyCLR_Result::InvalidOperation;
}

#else

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::NativeAcquire___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::NativeRelease___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetBitTiming___VOID__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ReadMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::WriteMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_UnreadMessageCount___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_UnwrittenMessageCount___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::Reset___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetExplicitFilters___VOID__SZARRAY_U4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetGroupFilters___VOID__SZARRAY_U4__SZARRAY_U4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_IsWritingAllowed___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_ReadErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_WriteErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_SourceClock___U4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_ReadBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::set_ReadBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::get_WriteBufferSize___U4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::set_WriteBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}
#endif //INCLUDE_CAN
