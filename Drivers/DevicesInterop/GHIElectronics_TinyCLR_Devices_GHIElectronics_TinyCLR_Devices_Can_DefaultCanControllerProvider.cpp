#include "GHIElectronics_TinyCLR_Devices.h"

void TinyCLR_Can_ErrorReceivedIsr(const TinyCLR_Can_Provider* self, int32_t channel, TinyCLR_Can_Error error) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Can.ErrorReceived", self->Parent->Name, channel, (uint64_t)error, 0, 0);
}

void TinyCLR_Can_MessageReceivedIsr(const TinyCLR_Can_Provider* self, int32_t channel, size_t count) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Can.MessageReceived", self->Parent->Name, channel, (uint64_t)count, 0, 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::NativeAcquire___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;
    int32_t channel = channelArg.Data.Numeric->I4;

    if (provider != nullptr) {
        if (provider->Acquire(provider, channel) == TinyCLR_Result::Success) {
            provider->SetMessageReceivedHandler(provider, channel, TinyCLR_Can_MessageReceivedIsr);
            provider->SetErrorReceivedHandler(provider, channel, TinyCLR_Can_ErrorReceivedIsr);

            return TinyCLR_Result::Success;
        }

        return TinyCLR_Result::SharingViolation;
    }

    return TinyCLR_Result::ArgumentNull;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::NativeRelease___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);

    int32_t channel = channelArg.Data.Numeric->I4;
    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    if (provider != nullptr)
        provider->Release(provider, channel);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::Reset___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    return provider->Reset(provider, channel);

}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetBitTiming___VOID__I4__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, channelArg, arg2, arg3, arg4, arg5, arg6, arg7, arg8;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetArgument(interop, md.Stack, 2, arg2);

    interop->GetField(interop, arg2.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Propagation__BackingField___I4, arg3);
    interop->GetField(interop, arg2.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase1__BackingField___I4, arg4);
    interop->GetField(interop, arg2.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase2__BackingField___I4, arg5);
    interop->GetField(interop, arg2.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___BaudratePrescaler__BackingField___I4, arg6);
    interop->GetField(interop, arg2.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___SynchronizationJumpWidth__BackingField___I4, arg7);
    interop->GetField(interop, arg2.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___UseMultiBitSampling__BackingField___BOOLEAN, arg8);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    int32_t propagation = arg3.Data.Numeric->I4;
    int32_t phase1 = arg4.Data.Numeric->I4;
    int32_t phase2 = arg5.Data.Numeric->I4;

    int32_t brp = arg6.Data.Numeric->I4;
    int32_t synchronizationJumpWidth = arg7.Data.Numeric->I4;
    int32_t useMultiBitSampling = arg8.Data.Numeric->Boolean;

    if (provider != nullptr)
        provider->SetBitTiming(provider, channel, propagation, phase1, phase2, brp, synchronizationJumpWidth, useMultiBitSampling);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ReadMessages___I4__I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    uint8_t* data;

    uint32_t arbID;
    size_t length;

    bool extendedId;
    bool remoteTransmissionRequest;

    uint64_t ts;

    int32_t offset;
    int32_t count;
    int32_t sent = 0;

    const TinyCLR_Interop_ClrObject* self;
    const TinyCLR_Interop_ClrObject* msgObj;

    TinyCLR_Interop_ClrValue fld, managedValueMessages, managedValueOffset, managedValueCount, ret, channelArg;
    TinyCLR_Interop_ClrValue fldData, fldarbID, fldLen, fldRtr, fldEid, fldts;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetArgument(interop, md.Stack, 2, managedValueMessages);
    interop->GetArgument(interop, md.Stack, 3, managedValueOffset);
    interop->GetArgument(interop, md.Stack, 4, managedValueCount);

    int32_t channel = channelArg.Data.Numeric->I4;

    interop->GetReturn(interop, md.Stack, ret);

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = (TinyCLR_Interop_ClrObjectReference*)managedValueMessages.Data.SzArray.Data;

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    size_t availableMsgCount;

    provider->GetUnreadMessageCount(provider, channel, availableMsgCount);

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


        if (provider->ReadMessage(provider, channel, arbID, extendedId, remoteTransmissionRequest, ts, data, length) != TinyCLR_Result::Success)
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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::WriteMessages___I4__I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    uint8_t* data;

    uint32_t arbID;

    bool extendedId;
    bool remoteTransmissionRequest;

    size_t length;

    int32_t offset;
    int32_t count;
    int32_t sent = 0;

    const TinyCLR_Interop_ClrObject* self;
    const TinyCLR_Interop_ClrObject* msgObj;

    TinyCLR_Interop_ClrValue fld, managedValueMessages, managedValueOffset, managedValueCount, ret, channelArg;
    TinyCLR_Interop_ClrValue fldData, fldarbID, fldLen, fldRtr, fldEid;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetArgument(interop, md.Stack, 2, managedValueMessages);
    interop->GetArgument(interop, md.Stack, 3, managedValueOffset);
    interop->GetArgument(interop, md.Stack, 4, managedValueCount);
    interop->GetReturn(interop, md.Stack, ret);

    int32_t channel = channelArg.Data.Numeric->I4;

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = (TinyCLR_Interop_ClrObjectReference*)managedValueMessages.Data.SzArray.Data;

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

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

        if (provider->WriteMessage(provider, channel, arbID, extendedId, remoteTransmissionRequest, data, length) != TinyCLR_Result::Success)
            break;

        msgArray++;
        sent++;
    }

    ret.Data.Numeric->I4 = sent;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::UnreadMessageCount___I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    size_t availableMsgCount;

    provider->GetUnreadMessageCount(provider, channel, availableMsgCount);

    ret.Data.Numeric->I4 = availableMsgCount;

    if (!availableMsgCount)
        return TinyCLR_Result::NoDataAvailable;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::UnwrittenMessageCount___I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    size_t availableMsgCount;

    provider->GetUnwrittenMessageCount(provider, channel, availableMsgCount);

    ret.Data.Numeric->I4 = availableMsgCount;

    if (!availableMsgCount)
        return TinyCLR_Result::NoDataAvailable;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetExplicitFilters___VOID__I4__SZARRAY_U4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, fldData, arg2, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetArgument(interop, md.Stack, 2, arg2);

    uint8_t* filterData = (uint8_t*)arg2.Data.SzArray.Data;

    int32_t length = arg2.Data.SzArray.Length;
    int32_t channel = channelArg.Data.Numeric->I4;

    if (filterData == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return provider->SetExplicitFilters(provider, channel, filterData, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetGroupFilters___VOID__I4__SZARRAY_U4__SZARRAY_U4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;

    TinyCLR_Interop_ClrValue fld, fldData1, fldData2;
    TinyCLR_Interop_ClrValue arg2, arg3, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetArgument(interop, md.Stack, 2, arg2);
    interop->GetArgument(interop, md.Stack, 3, arg3);

    int32_t channel = channelArg.Data.Numeric->I4;
    uint8_t* lowerBounds = (uint8_t*)arg2.Data.SzArray.Data;
    uint8_t* upperBounds = (uint8_t*)arg3.Data.SzArray.Data;

    size_t length = arg2.Data.SzArray.Length;

    if (lowerBounds == nullptr || upperBounds == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return provider->SetGroupFilters(provider, channel, lowerBounds, upperBounds, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ClearReadBuffer___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    return provider->ClearReadBuffer(provider, channel);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ClearWriteBuffer___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    return provider->ClearWriteBuffer(provider, channel);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::IsWritingAllowed___BOOLEAN__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    bool val = true;

    provider->IsWritingAllowed(provider, channel, val);

    ret.Data.Numeric->Boolean = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::ReadErrorCount___I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    size_t val = 0;

    provider->GetReadErrorCount(provider, channel, val);

    ret.Data.Numeric->I4 = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::WriteErrorCount___I4__I4(const TinyCLR_Interop_MethodData md) {
    size_t val = 0;

    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    provider->GetWriteErrorCount(provider, channel, val);

    ret.Data.Numeric->I4 = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SourceClock___U4__I4(const TinyCLR_Interop_MethodData md) {
    uint32_t val = 0;

    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    provider->GetSourceClock(provider, channel, val);

    ret.Data.Numeric->U4 = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::GetReadBufferSize___U4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetReturn(interop, md.Stack, ret);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;

    int32_t channel = channelArg.Data.Numeric->I4;

    size_t val;
    auto result = provider->GetReadBufferSize(provider, channel, val);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetReadBufferSize___VOID__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, arg2, channelArg;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetArgument(interop, md.Stack, 2, arg2);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;
    int32_t channel = channelArg.Data.Numeric->I4;

    return provider->SetReadBufferSize(provider, channel, (size_t)arg2.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::GetWriteBufferSize___U4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret, channelArg;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetReturn(interop, md.Stack, ret);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;
    int32_t channel = channelArg.Data.Numeric->I4;
    size_t val;
    auto result = provider->GetWriteBufferSize(provider, channel, val);

    ret.Data.Numeric->U4 = val;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::SetWriteBufferSize___VOID__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, arg2, channelArg;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetArgument(interop, md.Stack, 2, arg2);
    interop->GetArgument(interop, md.Stack, 1, channelArg);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider::FIELD___nativeProvider___I, fld);

    auto provider = (const TinyCLR_Can_Provider*)fld.Data.Numeric->I;
    int32_t channel = channelArg.Data.Numeric->I4;

    return provider->SetWriteBufferSize(provider, channel, (size_t)arg2.Data.Numeric->U4);
}


