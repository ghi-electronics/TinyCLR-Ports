#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

void TinyCLR_Can_ErrorReceivedIsr(const TinyCLR_Can_Controller* self, TinyCLR_Can_Error error) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    auto controllerIndex = *(reinterpret_cast<int32_t*>(self->ApiInfo->State));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Can.ErrorReceived", self->ApiInfo->Name, controllerIndex, (uint64_t)error, 0, 0);
}

void TinyCLR_Can_MessageReceivedIsr(const TinyCLR_Can_Controller* self, size_t count) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    auto controllerIndex = *(reinterpret_cast<int32_t*>(self->ApiInfo->State));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Can.MessageReceived", self->ApiInfo->Name, controllerIndex, (uint64_t)count, 0, 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetWriteBufferSize(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto arg1 = TinyCLR_Interop_GetArguments(md, 0);

    return provider->SetWriteBufferSize(provider, arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetReadBufferSize(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto arg1 = TinyCLR_Interop_GetArguments(md, 0);

    return provider->SetReadBufferSize(provider, arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_MessagesToWrite___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetMessagesToWrite(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_MessagesToRead___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetMessagesToRead(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_CanWriteMessage___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->Boolean = provider->CanWriteMessage(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_CanReadMessage___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->Boolean = provider->CanReadMessage(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_WriteErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetWriteErrorCount(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_ReadErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetReadErrorCount(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_SourceClock___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetSourceClock(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Enable___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    return provider->Enable(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Disable___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    return provider->Disable(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::WriteMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = md.InteropManager;

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

    managedValueMessages = TinyCLR_Interop_GetArguments(md, 0);
    managedValueOffset = TinyCLR_Interop_GetArguments(md, 1);
    managedValueCount = TinyCLR_Interop_GetArguments(md, 2);
    ret = TinyCLR_Interop_GetReturn(md);

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = reinterpret_cast<TinyCLR_Interop_ClrObjectReference*>(managedValueMessages.Data.SzArray.Data);

    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    int i = 0;

    while (i < offset) {
        i++;
        msgArray++;
    }

    for (i = 0; i < count; i++) {
        interop->ExtractObjectFromReference(interop, msgArray, msgObj);

        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___data___SZARRAY_U1, fldData);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___ArbitrationId__BackingField___I4, fldarbID);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___Length__BackingField___I4, fldLen);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN, fldRtr);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsExtendedId__BackingField___BOOLEAN, fldEid);

        data = reinterpret_cast<uint8_t*>(fldData.Data.SzArray.Data);
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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::ReadMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = md.InteropManager;

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

    managedValueMessages = TinyCLR_Interop_GetArguments(md, 0);
    managedValueOffset = TinyCLR_Interop_GetArguments(md, 1);
    managedValueCount = TinyCLR_Interop_GetArguments(md, 2);
    ret = TinyCLR_Interop_GetReturn(md);

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = reinterpret_cast<TinyCLR_Interop_ClrObjectReference*>(managedValueMessages.Data.SzArray.Data);

    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    auto availableMsgCount = provider->GetMessagesToRead(provider);

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
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___ArbitrationId__BackingField___I4, fldarbID);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___Length__BackingField___I4, fldLen);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN, fldRtr);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsExtendedId__BackingField___BOOLEAN, fldEid);
        interop->GetField(interop, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___TimeStamp__BackingField___mscorlibSystemDateTime, fldts);

        data = reinterpret_cast<uint8_t*>(fldData.Data.SzArray.Data);

        if (provider->ReadMessage(provider, arbID, extendedId, remoteTransmissionRequest, data, length, ts) != TinyCLR_Result::Success)
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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::SetBitTiming___VOID__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 0);
    auto arg1 = TinyCLR_Interop_GetField(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Propagation__BackingField___I4);
    auto arg2 = TinyCLR_Interop_GetField(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase1__BackingField___I4);
    auto arg3 = TinyCLR_Interop_GetField(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase2__BackingField___I4);
    auto arg4 = TinyCLR_Interop_GetField(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___BaudratePrescaler__BackingField___I4);
    auto arg5 = TinyCLR_Interop_GetField(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___SynchronizationJumpWidth__BackingField___I4);
    auto arg6 = TinyCLR_Interop_GetField(md, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___UseMultiBitSampling__BackingField___BOOLEAN);

    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    int32_t propagation = arg1.Data.Numeric->I4;
    int32_t phase1 = arg2.Data.Numeric->I4;
    int32_t phase2 = arg3.Data.Numeric->I4;

    int32_t brp = arg4.Data.Numeric->I4;
    int32_t synchronizationJumpWidth = arg5.Data.Numeric->I4;
    int32_t useMultiBitSampling = arg6.Data.Numeric->Boolean;

    return  provider->SetBitTiming(provider, propagation, phase1, phase2, brp, synchronizationJumpWidth, useMultiBitSampling);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::SetExplicitFilters___VOID__SZARRAY_I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));
    auto arg = TinyCLR_Interop_GetArguments(md, 0);

    uint32_t* filterData = reinterpret_cast<uint32_t*>(arg.Data.SzArray.Data);

    int32_t length = arg.Data.SzArray.Length;

    if (filterData == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return provider->SetExplicitFilters(provider, filterData, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::SetGroupFilters___VOID__SZARRAY_I4__SZARRAY_I4(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    auto arg0 = TinyCLR_Interop_GetArguments(md, 0);
    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    uint32_t* lowerBounds = reinterpret_cast<uint32_t*>(arg0.Data.SzArray.Data);
    uint32_t* upperBounds = reinterpret_cast<uint32_t*>(arg1.Data.SzArray.Data);

    size_t length = arg0.Data.SzArray.Length;

    if (lowerBounds == nullptr || upperBounds == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return provider->SetGroupFilters(provider, lowerBounds, upperBounds, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    return provider->ClearWriteBuffer(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    return provider->ClearReadBuffer(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetArgument(md, FIELD___impl___I));

    return provider->Release(provider);
}
