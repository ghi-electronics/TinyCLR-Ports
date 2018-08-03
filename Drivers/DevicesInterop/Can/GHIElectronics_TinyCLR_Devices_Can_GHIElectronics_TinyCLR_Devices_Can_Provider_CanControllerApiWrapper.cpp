#include "GHIElectronics_TinyCLR_Devices.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

static void TinyCLR_Can_ErrorReceivedIsr(const TinyCLR_Can_Controller* self, TinyCLR_Can_Error error) {
    extern const TinyCLR_Api_Manager* apiManager;
    auto interopManager = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    auto controllerIndex = *(reinterpret_cast<int32_t*>(self->ApiInfo->State));

    if (interopManager != nullptr)
        interopManager->RaiseEvent(interopManager, "GHIElectronics.TinyCLR.NativeEventNames.Can.ErrorReceived", self->ApiInfo->Name, controllerIndex, (uint64_t)error, 0, 0);
}

static void TinyCLR_Can_MessageReceivedIsr(const TinyCLR_Can_Controller* self, size_t count) {
    extern const TinyCLR_Api_Manager* apiManager;

    auto interopManager = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    auto controllerIndex = *(reinterpret_cast<int32_t*>(self->ApiInfo->State));

    if (interopManager != nullptr)
        interopManager->RaiseEvent(interopManager, "GHIElectronics.TinyCLR.NativeEventNames.Can.MessageReceived", self->ApiInfo->Name, controllerIndex, (uint64_t)count, 0, 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetWriteBufferSize(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    TinyCLR_Interop_ClrValue arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    return api->SetWriteBufferSize(api, arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetReadBufferSize(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    TinyCLR_Interop_ClrValue arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    return api->SetReadBufferSize(api, arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_MessagesToWrite___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMessagesToWrite(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_MessagesToRead___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMessagesToRead(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_CanWriteMessage___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->Boolean = api->CanWriteMessage(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_CanReadMessage___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->Boolean = api->CanReadMessage(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_WriteErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetWriteErrorCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_ReadErrorCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetReadErrorCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::get_SourceClock___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetSourceClock(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Enable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Enable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Disable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Disable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::WriteMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
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

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, managedValueMessages);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, managedValueOffset);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, managedValueCount);

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = reinterpret_cast<TinyCLR_Interop_ClrObjectReference*>(managedValueMessages.Data.SzArray.Data);

    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    int i = 0;

    while (i < offset) {
        i++;
        msgArray++;
    }

    for (i = 0; i < count; i++) {
        md.InteropManager->ExtractObjectFromReference(md.InteropManager, msgArray, msgObj);

        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___data___SZARRAY_U1, fldData);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___ArbitrationId__BackingField___I4, fldarbID);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___Length__BackingField___I4, fldLen);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN, fldRtr);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsExtendedId__BackingField___BOOLEAN, fldEid);

        data = reinterpret_cast<uint8_t*>(fldData.Data.SzArray.Data);
        arbID = fldarbID.Data.Numeric->I4;
        length = fldLen.Data.Numeric->I4 & 0xFF;

        remoteTransmissionRequest = (fldRtr.Data.Numeric->I4 != 0) ? true : false;
        extendedId = (fldEid.Data.Numeric->I4 != 0) ? true : false;

        if (api->WriteMessage(api, arbID, extendedId, remoteTransmissionRequest, data, length) != TinyCLR_Result::Success)
            break;

        msgArray++;
        sent++;
    }

    ret.Data.Numeric->I4 = sent;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::ReadMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md) {
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

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, managedValueMessages);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, managedValueOffset);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, managedValueCount);

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    offset = managedValueOffset.Data.Numeric->I4;
    count = managedValueCount.Data.Numeric->I4;

    auto msgArray = reinterpret_cast<TinyCLR_Interop_ClrObjectReference*>(managedValueMessages.Data.SzArray.Data);

    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    auto availableMsgCount = api->GetMessagesToRead(api);

    int32_t read = 0;

    if (availableMsgCount > count)
        availableMsgCount = count;

    int i = 0;

    while (i < offset) {
        i++;
        msgArray++;
    }

    for (i = 0; i < availableMsgCount; i++) {
        md.InteropManager->ExtractObjectFromReference(md.InteropManager, msgArray, msgObj);

        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___data___SZARRAY_U1, fldData);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___ArbitrationId__BackingField___I4, fldarbID);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___Length__BackingField___I4, fldLen);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN, fldRtr);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___IsExtendedId__BackingField___BOOLEAN, fldEid);
        md.InteropManager->GetField(md.InteropManager, msgObj, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage::FIELD___TimeStamp__BackingField___mscorlibSystemDateTime, fldts);

        data = reinterpret_cast<uint8_t*>(fldData.Data.SzArray.Data);

        if (api->ReadMessage(api, arbID, extendedId, remoteTransmissionRequest, data, length, ts) != TinyCLR_Result::Success)
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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::SetBitTiming___VOID__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue arg, arg1, arg2, arg3, arg4, arg5, arg6;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    md.InteropManager->GetField(md.InteropManager, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Propagation__BackingField___I4, arg1);
    md.InteropManager->GetField(md.InteropManager, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase1__BackingField___I4, arg2);
    md.InteropManager->GetField(md.InteropManager, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___Phase2__BackingField___I4, arg3);
    md.InteropManager->GetField(md.InteropManager, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___BaudratePrescaler__BackingField___I4, arg4);
    md.InteropManager->GetField(md.InteropManager, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___SynchronizationJumpWidth__BackingField___I4, arg5);
    md.InteropManager->GetField(md.InteropManager, arg.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming::FIELD___UseMultiBitSampling__BackingField___BOOLEAN, arg6);

    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    int32_t propagation = arg1.Data.Numeric->I4;
    int32_t phase1 = arg2.Data.Numeric->I4;
    int32_t phase2 = arg3.Data.Numeric->I4;

    int32_t brp = arg4.Data.Numeric->I4;
    int32_t synchronizationJumpWidth = arg5.Data.Numeric->I4;
    int32_t useMultiBitSampling = arg6.Data.Numeric->Boolean;

    return  api->SetBitTiming(api, propagation, phase1, phase2, brp, synchronizationJumpWidth, useMultiBitSampling);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::SetExplicitFilters___VOID__SZARRAY_I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    uint32_t* filterData = reinterpret_cast<uint32_t*>(arg.Data.SzArray.Data);

    int32_t length = arg.Data.SzArray.Length;

    if (filterData == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return api->SetExplicitFilters(api, filterData, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::SetGroupFilters___VOID__SZARRAY_I4__SZARRAY_I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg1);

    uint32_t* lowerBounds = reinterpret_cast<uint32_t*>(arg0.Data.SzArray.Data);
    uint32_t* upperBounds = reinterpret_cast<uint32_t*>(arg1.Data.SzArray.Data);

    size_t length = arg0.Data.SzArray.Length;

    if (lowerBounds == nullptr || upperBounds == nullptr || length == 0)
        return  TinyCLR_Result::NullReference;

    return api->SetGroupFilters(api, lowerBounds, upperBounds, length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->ClearWriteBuffer(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->ClearReadBuffer(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    if (api != nullptr) {
        if (api->Acquire(api) == TinyCLR_Result::Success) {
            api->SetMessageReceivedHandler(api, TinyCLR_Can_MessageReceivedIsr);
            api->SetErrorReceivedHandler(api, TinyCLR_Can_ErrorReceivedIsr);

            return TinyCLR_Result::Success;
        }

        return TinyCLR_Result::SharingViolation;
    }

    return TinyCLR_Result::ArgumentNull;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Can_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Can_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
