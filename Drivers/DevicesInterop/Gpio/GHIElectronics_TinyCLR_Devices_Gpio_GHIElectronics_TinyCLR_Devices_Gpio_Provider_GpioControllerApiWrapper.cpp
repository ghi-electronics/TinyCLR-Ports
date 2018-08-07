#include "GHIElectronics_TinyCLR_Devices_Gpio.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

static void TinyCLR_Gpio_PinChangeIsr(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge) {
    extern const TinyCLR_Api_Manager* apiManager;
    auto interopManager = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    if (interopManager != nullptr)
        interopManager->RaiseEvent(interopManager, "GHIElectronics.TinyCLR.NativeEventNames.Gpio.PinChanged", self->ApiInfo->Name, (uint64_t)pin, (uint64_t)(edge == TinyCLR_Gpio_PinChangeEdge::RisingEdge), 0, 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::get_PinCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetPinCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::OpenPin___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    return api->OpenPin(api, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::ClosePin___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    return api->ClosePin(api, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::GetDebounceTimeout___mscorlibSystemTimeSpan__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret, arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    auto pin = arg.Data.Numeric->U4;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I8 = api->GetDebounceTimeout(api, pin);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::SetDebounceTimeout___VOID__I4__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg[2], ret;
    TinyCLR_Interop_GetArguments(md, &arg[0], 0, 2);


    auto pin = arg[0].Data.Numeric->I4;
    auto value = arg[1].Data.Numeric->I8;

    return api->SetDebounceTimeout(api, pin, value);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::GetDriveMode___GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret, arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);
    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    auto pin = arg.Data.Numeric->I4;

    ret.Data.Numeric->I4 = static_cast<int32_t>(api->GetDriveMode(api, pin));

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::SetDriveMode___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    auto keepPinState = false;

    TinyCLR_Interop_ClrValue arg[2], ret;
    TinyCLR_Interop_GetArguments(md, &arg[0], 0, 2);

    auto pin = arg[0].Data.Numeric->I4;

    auto driveMode = static_cast<TinyCLR_Gpio_PinDriveMode>(arg[1].Data.Numeric->I4);

    TinyCLR_Result result;

    switch (driveMode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
        TinyCLR_Gpio_PinValue pinValue;

        if (api->GetDriveMode(api, pin) == TinyCLR_Gpio_PinDriveMode::Input || api->GetDriveMode(api, pin) == TinyCLR_Gpio_PinDriveMode::InputPullUp || api->GetDriveMode(api, pin) == TinyCLR_Gpio_PinDriveMode::InputPullDown) {
            keepPinState = true;

            api->Read(api, pin, pinValue);
        }

        result = api->SetDriveMode(api, pin, TinyCLR_Gpio_PinDriveMode::Output);

        if (result == TinyCLR_Result::Success && keepPinState == true)
            result = api->Write(api, pin, pinValue);
        break;

    case TinyCLR_Gpio_PinDriveMode::Input:
        result = api->SetDriveMode(api, pin, TinyCLR_Gpio_PinDriveMode::Input);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        result = api->SetDriveMode(api, pin, TinyCLR_Gpio_PinDriveMode::InputPullUp);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        result = api->SetDriveMode(api, pin, TinyCLR_Gpio_PinDriveMode::InputPullDown);
        break;

    default:
        return TinyCLR_Result::ArgumentInvalid;
    }

    return result;

}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::Read___GHIElectronicsTinyCLRDevicesGpioGpioPinValue__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    TinyCLR_Interop_ClrValue arg0, ret;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    TinyCLR_Gpio_PinValue value;

    api->Read(api, arg0.Data.Numeric->I4, value);

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = static_cast<int32_t>(value);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::Write___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinValue(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    TinyCLR_Interop_ClrValue arg0, arg1;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);

    TinyCLR_Gpio_PinValue value = static_cast<TinyCLR_Gpio_PinValue>(arg1.Data.Numeric->I4);

    return api->Write(api, arg0.Data.Numeric->I4, value);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::IsDriveModeSupported___BOOLEAN__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    TinyCLR_Interop_ClrValue arg0, arg1, ret;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    auto driveMode = static_cast<TinyCLR_Gpio_PinDriveMode>(arg1.Data.Numeric->I4);

    ret.Data.Numeric->Boolean = api->IsDriveModeSupported(api, arg0.Data.Numeric->I4, driveMode);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::SetPinChangedEdge___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinEdge(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);

    auto pin = arg0.Data.Numeric->I4;
    auto edge = static_cast<TinyCLR_Gpio_PinChangeEdge>(arg1.Data.Numeric->I4);

    return api->SetPinChangedHandler(api, pin, edge, TinyCLR_Gpio_PinChangeIsr);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Gpio_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper::ClearPinChangedEdge___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Gpio_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    auto pin = arg0.Data.Numeric->I4;

    return api->SetPinChangedHandler(api, pin, TinyCLR_Gpio_PinChangeEdge::FallingEdge, nullptr);
}
