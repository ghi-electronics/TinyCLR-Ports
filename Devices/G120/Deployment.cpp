// Copyright Microsoft Corporation
// Copyright GHI Electronics, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <LPC17.h>
#include "../../Drivers/S25FL032_Flash/S25FL032_Flash.h"

#define TOTAL_DEPLOYMENT_CONTROLLERS 1

static TinyCLR_Storage_Controller deploymentControllers[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Api_Info deploymentApi[TOTAL_DEPLOYMENT_CONTROLLERS];

const TinyCLR_Api_Info* LPC17_Deployment_GetApi() {
    for (int32_t i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &LPC17_Deployment_Acquire;
        deploymentControllers[i].Release = &LPC17_Deployment_Release;
        deploymentControllers[i].Read = &LPC17_Deployment_Read;
        deploymentControllers[i].Write = &LPC17_Deployment_Write;
        deploymentControllers[i].Erase = &LPC17_Deployment_Erase;
        deploymentControllers[i].IsErased = &LPC17_Deployment_IsErased;
        deploymentControllers[i].GetDescriptor = &LPC17_Deployment_GetDescriptor;
        deploymentControllers[i].IsPresent = &LPC17_Deployment_IsPresent;
        deploymentControllers[i].SetPresenceChangedHandler = &LPC17_Deployment_SetPresenceChangedHandler;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.DeploymentController";
        deploymentApi[i].Type = TinyCLR_Api_Type::DeploymentController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = nullptr;
    }

    return (const TinyCLR_Api_Info*)&deploymentApi;
}

TinyCLR_Result LPC17_Deployment_Acquire(const TinyCLR_Storage_Controller* self) {
    const TinyCLR_Api_Info* spiApi = &CONCAT(DEVICE_TARGET, _Spi_GetApi)()[LPC17_DEPLOYMENT_SPI_PORT];

    TinyCLR_Spi_Controller* spiController = (TinyCLR_Spi_Controller*)spiApi->Implementation;

    return S25FL032_Flash_Acquire(spiController, LPC17_DEPLOYMENT_SPI_ENABLE_PIN);
}

TinyCLR_Result LPC17_Deployment_Release(const TinyCLR_Storage_Controller* self) {
    return S25FL032_Flash_Release();
}

TinyCLR_Result LPC17_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    return S25FL032_Flash_Read(address, count, data);
}

TinyCLR_Result LPC17_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    return S25FL032_Flash_Write(address, count, data);;
}

TinyCLR_Result LPC17_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    sector += LPC17_DEPLOYMENT_SECTOR_START;

    return S25FL032_Flash_EraseBlock(sector);
}

TinyCLR_Result LPC17_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, bool& erased) {
    sector += LPC17_DEPLOYMENT_SECTOR_START;

    return S25FL032_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result LPC17_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
    return S25FL032_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result LPC17_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    S25FL032_Flash_GetDescriptor(addresses, sizes, count);

    addresses += LPC17_DEPLOYMENT_SECTOR_START;
    sizes += LPC17_DEPLOYMENT_SECTOR_START;
    count = LPC17_DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Flash_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Flash_IsPresent(const TinyCLR_Storage_Controller* self, bool& present) {
    return TinyCLR_Result::NotImplemented;
}

