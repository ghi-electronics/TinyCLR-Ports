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

#include "AT91.h"

#ifdef INCLUDE_SD

void AT91_SdCard_AddApi(const TinyCLR_Api_Manager* apiManager) {
    return nullptr;
}

TinyCLR_Result AT91_SdCard_Acquire(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result AT91_SdCard_Release(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result AT91_SdCard_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result AT91_SdCard_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result AT91_SdCard_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result AT91_SdCard_Erases(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result AT91_SdCard_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result AT91_SdCard_Reset() {
    return TinyCLR_Result::NotSupported;
}
#endif // INCLUDE_SD