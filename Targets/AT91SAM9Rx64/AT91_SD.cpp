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

const TinyCLR_Api_Info* AT91_SdCard_GetApi() {
    return nullptr;
}

TinyCLR_Result AT91_SdCard_Acquire(const TinyCLR_SdCard_Controller* self) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_Release(const TinyCLR_SdCard_Controller* self) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_GetControllerCount(const TinyCLR_SdCard_Controller* self, int32_t& count) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_WriteSectors(const TinyCLR_SdCard_Controller* self, uint64_t sector, size_t& count, const uint8_t* data, uint32_t timeout) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_ReadSectors(const TinyCLR_SdCard_Controller* self, uint64_t sector, size_t& count, uint8_t* data, uint32_t timeout) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_IsSectorErased(const TinyCLR_SdCard_Controller* self, uint64_t sector, bool& erased) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_EraseSectors(const TinyCLR_SdCard_Controller* self, uint64_t sector, size_t& count, uint32_t timeout) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_GetSectorMap(const TinyCLR_SdCard_Controller* self, const size_t*& sizes, size_t& count, bool& isUniform) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_Reset() {
    return TinyCLR_Result::NotImplemented;
}
#endif // INCLUDE_SD