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

#if defined(__GNUC__)
#define PACKED(x)       x __attribute__((packed))
#elif defined(arm) || defined(__arm)
#define __section(x)
#define PACKED(x)       __packed x
#endif

#define SIZEOF_CONST_ARRAY(x)               (sizeof(x)/sizeof(x[0]))

#define CONCAT2(a, b) a##b
#define CONCAT(a, b) CONCAT2(a, b)
