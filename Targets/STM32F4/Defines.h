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

#define ONE_MHZ                             1000000
#define TEN_MHZ                             (10 * ONE_MHZ)

#define GPIO_PIN_NONE                       0xFFFFFFFF
#define GPIO_ALT_MODE(x)                    x

#ifndef __max
#define __max(a,b)  (((a) > (b)) ? (a) : (b))
#endif

#ifndef __min
#define __min(a,b)  (((a) < (b)) ? (a) : (b))
#endif
