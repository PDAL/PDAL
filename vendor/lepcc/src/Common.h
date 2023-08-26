/*
Copyright 2016 Esri

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

A local copy of the license and additional notices are located with the
source distribution at:

http://github.com/Esri/lepcc/

Contributors:  Lucian Plesea
*/

#ifndef COMMON_H
#define COMMON_H

#include "lepcc_types.h"

namespace lepcc
{
  class Common
  {
  public:

    // from  https://en.wikipedia.org/wiki/Fletcher's_checksum
    // modified from ushorts to bytes (by Lucian Plesea)

    static uint32 ComputeChecksumFletcher32(const Byte* pByte, uint64 len);
  };

}    // namespace

#endif
