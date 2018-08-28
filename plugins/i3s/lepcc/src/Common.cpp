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

Contributors:  Lucian Plesea, Thomas Maurer
*/

#include <cstring>
#include "Common.h"

using namespace lepcc;

// -------------------------------------------------------------------------- ;

uint32 Common::ComputeChecksumFletcher32(const Byte* pByte, uint64 len)    // no 4 GB limit
{
  uint32 sum1 = 0xffff, sum2 = 0xffff;
  uint64 words = len / 2;

  while (words)
  {
    uint32 tlen = (words >= 359) ? 359 : (uint32)words;
    words -= tlen;
    do {
      sum1 += (*pByte++ << 8);
      sum2 += sum1 += *pByte++;
    } while (--tlen);

    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }

  // add the straggler byte if it exists
  if (len & 1)
    sum2 += sum1 += (*pByte << 8);

  // second reduction step to reduce sums to 16 bits
  sum1 = (sum1 & 0xffff) + (sum1 >> 16);
  sum2 = (sum2 & 0xffff) + (sum2 >> 16);

  return sum2 << 16 | sum1;
}

// -------------------------------------------------------------------------- ;

