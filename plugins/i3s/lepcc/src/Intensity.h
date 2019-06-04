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

Contributors:  Thomas Maurer
*/

#ifndef INTENSITY_H
#define INTENSITY_H

#include <vector>
#include "lepcc_types.h"

namespace lepcc
{
  /** encode intensity values; can be 8 bit or 16 bit, can be upscaled to 16 bit;
  *   so try to find upscale factor and use if there, then bit stuff;
  */

  class Intensity
  {
  public:
    Intensity() : m_upscaleFactor(0), m_numBytesNeeded(0), m_bpp(0)  {}
    virtual ~Intensity()  { Clear(); }

    ErrCode ComputeNumBytesNeededToEncode(uint32 nElem, const uint16* intensities, int64& nBytes);

    // dst buffer is already allocated. byte ptr is moved like a file pointer
    ErrCode Encode(Byte** ppByte, int64 bufferSize, uint32 nElem, const uint16* intensities) const;

    static ErrCode GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize);
    static ErrCode GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32& nPts) ;

    ErrCode Decode(const Byte** ppByte, int64 bufferSize, uint32& nElemInOut, uint16* intensities);

    void Clear();

  private:
    static const int kCurrVersion = 1;
    int    m_upscaleFactor;
    int64  m_numBytesNeeded;
    int    m_bpp;

    mutable std::vector<uint32>  m_dataVec;    // temp buffer

    static int HeaderSize() ;
    int FindUpscaleFactor(uint32 nElem, const uint16* intensities, uint16 maxElem) const;


    // file header structs:

    struct TopHeader
    {
      char    fileKey[10];
      uint16  version;    // file version
      uint32  checkSum;

      TopHeader() : version(kCurrVersion), checkSum(0)
      {
        static const char kIntensity[] = "Intensity ";
        memcpy(fileKey, kIntensity, 10);
      }

      static int FileKeyLength()  { return 10; }
    };
    static_assert(sizeof(TopHeader) == 16, "Unexpected size/packing");

    struct Header1
    {
      int64   blobSize;
      uint32  numPoints;
      uint16  scaleFactor;
      Byte    bpp;
      Byte    reserved;    // make size multiple of 8

      Header1() : blobSize(0), numPoints(0), scaleFactor(0), bpp(0), reserved(0)  {}
    };
    static_assert(sizeof(Header1) == 16, "Unexpected size/packing");

    static ErrCode ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1) ;
  };

}    // namespace

#endif
