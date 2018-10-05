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

#ifndef FLAGBYTES_H
#define FLAGBYTES_H

#include <vector>
#include "lepcc_types.h"
#include "Huffman.h"

namespace lepcc
{
  /** encode flag bytes such as Las return type, class type, etc;
  *   try both Huffman and bit stuff and take the better of the two;
  */

  class FlagBytes
  {
  public:
    FlagBytes() : m_numBytesNeeded(0), m_minValue(0)  {}
    virtual ~FlagBytes()  { Clear(); }

    ErrCode ComputeNumBytesNeededToEncode(uint32 nElem, const Byte* flagBytes, int64& nBytes);

    // dst buffer is already allocated. byte ptr is moved like a file pointer
    ErrCode Encode(Byte** ppByte, int64 bufferSize, uint32 nElem, const Byte* flagBytes) const;

    static ErrCode GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize);
    static ErrCode GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32& nPts);

    ErrCode Decode(const Byte** ppByte, int64 bufferSize, uint32& nElemInOut, Byte* flagBytes);

    void Clear();

  private:
    enum CompressionMethod : Byte { BitStuff = 0, HuffmanCodec };

    static const int kCurrVersion = 1;

    int64  m_numBytesNeeded;
    Byte   m_minValue;

    CompressionMethod  m_compressionMethod;
    Huffman            m_huffman;

    mutable std::vector<uint32>  m_dataVec;    // temp buffer
    mutable std::vector<Byte>    m_byteVec;

    static int HeaderSize();
    void ComputeHisto(uint32 nElem, const Byte* flagBytes, std::vector<int>& histoVec, int& numNonZeroBins) const;


    // file header structs:

    struct TopHeader
    {
      char    fileKey[10];
      uint16  version;    // file version
      uint32  checkSum;

      TopHeader() : version(kCurrVersion), checkSum(0)
      {
        static const char kFlags[] = "FlagBytes ";
        memcpy(&fileKey[0], kFlags, 10);
      }

      static int FileKeyLength()  { return 10; }
    };
    static_assert(sizeof(TopHeader) == 16, "Unexpected size/packing");

    struct Header1
    {
      int64   blobSize;
      uint32  numPoints;
      Byte    compressionMethod;
      Byte    minValue;
      uint16  reserved;    // make size multiple of 8

      Header1() : blobSize(0), numPoints(0), compressionMethod(0), minValue(0), reserved(0)  {}
    };
    static_assert(sizeof(Header1) == 16, "Unexpected size/packing");

    static ErrCode ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1);
  };

}    // namespace

#endif
