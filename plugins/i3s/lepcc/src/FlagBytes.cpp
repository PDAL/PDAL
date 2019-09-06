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
#include <climits>
#include <algorithm>
#include <cstring>
#include "FlagBytes.h"
#include "BitStuffer2.h"
#include "Common.h"
#include "utl_const_array.h"

using namespace std;
using namespace lepcc;

// -------------------------------------------------------------------------- ;

ErrCode FlagBytes::ComputeNumBytesNeededToEncode(uint32 nElem, const Byte* flagBytes, int64& nBytes)
{
  nBytes = -1;

  if (!nElem || !flagBytes)
    return ErrCode::WrongParam;

  // calc histo
  int numNonZeroBins = 0;
  vector<int> histoVec;
  ComputeHisto(nElem, flagBytes, histoVec, numNonZeroBins);

  // from histo, compute numBytesNeeded for Huffman
  nBytes = 0;
  if (numNonZeroBins > 1)
  {
    m_compressionMethod = HuffmanCodec;
    m_minValue = 0;
    nBytes = m_huffman.ComputeNumBytesNeededToEncode(histoVec);    // try Huffman
  }

  // from histo, get range, get numBytesNeeded for bit stuff
  int i0 = 0, i1 = 255;
  while (!histoVec[i0]) i0++;
  while (!histoVec[i1]) i1--;

  Byte maxElem = (Byte)(i1 - i0);
  BitStuffer2 bitStuffer2;
  int64 nBytesBitStuff = bitStuffer2.ComputeNumBytesNeededSimple(nElem, maxElem);

  if (nBytes <= 0 || nBytes >= nBytesBitStuff)
  {
    m_compressionMethod = BitStuff;
    m_minValue = (Byte)i0;
    nBytes = nBytesBitStuff;
  }

  nBytes += HeaderSize();
  m_numBytesNeeded = nBytes;

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode FlagBytes::Encode(Byte** ppByte, int64 bufferSize, uint32 nElem, const Byte* flagBytes) const
{
  if (!ppByte || !nElem || !flagBytes)
    return ErrCode::WrongParam;

  int headerSize = HeaderSize();

  if (bufferSize <= headerSize || bufferSize < m_numBytesNeeded)
    return ErrCode::BufferTooSmall;

  Byte* ptr = *ppByte;
  Byte* ptrStart = ptr;    // keep for later

  TopHeader topHd;
  memcpy(ptr, &topHd, sizeof(topHd));
  ptr += sizeof(topHd);

  Header1 hd1;
  hd1.blobSize = 0;    // overide later when done
  hd1.numPoints = nElem;
  hd1.compressionMethod = m_compressionMethod;
  hd1.minValue = m_minValue;

  memcpy(ptr, &hd1, sizeof(hd1));
  ptr += sizeof(hd1);

  *ppByte = ptr;

  const_array<Byte> flagByteArr(flagBytes, nElem);    // safe wrapper

  if (m_compressionMethod == BitStuff)
  {
    m_dataVec.resize(nElem);

    for (uint32 i = 0; i < nElem; i++)
      m_dataVec[i] = flagByteArr[i] - m_minValue;    // subtract offset

    BitStuffer2 bitStuffer2;
    if (!bitStuffer2.EncodeSimple(ppByte, m_dataVec))
      return ErrCode::Failed;
  }

  else if (m_compressionMethod == HuffmanCodec)
  {
    m_byteVec.resize(nElem);
    memcpy(&m_byteVec[0], flagBytes, nElem);

    if (!m_huffman.Encode(ppByte, bufferSize, m_byteVec))
      return ErrCode::Failed;
  }
  else
    return ErrCode::Failed;

  // add blob size
  uint32 numBytes = (uint32)(*ppByte - ptrStart);
  memcpy(ptrStart + sizeof(topHd), &numBytes, sizeof(numBytes));    // overide with the real num bytes

  // add check sum
  topHd.checkSum = Common::ComputeChecksumFletcher32(ptrStart + sizeof(topHd), (numBytes - sizeof(topHd)));
  memcpy(ptrStart, &topHd, sizeof(topHd));

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode FlagBytes::GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize)
{
  blobSize = 0;

  if (!pByte)
    return ErrCode::WrongParam;

  TopHeader refHd;
  Header1 hd1;
  if (bufferSize < (int64)(sizeof(refHd) + sizeof(hd1.blobSize)))
    return ErrCode::BufferTooSmall;

  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotFlagBytes;

  // get blob size
  pByte += sizeof(refHd);
  int64 nBytes = 0;
  memcpy(&nBytes, pByte, sizeof(nBytes));

  if (nBytes < bufferSize || nBytes > UINT_MAX)
    return ErrCode::Failed;

  blobSize = (uint32)nBytes;
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode FlagBytes::GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32& nPts)
{
  nPts = 0;

  TopHeader topHd;
  Header1 hd1;
  ErrCode errCode;
  if ((errCode = ReadHeaders(pByte, bufferSize, topHd, hd1)) != ErrCode::Ok)
    return errCode;

  nPts = hd1.numPoints;
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode FlagBytes::Decode(const Byte** ppByte, int64 bufferSize, uint32& nElemInOut, Byte* flagBytes)
{
  if (!ppByte || !*ppByte || !nElemInOut || !flagBytes)
    return ErrCode::WrongParam;

  int headerSize = HeaderSize();

  if (bufferSize <= headerSize)
    return ErrCode::BufferTooSmall;

  const Byte* ptr = *ppByte;
  const Byte* ptrStart = ptr;    // keep for later

  TopHeader topHd;
  Header1 hd1;
  ErrCode errCode;
  if ((errCode = ReadHeaders(ptr, bufferSize, topHd, hd1)) != ErrCode::Ok)
    return errCode;

  ptr += headerSize;

  if (bufferSize < hd1.blobSize)
    return ErrCode::BufferTooSmall;

  // test check sum
  uint32 checkSum = Common::ComputeChecksumFletcher32(ptrStart + sizeof(topHd), hd1.blobSize - sizeof(topHd));
  if (checkSum != topHd.checkSum)
    return ErrCode::WrongCheckSum;

  uint32 nElem = hd1.numPoints;

  if (nElem > nElemInOut)
    return ErrCode::OutArrayTooSmall;

  *ppByte = ptr;

  if (hd1.compressionMethod == BitStuff)
  {
    m_dataVec.resize(nElem);

    BitStuffer2 bitStuffer2;
    if (!bitStuffer2.Decode(ppByte, m_dataVec, 3))
      return ErrCode::Failed;

    for (uint32 i = 0; i < nElem; i++)
      flagBytes[i] = (Byte)(m_dataVec[i] + hd1.minValue);    // add offset
  }

  else if (hd1.compressionMethod == HuffmanCodec)
  {
    m_byteVec.resize(nElem);

    if (!m_huffman.Decode(ppByte, bufferSize, m_byteVec))
      return ErrCode::Failed;

    memcpy(flagBytes, &m_byteVec[0], nElem);
  }
  else
    return ErrCode::Failed;

  int64 nBytesRead = (int64)(*ppByte - ptrStart);
  if (nBytesRead != hd1.blobSize || nBytesRead > bufferSize)
    return ErrCode::Failed;

  nElemInOut = nElem;
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

void FlagBytes::Clear()
{
  m_dataVec.clear();
  m_byteVec.clear();
  m_huffman.Clear();
  m_numBytesNeeded = 0;
  m_minValue = 0;
}

// -------------------------------------------------------------------------- ;
// -------------------------------------------------------------------------- ;

int FlagBytes::HeaderSize()
{
  return (int)(sizeof(TopHeader) + sizeof(Header1));
}

// -------------------------------------------------------------------------- ;

void FlagBytes::ComputeHisto(uint32 nElem, const Byte* flagBytes,
  vector<int>& histoVec, int& numNonZeroBins) const
{
  histoVec.resize(256);
  memset(&histoVec[0], 0, 256);
  numNonZeroBins = 0;

  for (uint32 i = 0; i < nElem; i++)
  {
    Byte byte = flagBytes[i];
    numNonZeroBins += (histoVec[byte] == 0) ? 1 : 0;
    histoVec[byte]++;
  }
}

// -------------------------------------------------------------------------- ;

ErrCode FlagBytes::ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1)
{
  if (!pByte)
    return ErrCode::WrongParam;

  if (bufferSize <= HeaderSize())
    return ErrCode::BufferTooSmall;

  TopHeader refHd;
  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotFlagBytes;

  memcpy(&topHd, pByte, sizeof(topHd));
  pByte += sizeof(topHd);

  if (topHd.version > kCurrVersion)    // this reader is outdated
    return ErrCode::WrongVersion;

  memcpy(&hd1, pByte, sizeof(hd1));
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;
