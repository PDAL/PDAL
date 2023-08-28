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

#include <algorithm>
#include <cstring>
#include "Intensity.h"
#include "BitMask.h"
#include "BitStuffer2.h"
#include "Common.h"
#include "utl_const_array.h"
#include <climits>

using namespace std;
using namespace lepcc;

// -------------------------------------------------------------------------- ;

ErrCode Intensity::ComputeNumBytesNeededToEncode(uint32 nElem, const uint16* intensities, int64& nBytes)
{
  nBytes = -1;

  if (!nElem || !intensities)
    return ErrCode::WrongParam;

  const_array<uint16> intensityArr(intensities, nElem);    // safe wrapper

  uint16 maxElem = *std::max_element(intensityArr.begin(), intensityArr.end());

  // determine upscale factor if any
  m_upscaleFactor = FindUpscaleFactor(nElem, intensities, maxElem);    // >= 1
  maxElem /= (uint16)m_upscaleFactor;

  // determine bpp. if 8 or 16, write out raw binary instead of bit stuffing.
  m_bpp = 0;
  while ((m_bpp < 16) && (maxElem >> m_bpp))
    m_bpp++;

  if (m_bpp == 8 || m_bpp == 16)
  {
    m_numBytesNeeded = HeaderSize() + nElem * (m_bpp / 8);
  }
  else
  {
    BitStuffer2 bitStuffer2;
    m_numBytesNeeded = HeaderSize() + bitStuffer2.ComputeNumBytesNeededSimple(nElem, maxElem);
  }

  nBytes = m_numBytesNeeded;

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode Intensity::Encode(Byte** ppByte, int64 bufferSize, uint32 nElem, const uint16* intensities) const
{
  if (!ppByte || !nElem || !intensities || m_upscaleFactor <= 0)
    return ErrCode::WrongParam;

  if (bufferSize <= HeaderSize() || bufferSize < m_numBytesNeeded)
    return ErrCode::BufferTooSmall;

  Byte* ptr = *ppByte;
  Byte* ptrStart = ptr;    // keep for later

  TopHeader topHd;
  memcpy(ptr, &topHd, sizeof(topHd));
  ptr += sizeof(topHd);

  Header1 hd1;
  hd1.blobSize = 0;    // overide later when done
  hd1.scaleFactor = (uint16)m_upscaleFactor;
  hd1.numPoints = nElem;
  hd1.bpp = (Byte)m_bpp;

  memcpy(ptr, &hd1, sizeof(hd1));
  ptr += sizeof(hd1);

  *ppByte = ptr;

  const_array<uint16> intensityArr(intensities, nElem);    // safe wrapper

  if (m_bpp == 16)
  {
    int n = nElem * sizeof(uint16);
    memcpy(*ppByte, intensities, n);
    *ppByte += n;
  }
  else if (m_bpp == 8 && m_upscaleFactor == 1)    // common case
  {
    for (uint32 i = 0; i < nElem; i++)
      *ptr++ = (Byte)intensityArr[i];    // copy ls byte

    *ppByte += nElem;
  }
  else
  {
    m_dataVec.resize(nElem);

    if (m_upscaleFactor == 1)
    {
      for (uint32 i = 0; i < nElem; i++)
        m_dataVec[i] = intensityArr[i];
    }
    else
    {
      for (uint32 i = 0; i < nElem; i++)
        m_dataVec[i] = intensityArr[i] / m_upscaleFactor;
    }

    if (m_bpp == 8)
    {
      for (uint32 i = 0; i < nElem; i++)
        *ptr++ = (Byte)m_dataVec[i];    // copy ls byte

      *ppByte += nElem;
    }
    else
    {
      BitStuffer2 bitStuffer2;
      if (!bitStuffer2.EncodeSimple(ppByte, m_dataVec))
        return ErrCode::Failed;
    }
  }

  // add blob size
  uint32 numBytes = (uint32)(*ppByte - ptrStart);
  memcpy(ptrStart + sizeof(topHd), &numBytes, sizeof(numBytes));    // overide with the real num bytes

  // add check sum
  topHd.checkSum = Common::ComputeChecksumFletcher32(ptrStart + sizeof(topHd), (numBytes - sizeof(topHd)));
  memcpy(ptrStart, &topHd, sizeof(topHd));

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode Intensity::GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize)
{
  blobSize = 0;

  if (!pByte)
    return ErrCode::WrongParam;

  TopHeader refHd;
  Header1 hd1;
  if (bufferSize < (int64)(sizeof(refHd) + sizeof(hd1.blobSize)))
    return ErrCode::BufferTooSmall;

  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotIntensity;

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

ErrCode Intensity::GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32& nPts) 
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

ErrCode Intensity::Decode(const Byte** ppByte, int64 bufferSize, uint32& nElemInOut, uint16* intensities)
{
  if (!ppByte || !*ppByte || !nElemInOut || !intensities)
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

  uint32 numElem = hd1.numPoints;
  if (numElem > nElemInOut)
    return ErrCode::OutArrayTooSmall;

  uint16 scale = hd1.scaleFactor;
  if (scale < 1)
    return ErrCode::Failed;

  int bpp = hd1.bpp;
  if (bpp > 16)
    return ErrCode::Failed;

  *ppByte = ptr;

  if (bpp == 16)
  {
    int n = numElem * sizeof(uint16);
    memcpy(intensities, *ppByte, n);
    *ppByte += n;
  }
  else if (bpp == 8 && scale == 1)    // common case
  {
    for (uint32 i = 0; i < numElem; i++)
      intensities[i] = *ptr++;

    *ppByte += numElem;
  }
  else
  {
    if (bpp == 8)
    {
      m_dataVec.resize(numElem);

      for (uint32 i = 0; i < numElem; i++)
        m_dataVec[i] = *ptr++;

      *ppByte += numElem;
    }
    else
    {
      BitStuffer2 bitStuffer2;
      if (!bitStuffer2.Decode(ppByte, m_dataVec, 3))
        return ErrCode::Failed;
    }

    if ((uint32)m_dataVec.size() > nElemInOut)
      return ErrCode::Failed;

    for (uint32 i = 0; i < numElem; i++)
      intensities[i] = (uint16)(m_dataVec[i] * scale);
  }

  int64 nBytesRead = (int64)(*ppByte - ptrStart);
  if (nBytesRead != hd1.blobSize || nBytesRead > bufferSize)
    return ErrCode::Failed;

  nElemInOut = numElem;
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

void Intensity::Clear()
{
  m_dataVec.clear();
  m_upscaleFactor = 0;
  m_numBytesNeeded = 0;
}

// -------------------------------------------------------------------------- ;
// -------------------------------------------------------------------------- ;

int Intensity::HeaderSize() 
{
  return (int)(sizeof(TopHeader) + sizeof(Header1));
}

// -------------------------------------------------------------------------- ;

int Intensity::FindUpscaleFactor(uint32 nElem, const uint16* intensities, uint16 maxElem) const
{
  if (maxElem == 0)
    return 1;

  BitMask histoMask(1 + maxElem, 1);
  histoMask.SetAllInvalid();

  const_array<uint16> intensityArr(intensities, nElem);    // safe wrapper

  for (uint32 i = 0; i < nElem; i++)
    histoMask.SetValid(intensityArr[i]);

  int k0 = histoMask.NextValidBit(0);    // first valid entry
  int k1 = k0;
  int minDelta = k0;

  while ((k1 = histoMask.NextValidBit(k0 + 1)) > 0)    // 1st pass, find the min delta
  {
    minDelta = min(k1 - k0, minDelta);
    k0 = k1;
    if (minDelta <= 1)
      return 1;
  }

  k0 = -1;
  while ((k1 = histoMask.NextValidBit(k0 + 1)) > 0)    // 2nd pass, check each entry is a multiple of min delta
  {
    k0 = k1;
    if (k1 % minDelta)
      return 1;
  }

  return minDelta;
}

// -------------------------------------------------------------------------- ;

ErrCode Intensity::ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1) 
{
  if (!pByte)
    return ErrCode::WrongParam;

  if (bufferSize <= HeaderSize())
    return ErrCode::BufferTooSmall;

  TopHeader refHd;
  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotIntensity;

  memcpy(&topHd, pByte, sizeof(topHd));
  pByte += sizeof(topHd);

  if (topHd.version > kCurrVersion )    // this reader is outdated
    return ErrCode::WrongVersion;

  memcpy(&hd1, pByte, sizeof(hd1));
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;
