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
#include <climits>
#include "LEPCC.h"
#include "BitStuffer2.h"
#include "Common.h"
#include "utl_const_array.h"

using namespace std;
using namespace lepcc;

// -------------------------------------------------------------------------- ;

ErrCode LEPCC::ComputeNumBytesNeededToEncode(uint32 nPts, const Point3D* pts,
  double maxXErr, double maxYErr, double maxZErr, int64& nBytes)
{
  nBytes = -1;
  m_numBytesNeeded = 0;

  if (!nPts || !pts || maxXErr <= 0 || maxYErr <= 0 || maxZErr <= 0)
    return ErrCode::WrongParam;

  m_maxError = Point3D(maxXErr, maxYErr, maxZErr);

  m_extent3D = Compute3DExtent(nPts, pts);

  ErrCode errCode;
  if ((errCode = Quantize(nPts, pts)) != ErrCode::Ok)
    return errCode;

  if ((errCode = ConvertToDeltaModel()) != ErrCode::Ok)
    return errCode;

  nBytes = HeaderSize();

  nBytes += ComputeNumBytes_CutInSegments(m_yDeltaVec, m_sectionSize);
  nBytes += ComputeNumBytes_CutInSegments(m_numPointsPerRowVec, m_sectionSize);
  nBytes += ComputeNumBytes_CutInSegments(m_xDeltaVec, m_sectionSize);
  nBytes += ComputeNumBytes_CutInSegments(m_zVec, m_sectionSize);

  m_numBytesNeeded = nBytes;

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

void LEPCC::GetOrigPointIndexes(std::vector<uint32>& origPointIndexVec) const
{
  size_t nPts = m_cell3DVec.size();
  origPointIndexVec.resize(nPts);

  for (size_t i = 0; i < nPts; i++)
    origPointIndexVec[i] = m_cell3DVec[i].origPtIndex;
}

bool LEPCC::GetOrigPointIndexes(uint32* origPointIndexVec, int maxPtCount) const
{
  size_t nPts = m_cell3DVec.size();
  if (maxPtCount < (int)nPts)
    return false;
  for (size_t i = 0; i < nPts; i++)
    origPointIndexVec[i] = m_cell3DVec[i].origPtIndex;
  return true;
}

// -------------------------------------------------------------------------- ;

ErrCode LEPCC::Encode(Byte** ppByte, int64 bufferSize) const
{
  if (!ppByte)
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
  hd1.extent3D = m_extent3D;
  hd1.maxError3D = m_maxError;
  hd1.numPoints = (uint32)m_zVec.size();

  memcpy(ptr, &hd1, sizeof(hd1));
  ptr += sizeof(hd1);

  *ppByte = ptr;

  if (!Encode_CutInSegments(ppByte, m_yDeltaVec, m_sectionSize))
    return ErrCode::Failed;
  if (!Encode_CutInSegments(ppByte, m_numPointsPerRowVec, m_sectionSize))
    return ErrCode::Failed;
  if (!Encode_CutInSegments(ppByte, m_xDeltaVec, m_sectionSize))
    return ErrCode::Failed;
  if (!Encode_CutInSegments(ppByte, m_zVec, m_sectionSize))
    return ErrCode::Failed;

  // add blob size
  int64 numBytes = (int64)(*ppByte - ptrStart);
  memcpy(ptrStart + sizeof(topHd), &numBytes, sizeof(numBytes));    // overide with the real num bytes

  // add check sum
  topHd.checkSum = Common::ComputeChecksumFletcher32(ptrStart + sizeof(topHd), (numBytes - sizeof(topHd)));
  memcpy(ptrStart, &topHd, sizeof(topHd));

  if (numBytes != m_numBytesNeeded)
    return ErrCode::Failed;

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode LEPCC::GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize)
{
  blobSize = 0;

  if (!pByte)
    return ErrCode::WrongParam;

  TopHeader refHd;
  Header1 hd1;
  if (bufferSize < (int64)(sizeof(refHd) + sizeof(hd1.blobSize)))
    return ErrCode::BufferTooSmall;

  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotLepcc;

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

ErrCode LEPCC::GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32& nPts) 
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

ErrCode LEPCC::GetExtent3DFromHeader(const Byte* pByte, int64 bufferSize, Extent3D& ext) 
{
  ext = Extent3D();

  TopHeader topHd;
  Header1 hd1;
  ErrCode errCode;
  if ((errCode = ReadHeaders(pByte, bufferSize, topHd, hd1)) != ErrCode::Ok)
    return errCode;

  ext = hd1.extent3D;
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode LEPCC::Decode(const Byte** ppByte, int64 bufferSize, uint32& nPtsInOut, Point3D* pts)
{
  if (!ppByte || !*ppByte || !nPtsInOut || !pts)
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

  int64 numBytes = hd1.blobSize;
  m_extent3D = hd1.extent3D;
  m_maxError = hd1.maxError3D;

  if (hd1.numPoints > nPtsInOut)
    return ErrCode::OutArrayTooSmall;

  *ppByte = ptr;

  if (!Decode_CutInSegments(ppByte, m_yDeltaVec))
    return ErrCode::Failed;
  if (!Decode_CutInSegments(ppByte, m_numPointsPerRowVec))
    return ErrCode::Failed;
  if (!Decode_CutInSegments(ppByte, m_xDeltaVec))
    return ErrCode::Failed;
  if (!Decode_CutInSegments(ppByte, m_zVec))
    return ErrCode::Failed;

  Point3D p0 = m_extent3D.lower;
  Point3D p1 = m_extent3D.upper;

  Point3D cw(2 * m_maxError.x, 2 * m_maxError.y, 2 * m_maxError.z);

  // reconstruct the points
  int cnt = 0;
  int iy = 0;
  int nRows = (int)m_yDeltaVec.size();

  for (int i = 0; i < nRows; i++)
  {
    iy += m_yDeltaVec[i];
    int ix = 0;
    int nPts = m_numPointsPerRowVec[i];

    for (int j = 0; j < nPts; j++)
    {
      ix += m_xDeltaVec[cnt];
      int iz = m_zVec[cnt];

      double x = p0.x + ix * cw.x;
      double y = p0.y + iy * cw.y;
      double z = p0.z + iz * cw.z;

      pts[cnt] = Point3D(min(x, p1.x), min(y, p1.y), min(z, p1.z));    // ensure we don't go outside the orig extent
      cnt++;
    }
  }

  nPtsInOut = hd1.numPoints;

  int64 nBytesRead = (int64)(*ppByte - ptrStart);
  if (nBytesRead != hd1.blobSize || nBytesRead > bufferSize)
    return ErrCode::Failed;

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

void LEPCC::Clear()
{
  m_cell3DVec.clear();
  m_yDeltaVec.clear();
  m_numPointsPerRowVec.clear();
  m_xDeltaVec.clear();
  m_zVec.clear();
}

// -------------------------------------------------------------------------- ;
// -------------------------------------------------------------------------- ;

int LEPCC::HeaderSize() 
{
  return (int)(sizeof(TopHeader) + sizeof(Header1));
}

// -------------------------------------------------------------------------- ;

Extent3D LEPCC::Compute3DExtent(uint32 nPts, const Point3D* pts) const
{
  if (!nPts || !pts)
    return Extent3D();

  Extent3D ext;
  ext.lower = pts[0];
  ext.upper = ext.lower;

  const_array<Point3D> pointArr(pts, nPts);    // safe wrapper

  for (uint32 i = 0; i < nPts; i++)
  {
    ext.lower.x = min(ext.lower.x, pointArr[i].x);
    ext.lower.y = min(ext.lower.y, pointArr[i].y);
    ext.lower.z = min(ext.lower.z, pointArr[i].z);

    ext.upper.x = max(ext.upper.x, pointArr[i].x);
    ext.upper.y = max(ext.upper.y, pointArr[i].y);
    ext.upper.z = max(ext.upper.z, pointArr[i].z);
  }

  return ext;
}

// -------------------------------------------------------------------------- ;

ErrCode LEPCC::Quantize(uint32 nPts, const Point3D* pts)
{
  if (!nPts || !pts)
    return ErrCode::WrongParam;

  Point3D p0 = m_extent3D.lower;
  Point3D p1 = m_extent3D.upper;

  Point3D cw(2 * m_maxError.x, 2 * m_maxError.y, 2 * m_maxError.z);

  int64 nx64 = (int64)((p1.x - p0.x) / cw.x + 0.5) + 1;
  int64 ny64 = (int64)((p1.y - p0.y) / cw.y + 0.5) + 1;
  int64 nz64 = (int64)((p1.z - p0.z) / cw.z + 0.5) + 1;

  if (nx64 <= 0 || ny64 <= 0 || nz64 <= 0 || nx64 > INT_MAX || ny64 > INT_MAX || nz64 > INT_MAX)
    return ErrCode::QuantizeVirtualRasterTooBig;

  int nx = (int)nx64;
  int ny = (int)ny64;
  int nz = (int)nz64;

  m_cell3DVec.resize(0);
  m_cell3DVec.reserve(nPts);

  const_array<Point3D> pointArr(pts, nPts);    // safe wrapper

  for (uint32 i = 0; i < nPts; i++)
  {
    int ix = (int)((pointArr[i].x - p0.x) / cw.x + 0.5);    // relative to (x0, y0, z0)
    int iy = (int)((pointArr[i].y - p0.y) / cw.y + 0.5);
    int iz = (int)((pointArr[i].z - p0.z) / cw.z + 0.5);

    if (ix >= nx || iy >= ny || iz >= nz)
      return ErrCode::QuantizeIndexOutOfRange;

    int64 cellIndex = (int64)iy * (int64)nx + (int64)ix;
    Cell3D cell3D = { ix, iy, iz, (int)i, cellIndex };

    m_cell3DVec.push_back(cell3D);
  }

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode LEPCC::ConvertToDeltaModel()
{
  if (m_cell3DVec.empty())
    return ErrCode::Failed;

  int numPoints = (int)m_cell3DVec.size();

  // sort the points, top to bottom, left to right
  std::sort(m_cell3DVec.begin(), m_cell3DVec.end(), MyLessThanOp());

  m_yDeltaVec.resize(0);
  m_numPointsPerRowVec.resize(0);

  uint32 nPtsPerRow = 0;
  int prevRow = 0;
  int yCurr = m_cell3DVec[0].y;

  for (int i = 0; i < numPoints; i++)
  {
    int iy = m_cell3DVec[i].y;

    if (iy == yCurr)
      nPtsPerRow++;
    else
    {
      m_yDeltaVec.push_back((uint32)(yCurr - prevRow));
      m_numPointsPerRowVec.push_back(nPtsPerRow);

      nPtsPerRow = 1;
      prevRow = yCurr;
      yCurr = iy;
    }
  }

  m_yDeltaVec.push_back((uint32)(yCurr - prevRow));
  m_numPointsPerRowVec.push_back(nPtsPerRow);

  m_xDeltaVec.resize(0);
  m_xDeltaVec.reserve(numPoints);

  m_zVec.resize(0);
  m_zVec.reserve(numPoints);

  int numOccupiedRows = (int)m_yDeltaVec.size();
  int iy = 0;
  int cnt = 0;

  for (int i = 0; i < numOccupiedRows; i++)
  {
    iy += m_yDeltaVec[i];
    int prevCol = 0;

    for (int j = 0; j < (int)m_numPointsPerRowVec[i]; j++)
    {
      const Cell3D& pt = m_cell3DVec[cnt++];
      if (pt.y != iy)
        return ErrCode::Failed;

      int xDelta = pt.x - prevCol;    // can be 0 if > 1 point per cell
      m_xDeltaVec.push_back((uint32)xDelta);
      prevCol = pt.x;

      m_zVec.push_back((uint32)pt.z);
    }
  }

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

int LEPCC::ComputeNumBytes_CutInSegments(const vector<uint32>& dataVec, int sectionSize) const
{
  int numSections = (int)((dataVec.size() + (sectionSize - 1)) / sectionSize);
  int lenLastSection = (int)(dataVec.size() - (numSections - 1) * sectionSize);

  int nBytes = 0;
  uint32 totalMax = 0;

  vector<uint32> sectionMinVec;
  sectionMinVec.reserve(numSections);

  BitStuffer2 bitStuffer2;

  for (int i = 0; i < numSections; i++)
  {
    int len = (i < numSections - 1) ? sectionSize : lenLastSection;

    const uint32* pData = &dataVec[i * sectionSize];
    uint32 minElem = pData[0], maxElem = minElem;

    for (int j = 0; j < len; j++)
    {
      minElem = min(minElem, pData[j]);
      maxElem = max(maxElem, pData[j]);
    }

    sectionMinVec.push_back(minElem);
    totalMax = max(totalMax, maxElem);

    uint32 range = maxElem - minElem;
    nBytes += bitStuffer2.ComputeNumBytesNeededSimple((uint32)len, range);
  }

  uint32 maxOfSectionMins = *max_element(sectionMinVec.begin(), sectionMinVec.end());
  nBytes += bitStuffer2.ComputeNumBytesNeededSimple((uint32)numSections, maxOfSectionMins);

  //// compare to undivided and take the better
  //int nBytesUndivided = (int)bitStuffer2.ComputeNumBytesNeededSimple((uint32)dataVec.size(), totalMax);
  //return 1 + min(nBytes, nBytesUndivided);

  return nBytes;
}

// -------------------------------------------------------------------------- ;

bool LEPCC::Encode_CutInSegments(Byte** ppByte, const std::vector<uint32>& dataVec, int sectionSize) const
{
  if (!ppByte || dataVec.empty() || sectionSize <= 0)
    return false;

  int numSections = (int)((dataVec.size() + (sectionSize - 1)) / sectionSize);
  int lenLastSection = (int)(dataVec.size() - (numSections - 1) * sectionSize);

  // collect all mins, one for each section
  vector<uint32> sectionMinVec;
  sectionMinVec.reserve(numSections);
  vector<uint32>::const_iterator it, it0 = dataVec.begin();

  for (int i = 0; i < numSections; i++)
  {
    int len = (i < numSections - 1) ? sectionSize : lenLastSection;
    it = it0 + i * sectionSize;

    uint32 minElem = *min_element(it, it + len);
    sectionMinVec.push_back(minElem);
  }

  // write out these mins bit stuffed
  BitStuffer2 bitStuffer2;
  if (!bitStuffer2.EncodeSimple(ppByte, sectionMinVec))
    return false;

  // now to the sections
  vector<uint32> zeroBasedDataVec(sectionSize, 0);

  for (int i = 0; i < numSections; i++)
  {
    int len = (i < numSections - 1) ? sectionSize : lenLastSection;
    const uint32* pData = &dataVec[i * sectionSize];

    zeroBasedDataVec.resize(len);
    uint32 minElem = sectionMinVec[i];

    for (int j = 0; j < len; j++)
      zeroBasedDataVec[j] = pData[j] - minElem;

    if (!bitStuffer2.EncodeSimple(ppByte, zeroBasedDataVec))
      return false;
  }

  return true;
}

// -------------------------------------------------------------------------- ;

bool LEPCC::Decode_CutInSegments(const Byte** ppByte, std::vector<uint32>& dataVec) const
{
  if (!ppByte || !(*ppByte))
    return false;

  dataVec.resize(0);

  // decode the section mins
  vector<uint32> sectionMinVec, zeroBasedDataVec;
  BitStuffer2 bitStuffer2;
  if (!bitStuffer2.Decode(ppByte, sectionMinVec, 3))
    return false;

  int numSections = (int)sectionMinVec.size();

  dataVec.reserve(numSections * m_sectionSize);

  for (int i = 0; i < numSections; i++)
  {
    if (!bitStuffer2.Decode(ppByte, zeroBasedDataVec, 3))
      return false;

    int len = (int)zeroBasedDataVec.size();
    uint32 minElem = sectionMinVec[i];

    for (int j = 0; j < len; j++)
      dataVec.push_back(zeroBasedDataVec[j] + minElem);
  }

  return true;
}

// -------------------------------------------------------------------------- ;

ErrCode LEPCC::ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1) 
{
  if (!pByte)
    return ErrCode::WrongParam;

  if (bufferSize <= HeaderSize())
    return ErrCode::BufferTooSmall;

  TopHeader refHd;
  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotLepcc;

  memcpy(&topHd, pByte, sizeof(topHd));
  pByte += sizeof(topHd);

  if (topHd.version > kCurrVersion)    // this reader is outdated
    return ErrCode::WrongVersion;

  memcpy(&hd1, pByte, sizeof(hd1));
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

