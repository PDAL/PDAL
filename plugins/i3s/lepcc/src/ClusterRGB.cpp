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
#include "ClusterRGB.h"
#include "Common.h"
#include "utl_const_array.h"

using namespace std;
using namespace lepcc;

// -------------------------------------------------------------------------- ;

ErrCode ClusterRGB::ComputeNumBytesNeededToEncode(uint32 nPts, const RGB_t* colors, int64& nBytes)
{
  nBytes = -1;

  if (!nPts || !colors)
    return ErrCode::WrongParam;

  // count the colors
  int numOrigColors = 0;
  m_trueColorMask.SetSize(256 * 256, 256);    // 2 MB
  m_trueColorMask.SetAllInvalid();

  // if few enough colors for lossless coding, collect them here
  vector<int> losslessColors;

  // init level 6 rgb histogram, for median cut method
  const int numColorSteps = 1 << 6;
  const int numCubes6 = numColorSteps * numColorSteps * numColorSteps;
  const int colorShift6 = 8 - 6;

  m_level6Histo.resize(numCubes6);    // 1 MB
  memset(&m_level6Histo[0], 0, numCubes6 * sizeof(int));

  const_array<RGB_t> rgbArr(colors, nPts);    // safe wrapper
  vector<Box> boxVec;

  // first pass:  count orig colors, compute rgb histograms, init bounding box
  {
    Box box = { 0, 0, 256, 256, 256, -1, -1, -1 };

    for (uint32 i = 0; i < nPts; i++)
    {
      Byte r = rgbArr[i].r;
      Byte g = rgbArr[i].g;
      Byte b = rgbArr[i].b;

      // count orig colors
      int n = (r << 8) + g;
      if (!m_trueColorMask.IsValid(b, n))
      {
        m_trueColorMask.SetValid(b, n);
        numOrigColors++;
        if (numOrigColors <= m_maxNumColors)
          losslessColors.push_back(Compute3DArrayIndex(r, g, b, 8));
      }

      int m = Compute3DArrayIndex(r, g, b, 6);
      m_level6Histo[m]++;

      // update bounding box
      int sh = colorShift6;
      box.rMin = min(r >> sh, box.rMin);
      box.gMin = min(g >> sh, box.gMin);
      box.bMin = min(b >> sh, box.bMin);
      box.rMax = max(r >> sh, box.rMax);
      box.gMax = max(g >> sh, box.gMax);
      box.bMax = max(b >> sh, box.bMax);

      box.numPoints++;
    }

    box.volume = (box.rMax - box.rMin + 1) *
      (box.gMax - box.gMin + 1) *
      (box.bMax - box.bMin + 1);

    boxVec.push_back(box);

    //printf("num valid pixel = %d\n", box.numPoints);
    //printf("num orig colors = %d\n", numOrigColors);

  }    // first pass

  int headerSize = HeaderSize();

  // compare to number of points and decide
  if (2 * nPts <= (uint32)3 * min(numOrigColors, m_maxNumColors))
  {
    m_colorLookupMethod = None;

    m_rgbVec.resize(nPts);
    memcpy(&m_rgbVec[0], &rgbArr[0], nPts * sizeof(rgbArr[0]));    // keep the colors for later encoding

    nBytes = headerSize + nPts * 3;    // no colormap, store the colors raw per point
    return ErrCode::Ok;
  }
  else if (numOrigColors <= m_maxNumColors)
  {
    m_colorLookupMethod = Lossless;

    GenerateColormapLossless(losslessColors);

    if (!TurnColorsToIndexes(nPts, colors, m_colorIndexVec))
      return ErrCode::Failed;

    nBytes = ComputeNumBytesNeededToEncodeColorIndexes();
    if (nBytes < 0)
      return ErrCode::Failed;

    nBytes += headerSize + numOrigColors * 3;    // lossless colormap
    return ErrCode::Ok;
  }
  else
  {
    m_colorLookupMethod = Array3D;

    // main loop: find the most populated box and split it in 2 if possible, iterate
    int index = 0;
    while ((int)boxVec.size() < m_maxNumColors && index != -1)
    {
      if ((int)boxVec.size() < (m_maxNumColors >> 1))
        index = FindNextBox(boxVec, NumPixel);
      else
        index = FindNextBox(boxVec, NumPixelTimesVolume);

      if (index > -1)
      {
        Box box1, box2;
        SplitBox(boxVec[index], box1, box2, m_level6Histo, numColorSteps);
        boxVec[index] = box1;
        boxVec.push_back(box2);
      }
    }

    m_colorIndexLUT.resize(0);
    m_colorIndexLUT.assign(numCubes6, -1);    // 1 MB

    int size = (int)boxVec.size();
    for (int i = 0; i < size; i++)
    {
      const Box& box = boxVec[i];
      for (int ir = box.rMin; ir <= box.rMax; ir++)
        for (int ig = box.gMin; ig <= box.gMax; ig++)
          for (int ib = box.bMin; ib <= box.bMax; ib++)
          {
            int k = Compute3DArrayIndex2(ir, ig, ib, numColorSteps);
            m_colorIndexLUT[k] = i;
          }
    }

    // resize the color map to the number of boxes found
    m_colorMap.resize(size);
  }

  // compute the palette colors as the means over their median cut boxes
  int numColors = (int)m_colorMap.size();

  vector<Point3D> clusterCenters(numColors);
  vector<int> counts(numColors, 0);

  // second pass:  cluster all RGB points
  for (uint32 i = 0; i < nPts; i++)
  {
    Byte r = rgbArr[i].r;
    Byte g = rgbArr[i].g;
    Byte b = rgbArr[i].b;

    int k = Compute3DArrayIndex(r, g, b, 6);
    int index = m_colorIndexLUT[k];

    Point3D& p = clusterCenters[index];
    p.x += r;
    p.y += g;
    p.z += b;

    counts[index]++;
  }

  // finalize the color map
  for (int i = 0; i < numColors; i++)
  {
    Point3D p = clusterCenters[i];
    int n = counts[i];
    n = max(1, n);

    Byte r = ClampToByte((int)(p.x / n + 0.5));
    Byte g = ClampToByte((int)(p.y / n + 0.5));
    Byte b = ClampToByte((int)(p.z / n + 0.5));

    m_colorMap[i] = { r, g, b, 0 };
  }

  if (!TurnColorsToIndexes(nPts, colors, m_colorIndexVec))
    return ErrCode::Failed;

  nBytes = ComputeNumBytesNeededToEncodeColorIndexes();
  if (nBytes < 0)
    return ErrCode::Failed;

  nBytes += headerSize + m_colorMap.size() * 3;    // lossy colormap
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode ClusterRGB::Encode(Byte** ppByte, int64 bufferSize) const
{
  if (!ppByte)
    return ErrCode::WrongParam;

  int headerSize = HeaderSize();

  if (bufferSize <= headerSize)
    return ErrCode::BufferTooSmall;

  Byte* ptr = *ppByte;
  Byte* ptrStart = ptr;    // keep for later

  TopHeader topHd;
  memcpy(ptr, &topHd, sizeof(topHd));
  ptr += sizeof(topHd);

  Header1 hd1;
  hd1.blobSize = 0;    // overide later when done
  hd1.numPoints = (uint32)((m_colorLookupMethod != None) ? m_colorIndexVec.size() : m_rgbVec.size());
  hd1.numColorsInColormap = (m_colorLookupMethod != None) ? (uint16)m_colorMap.size() : 0;
  hd1.colorLookupMethod = (Byte)m_colorLookupMethod;
  hd1.colorIndexCompressionMethod = (Byte)m_colorIndexCompressionMethod;

  memcpy(ptr, &hd1, sizeof(hd1));
  ptr += sizeof(hd1);

  if (m_colorLookupMethod == None)
  {
    size_t rgbSize = m_rgbVec.size() * sizeof(RGB_t);

    if (bufferSize < (int64)(headerSize + rgbSize))
      return ErrCode::BufferTooSmall;

    memcpy(ptr, &m_rgbVec[0], rgbSize);    // write out the colors per point raw, no colormap
    ptr += rgbSize;
  }
  else
  {
    if (bufferSize < (int64)(headerSize + hd1.numColorsInColormap * 3))
      return ErrCode::BufferTooSmall;

    for (uint16 i = 0; i < hd1.numColorsInColormap; i++)    // write colormap
    {
      memcpy(ptr, &m_colorMap[i], 3);
      ptr += 3;
    }

    if (m_colorIndexCompressionMethod == NoCompression)
    {
      if (bufferSize < (int64)(headerSize + hd1.numColorsInColormap * 3 + hd1.numPoints * 1))
        return ErrCode::BufferTooSmall;

      memcpy(ptr, &m_colorIndexVec[0], m_colorIndexVec.size());    // write color indexes uncompressed
      ptr += m_colorIndexVec.size();
    }
    else if (m_colorIndexCompressionMethod == AllConst)
    {
    }

#ifdef TryHuffmanOnColor
    else if (m_colorIndexCompressionMethod == HuffmanCodec)
    {
      int64 bufferSizeLeft = (int64)(ptr - *ppByte);
      *ppByte = ptr;
      if (!m_huffman.Encode(ppByte, bufferSizeLeft, m_colorIndexVec))
        return ErrCode::Failed;

      ptr = *ppByte;
    }
#endif

    else
      return ErrCode::Failed;
  }

  *ppByte = ptr;

  // add blob size
  uint32 numBytes = (uint32)(*ppByte - ptrStart);
  memcpy(ptrStart + sizeof(topHd), &numBytes, sizeof(numBytes));    // overide with the real num bytes

  // add check sum
  topHd.checkSum = Common::ComputeChecksumFletcher32(ptrStart + sizeof(topHd), (numBytes - sizeof(topHd)));
  memcpy(ptrStart, &topHd, sizeof(topHd));

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

ErrCode ClusterRGB::GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize)
{
  blobSize = 0;

  if (!pByte)
    return ErrCode::WrongParam;

  TopHeader refHd;
  Header1 hd1;
  if (bufferSize < (int64)(sizeof(refHd) + sizeof(hd1.blobSize)))
    return ErrCode::BufferTooSmall;

  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotClusterRGB;

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

ErrCode ClusterRGB::GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32& nPts) 
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

ErrCode ClusterRGB::Decode(const Byte** ppByte, int64 bufferSize, uint32& nPtsInOut, RGB_t* colors)
{
  if (!ppByte || !*ppByte || !nPtsInOut || !colors)
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

  if (bufferSize < hd1.blobSize)    // allocated buffer is too small
    return ErrCode::BufferTooSmall;

  // test check sum
  uint32 checkSum = Common::ComputeChecksumFletcher32(ptrStart + sizeof(topHd), hd1.blobSize - sizeof(topHd));
  if (checkSum != topHd.checkSum)
    return ErrCode::WrongCheckSum;

  uint16 numColors = hd1.numColorsInColormap;

  if (hd1.numPoints > nPtsInOut)
    return ErrCode::OutArrayTooSmall;

  if (numColors == 0)
  {
    int nBytes = hd1.numPoints * 3;

    if (bufferSize < (int64)(headerSize + nBytes))
      return ErrCode::BufferTooSmall;

    memcpy(colors, ptr, nBytes);    // read in rgb per point for numPoints
    ptr += nBytes;
  }
  else
  {
    if (bufferSize < (int64)(headerSize + numColors * 3))
      return ErrCode::BufferTooSmall;

    m_colorMap.resize(numColors);

    for (uint16 i = 0; i < numColors; i++)    // read colormap
    {
      m_colorMap[i].r = *ptr++;
      m_colorMap[i].g = *ptr++;
      m_colorMap[i].b = *ptr++;
    }

    RGB_t* dstPtr = colors;

    if (hd1.colorIndexCompressionMethod == NoCompression)
    {
      if (bufferSize < (int64)(headerSize + numColors * 3 + hd1.numPoints * 1))
        return ErrCode::BufferTooSmall;

      for (uint32 i = 0; i < hd1.numPoints; i++)    // read color index per point
      {
        Byte index = *ptr++;
        *dstPtr++ = m_colorMap[index];
      }
    }
    else if (hd1.colorIndexCompressionMethod == AllConst)
    {
      for (uint32 i = 0; i < hd1.numPoints; i++)    // special case all points have same color
      {
        *dstPtr++ = m_colorMap[0];
      }
    }

#ifdef TryHuffmanOnColor
    else if (hd1.colorIndexCompressionMethod == HuffmanCodec)
    {
      m_colorIndexVec.resize(hd1.numPoints);
      int64 bufferSizeLeft = (int64)(ptr - *ppByte);
      *ppByte = ptr;
      if (!m_huffman.Decode(ppByte, bufferSizeLeft, m_colorIndexVec))
        return ErrCode::Failed;

      ptr = *ppByte;

      for (uint32 i = 0; i < hd1.numPoints; i++)
      {
        Byte index = m_colorIndexVec[i];
        memcpy(dstPtr, &m_colorMap[index], 3);
        dstPtr++;
      }
    }
#endif

    else
      return ErrCode::Failed;
  }

  *ppByte = ptr;

  nPtsInOut = hd1.numPoints;    // num points really decoded

  int64 nBytesRead = (int64)(*ppByte - ptrStart);
  if (nBytesRead != hd1.blobSize || nBytesRead > bufferSize)
    return ErrCode::Failed;

  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;

void ClusterRGB::Clear()
{
  m_colorMap.clear();
  m_trueColorMask.Clear();
  m_level6Histo.clear();
  m_colorIndexLUT.clear();
  m_rgbVec.clear();
  m_colorIndexVec.clear();
  m_mapRGBValueToColormapIndex.clear();

#ifdef TryHuffmanOnColor
  m_huffman.Clear();
#endif
}

// -------------------------------------------------------------------------- ;
// -------------------------------------------------------------------------- ;

int ClusterRGB::HeaderSize() 
{
  return (int)(sizeof(TopHeader) + sizeof(Header1));
}

// -------------------------------------------------------------------------- ;

void ClusterRGB::GenerateColormapLossless(const vector<int>& losslessColors)
{
  int numColors = (int)losslessColors.size();
  //assert(numColors <= m_maxNumColors);
  m_colorMap.resize(numColors);
  m_mapRGBValueToColormapIndex.clear();

  for (int i = 0; i < numColors; i++)
  {
    int n = losslessColors[i];
    Byte r = (n >> 16) & 255;
    Byte g = (n >>  8) & 255;
    Byte b = n & 255;

    m_colorMap[i] = { r, g, b, 0 };
    m_mapRGBValueToColormapIndex[n] = i;
  }
}

// -------------------------------------------------------------------------- ;

int ClusterRGB::FindNextBox(const vector<Box>& boxVec, enum FindNextBoxMethod method) const
{
  double maxPixVol = -1;
  int index = -1;
  int size = (int)boxVec.size();
  for (int i = 0; i < size; i++)
  {
    const Box& box = boxVec[i];
    double vol = (method == NumPixelTimesVolume) ? box.volume : 1;

    if ((box.rMax > box.rMin || 
         box.gMax > box.gMin ||
         box.bMax > box.bMin) && box.numPoints * vol > maxPixVol)
    {
      maxPixVol = box.numPoints * vol;
      index = i;
    }
  }
  return index;
}

// -------------------------------------------------------------------------- ;

void ClusterRGB::ProjectHistogram(const vector<int>& histogram,
                                         int numColorSteps,
                                         const Box& box,
                                         enum ColorAxis axis,
                                         vector<int>& oneAxisHistogram) const
{
  oneAxisHistogram.resize(0);
  oneAxisHistogram.assign(numColorSteps, 0);

  if (axis == RED)
  {
    for (int ir = box.rMin; ir <= box.rMax; ir++)
    {
      int cnt = 0;
      for (int ig = box.gMin; ig <= box.gMax; ig++)
      {
        int k = Compute3DArrayIndex2(ir, ig, box.bMin, numColorSteps);
        const int* ptr = &histogram[k];
        for (int ib = box.bMin; ib <= box.bMax; ib++)
          cnt += *ptr++;
      }
      oneAxisHistogram[ir] = cnt;
    }
  }
  else if (axis == GREEN)
  {
    for (int ig = box.gMin; ig <= box.gMax; ig++)
    {
      int cnt = 0;
      for (int ir = box.rMin; ir <= box.rMax; ir++)
      {
        int k = Compute3DArrayIndex2(ir, ig, box.bMin, numColorSteps);
        const int* ptr = &histogram[k];
        for (int ib = box.bMin; ib <= box.bMax; ib++)
          cnt += *ptr++;
      }
      oneAxisHistogram[ig] = cnt;
    }
  }
  else if (axis == BLUE)
  {
    for (int ib = box.bMin; ib <= box.bMax; ib++)
    {
      int cnt = 0;
      for (int ir = box.rMin; ir <= box.rMax; ir++)
      {
        int k = Compute3DArrayIndex2(ir, box.gMin, ib, numColorSteps);
        const int* ptr = &histogram[k];
        for (int ig = box.gMin; ig <= box.gMax; ig++)
        {
          cnt += *ptr;
          ptr += numColorSteps;
        }
      }
      oneAxisHistogram[ib] = cnt;
    }
  }
}

// -------------------------------------------------------------------------- ;

//  See which axis is the largest, do a histogram along that axis.
//  Split at median point. Shrink both new boxes to fit points and return.

void ClusterRGB::SplitBox(const Box& box0, Box& box1, Box& box2,
                                 const vector<int>& histogram,
                                 int numColorSteps) const
{
  vector<int> oneAxisHisto;

  int dr = box0.rMax - box0.rMin;
  int dg = box0.gMax - box0.gMin;
  int db = box0.bMax - box0.bMin;
  int first, last;

  if (dr >= dg && dr >= db)    // split along red axis
  {
    ProjectHistogram(histogram, numColorSteps, box0, RED, oneAxisHisto);
    first = box0.rMin;
    last  = box0.rMax;
  }
  else if (dg >= db)    // split along green axis
  {
    ProjectHistogram(histogram, numColorSteps, box0, GREEN, oneAxisHisto);
    first = box0.gMin;
    last  = box0.gMax;
  }
  else    // split along blue axis
  {
    ProjectHistogram(histogram, numColorSteps, box0, BLUE, oneAxisHisto);
    first = box0.bMin;
    last  = box0.bMax;
  }

  // the famous median cut
  int halfNumPixel = box0.numPoints / 2;
  int sum = 0;
  int median = first;
  while (sum < halfNumPixel)
    sum += oneAxisHisto[median++];

  // clamp median to avoid empty box
  median = max(first + 1, min(median, last));

  int sum1 = 0;
  for (int i = first; i < median; i++)
    sum1 += oneAxisHisto[i];

  int sum2 = 0;
  for (int i = median; i < numColorSteps; i++)
    sum2 += oneAxisHisto[i];

  box1 = box0;
  box2 = box0;

  //assert(sum1 > 0);
  //assert(sum2 > 0);

  box1.numPoints = sum1;
  box2.numPoints = sum2;

  if (dr >= dg && dr >= db)
  {
    box1.rMax = median - 1;
    box2.rMin = median;
  }
  else if (dg >= db)
  {
    box1.gMax = median - 1;
    box2.gMin = median;
  }
  else
  {
    box1.bMax = median - 1;
    box2.bMin = median;
  }

  ShrinkBox(box1, histogram, numColorSteps);
  ShrinkBox(box2, histogram, numColorSteps);
}

// -------------------------------------------------------------------------- ;

void ClusterRGB::ShrinkBox(Box& box, const vector<int>& histogram, int numColorSteps) const
{
  vector<int> oneAxisHisto;

  if (box.rMax > box.rMin)
  {
    ProjectHistogram(histogram, numColorSteps, box, RED, oneAxisHisto);
    while (oneAxisHisto[box.rMin] == 0) box.rMin++;
    while (oneAxisHisto[box.rMax] == 0) box.rMax--;
  }
  if (box.gMax > box.gMin)
  {
    ProjectHistogram(histogram, numColorSteps, box, GREEN, oneAxisHisto);
    while (oneAxisHisto[box.gMin] == 0) box.gMin++;
    while (oneAxisHisto[box.gMax] == 0) box.gMax--;
  }
  if (box.bMax > box.bMin)
  {
    ProjectHistogram(histogram, numColorSteps, box, BLUE, oneAxisHisto);
    while (oneAxisHisto[box.bMin] == 0) box.bMin++;
    while (oneAxisHisto[box.bMax] == 0) box.bMax--;
  }

  box.volume = (box.rMax - box.rMin + 1) *
               (box.gMax - box.gMin + 1) *
               (box.bMax - box.bMin + 1);
}

// -------------------------------------------------------------------------- ;

bool ClusterRGB::TurnColorsToIndexes(uint32 nPts, const RGB_t* colors, vector<Byte>& colorIndexVec) const
{
  if (!nPts || !colors)
    return false;

  if (m_colorLookupMethod != Array3D && m_colorLookupMethod != Lossless)
    return false;

  colorIndexVec.resize(nPts);

  const bool useLUT = (m_colorLookupMethod == Array3D);
  const int shift = useLUT ? 6 : 8;
  const RGB_t* p = colors;

  for (uint32 i = 0; i < nPts; i++)
  {
    int k = Compute3DArrayIndex(p->r, p->g, p->b, shift);
    p++;
    int index = useLUT ? m_colorIndexLUT[k] : m_mapRGBValueToColormapIndex.find(k)->second;

    if (index >= 256)    // for > 256 colors in colormap change the currently fixed byte array to bit stuffed array
      return false;

    colorIndexVec[i] = (Byte)index;
  }

  return true;
}

// -------------------------------------------------------------------------- ;

int64 ClusterRGB::ComputeNumBytesNeededToEncodeColorIndexes()
{
  int numPoints = (int)m_colorIndexVec.size();
  if (numPoints == 0)
    return -1;

  vector<int> histoVec;
  int numNonZeroBins = 0;
  ComputeHistoOnColorIndexes(m_colorIndexVec, histoVec, numNonZeroBins);

  m_colorIndexCompressionMethod = AllConst;
  int64 nBytes = 0;    // if all indexes are the same

  if (numNonZeroBins > 1)
  {
    m_colorIndexCompressionMethod = HuffmanCodec;
#ifdef TryHuffmanOnColor
    nBytes = m_huffman.ComputeNumBytesNeededToEncode(histoVec);    // try Huffman on indexes
#endif
    if (nBytes <= 0 || nBytes >= numPoints * 1)
    {
      m_colorIndexCompressionMethod = NoCompression;
      nBytes = numPoints * 1;
    }
  }

  return nBytes;
}

// -------------------------------------------------------------------------- ;

void ClusterRGB::ComputeHistoOnColorIndexes(const vector<Byte>& colorIndexVec, 
  vector<int>& histoVec, int& numNonZeroBins) const
{
  histoVec.resize(256);
  memset(&histoVec[0], 0, 256);
  numNonZeroBins = 0;

  int len = (int)colorIndexVec.size();
  for (int i = 0; i < len; i++)
  {
    int index = colorIndexVec[i];
    numNonZeroBins += (histoVec[index] == 0) ? 1 : 0;
    histoVec[index]++;
  }
}

// -------------------------------------------------------------------------- ;

ErrCode ClusterRGB::ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1) 
{
  if (!pByte)
    return ErrCode::WrongParam;

  if (bufferSize <= HeaderSize())
    return ErrCode::BufferTooSmall;

  TopHeader refHd;
  if (0 != memcmp(pByte, refHd.fileKey, refHd.FileKeyLength()))    // file key
    return ErrCode::NotClusterRGB;

  memcpy(&topHd, pByte, sizeof(topHd));
  pByte += sizeof(topHd);

  if (topHd.version > kCurrVersion )    // this reader is outdated
    return ErrCode::WrongVersion;

  memcpy(&hd1, pByte, sizeof(hd1));
  return ErrCode::Ok;
}

// -------------------------------------------------------------------------- ;
