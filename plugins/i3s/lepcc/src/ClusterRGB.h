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

#ifndef CLUSTERRGB_H
#define CLUSTERRGB_H

#include <string>
#include <vector>
#include <unordered_map>
#include "BitMask.h"

#ifdef TryHuffmanOnColor
  #include "Huffman.h"
#endif

namespace lepcc
{
  /** color quantize or cluster to the best M (dflt 256) colors using the median cut algorithm;
  *   if number of different colors <= M, the method is lossless;
  */

  class ClusterRGB
  {
  public:
    ClusterRGB() : m_maxNumColors(256),                // [0..255]
      m_colorLookupMethod(None), m_colorIndexCompressionMethod(NoCompression)  {}
    virtual ~ClusterRGB()  { Clear(); }

    ErrCode ComputeNumBytesNeededToEncode(uint32 nPts, const RGB_t* colors, int64& nBytes);

    // dst buffer is already allocated. byte ptr is moved like a file pointer.
    ErrCode Encode(Byte** ppByte, int64 bufferSize) const;

    static ErrCode GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize);
    static ErrCode GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32& nPts) ;

    ErrCode Decode(const Byte** ppByte, int64 bufferSize, uint32& nPtsInOut, RGB_t* colors);

    void Clear();

  private:

    struct Box
    {
      int numPoints;
      int volume;
      int rMin, gMin, bMin;
      int rMax, gMax, bMax;
    };

    struct RGBA_t : public RGB_t
    {
      Byte a;

      RGBA_t() : a{0}
      {}
      RGBA_t(Byte r0, Byte g0, Byte b0, Byte a0 = 0) :
          RGB_t(r0, g0, b0), a(a0)
      {}

      int DistRGB(RGBA_t v)
      {
        int dx = (int)(r) - v.r;
        int dy = (int)(g) - v.g;
        int dz = (int)(b) - v.b;
        return dx * dx + dy * dy + dz * dz;
      }
    };

    enum ColorAxis { RED, GREEN, BLUE };
    enum FindNextBoxMethod { NumPixel, NumPixelTimesVolume };
    enum ColorLookUpMethod { None = 0, Lossless, Array3D };
    enum ColorIndexCompressionMethod { NoCompression = 0, AllConst, HuffmanCodec };

    static const int kCurrVersion = 1;

    int                  m_maxNumColors;
    ColorLookUpMethod    m_colorLookupMethod;
    std::vector<RGBA_t>  m_colorMap;
    BitMask              m_trueColorMask;
    std::vector<int>     m_level6Histo, m_colorIndexLUT;
    std::vector<RGB_t>   m_rgbVec;    // small; only used if too few colors to justify colormap
    std::vector<Byte>    m_colorIndexVec;    // usually used

    std::unordered_map<int, int>  m_mapRGBValueToColormapIndex;    // used for lossless

    ColorIndexCompressionMethod   m_colorIndexCompressionMethod;

#ifdef TryHuffmanOnColor
    Huffman              m_huffman;
#endif


    int Compute3DArrayIndex(Byte r, Byte g, Byte b, int level) const
    {
      int sh = 8 - level;
      return ((r >> sh) << (level << 1)) + ((g >> sh) << level) + (b >> sh);
    }

    int Compute3DArrayIndex2(int ir, int ig, int ib, int nSteps) const
    {
      return ir * nSteps * nSteps + ig * nSteps + ib;
    }

    Byte ClampToByte(int i) const
    {
      return (Byte)(i > 0 ? (i < 255 ? i : 255) : 0);
    }

    static int HeaderSize() ;
    void GenerateColormapLossless(const std::vector<int>& losslessColors);

    int FindNextBox(const std::vector<Box>& boxVec, enum FindNextBoxMethod method) const;

    void ProjectHistogram(const std::vector<int>& histogram,
      int numColorSteps,
      const Box& box,
    enum ColorAxis axis,
      std::vector<int>& oneAxisHistogram) const;

    void SplitBox(const Box& box0, Box& box1, Box& box2, const std::vector<int>& histogram, int numColorSteps) const;

    void ShrinkBox(Box& box, const std::vector<int>& histogram, int numColorSteps) const;

    bool TurnColorsToIndexes(uint32 nPts, const RGB_t* colors, std::vector<Byte>& colorIndexVec) const;

    int64 ComputeNumBytesNeededToEncodeColorIndexes();

    void ComputeHistoOnColorIndexes(const std::vector<Byte>& colorIndexVec, std::vector<int>& histoVec, int& numNonZeroBins) const;


    // file header structs:

    struct TopHeader
    {
      char    fileKey[10];
      uint16  version;    // file version
      uint32  checkSum;

      TopHeader() : version(1), checkSum(0)
      {
        std::string fk = "ClusterRGB";
        memcpy(&fileKey[0], fk.c_str(), fk.length());
      }

      static int FileKeyLength()  { return 10; }
    };
    static_assert(sizeof(TopHeader) == 16, "Unexpected size/packing");

    struct Header1
    {
      int64   blobSize;
      uint32  numPoints;
      uint16  numColorsInColormap;
      Byte    colorLookupMethod;
      Byte    colorIndexCompressionMethod;

      Header1() : blobSize(0), numPoints(0), numColorsInColormap(0), colorLookupMethod(0), colorIndexCompressionMethod(0) {}
    };
    static_assert(sizeof(Header1) == 16, "Unexpected size/packing");

    static ErrCode ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1) ;
  };

}    // namespace

#endif
