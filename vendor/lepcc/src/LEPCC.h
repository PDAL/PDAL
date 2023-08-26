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

#ifndef LEPCC_H
#define LEPCC_H

#include <cstring>
#include <string>
#include <vector>
#include "lepcc_types.h"

namespace lepcc
{
  /** Limited Error Point Cloud Compression, based on LERC
  *
  */

  class LEPCC
  {
  public:
    LEPCC() :  m_sectionSize(128), m_numBytesNeeded(0)  {}
    virtual ~LEPCC()  {}

    ErrCode ComputeNumBytesNeededToEncode(uint32 nPts, const Point3D* pts,
      double maxXErr, double maxYErr, double maxZErr, int64& nBytes);

    Extent3D GetExtent3D() const  { return m_extent3D; }
    void GetOrigPointIndexes(std::vector<uint32>& origPointIndexVec) const;
    bool GetOrigPointIndexes(uint32* origPointIndexVec, int maxPtCount) const;

    // dst buffer is already allocated. byte ptr is moved like a file pointer.
    ErrCode Encode(Byte** ppByte, int64 bufferSize) const;

    static ErrCode GetBlobSize(const Byte* pByte, int64 bufferSize, uint32& blobSize);
    static ErrCode GetNumPointsFromHeader(const Byte* pByte, int64 bufferSize, uint32&  nPts);
    static ErrCode GetExtent3DFromHeader( const Byte* pByte, int64 bufferSize, Extent3D& ext) ;

    ErrCode Decode(const Byte** ppByte, int64 bufferSize, uint32& nPtsInOut, Point3D* pts);

    void Clear();


  private:

    struct Cell3D
    {
      int x, y, z;
      int origPtIndex;    // before sort
      int64 xyCellIndex;
    };

    struct MyLessThanOp
    {
      inline bool operator() (const Cell3D& p0, const Cell3D& p1)
      {
        return p0.xyCellIndex < p1.xyCellIndex;
      }
    };

    static const int kCurrVersion = 1;

    int         m_sectionSize;
    int64       m_numBytesNeeded;
    Extent3D    m_extent3D;
    Point3D     m_maxError;

    std::vector<Cell3D>  m_cell3DVec;
    std::vector<uint32>  m_yDeltaVec, m_numPointsPerRowVec, m_xDeltaVec, m_zVec;

    static int HeaderSize() ;
    Extent3D Compute3DExtent(uint32 nPts, const Point3D* pts) const;

    ErrCode Quantize(uint32 nPts, const Point3D* pts);

    ErrCode ConvertToDeltaModel();

    int ComputeNumBytes_CutInSegments(const std::vector<uint32>& dataVec, int sectionSize) const;

    bool Encode_CutInSegments(Byte** ppByte, const std::vector<uint32>& dataVec, int sectionSize) const;
    bool Decode_CutInSegments(const Byte** ppByte, std::vector<uint32>& dataVec) const;


    // file header structs:

    struct TopHeader
    {
      char    fileKey[10];
      uint16  version;    // file version
      uint32  checkSum;

      TopHeader() : version(1), checkSum(0)
      {
        std::string fk = "LEPCC     ";
        memcpy(&fileKey[0], fk.c_str(), fk.length());
      }

      static int FileKeyLength()  { return 10; }
    };
    static_assert(sizeof(TopHeader) == 16, "Unexpected size/packing");

    struct Header1
    {
      int64     blobSize;
      Extent3D  extent3D;
      Point3D   maxError3D;
      uint32    numPoints;
      uint32    reserved;    // make size multiple of 8

      Header1() : blobSize(0), numPoints(0), reserved(0)  {}
    };
    static_assert(sizeof(Header1) == 88, "Unexpected size/packing");

    static ErrCode ReadHeaders(const Byte* pByte, int64 bufferSize, TopHeader& topHd, Header1& hd1) ;
  };

}    // namespace

#endif
