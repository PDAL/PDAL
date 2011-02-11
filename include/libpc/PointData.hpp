/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following 
* conditions are met:
* 
*     * Redistributions of source code must retain the above copyright 
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright 
*       notice, this list of conditions and the following disclaimer in 
*       the documentation and/or other materials provided 
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the 
*       names of its contributors may be used to endorse or promote 
*       products derived from this software without specific prior 
*       written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
* OF SUCH DAMAGE.
****************************************************************************/

#ifndef INCLUDED_POINTDATA_HPP
#define INCLUDED_POINTDATA_HPP

#include "libpc/export.hpp"
#include "libpc/PointLayout.hpp"

namespace libpc
{

// a PointData object is just an untyped array of N bytes,
// where N is (the size of the given Layout * the number of points)
//
// That is, a PointData represents the underlying data for one or more points.
//
// A PointData object has an associated Layout object.
//
// Many of the methods take a first parameter "index", to specify which point in the
// collection is to be operated upon.
class LIBPC_DLL PointData
{
public:
    typedef unsigned char byte; // BUG

  // note that when we make a PointData object all the fields are initialized to inactive,
  // regardless of what the passed-in layout says -- this is because the field object 
  // represents the state within the owning object, which in this case is a completely
  // empty buffer (similarly, all the points in the buffer are marked "invalid")
  PointData(const PointLayout&, int numPoints);

  // number of points in this buffer
  int getNumPoints() const;

  // layout (number and kinds of fields) for a point in this buffer
  const PointLayout& getLayout() const;

  // "valid" means the data for the point can be used; if invalid, the point should
  // be ignored or skipped.  (This is done for efficiency; we don't want to have to
  // modify the buffer's size just to "delete" a point.)
  bool isValid(int pointIndex) const;
  void setValid(int pointIndex, bool value=true);

  // accessors to a particular field of a particular point in this buffer
  byte getField_U8(int pointIndex, int fieldIndex) const;
  float getField_F32(int pointIndex, int fieldIndex) const;
  double getField_F64(int pointIndex, int fieldIndex) const;

  // accessors to a particular field of a particular point in this buffer
  void setField_U8(int pointIndex, int fieldIndex, byte value);
  void setField_F32(int pointIndex, int fieldIndex, float value);
  void setField_F64(int pointIndex, int fieldIndex, double value);

  // handy functions to get at some some well-known fields
  float getX(int pointIndex) const;
  float getY(int pointIndex) const;
  float getZ(int pointIndex) const;
  void setX(int pointIndex, float value);
  void setY(int pointIndex, float value);
  void setZ(int pointIndex, float value);

  // bulk copy all the fields from the given point into this object
  // NOTE: this is only legal if the src and dest layouts are exactly the same
  // (later, this will be implemented properly, to handle the general cases slowly and the best case quickly)
  void copyFieldsFast(int destPointIndex, int srcPointIndex, const PointData& srcPointData);

  void dump(std::string indent="") const;
  void dump(int index, std::string indent="") const;

private:
  // access to the raw memory
  byte* getData(int pointIndex) const;

  template<class T> T getField(int fieldIndex, int itemOffset) const;
  template<class T> void setField(int fieldIndex, int itemOffset, T value);

  PointLayout m_layout;
  byte* m_data;
  int m_pointSize;
  int m_numPoints;
  std::vector<bool> m_isValid; // one bool for each point

  PointData(const PointData&); // not implemented
  PointData& operator=(const PointData&); // not implemented
};

}; // namespace libpc

#endif
