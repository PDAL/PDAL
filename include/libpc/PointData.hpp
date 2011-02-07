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

#include "PointLayout.hpp"

typedef unsigned char byte;

// a PointData object is just an untyped array of N bytes,
// where N is (the size of the given Layout * the number of points)
//
// That is, a PointData represents the underlying data for one or more points.
//
// A PointData object has an associated Layout object.
//
// Many of the methods take a first parameter "index", to specify which point in the
// collection is to be operated upon.
class PointData
{
public:
  PointData(const PointLayout&, int numPoints);

  byte* getData(int index) const;
  int getNumPoints() const;
  const PointLayout& getLayout() const;

  bool isValid(int index) const;
  void setValid(int index);
  void setInvalid(int index);

  byte getField_U8(int index, int itemOffset) const;
  float getField_F32(int index, int itemOffset) const;
  double getField_F64(int index, int itemOffset) const;

  void setField_U8(int index, int itemOffset, byte value);
  void setField_F32(int index, int itemOffset, float value);
  void setField_F64(int index, int itemOffset, double value);

  // some well-known fields
  float getX(int index) const;
  float getY(int index) const;
  float getZ(int index) const;
  void setX(int index, float value);
  void setY(int index, float value);
  void setZ(int index, float value);

  void dump() const;
  void dump(int index) const;

private:
  template<class T> T getField(int index, int itemOffset) const;
  template<class T> void setField(int index, int itemOffset, T value);

  PointLayout m_layout;
  byte* m_data;
  int m_pointSize;
  int m_numPoints;
  std::vector<bool> m_isValid;

  PointData(const PointData&); // not implemented
  PointData& operator=(const PointData&); // not implemented
};


#endif
