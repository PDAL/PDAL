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

#include "libpc/PointData.hpp"

#include <cassert>
#include <iostream>

using std::cout;
using std::endl;


PointData::PointData(const PointLayout& layout, int numPoints) :
  m_layout(layout),
  m_numPoints(numPoints),
  m_data(NULL),
  m_pointSize(0)
{
  m_pointSize = m_layout.getSizeInBytes();
  m_data = new byte[m_pointSize * m_numPoints];
  
  m_isValid.resize(m_numPoints);

  return;
}


bool 
PointData::isValid(int index) const
{
  return m_isValid[index];
}


void
PointData::setValid(int index)
{
  m_isValid[index] = true;
}


void
PointData::setInvalid(int index)
{
  m_isValid[index] = false;
}


byte* PointData::getData(int index) const
{
  return m_data + m_pointSize * index;
}


int PointData::getNumPoints() const
{
  return m_numPoints;
}


const PointLayout& PointData::getLayout() const
{
  return m_layout;
}


template <class T>
void PointData::setField(int index, int itemOffset, T value)
{
  int offset = (index * m_pointSize) + itemOffset;
  assert(offset + (int)sizeof(T) <= m_pointSize * m_numPoints);
  byte* p = m_data + offset;

  *(T*)p = value;
}


template <class T>
T PointData::getField(int index, int itemOffset) const
{
  int offset = (index * m_pointSize) + itemOffset;
  assert(offset + (int)sizeof(T) <= m_pointSize * m_numPoints);
  byte* p = m_data + offset;

  return *(T*)p;
}


void PointData::setField_U8(int index, int itemOffset, byte value)
{
  setField<byte>(index, itemOffset, value);
}


void PointData::setField_F32(int index, int itemOffset, float value)
{
  setField<float>(index, itemOffset, value);
}


void PointData::setField_F64(int index, int itemOffset, double value)
{
  setField<double>(index, itemOffset, value);

}


byte PointData::getField_U8(int index, int itemOffset) const
{
  return getField<byte>(index, itemOffset);
}


float PointData::getField_F32(int index, int itemOffset) const
{
  return getField<float>(index, itemOffset);
}


double PointData::getField_F64(int index, int itemOffset) const
{
  return getField<double>(index, itemOffset);
}


float PointData::getX(int index) const
{
  return getField_F32(index, m_layout.getFieldOffset_X());
}


float PointData::getY(int index) const
{
  return getField_F32(index, m_layout.getFieldOffset_Y());
}


float PointData::getZ(int index) const
{
  return getField_F32(index, m_layout.getFieldOffset_Z());
}


void PointData::setX(int index, float value)
{
  setField_F32(index, m_layout.getFieldOffset_X(), value);
}


void PointData::setY(int index, float value)
{
  setField_F32(index, m_layout.getFieldOffset_Y(), value);
}


void PointData::setZ(int index, float value)
{
  setField_F32(index, m_layout.getFieldOffset_Z(), value);
}


void PointData::dump() const
{
  for (int index=0; index<getNumPoints(); index++)
  {
    dump(index);
  }

  return;
}


void PointData::dump(int index) const
{
  const PointLayout& layout = getLayout();

  for (int f=0; f<layout.getNumFields(); f++)
  {
    const Field& field = layout.getField(f);
    
    cout << field.getName(field.item()) << ": ";

    switch (field.type())
    {
    case Field::U8:
      cout << this->getField_U8(index, field.offset());
      break;
    case Field::F32:
      cout << this->getField_F32(index, field.offset());
      break;
    case Field::F64:
      cout << this->getField_F64(index, field.offset());
      break;
    default:
      throw;
    }

    cout << endl;
  }

  return;
}
