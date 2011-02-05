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


PointData::PointData(const Layout& layout, int numPoints) :
  m_layout(layout),
  m_numPoints(numPoints),
  m_data(NULL),
  m_pointSize(0)
{
  m_pointSize = m_layout.getSizeInBytes();
  m_data = new byte[m_pointSize * m_numPoints];

  return;
}


byte* PointData::getData(int pointNumber) const
{
  return m_data + m_pointSize * pointNumber;
}


void PointData::setField_F32(int pointNumber, int itemOffset, float value)
{
  int offset = (pointNumber * m_pointSize) + itemOffset;
  assert(offset + (int)sizeof(float) <= m_pointSize * m_numPoints);
  byte* p = m_data + offset;

  *(float*)p = value;
}


void PointData::setField_F64(int pointNumber, int itemOffset, double value)
{
  int offset = (pointNumber * m_pointSize) + itemOffset;
  assert(offset + (int)sizeof(double) <= m_pointSize * m_numPoints);
  byte* p = m_data + offset;

  *(double*)p = value;
}
