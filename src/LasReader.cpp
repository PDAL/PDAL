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
*     * Neither the name of the Martin Isenburg or Iowa Department 
*       of Natural Resources nor the names of its contributors may be 
*       used to endorse or promote products derived from this software 
*       without specific prior written permission.
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

#include <cassert>

#include "libpc/LasReader.hpp"

using std::vector;
using std::string;


LasReader::LasReader(string file)
{
  return;
}


void LasReader::open()
{
  // pretend we read the header to determine the number of points and the layout

  vector<Field> fields;

  fields.push_back(Field(Field::XPos, 0, Field::F32));
  fields.push_back(Field(Field::YPos, 4, Field::F32));
  fields.push_back(Field(Field::ZPos, 8, Field::F32));
  fields.push_back(Field(Field::Time, 12, Field::F64));

  getHeader().getLayout().addFields(fields);

  getHeader().setNumPoints(100);

  return;
}


void LasReader::readNextPoints(PointBuffer& buffer)
{
  // make up some data and put it into the buffer

  int cnt = buffer.getNumPoints();
  assert(m_lastPointRead + cnt <= getHeader().getNumPoints());

  PointData& pointData = buffer.getPointData();

  int offsetX = getHeader().getLayout().findFieldOffset(Field::XPos);
  int offsetY = getHeader().getLayout().findFieldOffset(Field::YPos);
  int offsetZ = getHeader().getLayout().findFieldOffset(Field::ZPos);
  int offsetT = getHeader().getLayout().findFieldOffset(Field::Time);

  float v = (float)m_lastPointRead;

  for (int i=0; i<cnt; i++)
  {
    pointData.setField_F32(i, offsetX, v);
    pointData.setField_F32(i, offsetY, v + 0.1f);
    pointData.setField_F32(i, offsetZ, v + 0.2f);
    pointData.setField_F64(i, offsetT, v + 0.3f);

    ++v;
  }

  m_lastPointRead += cnt;

  return;
}
