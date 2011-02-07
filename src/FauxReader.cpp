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

#include <cassert>

#include "libpc/FauxReader.hpp"
#include "libpc/Utils.hpp"

using std::vector;
using std::string;


FauxReader::FauxReader(string file)
{
  return;
}


void FauxReader::initialize()
{
  Reader::initialize();

  // pretend we read the header to determine the number of points and the layout

  Header& header = getHeader();
  PointLayout& layout = header.getPointLayout();

  vector<Field> fields;

  fields.push_back(Field(Field::XPos, 0, Field::F32));
  fields.push_back(Field(Field::YPos, 4, Field::F32));
  fields.push_back(Field(Field::ZPos, 8, Field::F32));
  fields.push_back(Field(Field::Time, 12, Field::F64));

  layout.addFields(fields);

  header.setNumPoints(30);

  header.m_minX = 0.0;
  header.m_maxX = 100.0;
  header.m_minY = 0.0;
  header.m_maxY = 100.0;

  header.m_minZ = -100.0;
  header.m_maxZ = 100.0;

  return;
}


void FauxReader::readNextPoints(PointData& data)
{
  // make up some data and put it into the buffer

  int cnt = data.getNumPoints();
  assert(m_lastPointRead + cnt <= getHeader().getNumPoints());

  const PointLayout& layout = data.getLayout();

  int offsetT = layout.findFieldOffset(Field::Time);

  float v = (float)m_lastPointRead;

  for (int index=0; index<cnt; index++)
  {
    data.setValid(index);
    data.setX(index, Utils::random<float>(0,100));
    data.setY(index, Utils::random<float>(0,100));
    data.setZ(index, Utils::random<float>(-25,100));
    data.setField_F64(index, offsetT, v * 0.1);

    ++v;
  }

  m_lastPointRead += cnt;

  return;
}
