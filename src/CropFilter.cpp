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

#include "libpc/CropFilter.hpp"

CropFilter::CropFilter(Stage& prevStage, float minX, float maxX, float minY, float maxY, float minZ, float maxZ) 
  : Filter(prevStage),
  m_minX(minX), 
  m_maxX(maxX),
  m_minY(minY), 
  m_maxY(maxY),
  m_minZ(minZ), 
  m_maxZ(maxZ)
{
  return;
}


void CropFilter::initialize()
{
  Filter::initialize();
  return;
}


void CropFilter::updateLayout()
{
  m_prevStage.updateLayout();
  const PointLayout& layout = m_prevStage.getConstHeader().getConstPointLayout();

  getHeader().getPointLayout() = layout;

  // crop filter doesn't add any fields

  return;
}


void CropFilter::readNextPoints(PointData& data)
{
  m_prevStage.readNextPoints(data);

  int cnt = data.getNumPoints();
  //const PointLayout& layout = data.getLayout();

  for (int index=0; index<cnt; index++)
  {
    float x = data.getX(index);
    float y = data.getY(index);
    float z = data.getZ(index);
    if (x < m_minX || x > m_maxX || y < m_minY || y > m_maxY ||z < m_minZ || z > m_maxZ)
    {
      // remove this point, and update the lower bound for Z
      data.setInvalid(index);
    }
  }

  return;
}
