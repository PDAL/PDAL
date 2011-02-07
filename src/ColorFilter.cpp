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

#include "libpc/ColorFilter.hpp"

ColorFilter::ColorFilter(Stage& prevStage) 
  : Filter(prevStage)
{
  return;
}


void ColorFilter::initialize()
{
  Filter::initialize();
  return;
}


void ColorFilter::readNextPoints(PointData& currData)
{
  // This filter changes the layout: it adds three u8 fields
  // Therefore, we get the data from the prev stage using the prev stage's layout,
  // and then copy the contents into the layout for this stage (which was
  // passed in to us).

  const int chunk = currData.getNumPoints();
  int cnt = currData.getNumPoints();

  //const PointLayout& currLayout = getConstHeader().getConstPointLayout();
  const PointLayout& prevLayout = m_prevStage.getConstHeader().getConstPointLayout();

  PointData prevData(prevLayout, chunk);
  m_prevStage.readNextPoints(prevData);

  for (int index=0; index<cnt; index++)
  {
    if (prevData.isValid(index))
    {
      currData.setX(index, prevData.getX(index));
      currData.setY(index, prevData.getY(index));
      currData.setZ(index, prevData.getZ(index));
      currData.setField_F64(index, Field::Time, prevData.getField_F64(index, Field::Time));
      currData.setValid(index);
    }
  }

  for (int index=0; index<cnt; index++)
  {
    if (currData.isValid(index))
    {
      float z = currData.getZ(index);
      byte red, green, blue;
      getColor(z, red, green, blue);
      
      // now we would store th 3 u8's in the point data...
    }
  }

  return;
}
       

static void SlimDX_GetColor(float value, float minValue, float maxValue, float& red, float& green, float& blue)
{
  // initialize to white
  red = 1.0;
  green = 1.0;
  blue = 1.0;

  if (value < minValue)
  {
    value = minValue;
  }

  if (value > maxValue)
  {
    value = maxValue;
  }

  float dv = maxValue - minValue;

  if (value < (minValue + (0.25 * dv)))
  {
    red = 0;
    green = 4 * (value - minValue) / dv;
  }
  else if (value < (minValue + (0.5 * dv)))
  {
    red = 0;
    blue = 1 + (4 * (minValue + (0.25f * dv) - value) / dv);
  }
  else if (value < (minValue + (0.75 * dv)))
  {
    red = 4 * (value - minValue - (0.5f * dv)) / dv;
    blue = 0;
  }
  else
  {
    green = 1 + (4 * (minValue + (0.75f * dv) - value) / dv);
    blue = 0;
  }

  return;
}


// taken from SlimDXControl
void ColorFilter::getColor(float value, byte& red, byte& green, byte& blue)
{
  float fred, fgreen, fblue;

  SlimDX_GetColor(value, (float)getHeader().m_minZ, (float)getHeader().m_maxZ, fred, fblue, fgreen);

  red = (byte)(fred * 255.0);
  green = (byte)(fgreen * 255.0);
  blue = (byte)(fblue * 255.0);

  return;
}
