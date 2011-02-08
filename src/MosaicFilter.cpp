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
#include "libpc/MosaicFilter.hpp"


// BUG: will generalize to more than 2 inputs
MosaicFilter::MosaicFilter(Stage& prevStage, Stage& prevStage2)
  : Filter(prevStage),
  m_prevStage2(prevStage2)
{
  return;
}


void MosaicFilter::initialize()
{
  Filter::initialize();
  return;
}


void MosaicFilter::updateLayout()
{
  Header& header =  getHeader();
  const Header& header1 =  m_prevStage.getConstHeader();
  const Header& header2 =  m_prevStage2.getConstHeader();

  m_prevStage.updateLayout();
  const PointLayout& layout = header1.getConstPointLayout();

  header.getPointLayout() = layout;

  // BUG: update header bounds to include both stages
  Bounds bigbox(header1.getBounds());
  bigbox.grow(header2.getBounds());
  header.setBounds(bigbox);

  header.setNumPoints(header1.getNumPoints() + header2.getNumPoints());

  return;
}


void MosaicFilter::readNextPoints(PointData& destData)
{
  int numPoints = destData.getNumPoints();

  // We're given a buffer of size N to fill, but we have two sources
  // feeding us -- so we do a read of N/2 points from each one

  assert(numPoints % 2 == 0); // yeah right

  PointData srcData1(destData.getLayout(), numPoints / 2);
  PointData srcData2(destData.getLayout(), numPoints / 2);

  m_prevStage.readNextPoints(srcData1);

  m_prevStage2.readNextPoints(srcData2);

  int destPointIndex = 0;

  for (int srcPointIndex=0; srcPointIndex<numPoints/2; srcPointIndex++)
  {
    if (srcData1.isValid(srcPointIndex))
    {
      destData.copyFieldsFast(destPointIndex, srcPointIndex, srcData1);
      destData.setValid(destPointIndex, true);
    }
    else
    {
      destData.setValid(destPointIndex, false);
    }
    destPointIndex++;
  }
  for (int srcPointIndex=0; srcPointIndex<numPoints/2; srcPointIndex++)
  {
    if (srcData2.isValid(srcPointIndex))
    {
      destData.copyFieldsFast(destPointIndex, srcPointIndex, srcData2);
      destData.setValid(destPointIndex, true);
    }
    else
    {
      destData.setValid(destPointIndex, false);
    }
    destPointIndex++;
  }

  // BUG: when we're done, we will have gotten only half the data from our sources...!

  return;
}
