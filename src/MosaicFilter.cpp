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

namespace libpc
{

// BUG: will generalize to more than 2 inputs
MosaicFilter::MosaicFilter(Stage& prevStage, Stage& prevStage2)
    : Filter(prevStage),
      m_prevStage2(prevStage2)
{
    const Header& prevHeader1 =  m_prevStage.getHeader();
    const Header& prevHeader2 =  m_prevStage2.getHeader();

    {
        const Schema& prevSchema1 = prevHeader1.getSchema();
        const Schema& prevSchema2 = prevHeader2.getSchema();
        assert(prevSchema1==prevSchema2);
    }

    Header& thisHeader = getHeader();

    Bounds<double> bigbox(prevHeader1.getBounds());
    bigbox.grow(prevHeader2.getBounds());
    thisHeader.setBounds(bigbox);

    thisHeader.setNumPoints(prevHeader1.getNumPoints() + prevHeader2.getNumPoints());

    return;
}


boost::uint32_t MosaicFilter::readPoints(PointData& destData)
{
    boost::uint32_t numPoints = destData.getNumPoints();

    // BUG: this doesn't account for isValid()

    // BUG:
    // We're given a buffer of size N to fill, but we have two sources
    // feeding us -- so we do a read of N/2 points from each one

    assert(numPoints % 2 == 0); // yeah right

    PointData srcData1(destData.getSchemaLayout(), numPoints / 2);
    PointData srcData2(destData.getSchemaLayout(), numPoints / 2);

    m_prevStage.readPoints(srcData1);

    m_prevStage2.readPoints(srcData2);

    int destPointIndex = 0;

    for (boost::uint32_t srcPointIndex=0; srcPointIndex<numPoints/2; srcPointIndex++)
    {
        if (srcData1.isValid(srcPointIndex))
        {
            destData.copyPointFast(destPointIndex, srcPointIndex, srcData1);
            destData.setValid(destPointIndex, true);
        }
        else
        {
            destData.setValid(destPointIndex, false);
        }
        destPointIndex++;
    }
    for (boost::uint32_t srcPointIndex=0; srcPointIndex<numPoints/2; srcPointIndex++)
    {
        if (srcData2.isValid(srcPointIndex))
        {
            destData.copyPointFast(destPointIndex, srcPointIndex, srcData2);
            destData.setValid(destPointIndex, true);
        }
        else
        {
            destData.setValid(destPointIndex, false);
        }
        destPointIndex++;
    }

    // BUG: when we're done, we will have gotten only half the data from our sources...!

    return numPoints;
}


} // namespace libpc
