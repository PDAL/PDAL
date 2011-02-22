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
#include <iostream>

#include "libpc/FauxReader.hpp"
#include "libpc/Utils.hpp"

using std::vector;
using std::string;
using std::cout;

namespace libpc
{

FauxReader::FauxReader(const Bounds<double>& bounds, int numPoints)
    : Reader()
{
    Header* header = new Header;
    Schema& schema = header->getSchema();

    header->setNumPoints(numPoints);
    header->setBounds(bounds);

    schema.addDimension(Dimension("XPos", Dimension::Float));
    schema.addDimension(Dimension("YPos", Dimension::Float));
    schema.addDimension(Dimension("ZPos", Dimension::Float));
    schema.addDimension(Dimension("Time", Dimension::Double));

    setHeader(header);

    cout << header;

    return;
}


void FauxReader::readPoints(PointData& data)
{
    // make up some data and put it into the buffer

    boost::uint32_t numPoints = data.getNumPoints();
    assert(m_currentPointIndex + numPoints <= getHeader().getNumPoints());

    const Schema& schema = data.getSchema();
    Header& header = getHeader();

    std::size_t fieldIndexT;
    bool ok = schema.findDimensionIndex("Time", fieldIndexT);
    assert(ok);

    float v = (float)m_currentPointIndex;

    const Bounds<double>& bounds = header.getBounds();
    const std::vector<Range<double>>& dims = bounds.dimensions();
    const double minX = dims[0].getMinimum();
    const double maxX = dims[0].getMaximum();
    const double minY = dims[1].getMinimum();
    const double maxY = dims[1].getMaximum();
    const double minZ = dims[2].getMinimum();
    const double maxZ = dims[2].getMaximum();

    std::size_t offsetX;
    std::size_t offsetY;
    std::size_t offsetZ;

    ok = schema.findDimensionIndex("XPos", offsetX);
    assert(ok);
    ok = schema.findDimensionIndex("YPos", offsetY);
    assert(ok);
    ok = schema.findDimensionIndex("ZPos", offsetZ);
    assert(ok);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        const float x = (float)Utils::random(minX, maxX);
        const float y = (float)Utils::random(minY, maxY);
        const float z = (float)Utils::random(minZ, maxZ);

        data.setValid(pointIndex);

        data.setField<float>(pointIndex, offsetX, x);
        data.setField<float>(pointIndex, offsetY, y);
        data.setField<float>(pointIndex, offsetZ, z);

        data.setField<double>(pointIndex, fieldIndexT, v * 0.1);

        ++v;
    }

    m_currentPointIndex += numPoints;

    m_numPointsRead += numPoints;

    return;
}


} // namespace libpc
