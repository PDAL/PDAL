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

namespace libpc
{

FauxReader::FauxReader(const Bounds<double>& bounds, int numPoints)
    : Reader()
{
    Header& header = getHeader();
    PointLayout& layout = header.getLayout();

    header.setNumPoints(numPoints);
    header.setBounds(bounds);

    layout.addField(Dimension("XPos", Dimension::float_t));
    layout.addField(Dimension("YPos", Dimension::float_t));
    layout.addField(Dimension("ZPos", Dimension::float_t));
    layout.addField(Dimension("Time", Dimension::double_t));

    header.dump();

    return;
}


void FauxReader::readPoints(PointData& data)
{
    // make up some data and put it into the buffer

    boost::uint32_t numPoints = data.getNumPoints();
    assert(m_currentPointIndex + numPoints <= getHeader().getNumPoints());

    const PointLayout& layout = data.getLayout();
    Header& header = getHeader();

    std::size_t fieldIndexT;
    bool ok = layout.findFieldIndex("Time", fieldIndexT);
    assert(ok);

    float v = (float)m_currentPointIndex;

    const Bounds<double>& bounds = header.getBounds();
    const double minX = bounds.dims()[0].minimum();
    const double maxX = bounds.dims()[0].maximum();
    const double minY = bounds.dims()[1].minimum();
    const double maxY = bounds.dims()[1].maximum();
    const double minZ = bounds.dims()[2].minimum();
    const double maxZ = bounds.dims()[2].maximum();

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        const float x = (float)Utils::random(minX, maxX);
        const float y = (float)Utils::random(minY, maxY);
        const float z = (float)Utils::random(minZ, maxZ);

        data.setValid(pointIndex);

        std::size_t offsetX;
        std::size_t offsetY;
        std::size_t offsetZ;
        bool ok;
        
        ok = layout.findFieldIndex("XPos", offsetX);
        assert(ok);
        ok = layout.findFieldIndex("YPos", offsetY);
        assert(ok);
        ok = layout.findFieldIndex("ZPos", offsetZ);
        assert(ok);

        data.setField_F32(pointIndex, offsetX, x);
        data.setField_F32(pointIndex, offsetY, y);
        data.setField_F32(pointIndex, offsetZ, z);

        data.setField_F64(pointIndex, fieldIndexT, v * 0.1);

        ++v;
    }

    m_currentPointIndex += numPoints;

    m_numPointsRead += numPoints;

    return;
}


} // namespace libpc
