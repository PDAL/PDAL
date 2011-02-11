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
#include "libpc/Color.hpp"
#include "libpc/ColorFilter.hpp"

namespace libpc
{

ColorFilter::ColorFilter(Stage& prevStage)
    : Filter(prevStage)
{
    PointLayout& layout = getHeader().getLayout();

    // add the three u8 fields
    layout.addField(Field(Field::Zred, Field::U8));
    layout.addField(Field(Field::Zgreen, Field::U8));
    layout.addField(Field(Field::Zblue, Field::U8));

    return;
}


void ColorFilter::readNextPoints(PointData& data)
{
    m_prevStage.readNextPoints(data);

    int numPoints = data.getNumPoints();

    const PointLayout& layout = data.getLayout();

    int fieldIndexR = layout.findFieldIndex(Field::Zred);
    assert(fieldIndexR != -1);
    int fieldIndexG = layout.findFieldIndex(Field::Zgreen);
    assert(fieldIndexG != -1);
    int fieldIndexB = layout.findFieldIndex(Field::Zblue);
    assert(fieldIndexB != -1);

    for (int pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        float z = data.getZ(pointIndex);
        boost::uint8_t red, green, blue;
        getColor(z, red, green, blue);

        // now we store the 3 u8's in the point data...
        data.setField_U8(pointIndex, fieldIndexR, red);
        data.setField_U8(pointIndex, fieldIndexG, green);
        data.setField_U8(pointIndex, fieldIndexB, blue);

    }

    return;
}

void ColorFilter::getColor(float value, boost::uint8_t& red, boost::uint8_t& green, boost::uint8_t& blue)
{
    double fred, fgreen, fblue;

    const Range<double>& zrange = getHeader().getBounds().dims()[2];
    Color::interpolateColor(value, zrange.minimum(), zrange.maximum(), fred, fblue, fgreen);

    const double vmax = (std::numeric_limits<boost::uint8_t>::max());
    red = (boost::uint8_t)(fred * vmax);
    green = (boost::uint8_t)(fgreen * vmax);
    blue = (boost::uint8_t)(fblue * vmax);

    return;
}

} // namespace libpc
