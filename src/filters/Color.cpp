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

#include <pdal/filters/Color.hpp>

#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace filters
{


Color::Color(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    return;
}


Color::Color(Stage& prevStage)
    : Filter(prevStage, Options::none())
{
    return;
}

std::vector<Dimension> Color::getDefaultDimensions()
{
    std::vector<Dimension> output;
    Dimension red("Red", dimension::UnsignedInteger, 2);
    red.setUUID("de62488f-5559-440f-bb60-d74db2c78c63");
    red.setNamespace(s_getName());
    output.push_back(red);

    Dimension green("Green", dimension::UnsignedInteger, 2);
    green.setUUID("6c3ebeb4-ffc0-4b57-9b4a-479939631599");
    green.setNamespace(s_getName());
    output.push_back(green);

    Dimension blue("Blue", dimension::UnsignedInteger, 2);
    blue.setUUID("7cdd9ec4-a209-4f14-ad55-1eda94aec750");
    blue.setNamespace(s_getName());
    output.push_back(blue);
    
    return output;

}


void Color::initialize()
{
    Filter::initialize();
}


Options Color::getDefaultOptions()
{
    Options options;
    return options;
}


void Color::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const Schema& schema = data.getSchema();

    boost::optional<Dimension const&> dimZ = schema.getDimensionOptional("Z");
    boost::optional<Dimension const&> dimRed = schema.getDimensionOptional("Red");
    boost::optional<Dimension const&> dimGreen = schema.getDimensionOptional("Green");
    boost::optional<Dimension const&> dimBlue = schema.getDimensionOptional("Blue");

    if (!dimZ) throw pdal_error("Unable to get 'Z' dimension for colorization!");

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        const boost::int32_t zraw = data.getField<boost::int32_t>(*dimZ, pointIndex);
        const double z = dimZ->applyScaling(zraw);

        boost::uint16_t red, green, blue;
        this->getColor_F64_U16(z, red, green, blue);

        // now we store the 3 u16's in the point data...
        data.setField<boost::uint16_t>(*dimRed, pointIndex, red);
        data.setField<boost::uint16_t>(*dimGreen, pointIndex, green);
        data.setField<boost::uint16_t>(*dimBlue, pointIndex, blue);

        data.setNumPoints(pointIndex+1);
    }

    return;
}


// static function to impute a color from within a range
void Color::interpolateColor(double value, double minValue, double maxValue, double& red, double& green, double& blue)
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

    double dv = maxValue - minValue;

    if (value < (minValue + (0.25 * dv)))
    {
        red = 0;
        green = 4 * (value - minValue) / dv;
    }
    else if (value < (minValue + (0.5 * dv)))
    {
        red = 0;
        blue = 1 + (4 * (minValue + (0.25 * dv) - value) / dv);
    }
    else if (value < (minValue + (0.75 * dv)))
    {
        red = 4 * (value - minValue - (0.5 * dv)) / dv;
        blue = 0;
    }
    else
    {
        green = 1 + (4 * (minValue + (0.75 * dv) - value) / dv);
        blue = 0;
    }

    return;
}



void Color::getColor_F32_U8(float value, boost::uint8_t& red, boost::uint8_t& green, boost::uint8_t& blue) const
{
    double fred, fgreen, fblue;

    const Range<double>& zrange = getBounds().dimensions()[2];
    interpolateColor(value, zrange.getMinimum(), zrange.getMaximum(), fred, fblue, fgreen);

    const double vmax = (std::numeric_limits<boost::uint8_t>::max)();
    red = (boost::uint8_t)(fred * vmax);
    green = (boost::uint8_t)(fgreen * vmax);
    blue = (boost::uint8_t)(fblue * vmax);

    return;
}


void Color::getColor_F64_U16(double value, boost::uint16_t& red, boost::uint16_t& green, boost::uint16_t& blue) const
{
    double fred, fgreen, fblue;

    const Range<double>& zrange = getBounds().dimensions()[2];
    interpolateColor(value, zrange.getMinimum(), zrange.getMaximum(), fred, fblue, fgreen);

    const double vmax = (std::numeric_limits<boost::uint16_t>::max)();
    red = (boost::uint16_t)(fred * vmax);
    green = (boost::uint16_t)(fgreen * vmax);
    blue = (boost::uint16_t)(fblue * vmax);

    return;
}


pdal::StageSequentialIterator* Color::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Color(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Color::Color(const pdal::filters::Color& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_colorFilter(filter)
{
    return;
}


boost::uint32_t Color::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    m_colorFilter.processBuffer(data);

    return numRead;
}


boost::uint64_t Color::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Color::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

}
} // iterators::sequential

}
} // pdal::filters
