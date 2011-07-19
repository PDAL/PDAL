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

#include <pdal/filters/ColorFilter.hpp>

#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>
#include <pdal/exceptions.hpp>
#include <pdal/Color.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/ColorFilterIterator.hpp>

namespace pdal { namespace filters {

ColorFilter::ColorFilter(const DataStagePtr& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    checkImpedance();

    return;
}


void ColorFilter::checkImpedance()
{
    Schema& schema = getSchemaRef();

    Dimension dimZ(Dimension::Field_Z, Dimension::Int32);
    if (schema.hasDimension(dimZ) == false)
    {
        throw impedance_invalid("color filter does not have Z/Int32 field");
    }

    Dimension dimRed(Dimension::Field_Red, Dimension::Uint16);
    Dimension dimGreen(Dimension::Field_Green, Dimension::Uint16);
    Dimension dimBlue(Dimension::Field_Blue, Dimension::Uint16);

    // are there already u16 fields for color?
    if (!schema.hasDimension(dimRed))
    {
        schema.addDimension(dimRed);
    }
    if (!schema.hasDimension(dimGreen))
    {
        schema.addDimension(dimGreen);
    }
    if (!schema.hasDimension(dimBlue))
    {
        schema.addDimension(dimBlue);
    }

    return;
}


const std::string& ColorFilter::getDescription() const
{
    static std::string name("Color Filter");
    return name;
}

const std::string& ColorFilter::getName() const
{
    static std::string name("filters.color");
    return name;
}

void ColorFilter::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    const int indexR = schema.getDimensionIndex(Dimension::Field_Red, Dimension::Uint16);
    const int indexG = schema.getDimensionIndex(Dimension::Field_Green, Dimension::Uint16);
    const int indexB = schema.getDimensionIndex(Dimension::Field_Blue, Dimension::Uint16);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const Dimension& zDim = schema.getDimension(indexZ);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        const boost::int32_t zraw = data.getField<boost::int32_t>(pointIndex, indexZ);
        const double z = zDim.applyScaling(zraw);

        boost::uint16_t red, green, blue;
        this->getColor_F64_U16(z, red, green, blue);

        // now we store the 3 u16's in the point data...
        data.setField<boost::uint16_t>(pointIndex, indexR, red);
        data.setField<boost::uint16_t>(pointIndex, indexG, green);
        data.setField<boost::uint16_t>(pointIndex, indexB, blue);

        data.setNumPoints(pointIndex+1);
    }

    return;
}


void ColorFilter::getColor_F32_U8(float value, boost::uint8_t& red, boost::uint8_t& green, boost::uint8_t& blue) const
{
    double fred, fgreen, fblue;

    const Range<double>& zrange = getBounds().dimensions()[2];
    Color::interpolateColor(value, zrange.getMinimum(), zrange.getMaximum(), fred, fblue, fgreen);

    const double vmax = (std::numeric_limits<boost::uint8_t>::max)();
    red = (boost::uint8_t)(fred * vmax);
    green = (boost::uint8_t)(fgreen * vmax);
    blue = (boost::uint8_t)(fblue * vmax);

    return;
}


void ColorFilter::getColor_F64_U16(double value, boost::uint16_t& red, boost::uint16_t& green, boost::uint16_t& blue) const
{
    double fred, fgreen, fblue;

    const Range<double>& zrange = getBounds().dimensions()[2];
    Color::interpolateColor(value, zrange.getMinimum(), zrange.getMaximum(), fred, fblue, fgreen);

    const double vmax = (std::numeric_limits<boost::uint16_t>::max)();
    red = (boost::uint16_t)(fred * vmax);
    green = (boost::uint16_t)(fgreen * vmax);
    blue = (boost::uint16_t)(fblue * vmax);

    return;
}


pdal::StageSequentialIterator* ColorFilter::createSequentialIterator() const
{
    return new ColorFilterSequentialIterator(*this);
}

} } // namespaces
