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

#include <pdal/filters/Attribute.hpp>

#include <pdal/PointBuffer.hpp>

namespace pdal { namespace filters {


Attribute::Attribute(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    return;
}

void Attribute::initialize()
{
    Filter::initialize();
    
    std::string expression = getOptions().getValueOrDefault<std::string>("expression", "empty") ;
    log()->get(logDEBUG)  << "expression " << expression << std::endl;

    // if (parser::parse_doubles(expression.begin(), expression.end()))
    // {
    //     log()->get(logDEBUG)  << "parsed expression: " << expression << std::endl;
    // } else
    // {
    //     log()->get(logDEBUG)  << "unable to parse expression: " << expression << std::endl;
    // }
}


const Options Attribute::getDefaultOptions() const
{
    Options options;
    Option expression("expression", "");
    options.add(expression);
    return options;
}




void Attribute::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();
    // 
    // const Schema& schema = data.getSchema();
    // 
    // const int indexR = schema.getDimensionIndex(DimensionId::Red_u16);
    // const int indexG = schema.getDimensionIndex(DimensionId::Green_u16);
    // const int indexB = schema.getDimensionIndex(DimensionId::Blue_u16);
    // const int indexZ = schema.getDimensionIndex(DimensionId::Z_i32);
    // const Dimension& zDim = schema.getDimension(DimensionId::Z_i32);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        // const boost::int32_t zraw = data.getField<boost::int32_t>(pointIndex, indexZ);
        // const double z = zDim.applyScaling(zraw);
        // 
        // boost::uint16_t red, green, blue;
        // // this->getAttribute_F64_U16(z, red, green, blue);
        // 
        // // now we store the 3 u16's in the point data...
        // data.setField<boost::uint16_t>(pointIndex, indexR, red);
        // data.setField<boost::uint16_t>(pointIndex, indexG, green);
        // data.setField<boost::uint16_t>(pointIndex, indexB, blue);

        data.setNumPoints(pointIndex+1);
    }

    return;
}



pdal::StageSequentialIterator* Attribute::createSequentialIterator() const
{
    return new pdal::filters::iterators::sequential::Attribute(*this);
}

namespace iterators { namespace sequential {

Attribute::Attribute(const pdal::filters::Attribute& filter)
    : pdal::FilterSequentialIterator(filter)
    , m_attributeFilter(filter)
{
    return;
}


boost::uint32_t Attribute::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    m_attributeFilter.processBuffer(data);

    return numRead;
}


boost::uint64_t Attribute::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Attribute::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

} } // iterators::sequential

} } // pdal::filters
