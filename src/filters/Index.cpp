/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/filters/Index.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <algorithm>

#include <pdal/PointBuffer.hpp>

#ifdef PDAL_HAVE_GDAL
#include <gdal.h>
#include <ogr_spatialref.h>
#include <pdal/GDALUtils.hpp>
#endif

namespace pdal
{
namespace filters
{


Index::Index(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    return;
}

void Index::initialize()
{
    Filter::initialize();


    return;
}


const Options Index::getDefaultOptions() const
{
    Options options;

    Option x("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    Option y("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");

    pdal::Option red("dimension", "Red", "");
    pdal::Option b0("band",1, "");
    pdal::Option s0("scale", 1.0f, "scale factor for this dimension");
    pdal::Options redO;
    redO.add(b0);
    redO.add(s0);
    red.setOptions(redO);

    pdal::Option green("dimension", "Green", "");
    pdal::Option b1("band",2, "");
    pdal::Option s1("scale", 1.0f, "scale factor for this dimension");
    pdal::Options greenO;
    greenO.add(b1);
    greenO.add(s1);
    green.setOptions(greenO);

    pdal::Option blue("dimension", "Blue", "");
    pdal::Option b2("band",3, "");
    pdal::Option s2("scale", 1.0f, "scale factor for this dimension");
    pdal::Options blueO;
    blueO.add(b2);
    blueO.add(s2);
    blue.setOptions(blueO);

    pdal::Option reproject("reproject", false, "Reproject the input data into the same coordinate system as the raster?");

    options.add(x);
    options.add(y);
    options.add(red);
    options.add(green);
    options.add(blue);
    options.add(reproject);


    return options;
}



void Index::processBuffer(PointBuffer& /* data */) const
{
    return;
}


pdal::StageSequentialIterator* Index::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Index(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Index::Index(const pdal::filters::Index& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_stage(filter)
{
    return;
}

void Index::readBufferBeginImpl(PointBuffer& buffer)
{
    // Cache dimension positions

    pdal::Schema const& schema = buffer.getSchema();
 

}



boost::uint32_t Index::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    return numRead;
}


boost::uint64_t Index::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Index::atEndImpl() const
{
    return getPrevIterator().atEnd();
}


}
} // iterators::sequential


}
} // namespaces
