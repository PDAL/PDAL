/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/drivers/p2g/Writer.hpp>
#include <pdal/PointBuffer.hpp>
#include <iostream>

namespace pdal { namespace drivers { namespace p2g {


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
{

    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();
    return;
}


const Options Writer::getDefaultOptions() const
{
    Options options;

    // Option filename("filename", "", "file to read from");
    // Option compression("compression", false, "Do we LASzip-compress the data?");
    // Option format("format", PointFormat3, "Point format to write");
    // Option major_version("major_version", 1, "LAS Major version");
    // Option minor_version("minor_version", 2, "LAS Minor version");
    // Option day_of_year("day_of_year", 0, "Day of Year for file");
    // Option year("year", 2011, "4-digit year value for file");
    // Option system_id("system_id", LasHeader::SystemIdentifier, "System ID for this file");
    // Option software_id("software_id", LasHeader::SoftwareIdentifier, "Software ID for this file");
    // Option header_padding("header_padding", 0, "Header padding (space between end of VLRs and beginning of point data)");
    // 
    // options.add(major_version);
    // options.add(minor_version);
    // options.add(day_of_year);
    // options.add(year);
    // options.add(system_id);
    // options.add(software_id);
    // options.add(header_padding);
    // options.add(format);
    // options.add(filename);
    // options.add(compression);
    return options;
}


void Writer::writeBegin(boost::uint64_t targetNumPointsToWrite)
{
    // need to set properties of the header here, based on prev->getHeader() and on the user's preferences
    // m_lasHeader.setBounds( getPrevStage().getBounds() );

    const Schema& schema = getPrevStage().getSchema();

    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& pointBuffer)
{
    const Schema& schema = pointBuffer.getSchema();
    const Dimension& xDim = schema.getDimension(DimensionId::X_i32);
    const Dimension& yDim = schema.getDimension(DimensionId::Y_i32);
    const Dimension& zDim = schema.getDimension(DimensionId::Z_i32);


    boost::uint32_t numValidPoints = 0;

    boost::uint8_t buf[1024]; // BUG: fixed size

    for (boost::uint32_t pointIndex=0; pointIndex<pointBuffer.getNumPoints(); pointIndex++)
    {
        boost::uint8_t* p = buf;

        // // we always write the base fields
        // const boost::int32_t x = pointBuffer.getRawField<boost::int32_t>(pointIndex, positions.X);
        // const boost::int32_t y = pointBuffer.getRawField<boost::int32_t>(pointIndex, positions.Y);
        // const boost::int32_t z = pointBuffer.getRawField<boost::int32_t>(pointIndex, positions.Z);
        
        // std::clog << "x: " << x << " y: " << y << " z: " << z << std::endl;
        // std::clog << "positions.X: " << positions.X << " positions.Y: " << positions.Y << " positions.Z: " << positions.Z << std::endl;
        

    }

    // m_numPointsWritten = m_numPointsWritten+numValidPoints;
    return numValidPoints;
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

} } } // namespaces
