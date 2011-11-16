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
#include <algorithm>

#include <points2grid/Interpolation.hpp>


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
    
    m_GRID_DIST_X = getOptions().getValueOrDefault<double>("grid_dist_x", 6.0);
    m_GRID_DIST_Y = getOptions().getValueOrDefault<double>("grid_dist_y", 6.0);
    m_RADIUS_SQ = getOptions().getValueOrDefault<double>("radius", 8.4852813742385713);
    
    double min_x = (std::numeric_limits<double>::max)();
    double max_x = (std::numeric_limits<double>::min)();
    double min_y = (std::numeric_limits<double>::max)();
    double max_y = (std::numeric_limits<double>::min)();

    setBounds(pdal::Bounds<double>(min_x, min_y, max_x, max_y));
    
    return;
}


const Options Writer::getDefaultOptions() const
{
    Options options;
    
    Option grid_x("grid_dist_x", 6.0, "X grid distance");
    Option grid_y("grid_dist_y", 6.0, "Y grid distance");
    
    double default_radius = (double) sqrt(2.0) * grid_x.getValue<double>();
    Option radius("radius", default_radius);

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

    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{

    calculateGridSizes();
    log()->get(logDEBUG) << "m_GRID_SIZE_X: " << m_GRID_SIZE_X << std::endl;
    log()->get(logDEBUG) << "m_GRID_SIZE_Y: " << m_GRID_SIZE_Y << std::endl;
    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{
    const Schema& schema = data.getSchema();

    int xPos = schema.getDimensionIndex(pdal::DimensionId::X_i32);
    pdal::Dimension const& xDim = schema.getDimension(xPos);

    int yPos = schema.getDimensionIndex(pdal::DimensionId::Y_i32);
    pdal::Dimension const& yDim = schema.getDimension(yPos);
    
    int zPos = schema.getDimensionIndex(pdal::DimensionId::Z_i32);
    pdal::Dimension const& zDim = schema.getDimension(zPos);


    boost::uint32_t numPoints = 0;

    
    double xd(0.0);
    double yd(0.0);
    double zd(0.0);
    
    for (boost::uint32_t pointIndex=0; pointIndex < data.getNumPoints(); pointIndex++)
    {
            boost::int32_t x = data.getField<boost::int32_t>(pointIndex, xPos);
            boost::int32_t y = data.getField<boost::int32_t>(pointIndex, yPos);
            boost::int32_t z = data.getField<boost::int32_t>(pointIndex, zPos);
        
            xd = xDim.applyScaling<boost::int32_t>(x);
            yd = yDim.applyScaling<boost::int32_t>(y);
            zd = zDim.applyScaling<boost::int32_t>(z);
            
            m_bounds.setMinimum(0, (std::min)(xd, m_bounds.getMinimum(0)));
            m_bounds.setMinimum(1, (std::min)(yd, m_bounds.getMinimum(1)));
            m_bounds.setMaximum(0, (std::max)(xd, m_bounds.getMaximum(0)));
            m_bounds.setMaximum(1, (std::max)(yd, m_bounds.getMaximum(1)));
            xd -= getBounds().getMinimum(0);
            yd -= getBounds().getMinimum(1);
            m_coordinates.push_back(boost::tuple<double, double, double>(xd, yd, zd));
            numPoints++;
    }

    return numPoints;
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

void Writer::calculateGridSizes() 
{
    pdal::Bounds<double>& extent = getBounds();
    
    m_GRID_SIZE_X = (int)(ceil((extent.getMaximum(0) - extent.getMinimum(0))/m_GRID_DIST_X)) + 1;
    m_GRID_SIZE_Y = (int)(ceil((extent.getMaximum(1) - extent.getMinimum(1))/m_GRID_DIST_Y)) + 1;
}

} } } // namespaces
