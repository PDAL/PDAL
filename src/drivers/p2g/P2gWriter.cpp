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

#include <pdal/drivers/p2g/P2gWriter.hpp>
#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <points2grid/Interpolation.hpp>


namespace pdal
{
namespace drivers
{
namespace p2g
{


void P2gWriter::processOptions(const Options& options)
{
    m_GRID_DIST_X = options.getValueOrDefault<double>("grid_dist_x", 6.0);
    m_GRID_DIST_Y = options.getValueOrDefault<double>("grid_dist_y", 6.0);
    m_RADIUS_SQ = options.getValueOrDefault<double>("radius",
        8.4852813742385713);
    m_fill_window_size = options.getValueOrDefault<boost::uint32_t>(
        "fill_window_size", 3);
    m_filename = options.getValueOrThrow<std::string>("filename");

    std::vector<Option> types = options.getOptions("output_type");

    if (!types.size())
        m_outputTypes = OUTPUT_TYPE_ALL;
    else
    {
        for (auto i = types.begin(); i != types.end(); ++i)
        {
            if (boost::iequals(i->getValue<std::string>(), "min"))
                m_outputTypes |= OUTPUT_TYPE_MIN;
            if (boost::iequals(i->getValue<std::string>(), "max"))
                m_outputTypes |= OUTPUT_TYPE_MAX;
            if (boost::iequals(i->getValue<std::string>(), "mean"))
                m_outputTypes |= OUTPUT_TYPE_MEAN;
            if (boost::iequals(i->getValue<std::string>(), "idw"))
                m_outputTypes |= OUTPUT_TYPE_IDW;
            if (boost::iequals(i->getValue<std::string>(), "den"))
                m_outputTypes |= OUTPUT_TYPE_DEN;
            if (boost::iequals(i->getValue<std::string>(), "std"))
                m_outputTypes |= OUTPUT_TYPE_STD;
            if (boost::iequals(i->getValue<std::string>(), "all"))
                m_outputTypes = OUTPUT_TYPE_ALL;
        }
    }

    std::string output_format =
        options.getValueOrDefault<std::string>("output_format", "grid");
    if (boost::iequals(output_format, "grid"))
        m_outputFormat = OUTPUT_FORMAT_GRID_ASCII;
    else if (boost::iequals(output_format, "asc"))
        m_outputFormat = OUTPUT_FORMAT_ARC_ASCII;
    else if (boost::iequals(output_format, "tif"))
        m_outputFormat = OUTPUT_FORMAT_GDAL_GTIFF;
    else if (boost::iequals(output_format, "all"))
        m_outputFormat = OUTPUT_FORMAT_ALL;
    else
    {
        std::ostringstream oss;
        oss << "Unrecognized output format " << output_format;
        throw p2g_error("Unrecognized output format");
    }
}


/*
void P2gWriter::ready(PointContext ctx)
{
    double min_x = (std::numeric_limits<double>::max)();
    double max_x = (std::numeric_limits<double>::min)();
    double min_y = (std::numeric_limits<double>::max)();
    double max_y = (std::numeric_limits<double>::min)();
    setBounds(pdal::Bounds<double>(min_x, min_y, max_x, max_y));
}
*/


Options P2gWriter::getDefaultOptions()
{
    Options options;

    Option grid_x("grid_dist_x", 6.0, "X grid distance");
    Option grid_y("grid_dist_y", 6.0, "Y grid distance");

    double default_radius = (double) sqrt(2.0) * grid_x.getValue<double>();
    Option radius("radius", default_radius);

    Option fill_window_size("fill_window_size", 3);
    Option dim_z("Z", "Z", "Name of Z dimension to interpolate");
    options.add(dim_z);
    options.add(grid_x);
    options.add(grid_y);
    options.add(radius);
    options.add(fill_window_size);
    return options;
}


void P2gWriter::write(const PointBuffer& buf)
{
    std::string z_name = getOptions().getValueOrDefault<std::string>("Z", "Z");


    for (point_count_t idx = 0; idx < buf.size(); idx++)
    {
        double x = buf.getFieldAs<double>(Dimension::Id::X, idx);
        double y = buf.getFieldAs<double>(Dimension::Id::Y, idx);
        double z = buf.getFieldAs<double>(Dimension::Id::Z, idx);
        m_coordinates.push_back(boost::tuple<double, double, double>(x, y, z));
    }

    m_bounds = buf.calculateBounds();

    m_GRID_SIZE_X = (int)(ceil((m_bounds.getMaximum(0) - m_bounds.getMinimum(0))/m_GRID_DIST_X)) + 1;
    m_GRID_SIZE_Y = (int)(ceil((m_bounds.getMaximum(1) - m_bounds.getMinimum(1))/m_GRID_DIST_Y)) + 1;

    log()->get(LogLevel::Debug) << "X grid size: " << m_GRID_SIZE_X << std::endl;
    log()->get(LogLevel::Debug) << "Y grid size: " << m_GRID_SIZE_Y << std::endl;


    log()->floatPrecision(6);
    log()->get(LogLevel::Debug) << "X grid distance: " << m_GRID_DIST_X << std::endl;
    log()->get(LogLevel::Debug) << "Y grid distance: " << m_GRID_DIST_Y << std::endl;
    log()->clearFloat();

    boost::scoped_ptr<OutCoreInterp> p(new OutCoreInterp(m_GRID_DIST_X,
                                       m_GRID_DIST_Y,
                                       m_GRID_SIZE_X,
                                       m_GRID_SIZE_Y,
                                       m_RADIUS_SQ,
                                       m_bounds.getMinimum(0),
                                       m_bounds.getMaximum(0),
                                       m_bounds.getMinimum(1),
                                       m_bounds.getMaximum(1),
                                       m_fill_window_size));
    m_interpolator.swap(p);

    if (m_interpolator->init() < 0)
    {
        throw p2g_error("unable to initialize interpolator");
    }

    int rc(0);

    std::vector<boost::tuple<double, double, double> >::const_iterator i;
    for (i = m_coordinates.begin(); i!= m_coordinates.end(); ++i)
    {
        double x = i->get<0>();
        double y = i->get<1>();
        x = x - m_bounds.getMinimum(0);
        y = y - m_bounds.getMinimum(1);

        rc = m_interpolator->update(x, y, i->get<2>());
        if (rc < 0)
        {
            throw p2g_error("interp->update() error while processing ");
        }
    }

    double adfGeoTransform[6];
    adfGeoTransform[0] = m_bounds.getMinimum(0);
    adfGeoTransform[1] = m_GRID_DIST_X;
    adfGeoTransform[2] = 0.0;
    adfGeoTransform[3] = m_bounds.getMaximum(1);
    adfGeoTransform[4] = 0.0;
    adfGeoTransform[5] = -1 * m_GRID_DIST_Y;
    
    SpatialReference const& srs = getSpatialReference();

    if ((rc = m_interpolator->finish(const_cast<char*>(m_filename.c_str()), m_outputFormat, m_outputTypes, adfGeoTransform, srs.getWKT().c_str())) < 0)
    {
        throw p2g_error("interp->finish() error");
    }

    return;
}



}
}
} // namespaces
