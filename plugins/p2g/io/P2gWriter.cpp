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

#include "P2gWriter.hpp"
#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>

#include <iostream>
#include <algorithm>

#include <points2grid/Interpolation.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.p2g",
    "Points2Grid Writer",
    "http://pdal.io/stages/writers.p2g.html" );

CREATE_SHARED_PLUGIN(1, 0, P2gWriter, Writer, s_info)

std::string P2gWriter::getName() const { return s_info.name; }

void P2gWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("grid_dist_x", "X grid distance", m_GRID_DIST_X, 6.0);
    args.add("grid_dist_y", "Y grid distance", m_GRID_DIST_Y, 6.0);
    args.add("radius", "Radius", m_RADIUS, 8.4852813742385713);
    args.add("fill_window_size", "Fill window size", m_fill_window_size, 3U);
    args.add("output_type", "Output type", m_outputTypeSpec);
    args.add("output_format", "Output format", m_outputFormatSpec, "grid");
    args.add("bounds", "Output raster bounds", m_bounds);

}

void P2gWriter::initialize()
{
    m_outputTypes = 0;
    for (std::string& type : m_outputTypeSpec)
    {
        std::string val = Utils::tolower(type);
        if (val == "min")
            m_outputTypes |= OUTPUT_TYPE_MIN;
        else if (val == "max")
            m_outputTypes |= OUTPUT_TYPE_MAX;
        else if (val == "mean")
            m_outputTypes |= OUTPUT_TYPE_MEAN;
        else if (val == "idw")
            m_outputTypes |= OUTPUT_TYPE_IDW;
        else if (val == "den")
            m_outputTypes |= OUTPUT_TYPE_DEN;
        else if (val == "std")
            m_outputTypes |= OUTPUT_TYPE_STD;
        else if (val == "all")
            m_outputTypes = OUTPUT_TYPE_ALL;
        else
        {
            std::ostringstream oss;

            oss << "Unrecognized output type '" << type << "'."; 
            throw p2g_error(oss.str());
        }
    }
    if (m_outputTypes == 0)
        m_outputTypes = OUTPUT_TYPE_ALL;

    std::string fmt = Utils::tolower(m_outputFormatSpec);
    if (fmt == "grid")
        m_outputFormat = OUTPUT_FORMAT_GRID_ASCII;
    else if (fmt == "asc")
        m_outputFormat = OUTPUT_FORMAT_ARC_ASCII;
    else if (fmt == "tif")
        m_outputFormat = OUTPUT_FORMAT_GDAL_GTIFF;
    else if (fmt == "all")
        m_outputFormat = OUTPUT_FORMAT_ALL;
    else
    {
        std::ostringstream oss;

        oss << "Unrecognized output format '" << m_outputFormatSpec << "'";
        throw p2g_error(oss.str());
    }
}


void P2gWriter::ready(PointTableRef table)
{
    if (!table.spatialReferenceUnique())
    {
        std::ostringstream oss;

        oss << getName() << ": Can't write output with multiple spatial "
            "references.";
        throw pdal_error(oss.str());
    }
}


void P2gWriter::write(const PointViewPtr view)
{
    for (point_count_t idx = 0; idx < view->size(); idx++)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, idx);
        double y = view->getFieldAs<double>(Dimension::Id::Y, idx);
        double z = view->getFieldAs<double>(Dimension::Id::Z, idx);
        m_coordinates.push_back(Coordinate{x, y, z});
    }

    if (m_bounds.empty()) {
        view->calculateBounds(m_bounds);
    }
}

void P2gWriter::done(PointTableRef table)
{
    // If we never got any points, we're done.
    if (! m_coordinates.size()) return;

    m_GRID_SIZE_X = (int)(ceil((m_bounds.maxx - m_bounds.minx)/m_GRID_DIST_X)) + 1;
    m_GRID_SIZE_Y = (int)(ceil((m_bounds.maxy - m_bounds.miny)/m_GRID_DIST_Y)) + 1;

    log()->get(LogLevel::Debug) << "X grid size: " << m_GRID_SIZE_X << std::endl;
    log()->get(LogLevel::Debug) << "Y grid size: " << m_GRID_SIZE_Y << std::endl;


    log()->floatPrecision(6);
    log()->get(LogLevel::Debug) << "X grid distance: " << m_GRID_DIST_X << std::endl;
    log()->get(LogLevel::Debug) << "Y grid distance: " << m_GRID_DIST_Y << std::endl;
    log()->clearFloat();

    std::unique_ptr<OutCoreInterp> p(new OutCoreInterp(m_GRID_DIST_X,
                                       m_GRID_DIST_Y,
                                       m_GRID_SIZE_X,
                                       m_GRID_SIZE_Y,
                                       m_RADIUS * m_RADIUS,
                                       m_bounds.minx,
                                       m_bounds.maxx,
                                       m_bounds.miny,
                                       m_bounds.maxy,
                                       m_fill_window_size));
    m_interpolator.swap(p);

    if (m_interpolator->init() < 0)
    {
        throw p2g_error("unable to initialize interpolator");
    }

    for (auto coord : m_coordinates)
    {
        double x = coord.x - m_bounds.minx;
        double y = coord.y - m_bounds.miny;
        double z = coord.z;

        if (m_interpolator->update(x, y, z) < 0)
            throw p2g_error("interp->update() error while processing ");
    }

    double adfGeoTransform[6];
    adfGeoTransform[0] = m_bounds.minx - 0.5*m_GRID_DIST_X;
    adfGeoTransform[1] = m_GRID_DIST_X;
    adfGeoTransform[2] = 0.0;
    adfGeoTransform[3] = m_bounds.maxy + 0.5*m_GRID_DIST_Y;
    adfGeoTransform[4] = 0.0;
    adfGeoTransform[5] = -1 * m_GRID_DIST_Y;

    SpatialReference const& srs = table.spatialReference();

    log()->get(LogLevel::Debug) << "Output SRS  :'" << srs.getWKT() << "'" <<
        std::endl;

    // Strip off the extension if it was provided so that we don't get
    // file.asc.type.asc or file.asc.asc, as point2grid appends a file
    // extension.
    std::string extension = FileUtils::extension(m_filename);
    if (extension == ".asc" || extension == ".grid" || extension == ".tif")
        m_filename = m_filename.substr(0, m_filename.find_last_of("."));

    if (m_interpolator->finish(m_filename.c_str(),
        m_outputFormat, m_outputTypes, adfGeoTransform,
        srs.getWKT().c_str()) < 0)
    {
        throw p2g_error("interp->finish() error");
    }
}

} // namespaces
