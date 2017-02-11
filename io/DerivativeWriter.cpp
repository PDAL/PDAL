/******************************************************************************
* Copyright (c) 2015-2016, Bradley J Chambers, brad.chambers@gmail.com
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

#include "DerivativeWriter.hpp"

#include <pdal/EigenUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{
static PluginInfo const s_info =
    PluginInfo("writers.derivative", "Derivative writer",
               "http://pdal.io/stages/writers.derivative.html");

CREATE_STATIC_PLUGIN(1, 0, DerivativeWriter, Writer, s_info)

std::string DerivativeWriter::getName() const
{
    return s_info.name;
}

void DerivativeWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("edge_length", "Edge length", m_edgeLength, 15.0);
    args.add("primitive_type", "Primitive type", m_primTypesSpec, {"slope_d8"});
    args.add("slope_units", "Slope units (degrees/percent)", m_slopeUnit,
             "percent");
    args.add("altitude", "Illumination altitude (degrees)", m_illumAltDeg,
             45.0);
    args.add("azimuth", "Illumination azimuth (degrees)", m_illumAzDeg, 315.0);
    args.add("driver", "GDAL format driver", m_driver, "GTiff");
}


void DerivativeWriter::initialize()
{
    static std::map<std::string, PrimitiveType> primtypes =
    {
        {"slope_d8", SLOPE_D8},
        {"slope_fd", SLOPE_FD},
        {"aspect_d8", ASPECT_D8},
        {"aspect_fd", ASPECT_FD},
        {"hillshade", HILLSHADE},
        {"contour_curvature", CONTOUR_CURVATURE},
        {"profile_curvature", PROFILE_CURVATURE},
        {"tangential_curvature", TANGENTIAL_CURVATURE},
        {"total_curvature", TOTAL_CURVATURE}
    };

    auto hashPos = handleFilenameTemplate(m_filename);
    if (hashPos == std::string::npos && m_primTypesSpec.size() > 1)
        throwError("No template placeholder ('#') found in filename '" +
            m_filename + "' when one is required with multiple primitive "
            "types.");

    for (std::string os : m_primTypesSpec)
    {
        std::string s = Utils::tolower(os);
        auto pi = primtypes.find(s);
        if (pi == primtypes.end())
            throwError("Unrecognized primitive type '" + os + "'.");
        TypeOutput to;
        to.m_type = pi->second;
        to.m_filename = generateFilename(pi->first, hashPos);
        m_primitiveTypes.push_back(to);
    }
}


std::string DerivativeWriter::generateFilename(const std::string& primName,
        std::string::size_type hashPos) const
{
    std::string filename = m_filename;
    if (hashPos != std::string::npos)
        filename.replace(hashPos, 1, primName);
    return filename;
}


void DerivativeWriter::write(const PointViewPtr data)
{
    using namespace eigen;
    using namespace Eigen;

    // Bounds are required for computing number of rows and columns, and for
    // later indexing individual points into the appropriate raster cells.
    BOX2D bounds;
    data->calculateBounds(bounds);
    SpatialReference srs = data->spatialReference();

    // Determine the number of rows and columns at the given cell size.
    size_t cols = ((bounds.maxx - bounds.minx) / m_edgeLength) + 1;
    size_t rows = ((bounds.maxy - bounds.miny) / m_edgeLength) + 1;

    // Begin by creating a DSM of max elevations per XY cell.
    MatrixXd DSM = createMaxMatrix2(*data.get(), rows, cols, m_edgeLength,
                                    bounds);

    // We will pad the edges by 1 cell, though according to some texts we should
    // simply compute forward- or backward-difference as opposed to centered
    // difference at these points.
    MatrixXd paddedDSM = padMatrix(DSM, 1);

    // Prepare the out matrix.
    MatrixXd out(DSM.rows(), DSM.cols());
    out.setConstant(std::numeric_limits<double>::quiet_NaN());

    for (TypeOutput& to : m_primitiveTypes)
    {
        for (int r = 1; r < paddedDSM.rows()-1; ++r)
        {
            for (int c = 1; c < paddedDSM.cols()-1; ++c)
            {
                double val = paddedDSM(r, c);
                if (std::isnan(val))
                    continue;
                Matrix3d block = paddedDSM.block(r-1, c-1, 3, 3);
                if (to.m_type == SLOPE_D8)
                {
                    out(r-1, c-1) = computeSlopeD8(block, m_edgeLength);
                    if (Utils::iequals(m_slopeUnit, "degrees"))
                        out(r-1, c-1) = percentSlopeToDegrees(out(r-1, c-1));
                }
                if (to.m_type == SLOPE_FD)
                {
                    out(r-1, c-1) = computeSlopeFD(block, m_edgeLength);
                    if (Utils::iequals(m_slopeUnit, "degrees"))
                        out(r-1, c-1) = percentSlopeToDegrees(out(r-1, c-1));
                }
                if (to.m_type == ASPECT_D8)
                    out(r-1, c-1) = computeAspectD8(block, m_edgeLength);
                if (to.m_type == ASPECT_FD)
                    out(r-1, c-1) = computeAspectFD(block, m_edgeLength);
                if (to.m_type == HILLSHADE)
                    out(r-1, c-1) = computeHillshade(block, m_edgeLength,
                                                     m_illumAltDeg,
                                                     m_illumAzDeg);
                if (to.m_type == CONTOUR_CURVATURE)
                    out(r-1, c-1) = computeContour(block, m_edgeLength);
                if (to.m_type == PROFILE_CURVATURE)
                    out(r-1, c-1) = computeProfile(block, m_edgeLength);
                if (to.m_type == TANGENTIAL_CURVATURE)
                    out(r-1, c-1) = computeTangential(block, m_edgeLength);
                if (to.m_type == TOTAL_CURVATURE)
                    out(r-1, c-1) = computeTotal(block, m_edgeLength);
            }
        }

        // Finally, write our Matrix as a GDAL raster (specifically GTiff).
        writeMatrix(out, to.m_filename, m_driver, m_edgeLength, bounds, srs);
    }
}

} // namespace pdal
