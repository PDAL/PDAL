/******************************************************************************
* Copyright (c) 2014, Pete Gadomski <pete.gadomski@gmail.com>
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

#include "TransformationFilter.hpp"

#include <pdal/pdal_export.hpp>

#include <sstream>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.transformation",
    "Transform each point using a 4x4 transformation matrix",
    "http://pdal.io/stages/filters.transformation.html" );

CREATE_STATIC_PLUGIN(1, 0, TransformationFilter, Filter, s_info)

std::string TransformationFilter::getName() const { return s_info.name; }

TransformationMatrix transformationMatrixFromString(const std::string& s)
{
    std::istringstream iss(s);
    TransformationMatrix matrix;
    double entry;
    TransformationMatrix::size_type i = 0;
    while (iss >> entry)
    {
        if (i + 1 > matrix.size())
        {
            std::stringstream msg;
            msg << "Too many entries in transformation matrix, should be "
                << matrix.size();
            throw pdal_error(msg.str());
        }
        matrix[i++] = entry;
    }

    if (i != matrix.size())
    {
        std::stringstream msg;
        msg << "Too few entries in transformation matrix: "
            << i
            << " (should be "
            << matrix.size()
            << ")";

        throw pdal_error(msg.str());
    }

    return matrix;
}


void TransformationFilter::processOptions(const Options& options)
{
    m_matrix = transformationMatrixFromString(options.getValueOrThrow<std::string>("matrix"));
}


void TransformationFilter::filter(PointView& view)
{
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        double x = view.getFieldAs<double>(Dimension::Id::X, idx);
        double y = view.getFieldAs<double>(Dimension::Id::Y, idx);
        double z = view.getFieldAs<double>(Dimension::Id::Z, idx);

        view.setField(Dimension::Id::X, idx,
            x * m_matrix[0] + y * m_matrix[1] + z * m_matrix[2] + m_matrix[3]);

        view.setField(Dimension::Id::Y, idx,
            x * m_matrix[4] + y * m_matrix[5] + z * m_matrix[6] + m_matrix[7]);

        view.setField(Dimension::Id::Z, idx,
            x * m_matrix[8] + y * m_matrix[9] + z * m_matrix[10] + m_matrix[11]);
    }
}

} // namespace pdal
