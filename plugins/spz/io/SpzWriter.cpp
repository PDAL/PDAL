/******************************************************************************
* Copyright (c) 2025, Isaac Bell (isaac@hobu.co)
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

#include "SpzWriter.hpp"
#include "Util.hpp"

#include <pdal/util/OStream.hpp>

#include <arbiter/arbiter.hpp>

namespace pdal
{

static PluginInfo const s_info
{
        "writers.spz",
        "SPZ writer",
        "http://pdal.io/stages/writers.spz.html",
};

CREATE_SHARED_STAGE(SpzWriter, s_info)

SpzWriter::SpzWriter() : m_cloud(new spz::GaussianCloud)
{}

std::string SpzWriter::getName() const { return s_info.name; }

void SpzWriter::addArgs(ProgramArgs& args)
{
    args.add("antialiased", "Mark the data as antialiased", m_antialiased);
    args.add("input_orientation", "Coordinate system of points being written; default is 'RUB'. "
        "Note that SPZ will always write RUB", m_coordTransform, "RUB");
}

void SpzWriter::initialize()
{
    if (Utils::isRemote(filename()))
    {
        // swap our filename for a tmp file
        std::string tmpname = Utils::tempFilename(filename());
        m_remoteFilename = filename();
        setFilename(tmpname);
    }
    m_packOptions.from = spz::getCoordinateSystem(m_coordTransform);
}

Dimension::Id SpzWriter::tryFindDim(PointLayoutPtr layout, const std::string& dimName)
{
    Dimension::Id id = layout->findDim(dimName);
    if (id == Dimension::Id::Unknown)
        log()->get(LogLevel::Warning) << "Dimension " << dimName << " not found." << std::endl;
    return id;
}

void SpzWriter::checkDimensions(PointLayoutPtr layout)
{
    // looking for our spz-specific dims.

    // we expect 3 scale/color and 4 rotation dimensions with PLY-style labels
    for (int i = 0; i < 3; ++i)
    {
        m_scaleDims.push_back(tryFindDim(layout, "scale_" + std::to_string(i)));
        m_plyColorDims.push_back(tryFindDim(layout, "f_dc_" + std::to_string(i)));
        m_rotDims.push_back(tryFindDim(layout, "rot_" + std::to_string(i + 1)));
    }
    // rotation W component (rot_0) is added as the last item
    m_rotDims.push_back(tryFindDim(layout, "rot_0"));
    m_plyAlphaDim = tryFindDim(layout, "opacity");

    // find spherical harmonics dimensions, if there are any
    for (const auto& dim : layout->dimTypes())
    {
        std::string dimName = Utils::tolower(layout->dimName(dim.m_id));
        if (Utils::startsWith(dimName, "f_rest_"))
            m_shDims.push_back(dim.m_id);
    }

    // check spherical harmonics dimensions
    switch (m_shDims.size())
    {
        case 0:
            m_shDegree = 0;
            break;
        case 9:
            m_shDegree = 1;
            break;
        case 24:
            m_shDegree = 2;
            break;
        case 45:
            m_shDegree = 3;
            break;
        default:
            log()->get(LogLevel::Warning) << "Invalid spherical harmonics dimensions " <<
                "for '" << filename() << "': expected 0, 9, 24 or 45 dimensions " <<
                "labeled 'f_rest_*': found " << m_shDims.size() << std::endl;
            m_shDims.clear();
            m_shDegree = 0;
    }
    // check float RGB
    if (m_plyColorDims.size() != 3)
        m_plyColorDims.clear();
}

void SpzWriter::prepared(PointTableRef table)
{
    checkDimensions(table.layout());
}

void SpzWriter::write(const PointViewPtr data)
{
    point_count_t pointCount = data->size();
    //!! do some check for the max size of file here?

    m_cloud->numPoints = int(pointCount);
    m_cloud->shDegree = m_shDegree;
    m_cloud->antialiased = m_antialiased;

    m_cloud->positions.reserve(pointCount * 3);
    m_cloud->scales.reserve(pointCount * 3);
    m_cloud->rotations.reserve(pointCount * 3);
    m_cloud->alphas.reserve(pointCount);
    m_cloud->colors.reserve(pointCount * 3);
    m_cloud->sh.reserve(pointCount * m_shDims.size());

    int numSh = m_shDims.size() / 3;
    PointRef point(*data, 0);
    for (PointId idx = 0; idx < pointCount; ++idx)
    {
        point.setPointId(idx);

        m_cloud->positions.push_back(point.getFieldAs<float>(Dimension::Id::X));
        m_cloud->positions.push_back(point.getFieldAs<float>(Dimension::Id::Y));
        m_cloud->positions.push_back(point.getFieldAs<float>(Dimension::Id::Z));

        for (int i = 0; i < 3; ++i)
        {
            m_cloud->scales.push_back(point.getFieldAs<float>(m_scaleDims[i]));
        }
        for (int i = 0; i < 4; ++i)
        {
            m_cloud->rotations.push_back(point.getFieldAs<float>(m_rotDims[i]));
        }

        for (int i = 0; i < 3; ++i)
        m_cloud->colors.push_back(point.getFieldAs<float>(m_plyColorDims[i]));

        m_cloud->alphas.push_back(point.getFieldAs<float>(m_plyAlphaDim));

        if (m_shDegree)
        {
            for (int i = 0; i < numSh; i++) 
            {
                // R sh component
                m_cloud->sh.push_back(point.getFieldAs<float>(m_shDims[i]));
                // G sh component
                m_cloud->sh.push_back(point.getFieldAs<float>(m_shDims[i + numSh]));
                // B sh component
                m_cloud->sh.push_back(point.getFieldAs<float>(m_shDims[i + 2 * numSh]));
            }
        }
    }
}

void SpzWriter::done(PointTableRef table)
{
    if (!spz::saveSpz(*m_cloud.get(), m_packOptions, filename()))
        throwError("Unable to save SPZ data to " + filename());

    if (m_remoteFilename.size())
    {
        arbiter::Arbiter a;
        a.put(filename(), a.getBinary(filename()));

        FileUtils::deleteFile(filename());

        setFilename(m_remoteFilename);
        m_remoteFilename.clear();
    }
}

} // namespace pdal
