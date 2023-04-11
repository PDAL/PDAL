/******************************************************************************
 * Copyright (c) 2021 TileDB, Inc.
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

#include "XYZTmUtils.hpp"
#include <pdal/Metadata.hpp>

namespace pdal
{

static StaticPluginInfo const s_info{"readers.XYZTimeFauxReader",
                                     "XYZ time Faux Reader", "none"};

CREATE_STATIC_STAGE(XYZTimeFauxReader, s_info);

std::string XYZTimeFauxReader::getName() const
{
    return s_info.name;
}

void XYZTimeFauxReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "X/Y/Z/time limits", m_bounds,
             BOX4D(0., 0., 0., 0., 1., 1., 1., 1));
    args.add("xyz_mode", "mode for distribution of xyz dimension values",
             m_xyz_mode, Mode::Uniform);
    args.add("time_mode", "mode for distribution of time dimension values",
             m_tm_mode, Mode::Ramp);
    args.add("use_time",
             "Add a time dimension in addition to XYZ (default true)",
             m_use_time, true);
    args.add("dim4_name",
             "Use this to change the name of the 4th dimension from 'GpsTime'",
             m_dim4_name, "GpsTime");
    args.add("density", "Double value to set density dimension in points",
             m_density, 1.0);
}

void XYZTimeFauxReader::prepared(PointTableRef table)
{
    if (!m_countArg->set())
        throwError("Argument 'count' needs a value and none was provided.");
}

void XYZTimeFauxReader::initialize()
{

    m_generator.seed((uint32_t)std::time(NULL));

    if (m_xyz_mode == Mode::Uniform)
    {
        m_uniformX.reset(new urd(m_bounds.minx, m_bounds.maxx));
        m_uniformY.reset(new urd(m_bounds.miny, m_bounds.maxy));
        m_uniformZ.reset(new urd(m_bounds.minz, m_bounds.maxz));
    }
    else if (m_xyz_mode == Mode::Ramp)
    {
        if (m_count > 1)
        {
            m_delX = (m_bounds.maxx - m_bounds.minx) / (m_count - 1);
            m_delY = (m_bounds.maxy - m_bounds.miny) / (m_count - 1);
            m_delZ = (m_bounds.maxz - m_bounds.minz) / (m_count - 1);
        }
        else
        {
            m_delX = 0;
            m_delY = 0;
            m_delZ = 0;
        }
    }

    if (m_use_time)
    {
        if (m_tm_mode == Mode::Uniform)
        {
            m_uniformTm.reset(new urd(m_bounds.mintm, m_bounds.maxtm));
        }
        else if (m_tm_mode == Mode::Ramp)
        {
            if (m_count > 1)
            {
                m_delTm = (m_bounds.maxtm - m_bounds.mintm) / (m_count - 1);
            }
            else
            {
                m_delTm = 0;
            }
        }
    }
}

void XYZTimeFauxReader::addDimensions(PointLayoutPtr layout)
{
    Dimension::IdList ids;
    if (m_use_time && m_dim4_name == "GpsTime")
    {
        ids = {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z,
               Dimension::Id::GpsTime, Dimension::Id::Density};
    }
    else
    {
        ids = {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z,
               Dimension::Id::Density};
    }
    layout->registerDims(ids);
    if (m_use_time && m_dim4_name != "GpsTime")
        layout->registerOrAssignDim(m_dim4_name, Dimension::Type::Double);
}

void XYZTimeFauxReader::ready(PointTableRef table)
{
    m_index = 0;
}

#pragma warning(push)
#pragma warning(disable : 4244)

bool XYZTimeFauxReader::processOne(PointRef& point)
{
    double x(0);
    double y(0);
    double z(0);
    double tm(0);

    if (m_index >= m_count)
        return false;

    switch (m_xyz_mode)
    {
    case Mode::Uniform:
        x = (*m_uniformX)(m_generator);
        y = (*m_uniformY)(m_generator);
        z = (*m_uniformZ)(m_generator);
        break;
    case Mode::Ramp:
        x = m_bounds.minx + m_delX * m_index;
        y = m_bounds.miny + m_delY * m_index;
        z = m_bounds.minz + m_delZ * m_index;
        break;
    default:
        throwError("Invalid mode: only 'uniform' and 'ramp' implemented");
    }

    if (m_use_time)
    {
        switch (m_tm_mode)
        {
        case Mode::Uniform:
            tm = (*m_uniformTm)(m_generator);
            break;
        case Mode::Ramp:
            tm = m_bounds.mintm + m_delTm * m_index;
            break;
        default:
            throwError("Invalid mode: only 'uniform' and 'ramp' implemented");
        }
    }

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    if (m_use_time)
        point.setField(Dimension::id(m_dim4_name), tm);
    point.setField(Dimension::Id::Density, m_density);
    m_index++;
    return true;
}

#pragma warning(pop)

point_count_t XYZTimeFauxReader::read(PointViewPtr view, point_count_t count)
{
    for (PointId idx = 0; idx < count; ++idx)
    {
        PointRef point = view->point(idx);
        if (!processOne(point))
            break;
        if (m_cb)
            m_cb(*view, idx);
    };
    return count;
}

} // namespace pdal
