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

#include "FauxReader.hpp"

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <ctime>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.faux",
    "Faux Reader",
    "http://pdal.io/stages/readers.faux.html" );

CREATE_STATIC_PLUGIN(1, 0, FauxReader, Reader, s_info)

std::string FauxReader::getName() const { return s_info.name; }

void FauxReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "X/Y/Z limits", m_bounds, BOX3D(0, 0, 0, 1, 1, 1));
    args.add("mean_x", "X mean", m_mean_x);
    args.add("mean_y", "Y mean", m_mean_y);
    args.add("mean_z", "Z mean", m_mean_z);
    args.add("stdev_x", "X standard deviation", m_stdev_x, 1.0);
    args.add("stdev_y", "Y standard deviation", m_stdev_y, 1.0);
    args.add("stdev_z", "Z standard deviation", m_stdev_z, 1.0);
    args.add("mode", "Point creation mode", m_mode);
    args.add("number_of_returns", "Max number of returns", m_numReturns);
}


void FauxReader::initialize()
{
    if (m_numReturns > 10)
    {
        std::ostringstream oss;
        oss << getName() << ": Option 'number_of_returns' must be in the range "
            "[0,10].";
        throw pdal_error(oss.str());
    }
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


void FauxReader::addDimensions(PointLayoutPtr layout)
{
    Dimension::IdList ids = { Dimension::Id::X, Dimension::Id::Y,
        Dimension::Id::Z, Dimension::Id::OffsetTime };

    layout->registerDims(ids);
    if (m_numReturns > 0)
    {
        layout->registerDim(Dimension::Id::ReturnNumber);
        layout->registerDim(Dimension::Id::NumberOfReturns);
    }
}


void FauxReader::ready(PointTableRef /*table*/)
{
    m_returnNum = 1;
    m_time = 0;
    m_seed = (uint32_t)std::time(NULL);
    m_index = 0;
}


bool FauxReader::processOne(PointRef& point)
{
    double x(0);
    double y(0);
    double z(0);

    if (m_index >= m_count)
        return false;

    switch (m_mode)
    {
    case Mode::Random:
        x = Utils::random(m_bounds.minx, m_bounds.maxx);
        y = Utils::random(m_bounds.miny, m_bounds.maxy);
        z = Utils::random(m_bounds.minz, m_bounds.maxz);
        break;
    case Mode::Constant:
        x = m_bounds.minx;
        y = m_bounds.miny;
        z = m_bounds.minz;
        break;
    case Mode::Ramp:
        x = m_bounds.minx + m_delX * m_index;
        y = m_bounds.miny+ m_delY * m_index;
        z = m_bounds.minz + m_delZ * m_index;
        break;
    case Mode::Uniform:
        x = Utils::uniform(m_bounds.minx, m_bounds.maxx, m_seed++);
        y = Utils::uniform(m_bounds.miny, m_bounds.maxy, m_seed++);
        z = Utils::uniform(m_bounds.minz, m_bounds.maxz, m_seed++);
        break;
    case Mode::Normal:
        x = Utils::normal(m_mean_x, m_stdev_x, m_seed++);
        y = Utils::normal(m_mean_y, m_stdev_y, m_seed++);
        z = Utils::normal(m_mean_z, m_stdev_z, m_seed++);
        break;
    }

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::OffsetTime, m_time++);
    if (m_numReturns > 0)
    {
        point.setField(Dimension::Id::ReturnNumber, m_returnNum);
        point.setField(Dimension::Id::NumberOfReturns, m_numReturns);
        m_returnNum = (m_returnNum % m_numReturns) + 1;
    }
    m_index++;
    return true;
}


point_count_t FauxReader::read(PointViewPtr view, point_count_t count)
{
    for (PointId idx = 0; idx < count; ++idx)
    {
        PointRef point = view->point(idx);
        if (!processOne(point))
            break;
        if (m_cb)
            m_cb(*view, idx);
    }
    return count;
}

} // namespace pdal
