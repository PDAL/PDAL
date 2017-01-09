/******************************************************************************
* Copyright (c) 2017, Hobu Inc., info@hobu.co
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

#include "AssignFilter.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.assign",
    "Assign values for a dimension using a specified value.",
    "http://pdal.io/stages/filters.assign.html" );

CREATE_STATIC_PLUGIN(1, 0, AssignFilter, Filter, s_info)

void AssignFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension on which to filter", m_dimName).
        setPositional();
    args.add("value", "Value to set on matching points", m_value).
        setPositional();
}


void AssignFilter::prepared(PointTableRef table)
{
    m_dim = table.layout()->findDim(m_dimName);
    if (m_dim == Dimension::Id::Unknown)
        throw pdal_error(getName() + ": Dimension '" + m_dimName +
            "' not found.");
}


bool AssignFilter::processOne(PointRef& point)
{
    point.setField(m_dim, m_value);
    return true;
}


void AssignFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId id = 0; id < view.size(); ++id)
    {
        point.setPointId(id);
        processOne(point);
    }
}

} // namespace pdal

