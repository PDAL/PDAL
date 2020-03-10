/******************************************************************************
* Copyright (c) 2019, Hobu Inc., info@hobu.co
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "MemoryViewReader.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.memoryview",
    "Memory View Reader",
    "http://pdal.io/stages/readers.memoryview.html",
    {}
};

CREATE_STATIC_STAGE(MemoryViewReader, s_info)

std::string MemoryViewReader::getName() const { return s_info.name; }

MemoryViewReader::MemoryViewReader() : m_prepared(false)
{}


// NOTE: - Forces reading of the entire file.
/**
QuickInfo MemoryViewReader::inspect()
{
    QuickInfo qi;
    FixedPointTable t(100);

    StatsFilter f;
    f.setInput(*this);

    f.prepare(t);
    PointLayoutPtr layout = t.layout();
    for (Dimension::Id id : layout->dims())
        qi.m_dimNames.push_back(layout->dimName(id));
    f.execute(t);

    try
    {
        stats::Summary xSummary = f.getStats(Dimension::Id::X);
        qi.m_pointCount = xSummary.count();
        qi.m_bounds.minx = xSummary.minimum();
        qi.m_bounds.maxx = xSummary.maximum();
        stats::Summary ySummary = f.getStats(Dimension::Id::Y);
        qi.m_bounds.miny = ySummary.minimum();
        qi.m_bounds.maxy = ySummary.maximum();
        stats::Summary zSummary = f.getStats(Dimension::Id::Z);
        qi.m_bounds.minz = zSummary.minimum();
        qi.m_bounds.maxz = zSummary.maximum();
        qi.m_valid = true;
    }
    catch (pdal_error&)
    {}
    return qi;
}
**/


void MemoryViewReader::pushField(const Field& f)
{
    if (m_prepared)
        throwError("Can't pushField() after MemoryViewReader is prepared.");

    for (auto& tempField : m_fields)
        if (tempField.m_name == f.m_name)
            throwError("Attempt to push duplicate field with name '" +
                f.m_name + ".'");

    m_fields.emplace_back(f);
}


void MemoryViewReader::addDimensions(PointLayoutPtr layout)
{
    for (auto& f : m_fields)
        f.m_id = layout->registerOrAssignDim(f.m_name, f.m_type);
}


void MemoryViewReader::initialize()
{
    m_prepared = false;
}


void MemoryViewReader::prepared(PointTableRef)
{
    int xyz = 0;
    for (const FullField& f : m_fields)
    {
        if (f.m_name == "X")
            xyz |= 1;
        else if (f.m_name == "Y")
            xyz |= 2;
        else if (f.m_name == "Z")
            xyz |= 4;
    }
    if (xyz != 0 && xyz != 7)
        throwError("Fields must contain all or none of X, Y and Z.");
    if (xyz == 0 && !m_shape.valid())
        throwError("Fields don't contain X, Y and Z and no shape "
            "was provided.");
    if (xyz == 7 && m_shape.valid())
        throwError("Can't provide `shape` option when Fields contain "
            "X, Y and Z.");
    if (xyz == 0)
    {
        if (m_order == Order::RowMajor)
        {
            m_xDiv = 1;
            m_yDiv = m_shape.columns();
            m_zDiv = m_shape.columns() * m_shape.rows();

            m_xIter = m_shape.columns();
            m_yIter = m_shape.columns() * m_shape.rows();
            m_zIter = m_shape.columns() * m_shape.rows() * m_shape.depth();
        }
        else   // Column Major
        {
            m_xDiv = m_shape.depth() * m_shape.rows();
            m_yDiv = m_shape.depth();
            m_zDiv = 1;

            m_xIter = m_shape.depth() * m_shape.rows() * m_shape.columns();
            m_yIter = m_shape.depth() * m_shape.rows();
            m_zIter = m_shape.depth();
        }
    }
    m_prepared = true;
}


void MemoryViewReader::ready(PointTableRef)
{
    if (!m_incrementer)
        throwError("Points cannot be read without calling setIncrementer().");
    m_index = 0;
}


point_count_t MemoryViewReader::read(PointViewPtr v, point_count_t numPts)
{
    PointId idx = v->size();
    point_count_t cnt = 0;

    PointRef point(*v);
    while (cnt < numPts)
    {
        point.setPointId(idx);
        if (!processOne(point))
            break;
        cnt++;
        idx++;
    }
    return cnt;
}


bool MemoryViewReader::processOne(PointRef& point)
{
    char *base = m_incrementer(m_index);
    if (!base)
        return false;

    for (const FullField& f : m_fields)
        point.setField(f.m_id, f.m_type, (void *)(base + f.m_offset));

    if (m_shape.valid())
    {
        point.setField(Dimension::Id::X, (m_index % m_xIter) / m_xDiv);
        point.setField(Dimension::Id::Y, (m_index % m_yIter) / m_yDiv);
        point.setField(Dimension::Id::Z, (m_index % m_zIter) / m_zDiv);
    }

    m_index++;
    return true;
}

} // namespace pdal

