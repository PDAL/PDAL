/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <iomanip>

#include <pdal/KDIndex.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Algorithm.hpp>

#include "private/Raster.hpp"

namespace pdal
{

int PointView::m_lastId = 0;

PointView::PointView(PointTableRef pointTable) : m_pointTable(pointTable),
    m_layout(pointTable.layout()), m_size(0), m_id(0)
{
	m_id = ++m_lastId;
}

PointView::PointView(PointTableRef pointTable, const SpatialReference& srs) :
	m_pointTable(pointTable), m_layout(pointTable.layout()), m_size(0),
    m_id(0), m_spatialReference(srs)
{
	m_id = ++m_lastId;
}

PointView::~PointView()
{}


PointViewIter PointView::begin()
{
    return PointViewIter(this, 0);
}


PointViewIter PointView::end()
{
    return PointViewIter(this, size());
}


PointId PointView::tableId(PointId idx)
{
    if (idx > size())
        throw pdal_error("Point index must increment.");
    if (idx == size())
    {
        PointId rawId = m_pointTable.addPoint();
        m_index.push_back(rawId);
        m_size++;
        assert(m_temps.empty());
        return rawId;
    }
    return m_index[idx];
}


void PointView::setFieldInternal(Dimension::Id dim, PointId idx,
    const void *buf)
{
    PointId rawId = 0;
    if (idx == size())
    {
        rawId = m_pointTable.addPoint();
        m_index.push_back(rawId);
        m_size++;
        assert(m_temps.empty());
    }
    else if (idx > size())
    {
        std::cerr << "Point index must increment.\n";
        //error - throw?
        return;
    }
    else
    {
        rawId = m_index[idx];
    }
    m_pointTable.setFieldInternal(dim, rawId, buf);
}


void PointView::calculateBounds(BOX2D& output) const
{
    for (PointId idx = 0; idx < size(); idx++)
    {
        double x = getFieldAs<double>(Dimension::Id::X, idx);
        double y = getFieldAs<double>(Dimension::Id::Y, idx);

        output.grow(x, y);
    }
}


void PointView::calculateBounds(BOX3D& output) const
{
    for (PointId idx = 0; idx < size(); idx++)
    {
        double x = getFieldAs<double>(Dimension::Id::X, idx);
        double y = getFieldAs<double>(Dimension::Id::Y, idx);
        double z = getFieldAs<double>(Dimension::Id::Z, idx);

        output.grow(x, y, z);
    }
}


MetadataNode PointView::toMetadata() const
{
    MetadataNode node;

    const Dimension::IdList& dims = layout()->dims();

    for (PointId idx = 0; idx < size(); idx++)
    {
        MetadataNode pointnode = node.add(std::to_string(idx));
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            double v = getFieldAs<double>(*di, idx);
            pointnode.add(layout()->dimName(*di), v);
        }
    }
    return node;
}


TriangularMesh *PointView::createMesh(const std::string& name)
{
    if (Utils::contains(m_meshes, name))
        return nullptr;
    auto res = m_meshes.insert(std::make_pair(name,
        std::unique_ptr<TriangularMesh>(new TriangularMesh)));
    if (res.second)
        return res.first->second.get();
    return nullptr;
}


TriangularMesh *PointView::mesh(const std::string& name)
{
    auto it = m_meshes.find(name);
    if (it != m_meshes.end())
        return it->second.get();
    if (name.empty() && m_meshes.size())
        return m_meshes.begin()->second.get();
    return nullptr;
}


Rasterd *PointView::createRaster(const std::string& name, const RasterLimits& limits)
{
    if (Utils::contains(m_rasters, name))
        return nullptr;
    auto res = m_rasters.insert(std::make_pair(name,
        std::unique_ptr<Rasterd>(new Rasterd(limits))));
    if (res.second)
        return res.first->second.get();
    return nullptr;
}


Rasterd *PointView::raster(const std::string& name)
{
    auto it = m_rasters.find(name);
    if (it != m_rasters.end())
        return it->second.get();
    if (name.empty() && m_rasters.size())
        return m_rasters.begin()->second.get();
    return nullptr;
}


void PointView::invalidateProducts()
{
    m_index2.reset();
    m_index3.reset();
    // Should all meshes also be invalidated?
}


KD3Index& PointView::build3dIndex()
{
    //ABELL
    // Should we allow a force of point view build - perhaps the index has
    // changed or the point values have changed.
    if (!m_index3)
    {
        m_index3.reset(new KD3Index(*this));
        m_index3->build();
    }
    return *m_index3.get();
}


KD2Index& PointView::build2dIndex()
{
    //ABELL
    // Should we allow a force of point view build - perhaps the index has
    // changed or the point values have changed.
    if (!m_index2)
    {
        m_index2.reset(new KD2Index(*this));
        m_index2->build();
    }
    return *m_index2.get();
}


void PointView::dump(std::ostream& ostr) const
{
    using std::endl;
    PointLayoutPtr layout = m_pointTable.layout();
    const Dimension::IdList& dims = layout->dims();

    point_count_t numPoints = size();
    ostr << "Contains " << numPoints << "  points" << endl;
    for (PointId idx = 0; idx < numPoints; idx++)
    {
        ostr << "Point: " << idx << endl;

        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            Dimension::Id d = *di;
            const Dimension::Detail *dd = layout->dimDetail(d);
            ostr << layout->dimName(d) << " (" <<
                Dimension::interpretationName(dd->type()) << ") : ";

            switch (dd->type())
            {
            case Dimension::Type::Signed8:
                {
                    ostr << (int)(getFieldInternal<int8_t>(d, idx));
                    break;
                }
            case Dimension::Type::Signed16:
                {
                    ostr << getFieldInternal<int16_t>(d, idx);
                    break;
                }
            case Dimension::Type::Signed32:
                {
                    ostr << getFieldInternal<int32_t>(d, idx);
                    break;
                }
            case Dimension::Type::Signed64:
                {
                    ostr << getFieldInternal<int64_t>(d, idx);
                    break;
                }
            case Dimension::Type::Unsigned8:
                {
                    ostr << (unsigned)(getFieldInternal<uint8_t>(d, idx));
                    break;
                }
            case Dimension::Type::Unsigned16:
                {
                    ostr << getFieldInternal<uint16_t>(d, idx);
                    break;
                }
            case Dimension::Type::Unsigned32:
                {
                    ostr << getFieldInternal<uint32_t>(d, idx);
                    break;
                }
            case Dimension::Type::Unsigned64:
                {
                    ostr << getFieldInternal<uint64_t>(d, idx);
                    break;
                }
            case Dimension::Type::Float:
                {
                    ostr << getFieldInternal<float>(d, idx);
                    break;
                }
            case Dimension::Type::Double:
                {
                    ostr << getFieldInternal<double>(d, idx);
                    break;
                }
            case Dimension::Type::None:
                ostr << "NONE";
                break;
            }
            ostr << endl;
        }
    }
}


std::ostream& operator<<(std::ostream& ostr, const PointView& buf)
{
    buf.dump(ostr);
    return ostr;
}

} // namespace pdal
