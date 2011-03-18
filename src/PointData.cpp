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

#include <libpc/PointData.hpp>

#include <cassert>
#include <iostream>
#include <numeric>

using std::string;

namespace libpc
{


PointData::PointData(const SchemaLayout& schemaLayout, boost::uint32_t capacity)
    : m_schemaLayout(schemaLayout)
    , m_data(NULL)
    , m_pointSize(m_schemaLayout.getByteSize())
    , m_numPoints(0)
    , m_capacity(capacity)
    , m_bounds(Bounds<double>::getDefaultSpatialExtent())
{
    m_data = new boost::uint8_t[m_pointSize * m_capacity];

    return;
}

PointData::PointData(PointData const& other) 
    : m_schemaLayout(other.getSchemaLayout())
    , m_data(0)
    , m_pointSize(m_schemaLayout.getByteSize())
    , m_numPoints(other.m_numPoints)
    , m_capacity(other.m_capacity)
    , m_bounds(other.m_bounds)
{
    m_data = new boost::uint8_t[m_pointSize * m_capacity];
    
    memcpy(m_data, other.m_data, m_pointSize*m_capacity);

}

PointData& PointData::operator=(PointData const& rhs)
{
    if (&rhs != this)
    {
        m_schemaLayout = rhs.getSchemaLayout();
        m_data = new boost::uint8_t[m_schemaLayout.getByteSize() * rhs.getCapacity()];
        m_pointSize = m_schemaLayout.getByteSize();
        m_numPoints = rhs.getNumPoints();
        m_capacity = rhs.getCapacity();
        m_bounds = rhs.getSpatialBounds();
    }
    return *this;
}

PointData::~PointData()
{
    if (m_data)
        delete[] m_data;
}


const Bounds<double>& PointData::getSpatialBounds() const
{
    return m_bounds;
}


void PointData::setSpatialBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


boost::uint8_t* PointData::getData(std::size_t index) const
{
    return m_data + m_pointSize * index;
}


boost::uint32_t PointData::getNumPoints() const
{
    return m_numPoints;
}


void PointData::copyPointFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointData& srcPointData)
{
    assert(getSchemaLayout() == srcPointData.getSchemaLayout());

    boost::uint8_t* src = srcPointData.getData(srcPointIndex);
    boost::uint8_t* dest = getData(destPointIndex);
    std::size_t len = getSchemaLayout().getByteSize();

    memcpy(dest, src, len);

    assert(m_numPoints <= m_capacity);

    return;
}


void PointData::copyPointsFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointData& srcPointData, std::size_t numPoints)
{
    assert(getSchemaLayout() == srcPointData.getSchemaLayout());

    boost::uint8_t* src = srcPointData.getData(srcPointIndex);
    boost::uint8_t* dest = getData(destPointIndex);
    std::size_t len = getSchemaLayout().getByteSize();

    memcpy(dest, src, len * numPoints);
    
    assert(m_numPoints <= m_capacity);

    return;
}


std::ostream& operator<<(std::ostream& ostr, const PointData& pointData)
{
    using std::endl;

    const SchemaLayout& schemaLayout = pointData.getSchemaLayout();
    const std::vector<DimensionLayout>& dimensionLayouts = schemaLayout.getDimensionLayouts();
    const std::size_t numPoints = pointData.getNumPoints();

    int cnt = 0;
    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {

        ++cnt;
    }
    ostr << "Contains " << cnt << "  points (" << pointData.getNumPoints() << " total)" << endl;

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {

        
        ostr << "Point: " << pointIndex << endl;

        for (SchemaLayout::DimensionLayoutsCIter citer=dimensionLayouts.begin(); citer != dimensionLayouts.end(); ++citer)
        {
            const DimensionLayout& dimensionLayout = *citer;
            const Dimension& dimension = dimensionLayout.getDimension();
            std::size_t fieldIndex = dimensionLayout.getPosition();

            ostr << dimension.getFieldName() << " (" << dimension.getDataTypeName(dimension.getDataType()) << ") : ";

            switch (dimension.getDataType())
            {
            case Dimension::Int8:
                ostr << (int)(pointData.getField<boost::int8_t>(pointIndex, fieldIndex));
                break;
            case Dimension::Uint8:
                ostr << (int)(pointData.getField<boost::uint8_t>(pointIndex, fieldIndex));
                break;
            case Dimension::Int16:
                ostr << pointData.getField<boost::int16_t>(pointIndex, fieldIndex);
                break;
            case Dimension::Uint16:
                ostr << pointData.getField<boost::uint16_t>(pointIndex, fieldIndex);
                break;
            case Dimension::Int32:
                ostr << pointData.getField<boost::int32_t>(pointIndex, fieldIndex);
                break;
            case Dimension::Uint32:
                ostr << pointData.getField<boost::uint32_t>(pointIndex, fieldIndex);
                break;
            case Dimension::Int64:
                ostr << pointData.getField<boost::int64_t>(pointIndex, fieldIndex);
                break;
            case Dimension::Uint64:
                ostr << pointData.getField<boost::uint64_t>(pointIndex, fieldIndex);
                break;
            case Dimension::Float:
                ostr << pointData.getField<float>(pointIndex, fieldIndex);
                break;
            case Dimension::Double:
                ostr << pointData.getField<double>(pointIndex, fieldIndex);
                break;
            default:
                throw;
            }

            ostr << endl;
        }
    }

    return ostr;
}


} // namespace libpc
