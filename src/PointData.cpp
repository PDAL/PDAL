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

#include "libpc/PointData.hpp"

#include <cassert>
#include <iostream>

using std::cout;
using std::endl;
using std::string;

namespace libpc
{


PointData::PointData(const PointLayout& layout, boost::uint32_t numPoints) :
    m_layout(layout),
    m_numPoints(numPoints),
    m_data(NULL),
    m_pointSize(0)
{
    m_pointSize = m_layout.getSizeInBytes();
    m_data = new boost::uint8_t[m_pointSize * m_numPoints];

    // the points will all be set to invalid here
    m_isValid.resize(m_numPoints);

    return;
}


bool
PointData::isValid(std::size_t index) const
{
    return m_isValid[index];
}


void
PointData::setValid(std::size_t index, bool value)
{
    m_isValid[index] = value;
}


boost::uint8_t* PointData::getData(std::size_t index) const
{
    return m_data + m_pointSize * index;
}


boost::uint32_t PointData::getNumPoints() const
{
    return m_numPoints;
}


const PointLayout& PointData::getLayout() const
{
    return m_layout;
}


template <class T>
void PointData::setField(std::size_t pointIndex, std::size_t fieldIndex, T value)
{
    std::size_t offset = (pointIndex * m_pointSize) + m_layout.getField(fieldIndex).getOffset();
    assert(offset + sizeof(T) <= m_pointSize * m_numPoints);
    boost::uint8_t* p = m_data + offset;

    *(T*)p = value;
}


template <class T>
T PointData::getField(std::size_t pointIndex, std::size_t fieldIndex) const
{
    std::size_t offset = (pointIndex * m_pointSize) + m_layout.getField(fieldIndex).getOffset();
    assert(offset + sizeof(T) <= m_pointSize * m_numPoints);
    boost::uint8_t* p = m_data + offset;

    return *(T*)p;
}


void PointData::setField_U8(std::size_t pointIndex, std::size_t fieldIndex, boost::uint8_t value)
{
    setField<boost::uint8_t>(pointIndex, fieldIndex, value);
}


void PointData::setField_F32(std::size_t pointIndex, std::size_t fieldIndex, float value)
{
    setField<float>(pointIndex, fieldIndex, value);
}


void PointData::setField_F64(std::size_t pointIndex, std::size_t fieldIndex, double value)
{
    setField<double>(pointIndex, fieldIndex, value);

}


boost::uint8_t PointData::getField_U8(std::size_t pointIndex, std::size_t fieldIndex) const
{
    return getField<boost::uint8_t>(pointIndex, fieldIndex);
}


float PointData::getField_F32(std::size_t pointIndex, std::size_t fieldIndex) const
{
    return getField<float>(pointIndex, fieldIndex);
}


double PointData::getField_F64(std::size_t pointIndex, std::size_t fieldIndex) const
{
    return getField<double>(pointIndex, fieldIndex);
}


void PointData::copyFieldsFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointData& srcPointData)
{
    assert(getLayout() == srcPointData.getLayout());

    boost::uint8_t* src = srcPointData.getData(srcPointIndex);
    boost::uint8_t* dest = getData(destPointIndex);
    std::size_t len = getLayout().getSizeInBytes();

    memcpy(dest, src, len);

    setValid(destPointIndex, srcPointData.isValid(srcPointIndex));

    return;
}


void PointData::dump(string indent) const
{
    int cnt = 0;
    for (boost::uint32_t pointIndex=0; pointIndex<getNumPoints(); pointIndex++)
    {
        if (isValid(pointIndex))
            ++cnt;
    }
    cout << "Contains " << cnt << " valid points (" << m_numPoints << " total)" << endl;

    for (boost::uint32_t pointIndex=0; pointIndex<getNumPoints(); pointIndex++)
    {
        if (isValid(pointIndex))
        {
            cout << "Point: " << pointIndex << endl;
            dump(pointIndex, indent+"  ");
        }
    }

    return;
}


void PointData::dump(std::size_t pointIndex, string indent) const
{
    const PointLayout& layout = getLayout();

    for (boost::uint32_t fieldIndex=0; fieldIndex<layout.getNumFields(); fieldIndex++)
    {
        cout << indent;

        const Field& field = layout.getField(fieldIndex);

        cout << field.getName(field.getItem()) << ": ";

        switch (field.getType())
        {
        case Field::U8:
            cout << (int)(this->getField_U8(pointIndex, fieldIndex));
            break;
        case Field::F32:
            cout << this->getField_F32(pointIndex, fieldIndex);
            break;
        case Field::F64:
            cout << this->getField_F64(pointIndex, fieldIndex);
            break;
        default:
            throw;
        }

        cout << endl;
    }

    return;
}


} // namespace libpc
