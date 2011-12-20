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

#include <pdal/PointBuffer.hpp>

#include <boost/lexical_cast.hpp>


namespace pdal
{


PointBuffer::PointBuffer(const Schema& schema, boost::uint32_t capacity)
    : m_schema(schema)
    , m_data(new boost::uint8_t[m_schema.getByteSize() * capacity])
    , m_numPoints(0)
    , m_capacity(capacity)
    , m_bounds(Bounds<double>::getDefaultSpatialExtent())
    , m_byteSize(schema.getByteSize())
{

    return;
}

PointBuffer::PointBuffer(PointBuffer const& other) 
    : m_schema(other.getSchema())
    , m_data(new boost::uint8_t[m_schema.getByteSize() * other.m_capacity])
    , m_numPoints(other.m_numPoints)
    , m_capacity(other.m_capacity)
    , m_bounds(other.m_bounds)
    , m_byteSize(other.m_byteSize)
{
    if (other.m_data)
    {
        memcpy(m_data.get(), other.m_data.get(), m_schema.getByteSize()*m_capacity);
    }

}

PointBuffer& PointBuffer::operator=(PointBuffer const& rhs)
{
    if (&rhs != this)
    {
        m_schema = rhs.getSchema();
        m_numPoints = rhs.getNumPoints();
        m_capacity = rhs.getCapacity();
        m_bounds = rhs.getSpatialBounds();
        boost::scoped_array<boost::uint8_t> data( new boost::uint8_t[ m_schema.getByteSize()*m_capacity ] );
        m_data.swap(data);
        m_byteSize = rhs.m_byteSize;
        
        if (rhs.m_data.get())
            memcpy(m_data.get(), rhs.m_data.get(), m_byteSize*m_capacity);
        
        
    }
    return *this;
}

PointBuffer::~PointBuffer()
{
}


const Bounds<double>& PointBuffer::getSpatialBounds() const
{
    return m_bounds;
}


void PointBuffer::setSpatialBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


void PointBuffer::setData(boost::uint8_t* data, std::size_t index)
{
    memcpy(m_data.get() + m_byteSize * index, data, m_byteSize);
}

void PointBuffer::setAllData(boost::uint8_t* data, boost::uint32_t byteCount)
{
    memcpy(m_data.get(), data, byteCount);
}

void PointBuffer::setDataStride(boost::uint8_t* data, std::size_t index, boost::uint32_t byteCount)
{
    memcpy(m_data.get() + m_byteSize * index, data, byteCount);
}

boost::uint32_t PointBuffer::getNumPoints() const
{
    return m_numPoints;
}


void PointBuffer::getData(boost::uint8_t** data, std::size_t* array_size) const
{
    *array_size = m_byteSize;
    *data = (boost::uint8_t*) malloc (*array_size);
    memcpy(*data, m_data.get(), *array_size);
}


boost::property_tree::ptree PointBuffer::toPTree() const
{
    boost::property_tree::ptree tree;

    const Schema& schema = getSchema();
    schema::index_by_index const& dimensions = schema.getDimensions().get<schema::index>();

    const boost::uint32_t numPoints = getNumPoints();

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        const std::string pointstring = boost::lexical_cast<std::string>(pointIndex) + ".";
        
        boost::uint32_t i = 0;
        for (i=0; i<dimensions.size(); i++)
        {
            const Dimension& dimension = dimensions[i];
            boost::uint32_t const& size = dimension.getByteSize();
            
            const std::string key = pointstring + dimension.getName();
            
            std::string output = "";

            double scale = dimension.getNumericScale();
            double offset = dimension.getNumericOffset();
            
            bool applyScaling(false);
            if (!Utils::compare_distance(scale, 0.0) ||
                !Utils::compare_distance(offset, 0.0)
                )
                applyScaling = true;

            
            switch (dimension.getInterpretation())
            {
                    
#define GETFIELDAS(T) getField<T>(dimension, pointIndex)
#define STRINGIFY(x) boost::lexical_cast<std::string>(x)
                // note we convert 8-bit fields to ints, so they aren't treated as chars
            case dimension::SignedInteger:
            case dimension::SignedByte:
                if (size == 1)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::int8_t));
                    else
                    {
                        boost::int8_t v = GETFIELDAS(boost::int8_t);
                        double d = dimension.applyScaling<boost::int8_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                if (size == 2)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::int16_t));
                    else
                    {
                        boost::int16_t v = GETFIELDAS(boost::int16_t);
                        double d = dimension.applyScaling<boost::int16_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                if (size == 4)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::int32_t));
                    else
                    {
                        boost::int32_t v = GETFIELDAS(boost::int32_t);
                        double d = dimension.applyScaling<boost::int32_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                if (size == 8)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::int64_t));
                    else
                    {
                        boost::int64_t v = GETFIELDAS(boost::int64_t);
                        double d = dimension.applyScaling<boost::int64_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                break;
            case dimension::UnsignedInteger:
            case dimension::UnsignedByte:
                if (size == 1)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::uint8_t));
                    else
                    {
                        boost::uint8_t v = GETFIELDAS(boost::uint8_t);
                        double d = dimension.applyScaling<boost::uint8_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                if (size == 2)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::uint16_t));
                    else
                    {
                        boost::uint16_t v = GETFIELDAS(boost::uint16_t);
                        double d = dimension.applyScaling<boost::uint16_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                if (size == 4)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::uint32_t));
                    else
                    {
                        boost::uint32_t v = GETFIELDAS(boost::uint32_t);
                        double d = dimension.applyScaling<boost::uint32_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                if (size == 8)
                {
                    if (!applyScaling)
                        output += STRINGIFY(GETFIELDAS(boost::uint64_t));
                    else
                    {
                        boost::uint64_t v = GETFIELDAS(boost::uint64_t);
                        double d = dimension.applyScaling<boost::uint64_t>(v);
                        output += STRINGIFY(d);
                    }
                }
                break;


            case dimension::Float:
                if (dimension.getByteSize() == 4)
                    output += STRINGIFY(GETFIELDAS(float));
                else
                    output += STRINGIFY(GETFIELDAS(double));
                break;
            
            default:
                throw pdal_error("unknown dimension data type");
            }

            tree.add(key, output);
        }
    }
    return tree;
}


std::ostream& operator<<(std::ostream& ostr, const PointBuffer& pointBuffer)
{
    using std::endl;

    const Schema& schema = pointBuffer.getSchema();
    schema::index_by_index const& dimensions = schema.getDimensions().get<schema::index>();

    const std::size_t numPoints = pointBuffer.getNumPoints();

    ostr << "Contains " << numPoints << "  points" << endl;

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
       
        ostr << "Point: " << pointIndex << endl;

        boost::uint32_t i = 0;
        for (i=0; i<dimensions.size(); i++)
        {
            const Dimension& dimension = dimensions[i];

            ostr << dimension.getName() << " (" << dimension.getInterpretationName() << ") : ";

            switch (dimension.getInterpretation())
            {
            case dimension::SignedInteger:
            case dimension::SignedByte:
                if (dimension.getByteSize() == 1)
                    ostr << (int)(pointBuffer.getField<boost::int8_t>(dimension, pointIndex));
                if (dimension.getByteSize() == 2)
                    ostr << pointBuffer.getField<boost::int16_t>(dimension, pointIndex);
                if (dimension.getByteSize() == 4)
                    ostr << pointBuffer.getField<boost::int32_t>(dimension, pointIndex);
                if (dimension.getByteSize() == 8)
                    ostr << pointBuffer.getField<boost::int64_t>(dimension, pointIndex);
                break;
            case dimension::UnsignedInteger:
            case dimension::UnsignedByte:
                if (dimension.getByteSize() == 1)
                    ostr << (unsigned int)(pointBuffer.getField<boost::uint8_t>(dimension, pointIndex));
                if (dimension.getByteSize() == 2)
                    ostr << pointBuffer.getField<boost::uint16_t>(dimension, pointIndex);
                if (dimension.getByteSize() == 4)
                    ostr << pointBuffer.getField<boost::uint32_t>(dimension, pointIndex);
                if (dimension.getByteSize() == 8)
                    ostr << pointBuffer.getField<boost::uint64_t>(dimension, pointIndex);
                break;


            case dimension::Float:
                if (dimension.getByteSize() == 4)
                    ostr << pointBuffer.getField<float>(dimension, pointIndex);
                if (dimension.getByteSize() == 8)
                    ostr << pointBuffer.getField<double>(dimension, pointIndex);
                break;
            case dimension::Pointer:
                ostr << "pointer";
                break;
            default:
                throw;
            }

            ostr << endl;
        }
    }

    return ostr;
}


} // namespace pdal
