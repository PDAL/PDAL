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
#include <pdal/GlobalEnvironment.hpp>

#include <boost/lexical_cast.hpp>

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>

namespace pdal
{


PointBuffer::PointBuffer(const Schema& schema, boost::uint32_t capacity)
    : m_schema(schema)
    , m_numPoints(0)
    , m_capacity(capacity)
    , m_bounds(Bounds<double>::getDefaultSpatialExtent())
    , m_byteSize(schema.getByteSize())
    , m_metadata("pointbuffer")
    , m_segment(0)

{
    pointbuffer::PointBufferByteSize size = static_cast<pointbuffer::PointBufferByteSize>(schema.getByteSize()) * static_cast<pointbuffer::PointBufferByteSize>(capacity);

    m_data.reserve(size);
    m_data.resize(size);

    GlobalEnvironment& env = pdal::GlobalEnvironment::get();
    boost::uuids::basic_random_generator<boost::mt19937> gen(env.getRNG());
    m_uuid = gen();
    
    // boost::interprocess::shared_memory_object::remove("mySegmentObjectVector");
    // m_segment = new boost::interprocess::managed_shared_memory(boost::interprocess::create_only, "mySegmentObjectVector", size);    
    return;
}

PointBuffer::PointBuffer(PointBuffer const& other)
    : m_schema(other.getSchema())
    , m_data(other.m_data)
    , m_numPoints(other.m_numPoints)
    , m_capacity(other.m_capacity)
    , m_bounds(other.m_bounds)
    , m_byteSize(other.m_byteSize)
    , m_metadata(other.m_metadata)
{


}

PointBuffer::~PointBuffer()
{
    // delete m_segment;
    
}

PointBuffer& PointBuffer::operator=(PointBuffer const& rhs)
{
    if (&rhs != this)
    {
        m_schema = rhs.getSchema();
        m_numPoints = rhs.getNumPoints();
        m_capacity = rhs.getCapacity();
        m_bounds = rhs.getSpatialBounds();
        m_byteSize = rhs.m_byteSize;
        m_data = rhs.m_data;
        m_metadata = rhs.m_metadata;
    }
    return *this;
}

void PointBuffer::reset(Schema const& new_schema)
{
    boost::uint32_t old_size = m_schema.getByteSize();
    boost::uint32_t new_size = new_schema.getByteSize();

    m_schema = new_schema;
    m_byteSize = new_size;

    if (m_byteSize != old_size)
    {
        pointbuffer::PointBufferByteSize new_array_size = static_cast<pointbuffer::PointBufferByteSize>(new_size) * static_cast<pointbuffer::PointBufferByteSize>(m_capacity);
        if (new_array_size > m_data.size())
        {
            m_data.resize(new_array_size);
        }
    }

    m_numPoints = 0;

}

void PointBuffer::resize(boost::uint32_t const& capacity, bool bExact)
{
    if (capacity != m_capacity)
    {
        m_capacity = capacity;
        pointbuffer::PointBufferByteSize new_array_size = static_cast<pointbuffer::PointBufferByteSize>(m_schema.getByteSize()) * static_cast<pointbuffer::PointBufferByteSize>(m_capacity);
        if (new_array_size > m_data.size() || bExact)
        {
            m_data.resize(new_array_size);
        }

    }
}

const Bounds<double>& PointBuffer::getSpatialBounds() const
{
    return m_bounds;
}


void PointBuffer::setSpatialBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


void PointBuffer::setData(boost::uint8_t* data, boost::uint32_t pointIndex)
{
    boost::uint64_t position = static_cast<boost::uint64_t>(m_byteSize) * static_cast<boost::uint64_t>(pointIndex);
    memcpy(&(m_data.front()) + position, data, m_byteSize);
}

void PointBuffer::setDataStride(boost::uint8_t* data,
                                boost::uint32_t pointIndex,
                                boost::uint32_t byteCount)
{
    boost::uint64_t position = static_cast<boost::uint64_t>(m_byteSize) * static_cast<boost::uint64_t>(pointIndex);
    memcpy(&(m_data.front()) + position, data, byteCount);
}


void PointBuffer::getData(boost::uint8_t** data, boost::uint64_t* array_size) const
{
    *array_size = m_byteSize;
    *data = (boost::uint8_t*) malloc(static_cast<size_t>(*array_size));
    memcpy(*data, &(m_data.front()), static_cast<size_t>(*array_size));
}


pdal::Bounds<double> PointBuffer::calculateBounds(bool is3d) const
{
    pdal::Schema const& schema = getSchema();

    pdal::Bounds<double> output;

    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");

    Vector<double> v;

    bool first = true;
    for (boost::uint32_t pointIndex=0; pointIndex<getNumPoints(); pointIndex++)
    {
        boost::int32_t xi = getField<boost::int32_t>(dimX, pointIndex);
        boost::int32_t yi = getField<boost::int32_t>(dimY, pointIndex);
        boost::int32_t zi = getField<boost::int32_t>(dimZ, pointIndex);

        double xd = dimX.applyScaling(xi);
        double yd = dimY.applyScaling(yi);

        if (is3d)
        {
            double zd = dimZ.applyScaling(zi);
            if (first)
            {
                output = pdal::Bounds<double>(xd, yd, zd, xd, yd, zd);
                first = false;
                v.add(xd);
                v.add(yd);
                v.add(zd);
            }
            v[0] = xd;
            v[1] = yd;
            v[2] = zd;
            output.grow(v);

        }
        else
        {

            if (first)
            {
                output = pdal::Bounds<double>(xd, yd, xd, yd);
                first = false;
                v.add(xd);
                v.add(yd);
            }
            v[0] = xd;
            v[1] = yd;
            output.grow(v);
        }
    }

    return output;

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
            {
                applyScaling = true;
            }


            switch (dimension.getInterpretation())
            {

#define GETFIELDAS(T) getField<T>(dimension, pointIndex)
#define STRINGIFY(T,x) boost::lexical_cast<std::string>(boost::numeric_cast<T>(x))
                    // note we convert 8-bit fields to ints, so they aren't treated as chars
                case dimension::SignedInteger:
                    if (size == 1)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(boost::int32_t, GETFIELDAS(boost::int8_t));
                        else
                        {
                            boost::int8_t v = GETFIELDAS(boost::int8_t);
                            double d = dimension.applyScaling<boost::int8_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    if (size == 2)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(boost::int16_t, GETFIELDAS(boost::int16_t));
                        else
                        {
                            boost::int16_t v = GETFIELDAS(boost::int16_t);
                            double d = dimension.applyScaling<boost::int16_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    if (size == 4)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(boost::int32_t, GETFIELDAS(boost::int32_t));
                        else
                        {
                            boost::int32_t v = GETFIELDAS(boost::int32_t);
                            double d = dimension.applyScaling<boost::int32_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    if (size == 8)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(boost::int64_t, GETFIELDAS(boost::int64_t));
                        else
                        {
                            boost::int64_t v = GETFIELDAS(boost::int64_t);
                            double d = dimension.applyScaling<boost::int64_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    break;
                case dimension::UnsignedInteger:
                    if (size == 1)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(boost::uint32_t, GETFIELDAS(boost::uint8_t));
                        else
                        {
                            boost::uint8_t v = GETFIELDAS(boost::uint8_t);
                            double d = dimension.applyScaling<boost::uint8_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    if (size == 2)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(boost::uint16_t, GETFIELDAS(boost::uint16_t));
                        else
                        {
                            boost::uint16_t v = GETFIELDAS(boost::uint16_t);
                            double d = dimension.applyScaling<boost::uint16_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    if (size == 4)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(boost::uint32_t, GETFIELDAS(boost::uint32_t));
                        else
                        {
                            boost::uint32_t v = GETFIELDAS(boost::uint32_t);
                            double d = dimension.applyScaling<boost::uint32_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    if (size == 8)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(double, GETFIELDAS(boost::uint64_t));
                        else
                        {
                            boost::uint64_t v = GETFIELDAS(boost::uint64_t);
                            double d = dimension.applyScaling<boost::uint64_t>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    break;


                case dimension::Float:
                    if (size == 4)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(float, GETFIELDAS(float));
                        else
                        {
                            float v = GETFIELDAS(float);
                            double d = dimension.applyScaling<float>(v);
                            output += STRINGIFY(float, d);
                        }
                    }
                    else if (size == 8)
                    {
                        if (!applyScaling)
                            output += STRINGIFY(double, GETFIELDAS(double));
                        else
                        {
                            double v = GETFIELDAS(double);
                            double d = dimension.applyScaling<double>(v);
                            output += STRINGIFY(double, d);
                        }
                    }
                    else
                    {
                        output += STRINGIFY(double, GETFIELDAS(double));
                    }
                    break;

                case dimension::RawByte:
                    {
                        const boost::uint8_t* data  = getData(pointIndex) + dimension.getByteOffset();
                        std::vector<boost::uint8_t> bytes;
                        for (int i=0; i < dimension.getByteSize(); ++i)
                        {
                            bytes.push_back(data[i]);
                        }
                        output += Utils::binary_to_hex_string(bytes);
                        break;
                    }

                default:
                    throw pdal_error("unknown dimension data type");
            }

            tree.add(key, output);
        }
    }
    return tree;
}



double PointBuffer::applyScaling(Dimension const& d,
                                 std::size_t pointIndex) const
{
    double output(0.0);

    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);

    boost::uint32_t size = d.getByteSize();
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = getField<float>(d, pointIndex);
                output = static_cast<double>(flt);
            }
            if (size == 8)
            {
                output = getField<double>(d, pointIndex);
            }
            break;

        case dimension::SignedInteger:
            if (size == 1)
            {
                i8 = getField<boost::int8_t>(d, pointIndex);
                output = d.applyScaling<boost::int8_t>(i8);
            }
            if (size == 2)
            {
                i16 = getField<boost::int16_t>(d, pointIndex);
                output = d.applyScaling<boost::int16_t>(i16);
            }
            if (size == 4)
            {
                i32 = getField<boost::int32_t>(d, pointIndex);
                output = d.applyScaling<boost::int32_t>(i32);
            }
            if (size == 8)
            {
                i64 = getField<boost::int64_t>(d, pointIndex);
                output = d.applyScaling<boost::int64_t>(i64);
            }
            break;

        case dimension::UnsignedInteger:
            if (size == 1)
            {
                u8 = getField<boost::uint8_t>(d, pointIndex);
                output = d.applyScaling<boost::uint8_t>(u8);
            }
            if (size == 2)
            {
                u16 = getField<boost::uint16_t>(d, pointIndex);
                output = d.applyScaling<boost::uint16_t>(u16);
            }
            if (size == 4)
            {
                u32 = getField<boost::uint32_t>(d, pointIndex);
                output = d.applyScaling<boost::uint32_t>(u32);
            }
            if (size == 8)
            {
                u64 = getField<boost::uint64_t>(d, pointIndex);
                output = d.applyScaling<boost::uint64_t>(u64);
            }
            break;

        case dimension::RawByte:
        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be scaled in index filter");
    }

    return output;
}

void PointBuffer::copyLikeDimensions(PointBuffer const& source,
                                     PointBuffer& destination,
                                     schema::DimensionMap const& dimensions,
                                     boost::uint32_t source_starting_position,
                                     boost::uint32_t destination_starting_position,
                                     boost::uint32_t howMany)
{


    assert(howMany <= destination.getCapacity() - destination_starting_position);
    assert(howMany <= source.getCapacity() - source_starting_position);

    typedef schema::DimensionMap::const_iterator Iterator;

    for (Iterator d = dimensions.begin(); d != dimensions.end(); ++d)
    {

        Dimension const& source_dim = *d->first;
        Dimension const& dest_dim = *d->second;

        for (boost::uint32_t i = 0; i < howMany; ++i)
        {
            boost::uint8_t* source_position = source.getData(source_starting_position+i) + source_dim.getByteOffset();
            boost::uint8_t* destination_position = destination.getData(destination_starting_position + i) + dest_dim.getByteOffset();
            memcpy(destination_position, source_position, dest_dim.getByteSize());
        }
    }


}

std::ostream& operator<<(std::ostream& ostr, const PointBuffer& pointBuffer)
{
    using std::endl;

    const Schema& schema = pointBuffer.getSchema();
    schema::index_by_index const& dimensions = schema.getDimensions().get<schema::index>();

    const boost::uint32_t numPoints = pointBuffer.getNumPoints();

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
                case dimension::RawByte:
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

IndexedPointBuffer::IndexedPointBuffer( const Schema& schema, 
                                        boost::uint32_t capacity)
    : PointBuffer(schema, capacity)
    , m_dimX(0)
    , m_dimY(0)
    , m_dimZ(0)
    , m_is3D(true)
    , m_index(0)
{
    setCoordinateDimensions();
}

IndexedPointBuffer::IndexedPointBuffer(PointBuffer const& other) 
    : PointBuffer(other)
    , m_dimX(0)
    , m_dimY(0)
    , m_dimZ(0)
    , m_is3D(true)
    , m_index(0)
{
    setCoordinateDimensions();
}
IndexedPointBuffer::IndexedPointBuffer(IndexedPointBuffer const& other) 
    : PointBuffer(other)
    , m_dimX(other.m_dimX)
    , m_dimY(other.m_dimY)
    , m_dimZ(other.m_dimZ)
    , m_is3D(other.m_is3D)
    , m_index(0)
{
    setCoordinateDimensions();
    build(m_is3D);
}

void IndexedPointBuffer::setCoordinateDimensions()
{
    m_dimX = m_schema.getDimensionPtr("X");
    m_dimY = m_schema.getDimensionPtr("Y");
    m_dimZ = m_schema.getDimensionPtr("Z");
    if (!m_dimZ)
        m_is3D = false;
}

void IndexedPointBuffer::build(bool b3D)
{
    m_is3D = b3D;
    size_t nDims = m_is3D && m_dimZ ? 3 : 2;
    m_index =  new my_kd_tree_t(nDims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10, nDims ) );
    
    if (!m_dimX)
    {
        throw pdal_error("No 'X' dimension exists to build index with!");
    }
    if (!m_dimY)
    {
        throw pdal_error("No 'Y' dimension exists to build index with!");
    }

    m_index->buildIndex();
}

std::vector<size_t> IndexedPointBuffer::radius( double const& x, 
                                                double const& y, 
                                                double const& z, 
                                                double const& r)
{
    std::vector<size_t>   output;
    std::vector<std::pair<size_t,double> >   ret_matches;
    nanoflann::SearchParams params;
    params.sorted = true;
    
    std::vector<double> pt;
    pt.push_back(x); pt.push_back(y); pt.push_back(z);
    const size_t count = m_index->radiusSearch(&pt[0], r, ret_matches, params);
    
    for (size_t i = 0; i < count; ++i)
    {
        output.push_back(ret_matches[i].first);
    }
    return output;
}

std::vector<size_t> IndexedPointBuffer::neighbors(  double const& x,
                                                    double const& y, 
                                                    double const& z, 
                                                    double distance, 
                                                    boost::uint32_t k)
{


    std::vector<size_t> output(k);
    std::vector<double> out_dist_sqr(k);
    nanoflann::KNNResultSet<double> resultSet(k);
    
    resultSet.init(&output[0], &out_dist_sqr[0] );
    
    std::vector<double> pt;
    pt.push_back(x); pt.push_back(y);
    if (m_is3D) pt.push_back(z);
    m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    return output;
}



IndexedPointBuffer::~IndexedPointBuffer()
{
    delete m_index;
}
} // namespace pdal
