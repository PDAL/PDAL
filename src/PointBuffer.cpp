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
#include <algorithm>

namespace pdal
{


PointBuffer::PointBuffer(const Schema& schema, boost::uint32_t capacity)
    : m_schema(schema)
    , m_data_size(static_cast<pointbuffer::PointBufferByteSize>(schema.getByteSize()) * static_cast<pointbuffer::PointBufferByteSize>(capacity))
    , m_data(new boost::uint8_t[static_cast<pointbuffer::PointBufferByteSize>(schema.getByteSize()) * static_cast<pointbuffer::PointBufferByteSize>(capacity)])
    , m_numPoints(0)
    , m_capacity(capacity)
    , m_bounds(Bounds<double>::getDefaultSpatialExtent())
    , m_byteSize(schema.getByteSize())
    , m_orientation(schema.getOrientation())
    , m_metadata("pointbuffer")


{
    pointbuffer::PointBufferByteSize size = static_cast<pointbuffer::PointBufferByteSize>(schema.getByteSize()) * static_cast<pointbuffer::PointBufferByteSize>(capacity);

    GlobalEnvironment& env = pdal::GlobalEnvironment::get();
    boost::uuids::basic_random_generator<boost::mt19937> gen(env.getRNG());
    m_uuid = gen();
}

PointBuffer::PointBuffer(PointBuffer const& other)
    : m_schema(other.getSchema())
    , m_data_size(other.m_data_size)
    , m_data(new boost::uint8_t[other.m_data_size])
    , m_numPoints(other.m_numPoints)
    , m_capacity(other.m_capacity)
    , m_bounds(other.m_bounds)
    , m_byteSize(other.m_byteSize)
    , m_orientation(other.m_orientation)
    , m_metadata(other.m_metadata)
{
    
    std::copy(other.m_data.get(), other.m_data.get() + other.m_data_size, m_data.get());

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
        m_orientation = rhs.m_orientation;
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
    m_orientation = new_schema.getOrientation();

    if (m_byteSize != old_size)
    {
        pointbuffer::PointBufferByteSize new_array_size = static_cast<pointbuffer::PointBufferByteSize>(new_size) * static_cast<pointbuffer::PointBufferByteSize>(m_capacity);
        if (new_array_size > m_data_size)
        {
            boost::shared_array<boost::uint8_t> f(new boost::uint8_t[new_array_size]);
            m_data.swap(f);
            m_data_size = new_array_size;
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
        if (new_array_size > m_data_size || bExact)
        {
            boost::shared_array<boost::uint8_t> f(new boost::uint8_t[new_array_size]);
            m_data.swap(f);
            m_data_size = new_array_size;
            
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
    boost::uint64_t position = static_cast<pointbuffer::PointBufferByteSize>(m_byteSize) * static_cast<pointbuffer::PointBufferByteSize>(pointIndex);
    memcpy(m_data.get() + position, data, m_byteSize);
}

void PointBuffer::setDataStride(boost::uint8_t* data,
                                boost::uint32_t pointIndex,
                                boost::uint32_t byteCount)
{
    pointbuffer::PointBufferByteSize position(0);
    if (m_orientation == schema::POINT_INTERLEAVED)
        position = static_cast<pointbuffer::PointBufferByteSize>(m_byteSize) * static_cast<pointbuffer::PointBufferByteSize>(pointIndex);
    else if (m_orientation == schema::DIMENSION_INTERLEAVED)
        position = static_cast<pointbuffer::PointBufferByteSize>(pointIndex) * static_cast<pointbuffer::PointBufferByteSize>(m_capacity);
    
    assert ((byteCount - position) <= m_data_size);
    memcpy(m_data.get() + position, data, byteCount);
}


void PointBuffer::getData(boost::uint8_t** data, boost::uint64_t* array_size) const
{
    *array_size = m_byteSize;
    *data = (boost::uint8_t*) malloc(static_cast<size_t>(*array_size));
    memcpy(*data, m_data.get(), static_cast<size_t>(*array_size));
}

PointBuffer* PointBuffer::pack(bool bRemoveIgnoredDimensions) const
{

    // Creates a new buffer that has the ignored dimensions removed from
    // it.

    pdal::Schema const& schema = getSchema();
    schema::index_by_index const& idx = schema.getDimensions().get<schema::index>();
    pdal::Schema output_schema(schema);
    if (bRemoveIgnoredDimensions)
        output_schema = schema.pack();
    pdal::PointBuffer* output = new PointBuffer(output_schema, getNumPoints());
    
    PointBuffer::pack(this, output, true /* we need to pack out ignored dims*/, false /*no need to reset schema, already done */);
    return output;
}


void PointBuffer::pack( PointBuffer const* input, 
                        PointBuffer* output, 
                        bool bRemoveIgnoredDimensions,
                        bool bResetOutputSchema)
{

    // Creates a new buffer that has the ignored dimensions removed from
    // it.

    schema::index_by_index const& idx = input->getSchema().getDimensions().get<schema::index>();
    
    if (bResetOutputSchema)
    {
        pdal::Schema output_schema(input->getSchema());
        output_schema = input->getSchema().pack();
        output->reset(output_schema);
    }
    output->resize(input->getNumPoints(), true);

    boost::uint8_t* src = input->getData(0);
    
    boost::uint8_t* current_position = output->getData(0);
    
    schema::Orientation orientation = input->getSchema().getOrientation();
    if (orientation == schema::POINT_INTERLEAVED)
    {
        for (boost::uint32_t i = 0; i < input->getNumPoints(); ++i)
        {
            boost::uint8_t* data = input->getData(i);
            for (boost::uint32_t d = 0; d < idx.size(); ++d)
            {
                if (bRemoveIgnoredDimensions) 
                {
                    if (! idx[d].isIgnored())
                    {
                        memcpy(current_position, data, idx[d].getByteSize());
                        current_position = current_position+idx[d].getByteSize();                        
                    }
                } 
                else 
                {
                    memcpy(current_position, data, idx[d].getByteSize());
                    current_position = current_position+idx[d].getByteSize();                    
                }
                // if (! idx[d].isIgnored())
                // {
                //     memcpy(current_position, data, idx[d].getByteSize());
                //     current_position = current_position+idx[d].getByteSize();
                // }
                data = data + idx[d].getByteSize();
            }
        }
    }
    else if (orientation == schema::DIMENSION_INTERLEAVED)
    {
        for (boost::uint32_t d = 0; d < input->getSchema().size(); ++d)
        {
            // For each dimension, copy the data if it isn't ignored
            boost::uint8_t* data = input->getData(d);
            boost::uint64_t dimension_length = static_cast<boost::uint64_t>(idx[d].getByteSize()) * static_cast<boost::uint64_t>(input->getNumPoints());
            if (bRemoveIgnoredDimensions) 
            {
                if (! idx[d].isIgnored())
                {
                    memcpy(current_position, data, dimension_length);
                    current_position = current_position+dimension_length;
                }
            } 
            else 
            {
                memcpy(current_position, data, dimension_length);
                current_position = current_position+dimension_length;
            }
            data = data + dimension_length;
        }        
    }
    
    output->setNumPoints(input->getNumPoints());
}

PointBuffer* PointBuffer::flipOrientation() const
{

    // Creates a new buffer that has the ignored dimensions removed from
    // it.

    pdal::Schema schema = getSchema();
    schema::Orientation orientation = getSchema().getOrientation();
    if (orientation == schema::POINT_INTERLEAVED)
        schema.setOrientation(schema::DIMENSION_INTERLEAVED);
    else if (orientation == schema::DIMENSION_INTERLEAVED)
        schema.setOrientation(schema::POINT_INTERLEAVED);
    else
        throw pdal_error("schema orientation is not recognized for PointBuffer::flipOrientation!");
    
    schema::index_by_index const& idx = schema.getDimensions().get<schema::index>();

    pdal::PointBuffer* output = new PointBuffer(schema, getCapacity());

    if (orientation == schema::POINT_INTERLEAVED)
    {
        for (boost::uint32_t d = 0; d < idx.size(); ++d)
        {
            boost::uint8_t* write_position = output->getData(d);
            schema::size_type dimSize(idx[d].getByteSize());
            std::size_t offset(idx[d].getByteOffset());
            
            for (boost::uint32_t i = 0; i < getNumPoints(); ++i)
            {
                boost::uint8_t* point_start = getData(i);
                boost::uint8_t* read_position = point_start + offset;
                std::copy(read_position, read_position + dimSize, write_position);

                // memcpy(write_position, read_position, dimSize);
                write_position = write_position + dimSize;
            }
        }
    }
    else if (orientation == schema::DIMENSION_INTERLEAVED)
    {
        for (boost::uint32_t d = 0; d < idx.size(); ++d)
        {
            
            schema::size_type dimSize(idx[d].getByteSize());
            std::size_t offset(idx[d].getByteOffset());

            boost::uint8_t* read_start = getData(d);
                        
            for (boost::uint32_t i = 0; i < getNumPoints(); ++i)
            {
                boost::uint8_t* write_position = output->getData(i)+offset;
                boost::uint8_t* read_position = read_start + i*dimSize;
                std::copy(read_position, read_position + dimSize, write_position);
            }
        }   
    }
    
    output->setNumPoints(getNumPoints());
    return output;
    
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

            std::string output = printDimension(dimension, pointIndex);
            
            tree.add(key, output);
        }
    }
    return tree;
}

std::string PointBuffer::printDimension(Dimension const& dimension, boost::uint32_t index) const
{

    boost::uint32_t const& size = dimension.getByteSize();


    std::string output;

    double scale = dimension.getNumericScale();
    double offset = dimension.getNumericOffset();

    bool applyScaling(false);
    if (!Utils::compare_distance(scale, 1.0) ||
            !Utils::compare_distance(offset, 0.0)
       )
    {
        applyScaling = true;
    }


    switch (dimension.getInterpretation())
    {

#define GETFIELDAS(T) getField<T>(dimension, index)
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
                const boost::uint8_t* data  = getData(index) + dimension.getByteOffset();
                std::vector<boost::uint8_t> bytes;
                for (int i=0; i < dimension.getByteSize(); ++i)
                {
                    bytes.push_back(data[i]);
                }
                output += Utils::binary_to_hex_string(bytes);
                break;
            }

        default:
            output = std::string("unknown dimension data type");
    }
    return output;
    
}


std::ostream& PointBuffer::toRST(std::ostream& os) const
{
    const Schema& schema = getSchema();
    schema::index_by_index const& dimensions = schema.getDimensions().get<schema::index>();

    boost::uint32_t ns_column(32);    
    boost::uint32_t name_column(20);
    boost::uint32_t value_column(40);
    
    std::ostringstream hdr;
    for (int i = 0; i < 80; ++i)
        hdr << "-";

    for (std::size_t i=0; i< dimensions.size(); i++)
    {
        name_column = std::max(static_cast<std::size_t>(name_column), dimensions[i].getName().size());
        ns_column = std::max(static_cast<std::size_t>(name_column), dimensions[i].getNamespace().size());
    }
    
    std::ostringstream thdr;
    for (unsigned i = 0; i < name_column-1; ++i)
        thdr << "=";
    thdr << " ";
    for (unsigned i = 0; i < value_column-1; ++i)
        thdr << "=";        
    thdr << " ";
    for (unsigned i = 0; i < ns_column-1; ++i)
        thdr << "=";    
    thdr << " ";

    name_column--;
    unsigned step_back(3);

    for (boost::uint32_t pointIndex=0; pointIndex<getNumPoints(); pointIndex++)
    {
        os << "Point " << pointIndex << std::endl;
        os << hdr.str() << std::endl << std::endl;
        os << thdr.str() << std::endl;
        os << std::setw(name_column-step_back) << "Name" << std::setw(value_column-step_back) << "Value"  << std::setw(ns_column-step_back) << "Namespace" << std::endl;
        os << thdr.str() << std::endl;        
        for (unsigned i=0; i< dimensions.size(); i++)
        {
            const Dimension& dimension = dimensions[i];
            std::string value = printDimension(dimension, pointIndex);
            std::string name = dimension.getName();
            std::string ns = dimension.getNamespace();
            os   << std::left << std::setw(name_column) << name << std::right << std::setw(value_column) << value << std::setw(ns_column) << ns  << std::endl;
        }
        os << thdr.str() << std::endl << std::endl;
    }
    os << std::endl << std::endl;;
        
    return os;
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



inline void copyOver(boost::uint8_t *destination_position,
                     const boost::uint8_t *source_position,
                     boost::uint32_t source_bytesize) {
    if (source_bytesize == 1)
    {
        destination_position[0] = source_position[0];
    } else if (source_bytesize == 2)
    {
        destination_position[0] = source_position[0];
        destination_position[1] = source_position[1];
    } else if (source_bytesize == 4)
    {
        destination_position[0] = source_position[0];
        destination_position[1] = source_position[1];
        destination_position[2] = source_position[2];
        destination_position[3] = source_position[3];
    } else if (source_bytesize == 8)
    {
        destination_position[0] = source_position[0];
        destination_position[1] = source_position[1];
        destination_position[2] = source_position[2];
        destination_position[3] = source_position[3];
        destination_position[4] = source_position[4];
        destination_position[5] = source_position[5];
        destination_position[6] = source_position[6];
        destination_position[7] = source_position[7];
    } else
    {
        std::copy(source_position, source_position + source_bytesize, destination_position);
    }
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

    pdal::schema::Orientation source_orientation = source.getSchema().getOrientation();
    pdal::schema::Orientation destination_orientation = destination.getSchema().getOrientation();

    pointbuffer::PointBufferByteSize source_capacity = static_cast<pointbuffer::PointBufferByteSize>(source.getCapacity());
    pointbuffer::PointBufferByteSize destination_capacity = static_cast<pointbuffer::PointBufferByteSize>(destination.getCapacity());

    pointbuffer::PointBufferByteSize numCopyMapEntries = static_cast<pointbuffer::PointBufferByteSize>(dimensions.m.size());
    
    pointbuffer::PointBufferByteSize source_point_size = static_cast<pointbuffer::PointBufferByteSize>(source.getSchema().getByteSize());
    pointbuffer::PointBufferByteSize dest_point_size = static_cast<pointbuffer::PointBufferByteSize>(destination.getSchema().getByteSize());

    // setup fast paths
    if (source_orientation == schema::POINT_INTERLEAVED &&
            destination_orientation == schema::POINT_INTERLEAVED) {

        boost::uint8_t *source_ptr = source.getData(source_starting_position);
        boost::uint8_t *dst_ptr = destination.getData(destination_starting_position);


        boost::uint8_t* source_position = source_ptr;
        boost::uint8_t* destination_position = dst_ptr;

        // copy point by point
        //
        for (boost::uint32_t i = 0; i < howMany ; ++i)
        {
            for(pointbuffer::PointBufferByteSize iE = 0 ; iE < numCopyMapEntries ; ++iE) {
                boost::uint64_t const& encoded = dimensions.offsets[iE];

                boost::uint64_t source_byteoffset = (encoded >> 32);
                boost::uint64_t dest_byteoffset = (encoded >> 16) & 0xFFFF;
                boost::uint64_t source_bytesize = (encoded & 0xFFFF);

                copyOver(dst_ptr + dest_byteoffset,
                        source_ptr + source_byteoffset, source_bytesize);
            }

            source_ptr += source_point_size;
            dst_ptr += dest_point_size;
        }
    }
    else if (source_orientation == schema::DIMENSION_INTERLEAVED &&
            destination_orientation == schema::DIMENSION_INTERLEAVED) {
        boost::uint8_t *source_ptr = source.getData(0);
        boost::uint8_t *dst_ptr = destination.getData(0);

        // copy dimension by dimension
        // 
        for(pointbuffer::PointBufferByteSize iE = 0 ; iE < numCopyMapEntries ; ++iE) {
            boost::uint64_t const& encoded = dimensions.offsets[iE];

            boost::uint64_t source_byteoffset = (encoded >> 32);
            boost::uint64_t dest_byteoffset = (encoded >> 16) & 0xFFFF;
            boost::uint64_t source_bytesize = (encoded & 0xFFFF);

            boost::uint8_t* source_position = source_ptr +  source_capacity * source_byteoffset + 
                source_starting_position * source_bytesize;
            boost::uint8_t* destination_position = dst_ptr + destination_capacity * dest_byteoffset +
                destination_starting_position * source_bytesize;

            memcpy(destination_position, source_position, howMany * source_bytesize);

            //std::copy(destination_position, destination_position + howMany * source_bytesize, source_position);
        }
    }
    else if (source_orientation == schema::POINT_INTERLEAVED &&
            destination_orientation == schema::DIMENSION_INTERLEAVED) {
        // slowt case #1 when one of the dimensions doesn't match the other
        //
        boost::uint8_t *src_ptr = source.getData(source_starting_position);
        boost::uint8_t *dst_ptr = destination.getData(0);


        for(pointbuffer::PointBufferByteSize iE = 0 ; iE < numCopyMapEntries ; ++iE) {
            boost::uint64_t const& encoded = dimensions.offsets[iE];

            boost::uint64_t source_byteoffset = (encoded >> 32);
            boost::uint64_t dest_byteoffset = (encoded >> 16) & 0xFFFF;
            boost::uint64_t source_bytesize = (encoded & 0xFFFF);

            boost::uint8_t *this_src_ptr = src_ptr + source_byteoffset;
            boost::uint8_t *this_dst_ptr = dst_ptr + destination_capacity * dest_byteoffset +
                destination_starting_position * source_bytesize;

            for (boost::uint32_t i = 0; i < howMany; ++i) {
                copyOver(this_dst_ptr, this_src_ptr, source_bytesize);

                this_src_ptr += source_point_size;
                this_dst_ptr += source_bytesize;
            }
        }
    }
    else if (source_orientation == schema::DIMENSION_INTERLEAVED &&
            destination_orientation == schema::POINT_INTERLEAVED) {
        boost::uint8_t *src_ptr = source.getData(0);
        boost::uint8_t *dst_ptr = destination.getData(destination_starting_position);


        for(pointbuffer::PointBufferByteSize iE = 0 ; iE < numCopyMapEntries ; ++iE) {
            boost::uint64_t const& encoded = dimensions.offsets[iE];

            boost::uint64_t source_byteoffset = (encoded >> 32);
            boost::uint64_t dest_byteoffset = (encoded >> 16) & 0xFFFF;
            boost::uint64_t source_bytesize = (encoded & 0xFFFF);

            boost::uint8_t *this_src_ptr = src_ptr + source_capacity * source_byteoffset + source_starting_position * source_bytesize;
            boost::uint8_t *this_dst_ptr = dst_ptr + dest_byteoffset;

            for (boost::uint32_t i = 0; i < howMany; ++i) {
                copyOver(this_dst_ptr, this_src_ptr, source_bytesize);

                this_src_ptr += source_bytesize;
                this_dst_ptr += dest_point_size;
            }
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
