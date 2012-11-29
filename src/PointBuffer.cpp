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

#include <boost/uuid/uuid_io.hpp>


namespace pdal
{


PointBuffer::PointBuffer(const Schema& schema, boost::uint32_t capacity)
    : m_schema(schema)
    , m_data(new boost::uint8_t[schema.getByteSize() * capacity])
    , m_numPoints(0)
    , m_capacity(capacity)
    , m_bounds(Bounds<double>::getDefaultSpatialExtent())
    , m_byteSize(schema.getByteSize())
    , m_metadata("pointbuffer")
{

    return;
}

PointBuffer::PointBuffer(PointBuffer const& other)
    : m_schema(other.getSchema())
    , m_data(new boost::uint8_t[other.getSchema().getByteSize() * other.m_capacity])
    , m_numPoints(other.m_numPoints)
    , m_capacity(other.m_capacity)
    , m_bounds(other.m_bounds)
    , m_byteSize(other.m_byteSize)
    , m_metadata(other.m_metadata)
{
    if (other.m_data)
    {
        memcpy(m_data.get(), other.m_data.get(), other.getSchema().getByteSize()*m_capacity);
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
        boost::scoped_array<boost::uint8_t> data(new boost::uint8_t[ m_schema.getByteSize()*m_capacity ]());
        m_data.swap(data);
        m_byteSize = rhs.m_byteSize;
        if (rhs.m_data.get())
            memcpy(m_data.get(), rhs.m_data.get(), m_byteSize*m_capacity);

        m_metadata = rhs.m_metadata;
    }
    return *this;
}

void PointBuffer::reset(Schema const& new_schema)
{
    boost::uint32_t old_size = m_schema.getByteSize();
    boost::uint32_t new_size = m_schema.getByteSize();
    
    m_schema = new_schema;
    
    if (new_size != old_size)
    {
        boost::scoped_array<boost::uint8_t> data(new boost::uint8_t[ m_schema.getByteSize()*m_capacity ]());
        m_data.swap(data);
    }
    m_numPoints = 0;
    m_byteSize = new_schema.getByteSize();    
}

void PointBuffer::resize(boost::uint32_t const& capacity)
{
    if (capacity != m_capacity)
    {
        m_capacity = capacity;
        boost::scoped_array<boost::uint8_t> data(new boost::uint8_t[ m_schema.getByteSize()*m_capacity ]());
        m_data.swap(data);        
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


void PointBuffer::setData(boost::uint8_t* data, std::size_t pointIndex)
{
    memcpy(m_data.get() + m_byteSize * pointIndex, data, m_byteSize);
}

void PointBuffer::setDataStride(boost::uint8_t* data,
                                std::size_t pointIndex,
                                boost::uint32_t byteCount)
{
    memcpy(m_data.get() + m_byteSize * pointIndex, data, byteCount);
}


void PointBuffer::getData(boost::uint8_t** data, std::size_t* array_size) const
{
    *array_size = m_byteSize;
    *data = (boost::uint8_t*) malloc(*array_size);
    memcpy(*data, m_data.get(), *array_size);
}


pdal::Bounds<double> PointBuffer::calculateBounds(bool is3d) const
{
    pdal::Schema const& schema = getSchema();

    pdal::Bounds<double> output;

    boost::optional<Dimension const&> dimX = schema.getDimension("X");
    boost::optional<Dimension const&> dimY = schema.getDimension("Y");
    boost::optional<Dimension const&> dimZ = schema.getDimension("Z");


    bool first = true;
    for (boost::uint32_t pointIndex=0; pointIndex<getNumPoints(); pointIndex++)
    {
        boost::int32_t xi = getField<boost::int32_t>(*dimX, pointIndex);
        boost::int32_t yi = getField<boost::int32_t>(*dimY, pointIndex);
        boost::int32_t zi = getField<boost::int32_t>(*dimZ, pointIndex);

        double xd = dimX->applyScaling(xi);
        double yd = dimY->applyScaling(yi);
        
        if (is3d)
        {
            double zd = dimZ->applyScaling(zi);
            Vector<double> v(xd, yd, zd);
            if (first)
            {
                output = pdal::Bounds<double>(xd, yd, zd, xd, yd, zd);
                first = false;
            }
            output.grow(v);
        } else 
        {
            Vector<double> v(xd, yd);
            if (first)
            {
                output = pdal::Bounds<double>(xd, yd, xd, yd);
                first = false;
            }
            output.grow(v);
        }
    }

    return output;

}

//
// void PointBuffer::addMetadata(Metadata const& m)
// {
//     pointbuffer::index_by_name& index = m_metadata.get<pointbuffer::name>();
//
//     std::pair<pointbuffer::index_by_name::iterator, bool> q = index.insert(m);
//     if (!q.second)
//     {
//         std::ostringstream oss;
//         oss << "Could not insert into schema index because of " << q.first->getName();
//         throw metadata_error(oss.str());
//     }
//
//     return;
// }
//
//
// Metadata const& PointBuffer::getMetadata(std::string const& t, std::string const& ns) const
// {
//     pointbuffer::index_by_name const& name_index = m_metadata.get<pointbuffer::name>();
//     pointbuffer::index_by_name::const_iterator it = name_index.find(t);
//
//     pointbuffer::index_by_name::size_type count = name_index.count(t);
//
//     std::ostringstream oss;
//     oss << "Metadata with name '" << t << "' not found, unable to PointBuffer::getMetadata";
//
//     if (it != name_index.end()) {
//
//         if (ns.size())
//         {
//             while (it != name_index.end())
//             {
//                 if (boost::equals(ns, it->getNamespace()))
//                     return *it;
//                 ++it;
//             }
//
//         }
//
//         if (count > 1) {
//
//             std::pair<pointbuffer::index_by_name::const_iterator, pointbuffer::index_by_name::const_iterator> ret = name_index.equal_range(t);
//             boost::uint32_t num_parents(0);
//             boost::uint32_t num_children(0);
//             std::map<metadata::id, metadata::id> relationships;
//
//             // Test to make sure that the number of parent dimensions all with
//             // the same name is equal to only 1. If there are multiple
//             // dimensions with the same name, but no relationships defined,
//             // we are in an error condition
//             for (pointbuffer::index_by_name::const_iterator  o = ret.first; o != ret.second; ++o)
//             {
//                 // Put a map together that maps parents to children that
//                 // we are going to walk to find the very last child in the
//                 // graph.
//                 std::pair<metadata::id, metadata::id> p( o->getParent(), o->getUUID());
//                 relationships.insert(p);
//
//                 // The parent dimension should have a nil parent of its own.
//                 // nil_uuid is the default parent of all dimensions as the y
//                 // are created
//                 if (o->getParent().is_nil())
//                 {
//                     num_parents++;
//                 }
//                 else
//                 {
//                     num_children++;
//                 }
//
//             }
//
//             if (num_parents != 1)
//             {
//                 std::ostringstream oss;
//
//                 oss << "PointBuffer has multiple dimensions with name '" << t << "', but "
//                        "their parent/child relationships are not coherent. Multiple "
//                        "parents are present.";
//
//                 throw multiple_parent_metadata(oss.str());
//             }
//
//             metadata::id parent = boost::uuids::nil_uuid();
//
//             // Starting at the parent (nil uuid), walk the child/parent graph down to the
//             // end.  When we're done finding dimensions, what's left is the child
//             // at the end of the graph.
//             std::map<metadata::id, metadata::id>::const_iterator p = relationships.find(parent);
//             pdal::metadata::id child;
//             while (p != relationships.end())
//             {
//                 child = p->second;
//                 p = relationships.find(p->second);
//             }
//             pointbuffer::index_by_uid::const_iterator pi = m_metadata.get<pointbuffer::uid>().find(child);
//             if (pi != m_metadata.get<pointbuffer::uid>().end())
//             {
//                 return *pi;
//             }
//             else
//             {
//                 std::ostringstream errmsg;
//                 errmsg << "Unable to fetch subjugate metadata entry with id '" << child << "' in PointBuffer";
//                 throw metadata_not_found(errmsg.str());
//             }
//         }
//         return *it;
//     } else {
//         boost::uuids::uuid ps1;
//         try
//         {
//             boost::uuids::string_generator gen;
//             ps1 = gen(t);
//         } catch (std::runtime_error&)
//         {
//             // invalid string for uuid
//             throw metadata_not_found(oss.str());
//         }
//
//         pointbuffer::index_by_uid::const_iterator i = m_metadata.get<pointbuffer::uid>().find(ps1);
//
//         if (i != m_metadata.get<pointbuffer::uid>().end())
//         {
//             if (ns.size())
//             {
//                 while (i != m_metadata.get<pointbuffer::uid>().end())
//                 {
//                     if (boost::equals(ns, i->getNamespace()))
//                         return *i;
//                     ++i;
//                 }
//
//             }
//
//             return *i;
//         } else
//         {
//             oss.str("");
//             oss << "Metadata with name '" << t << "' not found, unable to PointBuffer::getMetadata";
//             throw metadata_not_found(oss.str());
//         }
//
//     }
//
// }
//
// Metadata const& PointBuffer::getMetadata(std::size_t t) const
// {
//     pointbuffer::index_by_index const& idx = m_metadata.get<pointbuffer::index>();
//
//     if (t >= idx.size())
//         throw dimension_not_found("Index position is not valid");
//
//     return idx.at(t);
// }
//
// boost::optional<Metadata const&> PointBuffer::getMetadataOptional(std::size_t t) const
// {
//     try
//     {
//         Metadata const& m = getMetadata(t);
//         return boost::optional<Metadata const&>(m);
//     } catch (pdal::dimension_not_found&)
//     {
//         return boost::optional<Metadata const&>();
//     }
// }
//
// Metadata const& PointBuffer::getMetadata(metadata::id const& t) const
// {
//     pointbuffer::index_by_uid::const_iterator it = m_metadata.get<pointbuffer::uid>().find(t);
//
//     if (it != m_metadata.get<pointbuffer::uid>().end())
//     {
//         return *it;
//     }
//
//     std::ostringstream oss;
//     oss << "getMetadata: metadata entry not found with uuid '" << boost::lexical_cast<std::string>(t) << "'";
//     throw metadata_not_found(oss.str());
//
// }
//
// boost::optional<Metadata const&> PointBuffer::getMetadataOptional(metadata::id const& t) const
// {
//     try
//     {
//         Metadata const& m = getMetadata(t);
//         return boost::optional<Metadata const&>(m);
//     } catch (pdal::metadata_not_found&)
//     {
//         return boost::optional<Metadata const&>();
//     }
// }
//
//
// boost::optional<Metadata const&> PointBuffer::getMetadataOptional(std::string const& t, std::string const& ns) const
// {
//
//     try
//     {
//         Metadata const& m = getMetadata(t, ns);
//         return boost::optional<Metadata const&>(m);
//     } catch (pdal::metadata_not_found&)
//     {
//         return boost::optional<Metadata const&>();
//     }
//
// }
//
// bool PointBuffer::setMetadata(Metadata const& m)
// {
//     pointbuffer::index_by_name& name_index = m_metadata.get<pointbuffer::name>();
//     pointbuffer::index_by_name::iterator it = name_index.find(m.getName());
//
//     // FIXME: If there are two metadata with the same name here, we're
//     // screwed if they both have the same namespace too
//     if (it != name_index.end()) {
//         while (it != name_index.end())
//         {
//             if (boost::equals(m.getNamespace(), it->getNamespace()))
//             {
//                 name_index.replace(it, m);
//                 return true;
//             }
//             ++it;
//         }
//     } else {
//         std::ostringstream oss;
//         oss << "Metadata with name '" << m.getName() << "' not found, unable to PointBuffer::setMetadata";
//         throw metadata_not_found(oss.str());
//     }
//
//     return true;
// }

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
                case dimension::SignedByte:
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
                case dimension::UnsignedByte:
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
