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

PointBuffer::PointBuffer(PointContext context) :
    m_bounds(Bounds<double>::getDefaultSpatialExtent()),
    m_context(context)
{}


const Bounds<double>& PointBuffer::getSpatialBounds() const
{
    return m_bounds;
}


void PointBuffer::setSpatialBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
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
    for (PointId idx = 0; idx < size(); idx++)
    {
        double x = getFieldAs<double>(dimX, idx);
        double y = getFieldAs<double>(dimY, idx);
        double z = getFieldAs<double>(dimZ, idx);

        if (is3d)
        {
            if (first)
            {
                output = pdal::Bounds<double>(x, y, z, x, y, z);
                first = false;
                v.add(x);
                v.add(y);
                v.add(z);
            }
            v[0] = x;
            v[1] = y;
            v[2] = z;
            output.grow(v);
        }
        else
        {
            if (first)
            {
                output = pdal::Bounds<double>(x, y, x, y);
                first = false;
                v.add(x);
                v.add(y);
            }
            v[0] = x;
            v[1] = y;
            output.grow(v);
        }
    }
    return output;
}


boost::property_tree::ptree PointBuffer::toPTree() const
{
    boost::property_tree::ptree tree;

    const Schema& schema = getSchema();
    schema::index_by_index const& dimensions =
        schema.getDimensions().get<schema::index>();

    for (PointId idx = 0; idx < size(); idx++)
    {
        std::string pointstring = boost::lexical_cast<std::string>(idx) + ".";

        for (size_t i = 0; i < dimensions.size(); i++)
        {
            const Dimension& dimension = dimensions[i];
            std::string key = pointstring + dimension.getName();
            double v = getFieldAs<double>(dimension, idx);
            std::string value = boost::lexical_cast<std::string>(v);
            tree.add(key, value);
        }
    }
    return tree;
}


std::ostream& PointBuffer::toRST(std::ostream& os) const
{
    const Schema& schema = getSchema();
    schema::index_by_index const& dimensions =
        schema.getDimensions().get<schema::index>();

    boost::uint32_t ns_column(32);    
    boost::uint32_t name_column(20);
    boost::uint32_t value_column(40);
    
    std::ostringstream hdr;
    for (int i = 0; i < 80; ++i)
        hdr << "-";

    for (std::size_t i=0; i< dimensions.size(); i++)
    {
        name_column = std::max(static_cast<std::size_t>(name_column),
            dimensions[i].getName().size());
        ns_column = std::max(static_cast<std::size_t>(name_column),
            dimensions[i].getNamespace().size());
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

    for (PointId idx = 0; idx < size(); ++idx)
    {
        os << "Point " << idx << std::endl;
        os << hdr.str() << std::endl << std::endl;
        os << thdr.str() << std::endl;
        os << std::setw(name_column-step_back) << "Name" <<
            std::setw(value_column-step_back) << "Value"  <<
            std::setw(ns_column-step_back) << "Namespace" << std::endl;
        os << thdr.str() << std::endl;        
        for (size_t i = 0; i < dimensions.size(); i++)
        {
            const Dimension& dimension = dimensions[i];
            double v = getFieldAs<double>(dimension, idx);
            std::string value = boost::lexical_cast<std::string>(v);
            std::string name = dimension.getName();
            std::string ns = dimension.getNamespace();
            os << std::left << std::setw(name_column) << name <<
                std::right << std::setw(value_column) << value <<
                std::setw(ns_column) << ns  << std::endl;
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
                output = d.applyScaling(i8);
            }
            if (size == 2)
            {
                i16 = getField<boost::int16_t>(d, pointIndex);
                output = d.applyScaling(i16);
            }
            if (size == 4)
            {
                i32 = getField<boost::int32_t>(d, pointIndex);
                output = d.applyScaling(i32);
            }
            if (size == 8)
            {
                i64 = getField<boost::int64_t>(d, pointIndex);
                output = d.applyScaling(i64);
            }
            break;

        case dimension::UnsignedInteger:
            if (size == 1)
            {
                u8 = getField<boost::uint8_t>(d, pointIndex);
                output = d.applyScaling(u8);
            }
            if (size == 2)
            {
                u16 = getField<boost::uint16_t>(d, pointIndex);
                output = d.applyScaling(u16);
            }
            if (size == 4)
            {
                u32 = getField<boost::uint32_t>(d, pointIndex);
                output = d.applyScaling(u32);
            }
            if (size == 8)
            {
                u64 = getField<boost::uint64_t>(d, pointIndex);
                output = d.applyScaling(u64);
            }
            break;

        case dimension::RawByte:
        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be scaled in "
                "index filter");
    }

    return output;
}


std::ostream& operator<<(std::ostream& ostr, const PointBuffer& pointBuffer)
{
    using std::endl;

    const Schema& schema = pointBuffer.getSchema();
    schema::index_by_index const& dimensions =
        schema.getDimensions().get<schema::index>();

    point_count_t numPoints = pointBuffer.size();

    ostr << "Contains " << numPoints << "  points" << endl;
    for (PointId pointIndex = 0; pointIndex < numPoints; pointIndex++)
    {
        ostr << "Point: " << pointIndex << endl;

        boost::uint32_t i = 0;
        for (i=0; i<dimensions.size(); i++)
        {
            const Dimension& dimension = dimensions[i];

            ostr << dimension.getName() << " (" <<
                dimension.getInterpretationName() << ") : ";

            switch (dimension.getInterpretation())
            {
                case dimension::SignedInteger:
                    if (dimension.getByteSize() == 1)
                        ostr << (int)(pointBuffer.getField<boost::int8_t>(
                            dimension, pointIndex));
                    if (dimension.getByteSize() == 2)
                        ostr << pointBuffer.getField<boost::int16_t>(
                            dimension, pointIndex);
                    if (dimension.getByteSize() == 4)
                        ostr << pointBuffer.getField<boost::int32_t>(
                            dimension, pointIndex);
                    if (dimension.getByteSize() == 8)
                        ostr << pointBuffer.getField<boost::int64_t>(
                            dimension, pointIndex);
                    break;
                case dimension::UnsignedInteger:
                case dimension::RawByte:
                    if (dimension.getByteSize() == 1)
                        ostr << (unsigned int)
                            (pointBuffer.getField<boost::uint8_t>(dimension,
                                pointIndex));
                    if (dimension.getByteSize() == 2)
                        ostr << pointBuffer.getField<boost::uint16_t>(
                            dimension, pointIndex);
                    if (dimension.getByteSize() == 4)
                        ostr << pointBuffer.getField<boost::uint32_t>(
                            dimension, pointIndex);
                    if (dimension.getByteSize() == 8)
                        ostr << pointBuffer.getField<boost::uint64_t>(
                            dimension, pointIndex);
                    break;

                case dimension::Float:
                    if (dimension.getByteSize() == 4)
                        ostr << pointBuffer.getField<float>(
                            dimension, pointIndex);
                    if (dimension.getByteSize() == 8)
                        ostr << pointBuffer.getField<double>(
                            dimension, pointIndex);
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

