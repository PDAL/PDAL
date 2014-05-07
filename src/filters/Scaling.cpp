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

#include <pdal/filters/Scaling.hpp>

#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>

namespace pdal
{
namespace filters
{


void Scaling::initialize()
{
    checkImpedance();
    //ABELL - buildSchema().
    m_schema = alterSchema(m_schema);
}


void Scaling::checkImpedance()
{
    std::vector<Option> dimensions = m_options.getOptions("dimension");

    for (Option& option : dimensions)
    {
        boost::optional<Options const&> dimOps = option.getOptions();

        scaling::Scaler scaler;
        scaler.name = option.getValue<std::string>();
        if (dimOps)
        {
            Dimension const& dim = getSchema().getDimension(scaler.name);
            scaler.scale = dimOps->getValueOrDefault<double>("scale",
                dim.getNumericScale());
            scaler.offset = dimOps->getValueOrDefault<double>("offset",
                dim.getNumericOffset());
            scaler.type = dimOps->getValueOrDefault<std::string>( "type",
                "SignedInteger");
            scaler.size = dimOps->getValueOrDefault<boost::uint32_t>("size", 4);
        }
        m_scalers.push_back(scaler);
    }
}


pdal::StageSequentialIterator*
Scaling::createSequentialIterator(PointBuffer& buffer) const
{
    return new iterators::sequential::Scaling(*this, buffer);
}

pdal::StageRandomIterator*
Scaling::createRandomIterator(PointBuffer& buffer) const
{
    return new iterators::random::Scaling(*this, buffer);
}


Options Scaling::getDefaultOptions()
{
    return Options();
}


Schema Scaling::alterSchema(Schema const& input_schema)
{
    typedef std::multimap<dimension::Interpretation,
        dimension::Interpretation>::const_iterator MapIterator;

    typedef std::pair<MapIterator, MapIterator> MapPair;
    std::vector<scaling::Scaler> const& scalers = getScalers();

    std::vector<scaling::Scaler>::const_iterator i;

    // Only certain scaling conversions are allowed. If we're not one of the
    // defined conversions, we're going to complain.

    Schema schema(input_schema);

    std::vector<std::pair<dimension::Interpretation, dimension::Interpretation>>
        conversions;

    conversions.push_back(
        std::make_pair(dimension::Float, dimension::Float));
    conversions.push_back(
        std::make_pair(dimension::Float, dimension::SignedInteger));
    conversions.push_back(
        std::make_pair(dimension::UnsignedInteger, dimension::Float));
    conversions.push_back(std::make_pair(
        dimension::UnsignedInteger, dimension::UnsignedInteger));
    conversions.push_back(
        std::make_pair(dimension::SignedInteger, dimension::Float));
    conversions.push_back(
        std::make_pair(dimension::SignedInteger, dimension::SignedInteger));

    // Loop through the options that the filter.Scaler collected. For each
    // dimension described, create a new dimension with the given parameters.
    // Create a map with the uuid of the new dimension that maps to the old
    // dimension to be scaled

    for (scaling::Scaler s : scalers)
    {
        boost::optional<Dimension const&> from_dimension =
            schema.getDimensionOptional(s.name);
        if (from_dimension)
        {
            Dimension to_dimension(s.name, getInterpretation(s.type), s.size,
                from_dimension->getDescription());
            to_dimension.setNumericScale(s.scale);
            to_dimension.setNumericOffset(s.offset);
            to_dimension.createUUID();
            to_dimension.setNamespace(getName());
            to_dimension.setParent(from_dimension->getUUID());

            auto target = std::make_pair(from_dimension->getInterpretation(),
                to_dimension.getInterpretation());
            if (std::find(conversions.begin(), conversions.end(), 
                target) == conversions.end())
            {
                throw pdal_error("Scaling between types not supported");
            }

            log()->get(logDEBUG2)  <<
                "Rescaling dimension " << from_dimension->getName() << " [" <<
                from_dimension->getInterpretation() << "/" <<
                from_dimension->getByteSize() << "]" << " from scale: " <<
                from_dimension->getNumericScale() << " offset: " <<
                from_dimension->getNumericOffset() << " to scale: " <<
                to_dimension.getNumericScale() << " offset: " <<
                to_dimension.getNumericOffset() << " datatype: " <<
                to_dimension.getInterpretation() << "/" <<
                to_dimension.getByteSize() << std::endl;

            m_scale_map.insert(std::make_pair(from_dimension->getUUID(),
                to_dimension.getUUID()));
            log()->get(logDEBUG2) << "scaling dimension with id '" <<
                from_dimension->getUUID() << "' to dimension with id: '" <<
                to_dimension.getUUID() <<"'" << std::endl;
            schema.appendDimension(to_dimension);
        }
    }

    bool markIgnored =
        getOptions().getValueOrDefault<bool>("ignore_old_dimensions", true);
    for (auto it = m_scale_map.begin(); it != m_scale_map.end(); ++it)
    {
        Dimension const& from_dimension = schema.getDimension(it->first);
        Dimension const& to_dimension = schema.getDimension(it->second);

        if (markIgnored)
        {
            log()->get(logDEBUG2) << "marking " << from_dimension.getName() <<
                " as ignored with uuid "  <<  from_dimension.getUUID() <<
                std::endl;

            Dimension d(from_dimension);
            boost::uint32_t flags = d.getFlags();
            d.setFlags(flags | dimension::IsIgnored);
            schema.setDimension(d);
        }

        log()->get(logDEBUG2) << "Map wants to do: " <<
            from_dimension.getName() << " [" <<
            from_dimension.getInterpretation() << "/" <<
            from_dimension.getByteSize() << "]" << " to scale: " <<
            to_dimension.getNumericScale() << " offset: " <<
            to_dimension.getNumericOffset() << " datatype: " <<
            to_dimension.getInterpretation() << "/" <<
            to_dimension.getByteSize() << std::endl;
    }
    return schema;
}

dimension::Interpretation Scaling::getInterpretation(std::string t) const
{
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "signedinteger")
        return dimension::SignedInteger;
    if (t == "unsignedinteger")
        return dimension::UnsignedInteger;
    if (t == "signedbyte")
        return dimension::RawByte;
    if (t == "unsignedbyte")
        return dimension::RawByte;
    if (t == "rawbyte")
        return dimension::RawByte;
    if (t == "float")
        return dimension::Float;
    if (t == "pointer")
        return dimension::Pointer;
    return dimension::Undefined;
}

namespace iterators
{

namespace sequential
{


Scaling::Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , scaling::IteratorBase(filter, buffer)
{} 


boost::uint64_t Scaling::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Scaling::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

boost::uint32_t Scaling::readBufferImpl(PointBuffer& buffer)
{
    const boost::uint32_t numRead = getPrevIterator().read(buffer);
    IteratorBase::scaleData(buffer, numRead);
    return numRead;
}


} // sequential

namespace random
{

Scaling::Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer)
    : pdal::FilterRandomIterator(filter, buffer)
    , scaling::IteratorBase(filter, buffer)
{}

boost::uint64_t Scaling::seekImpl(boost::uint64_t count)
{

    return getPrevIterator().seek(count);
}

boost::uint32_t Scaling::readBufferImpl(PointBuffer& buffer)
{
    const boost::uint32_t numRead = getPrevIterator().read(buffer);
    IteratorBase::scaleData(buffer, numRead);
    return numRead;
}


} // random

namespace scaling
{

IteratorBase::IteratorBase(const pdal::filters::Scaling& filter, PointBuffer&)
    : m_scalingFilter(filter)
{
    return;
}

void IteratorBase::readBufferBeginImpl(PointBuffer& buffer)
{
    pdal::Schema const& schema = buffer.getSchema();
    std::map<dimension::id, dimension::id>::const_iterator d;
    std::map<dimension::id, dimension::id> const& scale_map =
        m_scalingFilter.getScaleMap();
    for (d = scale_map.begin(); d != scale_map.end(); ++d)
    {
        boost::optional<pdal::Dimension const&> fr =
            schema.getDimensionOptional(d->first);
        boost::optional<pdal::Dimension const&> to =
            schema.getDimensionOptional(d->second);

        if (!fr)
            throw pdal_error("from dimension is not found on schema!");
        if (!to)
            throw pdal_error("to dimension is not found on schema!");

        //ABELL - We've checked they exist, seems we could dispense with the
        //  optional bit.
        std::pair<boost::optional<pdal::Dimension const&>,
            boost::optional<pdal::Dimension const&>> g(fr, to);
        m_dimension_map.insert(g);
    }
}

void IteratorBase::scaleData(PointBuffer& buffer, boost::uint32_t numRead)
{
    for (boost::uint32_t pointIndex = 0; pointIndex < numRead; pointIndex++)
    {
        for (auto i = m_dimension_map.begin(); i != m_dimension_map.end(); ++i)
        {
            boost::optional<pdal::Dimension const&> f = i->first;
            boost::optional<pdal::Dimension const&> t = i->second;
            //ABELL - Didn't we already check this?
            if (f && t)
            {
                Dimension const& from_dimension = *f;
                Dimension const& to_dimension = *t;
                writeScaledData(buffer, from_dimension, to_dimension,
                    pointIndex);
            }
        }
    }
}


#ifdef PDAL_COMPILER_MSVC
// the writeScaledData function causes lots of conversion warnings which
// are real but which we will ignore: blame the filter's user for casting badly
#  pragma warning(push)
#  pragma warning(disable: 4244)  // conversion from T1 to T2, possible loss of data
#endif

void IteratorBase::writeScaledData(PointBuffer& buffer,
    Dimension const& from, Dimension const& to, boost::uint32_t pointIndex)
{
    if (from.getInterpretation() == dimension::Float)
    {
        if (from.getByteSize() == 4)
        {
            float v = buffer.getField<float>(from, pointIndex);
            scale(from, to, v);
            if (to.getInterpretation() == dimension::SignedInteger)
            {
                if (to.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to, pointIndex, v);
                else if (to.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to, pointIndex, v);
                else if (to.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to, pointIndex, v);
                else if (to.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension::Float to "
                        "dimension::SignedInteger of size >8");
                return;
            }
        }
        else if (from.getByteSize() == 8)
        {
            double v = buffer.getField<double>(from, pointIndex);
            scale(from, to, v);
            if (to.getInterpretation() == dimension::SignedInteger)
            {
                if (to.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to, pointIndex, v);
                else if (to.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to, pointIndex, v);
                else if (to.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to, pointIndex, v);
                else if (to.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension::Float to "
                        "dimension::SignedInteger of size >8");
                return;
            }
        }
        else
        {
            std::ostringstream oss;
            oss << "Unable to interpret Float of size '" <<
                from.getByteSize() <<"'";
            throw pdal_error(oss.str());
        }
    }

    if (from.getInterpretation() == dimension::SignedInteger)
    {
        if (to.getInterpretation() == dimension::SignedInteger)
        {
            if (from.getByteSize() == 1)
            {
                boost::int8_t v = buffer.getField<boost::int8_t>(from,
                    pointIndex);
                scale(from, to, v);
                if (to.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to, pointIndex, v);
                else if (to.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to, pointIndex, v);
                else if (to.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to, pointIndex, v);
                else if (to.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 1 to SignedInteger of size > 8");
                return;
            }
            else if (from.getByteSize() == 2)
            {
                boost::int16_t v = buffer.getField<boost::int16_t>(from,
                    pointIndex);
                scale(from, to, v);
                if (to.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to, pointIndex, v);
                else if (to.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to, pointIndex, v);
                else if (to.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to, pointIndex, v);
                else if (to.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 2 to SignedInteger of size > 8");
                return;
            }
            else if (from.getByteSize() == 4)
            {
                boost::int32_t v = buffer.getField<boost::int32_t>(from,
                    pointIndex);
                scale(from, to, v);
                if (to.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to, pointIndex, v);
                else if (to.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to, pointIndex, v);
                else if (to.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to, pointIndex, v);
                else if (to.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 4 to SignedInteger of size > 8");
                return;
            }
            else if (from.getByteSize() == 8)
            {
                boost::int64_t v = buffer.getField<boost::int64_t>(from,
                    pointIndex);
                scale(from, to, v);
                if (to.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to, pointIndex, v);
                else if (to.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to, pointIndex, v);
                else if (to.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to, pointIndex, v);
                else if (to.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 8 to SignedInteger of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::SignedInteger "
                    ">8 to dimension::SignedInteger of size >8");
            }

        }
        else if (to.getInterpretation() == dimension::Float)
        {
            if (from.getByteSize() == 1)
            {
                boost::int8_t v = buffer.getField<boost::int8_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 1 to Float of size > 8");
                return;
            }
            else if (from.getByteSize() == 2)
            {
                boost::int16_t v = buffer.getField<boost::int16_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 2 to Float of size > 8");
                return;
            }
            else if (from.getByteSize() == 4)
            {
                boost::int32_t v = buffer.getField<boost::int32_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 4 to Float of size > 8");
                return;
            }
            else if (from.getByteSize() == 8)
            {
                boost::int64_t v = buffer.getField<boost::int64_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "SignedInteger 8 to Float of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::SignedInteger "
                    ">8 to dimension::Float of size >8");
            }
        }
    }

    if (from.getInterpretation() == dimension::UnsignedInteger)
    {

        if (to.getInterpretation() == dimension::UnsignedInteger)
        {
            if (from.getByteSize() == 1)
            {
                boost::uint8_t v = buffer.getField<boost::uint8_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);

                if (to.getByteSize() == 1)
                {
                    boost::uint8_t i = to.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 2)
                {
                    boost::uint16_t i = to.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 4)
                {
                    boost::uint32_t i = to.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 8)
                {
                    boost::uint64_t i = to.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 8 to UnsignedInteger of size > 8");
                return;
            }
            else if (from.getByteSize() == 2)
            {
                boost::uint16_t v = buffer.getField<boost::uint16_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);

                if (to.getByteSize() == 1)
                {
                    boost::uint8_t i = to.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 2)
                {
                    boost::uint16_t i = to.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 4)
                {
                    boost::uint32_t i = to.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 8)
                {
                    boost::uint64_t i = to.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 2 to UnsignedInteger of size > 8");
                return;
            }
            else if (from.getByteSize() == 4)
            {
                boost::uint32_t v = buffer.getField<boost::uint32_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);

                if (to.getByteSize() == 1)
                {
                    boost::uint8_t i = to.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 2)
                {
                    boost::uint16_t i = to.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 4)
                {
                    boost::uint32_t i = to.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 8)
                {
                    boost::uint64_t i = to.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 4 to UnsignedInteger of size > 8");
                return;
            }
            else if (from.getByteSize() == 8)
            {
                boost::uint64_t v = buffer.getField<boost::uint64_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);

                if (to.getByteSize() == 1)
                {
                    boost::uint8_t i = to.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 2)
                {
                    boost::uint16_t i = to.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 4)
                {
                    boost::uint32_t i = to.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to, pointIndex, i);
                }
                else if (to.getByteSize() == 8)
                {
                    boost::uint64_t i = to.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 8 to UnsignedInteger of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::UnsignedInteger "
                    ">8 to dimension::UnsignedInteger of size >8");
            }
        }
        else if (to.getInterpretation() == dimension::Float)
        {
            if (from.getByteSize() == 1)
            {
                boost::uint8_t v = buffer.getField<boost::uint8_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 1 to Float of size > 8");
                return;
            }
            else if (from.getByteSize() == 2)
            {
                boost::uint16_t v = buffer.getField<boost::uint16_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 2 to Float of size > 8");
                return;
            }
            else if (from.getByteSize() == 4)
            {
                boost::uint32_t v = buffer.getField<boost::uint32_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 4 to Float of size > 8");
                return;
            }
            else if (from.getByteSize() == 8)
            {
                boost::uint64_t v = buffer.getField<boost::uint64_t>(from,
                    pointIndex);
                double d = from.applyScaling<double>(v);
                d = to.removeScaling<double>(d);
                if (to.getByteSize() == 4)
                    buffer.setField<float>(to, pointIndex, (float)d);
                else if (to.getByteSize() == 8)
                    buffer.setField<double>(to, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from "
                        "UnsignedInteger 8 to Float of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::UnsignedInteger "
                    ">8 to dimension::Float of size >8");
            }

        }
    }
    throw pdal_error("Dimension data type unable to be scaled because it "
        "is Undefined");
}

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif

} // scaling

} // iterators


}
} // namespaces
