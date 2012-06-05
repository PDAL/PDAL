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


// ------------------------------------------------------------------------

Scaling::Scaling(Stage& prevStage, const Options& options)
    : Filter(prevStage, options)
{
    return;
}


void Scaling::initialize()
{
    Filter::initialize();

    checkImpedance();

    return;
}


void Scaling::checkImpedance()
{
    Options& options = getOptions();

    std::vector<Option> dimensions = options.getOptions("dimension");

    std::vector<Option>::const_iterator i;
    for (i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        boost::optional<Options const&> dimensionOptions = i->getOptions();

        scaling::Scaler scaler;
        scaler.name = i->getValue<std::string>();
        if (dimensionOptions)
        {
            scaler.scale = dimensionOptions->getValueOrDefault<double>("scale", 1.0);
            scaler.offset = dimensionOptions->getValueOrDefault<double>("offset", 0.0);
            scaler.type = dimensionOptions->getValueOrDefault<std::string>("type", "SignedInteger");
            scaler.size = dimensionOptions->getValueOrDefault<boost::uint32_t>("size", 4);
        }

        m_scalers.push_back(scaler);

    }
    return;
}


pdal::StageSequentialIterator* Scaling::createSequentialIterator(PointBuffer& buffer) const
{
    log()->get(logDEBUG4) << "Scaling::createSequentialIterator" << std::endl;
    return new pdal::filters::iterators::sequential::Scaling(*this, buffer);
}


const Options Scaling::getDefaultOptions() const
{
    Options options;
    return options;
}


namespace iterators
{
namespace sequential
{


Scaling::Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_scalingFilter(filter)
{
    alterSchema(buffer);
    m_scalingFilter.log()->get(logDEBUG4) << "iterators::sequential alterSchema! " << std::endl;

    return;
}

void Scaling::alterSchema(PointBuffer& buffer)
{
    typedef std::multimap<dimension::Interpretation, dimension::Interpretation>::const_iterator MapIterator;

    typedef std::pair<MapIterator, MapIterator> MapPair;
    std::vector<scaling::Scaler> const& scalers = m_scalingFilter.getScalers();

    std::vector<scaling::Scaler>::const_iterator i;

    // Only certain scaling conversions are allowed. If we're not one of the
    // defined conversions, we're going to complain.

    std::multimap<dimension::Interpretation, dimension::Interpretation> allowed_conversions;
    allowed_conversions.insert(std::make_pair(dimension::Float, dimension::Float));
    allowed_conversions.insert(std::make_pair(dimension::Float, dimension::SignedInteger));
    allowed_conversions.insert(std::make_pair(dimension::UnsignedInteger, dimension::Float));
    allowed_conversions.insert(std::make_pair(dimension::UnsignedInteger, dimension::UnsignedInteger));
    allowed_conversions.insert(std::make_pair(dimension::SignedInteger, dimension::Float));
    allowed_conversions.insert(std::make_pair(dimension::SignedInteger, dimension::SignedInteger));

    Schema schema = buffer.getSchema();

    // Loop through the options that the filter.Scaler collected. For each
    // dimension described, create a new dimension with the given parameters.
    // Create a map with the uuid of the new dimension that maps to the old
    // dimension to be scaled

    for (i = scalers.begin(); i != scalers.end(); ++i)
    {
        boost::optional<Dimension const&> from_dimension = schema.getDimensionOptional(i->name);
        if (from_dimension)
        {
            dimension::Interpretation interp = getInterpretation(i->type);

            Dimension to_dimension(i->name, interp, i->size, from_dimension->getDescription());
            to_dimension.setNumericScale(i->scale);
            to_dimension.setNumericOffset(i->offset);
            to_dimension.createUUID();
            to_dimension.setNamespace(m_scalingFilter.getName());
            to_dimension.setParent(from_dimension->getUUID());

            MapPair f = allowed_conversions.equal_range(from_dimension->getInterpretation());
            if (f.first == allowed_conversions.end())
            {
                throw pdal_error("Scaling between types not supported");
            }

            bool found(false);
            for (MapIterator  o = f.first; o != f.second; ++o)
            {
                if (to_dimension.getInterpretation() == o->second)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
                throw pdal_error("Scaling between types is not supported");

            m_scalingFilter.log()->get(logDEBUG4)  << "Rescaling dimension " << from_dimension->getName()
                                                   << " [" << from_dimension->getInterpretation() << "/" << from_dimension->getByteSize() << "]"
                                                   << " to scale: " << to_dimension.getNumericScale()
                                                   << " offset: " << to_dimension.getNumericOffset()
                                                   << " datatype: " << to_dimension.getInterpretation() << "/" << to_dimension.getByteSize()
                                                   << std::endl;
            std::pair<dimension::id, dimension::id> p(from_dimension->getUUID(), to_dimension.getUUID());
            
            m_scalingFilter.log()->get(logDEBUG4) << "scale map size was " << m_scale_map.size() << std::endl;
            m_scale_map.insert(p);
            m_scalingFilter.log()->get(logDEBUG3) << "scale map size is: " << m_scale_map.size() << std::endl;
            schema.appendDimension(to_dimension);
        }
    }


    std::map<dimension::id, dimension::id>::const_iterator d;
    for (d = m_scale_map.begin(); d != m_scale_map.end(); ++d)
    {
        Dimension const& from_dimension = schema.getDimension(d->first);
        Dimension const& to_dimension = schema.getDimension(d->second);
        m_scalingFilter.log()->get(logDEBUG4) << "Map wants to do: " << from_dimension.getName()
                                              << " [" << from_dimension.getInterpretation() << "/" << from_dimension.getByteSize() << "]"
                                              << " to scale: " << to_dimension.getNumericScale()
                                              << " offset: " << to_dimension.getNumericOffset()
                                              << " datatype: " << to_dimension.getInterpretation() << "/" << to_dimension.getByteSize()
                                              << std::endl;
    }


    buffer = PointBuffer(schema, buffer.getCapacity());
}

dimension::Interpretation Scaling::getInterpretation(std::string const& t) const
{
    if (boost::iequals(t, "SignedInteger"))
    {
        return dimension::SignedInteger;
    }
    if (boost::iequals(t, "UnsignedInteger"))
    {
        return dimension::UnsignedInteger;
    }
    if (boost::iequals(t, "SignedByte"))
    {
        return dimension::SignedByte;
    }
    if (boost::iequals(t, "UnsignedByte"))
    {
        return dimension::UnsignedByte;
    }
    if (boost::iequals(t, "Float"))
    {
        return dimension::Float;
    }
    if (boost::iequals(t, "Pointer"))
    {
        return dimension::Pointer;
    }

    return dimension::Undefined;

}

void Scaling::readBufferBeginImpl(PointBuffer& buffer)
{
    pdal::Schema const& schema = buffer.getSchema();
    std::map<dimension::id, dimension::id>::const_iterator d;
    for (d = m_scale_map.begin(); d != m_scale_map.end(); ++d)
    {
        boost::optional<pdal::Dimension const&> fr = schema.getDimensionOptional(d->first);
        boost::optional<pdal::Dimension const&> to = schema.getDimensionOptional(d->second);
        
        if (!fr) throw pdal_error("from dimension is not found on schema!");
        if (!to) throw pdal_error("to dimension is not found on schema!");
        
        std::pair<boost::optional<pdal::Dimension const&>, boost::optional<pdal::Dimension const&> > g(fr, to);
        m_dimension_map.insert(g);

    }
}

boost::uint32_t Scaling::readBufferImpl(PointBuffer& buffer)
{
    const boost::uint32_t numRead = getPrevIterator().read(buffer);

    for (boost::uint32_t pointIndex=0; pointIndex<numRead; pointIndex++)
    {
        std::map<boost::optional<pdal::Dimension const&>, boost::optional<pdal::Dimension const&> >::const_iterator d;
        
        for (d = m_dimension_map.begin(); d != m_dimension_map.end(); ++d)
        {
            boost::optional<pdal::Dimension const&> f = d->first;
            boost::optional<pdal::Dimension const&> t = d->second;
            Dimension const& from_dimension = *f;
            Dimension const& to_dimension = *t;
            writeScaledData(buffer, from_dimension, to_dimension, pointIndex);
        }
    }
    return numRead;
}


#ifdef PDAL_COMPILER_MSVC
// the writeScaledData function causes lots of conversion warnings which
// are real but which we will ignore: blame the filter's user for casting badly
#  pragma warning(push)
#  pragma warning(disable: 4244)  // conversion from T1 to T2, possible loss of data
#endif

void Scaling::writeScaledData(PointBuffer& buffer,
                              Dimension const& from_dimension,
                              Dimension const& to_dimension,
                              boost::uint32_t pointIndex)
{
    if (from_dimension.getInterpretation() == dimension::Float)
    {
        if (from_dimension.getByteSize() == 4)
        {
            float v = buffer.getField<float>(from_dimension, pointIndex);
            scale(from_dimension,
                  to_dimension,
                  v);
            if (to_dimension.getInterpretation() == dimension::SignedInteger)
            {
                if (to_dimension.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to_dimension, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension::Float to dimension::SignedInteger of size >8");
                return;
            }
        }
        else if (from_dimension.getByteSize() == 8)
        {
            double v = buffer.getField<double>(from_dimension, pointIndex);
            scale(from_dimension,
                  to_dimension,
                  v);
            if (to_dimension.getInterpretation() == dimension::SignedInteger)
            {
                if (to_dimension.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to_dimension, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension::Float to dimension::SignedInteger of size >8");
                return;
            }
        }
        else
        {
            std::ostringstream oss;
            oss << "Unable to interpret Float of size '" << from_dimension.getByteSize() <<"'";
            throw pdal_error(oss.str());
        }
    }

    if (from_dimension.getInterpretation() == dimension::SignedInteger)
    {

        if (to_dimension.getInterpretation() == dimension::SignedInteger)
        {
            if (from_dimension.getByteSize() == 1)
            {
                boost::int8_t v = buffer.getField<boost::int8_t>(from_dimension, pointIndex);
                scale(from_dimension,
                      to_dimension,
                      v);
                if (to_dimension.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to_dimension, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 1 to SignedInteger of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 2)
            {
                boost::int16_t v = buffer.getField<boost::int16_t>(from_dimension, pointIndex);
                scale(from_dimension,
                      to_dimension,
                      v);
                if (to_dimension.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to_dimension, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 2 to SignedInteger of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 4)
            {
                boost::int32_t v = buffer.getField<boost::int32_t>(from_dimension, pointIndex);
                scale(from_dimension,
                      to_dimension,
                      v);
                if (to_dimension.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to_dimension, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 4 to SignedInteger of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 8)
            {
                boost::int64_t v = buffer.getField<boost::int64_t>(from_dimension, pointIndex);
                scale(from_dimension,
                      to_dimension,
                      v);
                if (to_dimension.getByteSize() == 1)
                    buffer.setField<boost::int8_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 2)
                    buffer.setField<boost::int16_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 4)
                    buffer.setField<boost::int32_t>(to_dimension, pointIndex, v);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<boost::int64_t>(to_dimension, pointIndex, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 8 to SignedInteger of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::SignedInteger >8 to dimension::SignedInteger of size >8");
            }

        }
        else if (to_dimension.getInterpretation() == dimension::Float)
        {

            if (from_dimension.getByteSize() == 1)
            {
                boost::int8_t v = buffer.getField<boost::int8_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 1 to Float of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 2)
            {
                boost::int16_t v = buffer.getField<boost::int16_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 2 to Float of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 4)
            {
                boost::int32_t v = buffer.getField<boost::int32_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 4 to Float of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 8)
            {
                boost::int64_t v = buffer.getField<boost::int64_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 8 to Float of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::SignedInteger >8 to dimension::Float of size >8");
            }

        }
    }

    if (from_dimension.getInterpretation() == dimension::UnsignedInteger)
    {

        if (to_dimension.getInterpretation() == dimension::UnsignedInteger)
        {
            if (from_dimension.getByteSize() == 1)
            {
                boost::uint8_t v = buffer.getField<boost::uint8_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);

                if (to_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = to_dimension.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = to_dimension.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = to_dimension.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = to_dimension.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to_dimension, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 8 to UnsignedInteger of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 2)
            {
                boost::uint16_t v = buffer.getField<boost::uint16_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);

                if (to_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = to_dimension.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = to_dimension.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = to_dimension.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = to_dimension.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to_dimension, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 2 to UnsignedInteger of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 4)
            {
                boost::uint32_t v = buffer.getField<boost::uint32_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);

                if (to_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = to_dimension.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = to_dimension.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = to_dimension.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = to_dimension.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to_dimension, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 4 to UnsignedInteger of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 8)
            {
                boost::uint64_t v = buffer.getField<boost::uint64_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);

                if (to_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = to_dimension.removeScaling<boost::uint8_t>(d);
                    buffer.setField<boost::uint8_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = to_dimension.removeScaling<boost::uint16_t>(d);
                    buffer.setField<boost::uint16_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = to_dimension.removeScaling<boost::uint32_t>(d);
                    buffer.setField<boost::uint32_t>(to_dimension, pointIndex, i);
                }
                else if (to_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = to_dimension.removeScaling<boost::uint64_t>(d);
                    buffer.setField<boost::uint64_t>(to_dimension, pointIndex, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 8 to UnsignedInteger of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::UnsignedInteger >8 to dimension::UnsignedInteger of size >8");
            }

        }
        else if (to_dimension.getInterpretation() == dimension::Float)
        {

            if (from_dimension.getByteSize() == 1)
            {
                boost::uint8_t v = buffer.getField<boost::uint8_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 1 to Float of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 2)
            {
                boost::uint16_t v = buffer.getField<boost::uint16_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 2 to Float of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 4)
            {
                boost::uint32_t v = buffer.getField<boost::uint32_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 4 to Float of size > 8");
                return;
            }
            else if (from_dimension.getByteSize() == 8)
            {
                boost::uint64_t v = buffer.getField<boost::uint64_t>(from_dimension, pointIndex);
                double d = from_dimension.applyScaling<double>(v);
                d = to_dimension.removeScaling<double>(d);
                if (to_dimension.getByteSize() == 4)
                    buffer.setField<float>(to_dimension, pointIndex, (float)d);
                else if (to_dimension.getByteSize() == 8)
                    buffer.setField<double>(to_dimension, pointIndex, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 8 to Float of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::UnsignedInteger >8 to dimension::Float of size >8");
            }

        }
    }

    throw pdal_error("Dimension data type unable to be scaled because it is Undefined");
}

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif


boost::uint64_t Scaling::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Scaling::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

}
} // iterators::sequential


}
} // namespaces
