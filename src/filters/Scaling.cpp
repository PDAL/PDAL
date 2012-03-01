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

namespace pdal { namespace filters {


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


namespace iterators { namespace sequential {


Scaling::Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_scalingFilter(filter)
{
    alterSchema(buffer);
    return;
}

void Scaling::alterSchema(PointBuffer& buffer)
{
    std::vector<scaling::Scaler> const& scalers = m_scalingFilter.getScalers();
    
    std::vector<scaling::Scaler>::const_iterator i;
    
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
            std::pair<dimension::id, dimension::id> p(from_dimension->getUUID(), to_dimension.getUUID());
            m_scale_map.insert(p);
            schema.appendDimension(to_dimension);
        }
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

boost::uint32_t Scaling::readBufferImpl(PointBuffer& buffer)
{
    const Schema& schema = buffer.getSchema();

    const boost::uint32_t numRead = getPrevIterator().read(buffer);
    
    for (boost::uint32_t pointIndex=0; pointIndex<numRead; pointIndex++)
    {
        std::map<dimension::id, dimension::id>::const_iterator d;
        for (d = m_scale_map.begin(); d != m_scale_map.end(); ++d)
        {
            Dimension const& from_dimension = schema.getDimension(d->first);
            
            Dimension const& to_dimension = schema.getDimension(d->second);
            writeScaledData(buffer, from_dimension, to_dimension, pointIndex);
        }
    }

    return numRead;
}

void Scaling::writeScaledData(  PointBuffer& buffer, 
                                Dimension const& from_dimension, 
                                Dimension const& to_dimension, 
                                boost::uint32_t pointIndex)
{
    
    double dbl(0.0);
    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);

    switch (to_dimension.getInterpretation())
    {
        case dimension::SignedByte:
            i8 = buffer.getField<boost::int8_t>(from_dimension, pointIndex);
            scale(  from_dimension,
                    to_dimension,
                    i8);
            buffer.setField<boost::int8_t>(to_dimension, pointIndex, i8);
            
            break;
        case dimension::UnsignedByte:
            u8 = buffer.getField<boost::uint8_t>(from_dimension, pointIndex);
            scale(  from_dimension,
                    to_dimension,
                    u8);
            buffer.setField<boost::uint8_t>(to_dimension, pointIndex, u8);            
            break;
        case dimension::Float:
            if (from_dimension.getByteSize() == 4)
            {
                flt = buffer.getField<float>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        flt);
                buffer.setField<float>(to_dimension, pointIndex, flt);
                break;
            } 
            else if (from_dimension.getByteSize() == 8)
            {
                dbl = buffer.getField<double>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        dbl);
                buffer.setField<double>(to_dimension, pointIndex, dbl);
                break;
            }
            else 
            {
                std::ostringstream oss;
                oss << "Unable to interpret Float of size '" << from_dimension.getByteSize() <<"'";
                throw pdal_error(oss.str());
            }
            
        case dimension::SignedInteger:
            if (from_dimension.getByteSize() == 1)
            {
                i8 = buffer.getField<boost::int8_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        i8);
                buffer.setField<boost::int8_t>(to_dimension, pointIndex, i8);                
                break;
            } 
            else if (from_dimension.getByteSize() == 2)
            {
                i16 = buffer.getField<boost::int16_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        i16);
                buffer.setField<boost::int16_t>(to_dimension, pointIndex, i16);
                break;
            } else if (from_dimension.getByteSize() == 4)
            {
                i32 = buffer.getField<boost::int32_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        i32);
                buffer.setField<boost::int32_t>(to_dimension, pointIndex, i32);
                break;
            } 
            else if (from_dimension.getByteSize() == 8)
            {
                i64 = buffer.getField<boost::int64_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        i64);
                buffer.setField<boost::int64_t>(to_dimension, pointIndex, i64);
                break;
            }
            else 
            {
                std::ostringstream oss;
                oss << "Unable to interpret SignedInteger of size '" << from_dimension.getByteSize() <<"'";
                throw pdal_error(oss.str());
            }
        case dimension::UnsignedInteger:
            if (from_dimension.getByteSize() == 1)
            {
                u8 = buffer.getField<boost::uint8_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        u8);
                buffer.setField<boost::uint8_t>(to_dimension, pointIndex, u8);
                break;
            } 
            else if (from_dimension.getByteSize() == 2)
            {
                u16 = buffer.getField<boost::uint16_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        u16);
                buffer.setField<boost::uint16_t>(to_dimension, pointIndex, u16);
                break;
            } else if (from_dimension.getByteSize() == 4)
            {
                u32 = buffer.getField<boost::uint32_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        u32);
                buffer.setField<boost::uint32_t>(to_dimension, pointIndex, u32);
                break;
            } 
            else if (from_dimension.getByteSize() == 8)
            {
                u64 = buffer.getField<boost::uint64_t>(from_dimension, pointIndex);
                scale(  from_dimension,
                        to_dimension,
                        u64);
                buffer.setField<boost::uint64_t>(to_dimension, pointIndex, u64);
                break;
            }
            else 
            {
                std::ostringstream oss;
                oss << "Unable to interpret UnsignedInteger of size '" << from_dimension.getByteSize() <<"'";
                throw pdal_error(oss.str());
            }

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
            throw pdal_error("Dimension data type unable to be scaled because it is of interpretation Pointer");
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be scaled because it is Undefined");
    }    
    
    

    return;
}

boost::uint64_t Scaling::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Scaling::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

} } // iterators::sequential


} } // namespaces
