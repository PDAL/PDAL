/******************************************************************************
* Copyright (c) 2012, Howard Butler (hobu@hobu.net)
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

#include <pdal/filters/Selector.hpp>

#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/random_generator.hpp>

namespace pdal
{
namespace filters
{


// ------------------------------------------------------------------------

Selector::Selector(Stage& prevStage, const Options& options)
    : Filter(prevStage, options)
    , m_ignoreDefault(true)
{
    return;
}


void Selector::initialize()
{
    Filter::initialize();

    checkImpedance();

    return;
}


void Selector::checkImpedance()
{
    Options& options = getOptions();
    
    m_ignoreDefault = options.getValueOrDefault<bool>("ignore_default", true);
    std::vector<Option>::const_iterator i;
    
    try
    {
        Option ignored = options.getOption("ignore");
        boost::optional<Options const&> ignored_options = ignored.getOptions();
    
        
        if (ignored_options)
        {
            std::vector<Option> ignored_dimensions = ignored_options->getOptions("dimension");
            for (i = ignored_dimensions.begin(); i != ignored_dimensions.end(); ++i)
            {
                m_ignoredMap.insert(std::pair<std::string, bool>(i->getValue<std::string>(), true));
            }
        }
    }
    catch (option_not_found&)
    {
    }

    try
    {
        Option keep = options.getOption("keep");
        boost::optional<Options const&> keep_options = keep.getOptions();
    
        if (keep_options)
        {
            std::vector<Option> keep_dimensions = keep_options->getOptions("dimension");
            for (i = keep_dimensions.begin(); i != keep_dimensions.end(); ++i)
            {
                m_ignoredMap.insert(std::pair<std::string, bool>(i->getValue<std::string>(), false));
            }
        }
    }
    catch (option_not_found&)
    {
    }

    try
    {
        Option create = options.getOption("create");
        boost::optional<Options const&> create_options = create.getOptions();
    
        if (create_options)
        {
            std::vector<Option> create_dimensions = create_options->getOptions("dimension");
            for (i = create_dimensions.begin(); i != create_dimensions.end(); ++i)
            {
                Option const& o = *i;
                boost::optional<Options const&> dim_opts = o.getOptions();
                
                if (!dim_opts)
                    throw pdal_error("create dimension option has no sub options!");
                Options const& ops = *dim_opts;
                std::string name;
                boost::uint32_t size(0);
                std::string description;
                std::string interpretation;
                dimension::id uuid;
                dimension::id parent_uuid;
                double offset(0.0);
                double scale(0.0);
                double minimum(0.0);
                double maximum(0.0);
                EndianType endianness = Endian_Little;
                
                name = o.getValue<std::string>();
                scale = ops.getValueOrDefault<double>("scale", 1.0);
                offset = ops.getValueOrDefault<double>("offset", 0.0);
                size = ops.getValueOrDefault<boost::uint32_t>("size", 1);
                
                description = ops.getValueOrDefault<std::string>("description", "");
                interpretation = ops.getValueOrThrow<std::string>("interpretation");

                std::string endy = ops.getValueOrDefault<std::string>("endianness", "little");
                if (boost::iequals(endy, "big"))
                    endianness = Endian_Big;
                
                uuid = ops.getValueOrDefault<dimension::id>("uuid", boost::uuids::random_generator()());
                parent_uuid = ops.getValueOrDefault<dimension::id>("parent_uuid",  boost::uuids::nil_uuid());

                minimum = ops.getValueOrDefault<double>("minimum", 0.0);
                maximum = ops.getValueOrDefault<double>("maximum", 0.0);
                
                dimension::Interpretation interp = Dimension::getInterpretation(interpretation);

                Dimension d(name, interp, size, description);
                d.setUUID(uuid);
                d.setParent(parent_uuid);
                d.setNumericScale(scale);
                d.setNumericOffset(offset);
                d.setMinimum(minimum);
                d.setMaximum(maximum);
                m_createDimensions.push_back(d);
            }
        }
    }
    catch (option_not_found&)
    {
    }

    return;
}


void Selector::processBuffer(const PointBuffer& /*srcData*/, PointBuffer& /*dstData*/) const
{
    return;
}


pdal::StageSequentialIterator* Selector::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Selector(*this, buffer);
}


const Options Selector::getDefaultOptions() const
{
    Options options;
    Option ignore("ignore", "DimensionName", "An Options set with entries of name 'dimension' names to ignore");
    return options;
}


namespace iterators
{
namespace sequential
{


Selector::Selector(const pdal::filters::Selector& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_selectorFilter(filter)
{
    alterSchema(buffer);
    m_selectorFilter.log()->get(logDEBUG4) << "iterators::sequential alterSchema! " << std::endl;

    return;
}

void Selector::alterSchema(PointBuffer& buffer)
{
    Schema const& original_schema = buffer.getSchema();

    Schema new_schema = buffer.getSchema();
    
    std::map<std::string, bool> const& ignoredMap = m_selectorFilter.getIgnoredMap();
    // for (std::map<std::string, bool>::const_iterator i = ignoredMap.begin();
    //     i != ignoredMap.end(); ++i)
    // {
    //     boost::optional<Dimension const&> d = original_schema.getDimensionOptional(i->first);
    //     if (d)
    //     {
    //         Dimension d2(*d);
    //         boost::uint32_t flags = d2.getFlags();
    //         if (i->second)
    //             d2.setFlags(flags | dimension::IsIgnored);
    //         new_schema.setDimension(d2);
    //     }
    // }
    // 

    std::vector<Dimension> const& new_dimensions = m_selectorFilter.getCreatedDimensions();
    if (new_dimensions.size())
    {
        for (std::vector<Dimension>::const_iterator i = new_dimensions.begin(); i != new_dimensions.end(); ++i)
        {
            new_schema.appendDimension(*i);
        }
    }

    schema::Map dimensions = original_schema.getDimensions();
    schema::index_by_index const& dims = dimensions.get<schema::index>();
    for (schema::index_by_index::const_iterator t = dims.begin(); 
         t != dims.end(); 
         ++t)
    {
        
        std::map<std::string, bool>::const_iterator ignored = ignoredMap.find(t->getName());
        if (ignored != ignoredMap.end())
        {
            if (ignored->second) // marked to be dropped
            {
                // set to ignored
                Dimension d2(*t);
                boost::uint32_t flags = d2.getFlags();
                d2.setFlags(flags | dimension::IsIgnored);
                new_schema.setDimension(d2);
            }
        }
        
        else { // didn't find it in our map of specified dimensions
            
            if (m_selectorFilter.doIgnoreUnspecifiedDimensions())
            {
                // set to ignored
                Dimension d2(*t);
                boost::uint32_t flags = d2.getFlags();
                d2.setFlags(flags | dimension::IsIgnored);
                new_schema.setDimension(d2);
            }
        }

    }

    buffer = PointBuffer(new_schema, buffer.getCapacity());
}


boost::uint32_t Selector::readBufferImpl(PointBuffer& buffer)
{
    const boost::uint32_t numRead = getPrevIterator().read(buffer);

    return numRead;
}


boost::uint64_t Selector::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Selector::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

}
} // iterators::sequential


}
} // namespaces
