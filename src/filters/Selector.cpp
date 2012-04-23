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

namespace pdal
{
namespace filters
{


// ------------------------------------------------------------------------

Selector::Selector(Stage& prevStage, const Options& options)
    : Filter(prevStage, options)
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
    
    Option ignored = options.getOption("ignore");
    boost::optional<Options const&> ignored_options = ignored.getOptions();
    
    std::vector<Option>::const_iterator i;
    if (ignored_options)
    {
        std::vector<Option> ignored_dimensions = ignored_options->getOptions("dimension");
        for (i = ignored_dimensions.begin(); i != ignored_dimensions.end(); ++i)
        {
            m_ignoredDimensions.push_back( i->getValue<std::string>());
        }
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
    std::vector<std::string> const& ignored = m_selectorFilter.getIgnoredDimensionNames();
    
    std::vector<std::string>::const_iterator i;
    
    Schema const& original_schema = buffer.getSchema();

    Schema new_schema = buffer.getSchema();
    
    for (i = ignored.begin(); i != ignored.end(); ++i)
    {
        boost::optional<Dimension const&> d = original_schema.getDimensionOptional(*i);
        if (d)
        {
            Dimension t(*d);
            boost::uint32_t flags = t.getFlags();
            t.setFlags(flags | dimension::IsIgnored);
            new_schema.setDimension(t);
        }
    }

    buffer = PointBuffer(new_schema, buffer.getCapacity());
}


boost::uint32_t Selector::readBufferImpl(PointBuffer& buffer)
{
    const Schema& schema = buffer.getSchema();

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
