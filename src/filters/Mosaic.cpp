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

#include <pdal/filters/Mosaic.hpp>

#include <pdal/Bounds.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace filters
{


void Mosaic::initialize()
{
    const std::vector<Stage*>& stages = getPrevStages();

    const Stage& stage0 = *stages[0];
    const SpatialReference& srs0 = stage0.getSpatialReference();
    bool respectSrs = getOptions().getValueOrDefault<bool>("require_matching_srs", false);
//ABELL
/**
    const Schema& schema0 = stage0.getSchema();
    boost::uint64_t totalPoints = stage0.getNumPoints();
    Bounds<double> bigbox(stage0.getBounds());

    // we will only mosaic if all the stages have the same core
    // properties: SRS, schema, etc
    for (boost::uint32_t i=1; i<stages.size(); i++)
    {
        Stage& stage = *(stages[i]);
        if (respectSrs && stage.getSpatialReference() != srs0)
            throw impedance_invalid("mosaicked stages must have same srs");
        if (stage.getSchema() != schema0)
            throw impedance_invalid("mosaicked stages must have same schema");

        totalPoints += stage.getNumPoints();

        bigbox.grow(stage.getBounds());
    }
**/
}


Options Mosaic::getDefaultOptions()
{
    Options options;
    Option require_matching_srs("require_matching_srs",false,"Throw an exception when SRSs do not match");
    Option ignore_namespaces("ignore_namespaces",false,"Ignore namespaces and copy first matching dimension");
    
    options.add(require_matching_srs);
    options.add(ignore_namespaces);
    return options;
}


pdal::StageSequentialIterator*
Mosaic::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Mosaic(*this, buffer,
        log(), getOptions());
}


namespace iterators
{
namespace sequential
{


Mosaic::Mosaic(const pdal::filters::Mosaic& filter, PointBuffer& buffer,
        LogPtr log, const Options& options)
    : MultiFilterSequentialIterator(filter, buffer), m_log(log),
    m_options(options)
{}


Mosaic::~Mosaic()
{
    return;
}


boost::uint64_t Mosaic::skipImpl(boost::uint64_t targetCount)
{
    boost::uint64_t count = 0;

    while (count < targetCount)
    {
        m_prevIterator = m_prevIterators[m_iteratorIndex];

        // skip as much as we can in the current stage
        boost::uint64_t thisCount = m_prevIterator->skip(count);
        count += thisCount;

        if (m_prevIterators[m_iteratorIndex]->atEnd())
        {
            ++m_iteratorIndex;
        }
        if (m_iteratorIndex == m_prevIterators.size())
        {
            // no more points
            break;
        }
    }

    return count;
}


bool Mosaic::atEndImpl() const
{
    if (m_iteratorIndex == m_prevIterators.size())
        return true;
    if (m_prevIterators[m_iteratorIndex]->atEnd())
        return true;
    return false;
}


DimensionMapPtr Mosaic::fetchDimensionMap(PointBuffer const& user_buffer,
    PointBufferPtr stage_buffer)
{
    DimensionMaps::const_iterator i = m_dimensions.find(m_iteratorIndex);

    if (i != m_dimensions.end())
    {
        m_log->get(logDEBUG2) << "Mosaic::fetchDimensionMap: found existing "
            "DimensionMap with id " << m_iteratorIndex << std::endl;
        return i->second;
    }
    else
    {
        bool ignore_namespaces =
            m_options.getValueOrDefault<bool>("ignore_namespaces", false);
        DimensionMapPtr output  =
            DimensionMapPtr(stage_buffer->getSchema().
                mapDimensions(user_buffer.getSchema(), ignore_namespaces));
        m_log->get(logDEBUG2) << "DimensionMapPtr->size():  " <<
            output->m.size() << std::endl;
        if (!output->m.size())
            throw pdal_error("fetchDimensionMap map was unable to map "
                "any dimensions!");
        typedef std::map<Dimension const*,
            Dimension const*>::const_iterator Iterator;
        for (Iterator i = output->m.begin(); i != output->m.end(); ++i)
        {
            m_log->get(logDEBUG2) << "mapping " << i->first->getFQName() <<
                " to " << i->second->getFQName() << std::endl;
        }
        std::pair<int, DimensionMapPtr> p(m_iteratorIndex, output);
        m_dimensions.insert(p);
        m_log->get(logDEBUG2) << "IteratorBase::fetchDimensionMap: "
            "creating new DimensionMap with id " << m_iteratorIndex <<
            std::endl;
        return p.second;
    }
}



PointBufferPtr Mosaic::fetchPointBuffer(PointBuffer const& user_buffer)
{
    PointBufferMap::const_iterator i = m_buffers.find(m_iteratorIndex);

    if (i != m_buffers.end())
    {
        m_log->get(logDEBUG2) << "Mosaic::fetchPointBuffer: found existing "
            "PointBuffer with id " << m_iteratorIndex << std::endl;
        return i->second;
    }
    else
    {
        PointBufferPtr output(new PointBuffer(user_buffer.context()));

        std::pair<int, PointBufferPtr> p(m_iteratorIndex, output);
        m_buffers.insert(p);
        m_log->get(logDEBUG2)  << "Mosaic::fetchPointBuffer: creating "
            "new PointBuffer with id " << m_iteratorIndex << std::endl;
        return p.second;
    }
}

point_count_t Mosaic::readBufferImpl(PointBuffer& user_buffer)
{
    point_count_t totalNumPointsToRead = user_buffer.size();
    point_count_t totalNumPointsRead = 0;
    point_count_t destPointIndex = 0;

    // for each stage, we read as many points as we can
    
    while (totalNumPointsRead < totalNumPointsToRead)
    {
        assert(m_iteratorIndex < m_prevIterators.size());
        m_prevIterator = m_prevIterators[m_iteratorIndex];

        PointBufferPtr tmp = fetchPointBuffer(user_buffer);
        
        point_count_t howMany(totalNumPointsToRead-totalNumPointsRead);
        
        point_count_t numRead = m_prevIterator->read(*tmp);
        totalNumPointsRead += numRead;

        DimensionMapPtr d = fetchDimensionMap(user_buffer, tmp);

//ABELL
/**
        PointBuffer::copyLikeDimensions(*tmp, user_buffer,
                                        *d,
                                        0, user_buffer.size(),
                                        howMany);
**/
        if (m_prevIterator->atEnd())
        {
            ++m_iteratorIndex;
        }
        if (m_iteratorIndex == m_prevIterators.size())
        {
            break;
        }
    }
    return totalNumPointsRead;
}


}
} // iterators::sequential

}
} // namespaces
