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

#ifndef INCLUDED_FILTERS_STATSFILTER_HPP
#define INCLUDED_FILTERS_STATSFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>

#include <pdal/Range.hpp>
#include <pdal/PointBuffer.hpp>


namespace pdal { namespace filters {

class StatsFilterSequentialIterator;

class StatsCollector
{
public:
    StatsCollector();

    void insert(double value);
    void reset();

    double minimum() const { return m_minimum; }
    double maximum() const { return m_maximum; }
    double average() const { return m_sum / (double)m_count; }
    boost::uint64_t count() const { return m_count; }
    
    boost::property_tree::ptree toPTree() const;

private:
    boost::uint64_t m_count;
    double m_minimum;
    double m_maximum;
    double m_sum;
};


// this is just a pass-thorugh filter, which collects some stats about the points
// that are fed through it
class PDAL_DLL Stats : public Filter
{
public:
    SET_STAGE_NAME("filters.stats", "Statistics Filter")

    Stats(Stage& prevStage, const Options&);
    Stats(Stage& prevStage);
    ~Stats();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        if (t == StageIterator_Random ) return true;
        if (t == StageIterator_Block ) return true;
        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator() const;
    pdal::StageRandomIterator* createRandomIterator() const { return 0; } // BUG: add this
    pdal::StageBlockIterator* createBlockIterator() const { return 0; } // BUG: add this

    void processBuffer(PointBuffer& data) const;

    // returns the stats for field i
    const StatsCollector& getStats(DimensionId::Id id) const;

    // clears the counters for all fields
    void reset();

    // return a tree like this:
    //    X:
    //        cout: 100
    //        min: 1.0
    //        max: 100.0
    //    Y:
    //        cout: 100
    //        min: 11.0
    //        max: 110.0
    //
    boost::property_tree::ptree toStatsPTree() const;

private:
// the stats are keyed by the field name
    // BUG: not threadsafe, these should maybe live in the iterator
    std::map<DimensionId::Id,StatsCollector*> m_stats; // one Stats item per field in the schema

    Stats& operator=(const Stats&); // not implemented
    Stats(const Stats&); // not implemented
};



namespace iterators { namespace sequential {


class Stats : public pdal::FilterSequentialIterator
{
public:
    Stats(const pdal::filters::Stats& filter);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    const pdal::filters::Stats& m_statsFilter;
};


} } // iterators::sequential

} } // namespaces

#endif
