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

#pragma once

#include <pdal/Filter.hpp>

#include <pdal/Range.hpp>
#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/density.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace pdal
{
namespace filters
{
namespace stats
{

typedef boost::iterator_range<std::vector<std::pair<double, double> >::iterator > histogram_type;
#ifndef PDAL_COMPILER_MSVC  // See boost ticket 6535:  https://svn.boost.org/trac/boost/ticket/6535
typedef boost::accumulators::accumulator_set<double, boost::accumulators::features< boost::accumulators::droppable<boost::accumulators::tag::density> > > density_accumulator;
#else
typedef boost::accumulators::accumulator_set<double, boost::accumulators::features< boost::accumulators::tag::density > > density_accumulator ;
#endif
typedef boost::accumulators::accumulator_set<double, boost::accumulators::features<     boost::accumulators::droppable<boost::accumulators::tag::mean>,
        boost::accumulators::droppable<boost::accumulators::tag::max>,
        boost::accumulators::droppable<boost::accumulators::tag::min>,
        boost::accumulators::droppable<boost::accumulators::tag::count> > > summary_accumulator;

class PDAL_DLL Summary
{
public:
    Summary(std::string name, uint32_t num_bins, uint32_t sample_size, uint32_t cache_size,
        uint32_t seed, bool doExact, bool doSample) : 
        m_sample_size(sample_size), m_distribution(0, cache_size),
        m_doExact(doExact), m_doSample(doSample), m_name(name)
    {
        if (seed && m_doSample)
        {
            m_rng.seed(seed);
            m_distribution.reset();
        }
    }

    double minimum() const
        { return (boost::accumulators::min)(m_summary); }
    double maximum() const
        { return (boost::accumulators::max)(m_summary); }
    double average() const
        { return boost::accumulators::mean(m_summary); }
    boost::uint64_t count() const
        { return boost::accumulators::count(m_summary); }

    virtual boost::property_tree::ptree toPTree(PointContext ctx) const;
    void extractMetadata(MetadataNode &m) const;

    void reset()
    {
        m_summary.drop<boost::accumulators::tag::mean>();
        m_summary.drop<boost::accumulators::tag::count>();
        m_summary.drop<boost::accumulators::tag::max>();
        m_summary.drop<boost::accumulators::tag::min>();
        m_counts.clear();
    }

    template<class T> inline void insert(T value)
    {
        m_summary(static_cast<double>(value));
        if (m_doSample)
        {
            uint32_t sample = (uint32_t)m_distribution(m_rng);
            if (sample < m_sample_size)
                m_sample.push_back(static_cast<double>(value));
        }

        if (m_doExact == true)
            m_counts[(int32_t)value]++;
    }

private:
    summary_accumulator m_summary;
    std::vector<double> m_sample;
    uint32_t m_sample_size;
    boost::random::mt19937 m_rng;
    boost::random::uniform_int_distribution<> m_distribution;
    std::map<int32_t, uint32_t> m_counts;
    bool m_doExact;
    bool m_doSample;
    std::string m_name;
};

} // namespace stats
typedef boost::shared_ptr<stats::Summary> SummaryPtr;

// This is just a pass-thorugh filter, which collects some stats about
// the points that are fed through it
class PDAL_DLL Stats : public Filter
{
public:
    SET_STAGE_NAME("filters.stats", "Statistics Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.stats.html")  
    SET_STAGE_ENABLED(true)
    
    Stats(const Options& options) : Filter(options)
        {}

    static Options getDefaultOptions();

    boost::property_tree::ptree toPTree(PointContext ctx) const;
    const stats::Summary& getStats(Dimension::Id::Enum d) const;
    void reset();

private:
    Stats& operator=(const Stats&); // not implemented
    Stats(const Stats&); // not implemented
    virtual void processOptions(const Options& options);
    virtual void initialize();
    virtual void ready(PointContext ctx);
    virtual void done(PointContext ctx);
    virtual void filter(PointBuffer& data);
    void extractMetadata(PointContext ctx);

    std::string m_exact_dim_opt;
    std::string m_dim_opt;
    uint32_t m_cache_size;
    uint32_t m_sample_size;
    uint32_t m_seed;
    uint32_t m_bin_count;
    bool m_do_sample;
    point_count_t m_numPoints;
    std::set<std::string> m_dimension_names;
    std::set<std::string> m_exact_dimension_names;
    std::map<Dimension::Id::Enum, SummaryPtr> m_stats;
};

} // namespace filters
} // namespace pdal

