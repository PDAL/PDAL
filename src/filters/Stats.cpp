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

#include <pdal/filters/Stats.hpp>
#include <pdal/Utils.hpp>

#include <pdal/PointBuffer.hpp>
#include <boost/algorithm/string.hpp>

namespace pdal
{
namespace filters
{
namespace stats
{

void Summary::extractMetadata(MetadataNode &m) const
{
    uint32_t cnt = static_cast<uint32_t>(count());
    m.add("count", cnt, "count");
    m.add("minimum", minimum(), "minimum");
    m.add("maximum", maximum(), "maximum");
    m.add("average", average(), "average");
    m.add("name", m_name, "name");

    std::ostringstream sample;
    for (std::vector<double>::size_type i = 0; i < m_sample.size(); ++i)
        sample << m_sample[i] << " ";

    m.add("sample", sample.str(), "sample");

    if (m_doExact)
    {
        MetadataNode countsNode = m.add("counts");
        for (auto i = m_counts.begin(); i != m_counts.end(); ++i)
        {
            std::ostringstream binname;
            binname << "count-" << i->first;
            MetadataNode binNode = countsNode.add(binname.str());
            binNode.add("value", i->first);
            binNode.add("count", i->second);
        }
    }
}


boost::property_tree::ptree Summary::toPTree(PointContext ctx) const
{
    boost::property_tree::ptree tree;

    uint32_t cnt = static_cast<uint32_t>(count());
    tree.put("count", cnt);
    tree.put("minimum", minimum());
    tree.put("maximum", maximum());
    tree.put("average", average());

    std::ostringstream sample;
    for (size_t i = 0; i < m_sample.size(); ++i)
        sample << m_sample[i] << " ";
    tree.add("sample", sample.str());

    if (m_doExact == true)
    {
        boost::property_tree::ptree counts;
        for (auto i = m_counts.begin(); i != m_counts.end(); ++i)
        {
            boost::property_tree::ptree bin;
            bin.add("value", i->first);
            bin.add("count", i->second);
            std::ostringstream binname;
            binname << "count-" <<i->first;
            counts.add_child(binname.str(), bin);
        }
        tree.add_child("counts", counts);
    }
    return tree;
}

} // namespace stats


Options Stats::getDefaultOptions()
{
    Options options;
    options.add("sample_size", 1000, "Number of points to return for "
        "uniform random 'sample'");
    options.add("num_bins", 20, "Number of bins to use for histogram");
    options.add("stats_cache_size", 100000, "Number of points "
        "to use for histogram bin determination. Defaults to total number "
        "of points read if no option is specified.");
    options.add("seed", 0, "Seed to use for repeatable random sample. A "
        "seed value of 0 means no seed is used");
    options.add("num_points", 0, "Total number of points to be processed. "
        "(0 indicates all points will be processed).");
    return options;
}


void Stats::filter(PointBuffer& buffer)
{
    point_count_t limit = (std::min)(buffer.size(), m_numPoints);
    for (PointId idx = 0; idx < limit; ++idx)
    {
        for (auto p = m_stats.begin(); p != m_stats.end(); ++p)
        {
            Dimension::Id::Enum d = p->first;
            SummaryPtr c = p->second;
            c->insert(buffer.getFieldAs<double>(d, idx));
        }
    }
}


void Stats::done(PointContext ctx)
{
    extractMetadata(ctx);
}

void Stats::processOptions(const Options& options)
{
    m_exact_dim_opt = m_options.getValueOrDefault<std::string>(
        "exact_dimensions", "");
    m_dim_opt = m_options.getValueOrDefault<std::string>("dimensions", "");
    m_cache_size = m_options.getValueOrDefault<uint32_t>(
        "stats_cache_size", 1000);
    m_sample_size = m_options.getValueOrDefault<uint32_t>(
        "sample_size", 100000);
    m_seed = m_options.getValueOrDefault<uint32_t>("seed", 0);
    m_bin_count = m_options.getValueOrDefault<uint32_t>("num_bins", 20);
    m_numPoints = m_options.getValueOrDefault<point_count_t>("num_points", 0);
    if (m_numPoints == 0)
        m_numPoints = (std::numeric_limits<point_count_t>::max)();
    if (m_options.hasOption("do_sample"))
        m_do_sample = m_options.getValueOrThrow<bool>("do_sample");
    else
        m_do_sample = !m_exact_dim_opt.size() && !m_dim_opt.size();
}


void Stats::initialize()
{
    m_metadata.add("sample_size", m_sample_size);
    m_metadata.add("seed", m_seed);
    m_metadata.add("num_bins", m_bin_count);
    m_metadata.add("stats_cache_size", m_cache_size);
}


void Stats::ready(PointContext ctx)
{
    using namespace std;

    log()->get(LogLevel::Debug) << "Calculating histogram statistics for "
        "exact names '" << m_exact_dim_opt << "'"<< std::endl;

    vector<string> dims;
    boost::split(dims, m_exact_dim_opt, boost::is_any_of(" ,"));

    std::map<std::string, bool> exact_dimensions;
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        string dimName = *di;
        boost::trim(dimName);
        if (dimName.size())
        {
            log()->get(LogLevel::Debug) << "adding '" << dimName <<
                "' as exact dimension name to cumulate stats for" << std::endl;
            m_exact_dimension_names.insert(dimName);
            m_dimension_names.insert(dimName);
            exact_dimensions[dimName] = true;
        }
    }

    dims.clear();
    boost::split(dims, m_dim_opt, boost::is_any_of(" ,"));
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        string dimName = *di;
        boost::trim(dimName);
        if (dimName.size())
            m_dimension_names.insert(dimName);
    }

    if (m_dimension_names.size())
    {
        log()->get(LogLevel::Debug2) << "Explicit dimension size:" <<
            m_dimension_names.size() << std::endl;

        for (auto i = m_dimension_names.begin();
                i != m_dimension_names.end(); i++)
        {
            std::string const& name = *i;
            log()->get(LogLevel::Debug2) << "Requested to cumulate stats for "
                "dimension with name '" << name <<"'"<< std::endl;
            Dimension::Id::Enum d = ctx.findDim(name);
            if (d == Dimension::Id::Unknown)
                continue;
            log()->get(LogLevel::Debug2) << "Found dimension with name '" <<
                name << "'"<< std::endl;
            log()->get(LogLevel::Debug2) << "Cumulating stats for dimension " <<
                name << std::endl;

            bool doExact =
                (exact_dimensions.find(name) != exact_dimensions.end());

            m_stats[d] = SummaryPtr(new stats::Summary(*i, m_bin_count,
                m_sample_size, m_cache_size, m_seed, doExact, m_do_sample));
        }
    }
    else
    {
        Dimension::IdList dims = ctx.dims();
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            Dimension::Id::Enum d = *di;
            log()->get(LogLevel::Debug2) << "Cumulating stats for dimension " <<
                ctx.dimName(d) << std::endl;
            m_stats[d] = SummaryPtr(new stats::Summary(ctx.dimName(*di) ,m_bin_count,
                m_sample_size, m_cache_size, m_seed, false, m_do_sample));
        }
    }
}


void Stats::extractMetadata(PointContext ctx)
{

    boost::uint32_t position(0);
    
    // MetadataNode stat = m_metadata.add("statistic");
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Dimension::Id::Enum d = di->first;
        const SummaryPtr s = di->second;

        MetadataNode t = m_metadata.addList("statistic");
        t.add("position", position++);
        s->extractMetadata(t);
    }
}


boost::property_tree::ptree Stats::toPTree(PointContext ctx) const
{
    using namespace boost::property_tree ;
    ptree tree;

    tree.push_back(ptree::value_type("stats", ptree()));
    auto& p = tree.get_child("stats");
    
    boost::uint32_t position(0);
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Dimension::Id::Enum d = di->first;
        const SummaryPtr stat = di->second;

        boost::property_tree::ptree subtree = stat->toPTree(ctx);
        subtree.add("position", position++);
        p.add_child("", subtree);
    }
    return tree;
}


stats::Summary const& Stats::getStats(Dimension::Id::Enum dim) const
{
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Dimension::Id::Enum d = di->first;
        if (d == dim)
            return *(di->second);
    }
    throw pdal_error("Dimension not found");
}

} // namespace filters
} // namespace pdal
