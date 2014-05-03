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
#include <boost/tokenizer.hpp>


typedef boost::tokenizer<boost::char_separator<char> > tokenizer;


namespace pdal
{
namespace filters
{

//---------------------------------------------------------------------------
namespace stats
{

pdal::Metadata Summary::toMetadata() const
{
    boost::uint32_t cnt = static_cast<boost::uint32_t>(count());
    Metadata output;
    output.addMetadata("count", cnt, "count");
    output.addMetadata("minimum", minimum(), "minimum");
    output.addMetadata("maximum", maximum(), "maximum");
    output.addMetadata("average", average(), "average");

    std::ostringstream sample;
    for (std::vector<double>::size_type i =0; i < m_sample.size(); ++i)
    {
        sample << m_sample[i] << " ";
    };

    output.addMetadata("sample", sample.str(), "sample");

    if (m_doExact == true)
    {
        Metadata counts;
        counts.setType("metadata");
        counts.setName("counts");

        for (std::map<boost::int32_t, boost::uint32_t>::const_iterator i = m_counts.begin();
                i != m_counts.end(); ++i)
        {
            Metadata bin;
            std::ostringstream binname;
            binname << "count-" <<i->first;
            bin.setName(binname.str());
            bin.setType("metadata");
            bin.addMetadata("value", i->first);
            bin.addMetadata("count", i->second);
            counts.addMetadata(bin);
        }

        output.addMetadata(counts);
    }
    return output;

}

boost::property_tree::ptree Summary::toPTree() const
{
    boost::property_tree::ptree tree;

    boost::uint32_t cnt = static_cast<boost::uint32_t>(count());
    tree.put("count", cnt);
    tree.put("minimum", minimum());
    tree.put("maximum", maximum());
    tree.put("average", average());

    // boost::property_tree::ptree bins;
    // histogram_type hist = histogram();
    // for (boost::int32_t i = 0; i < hist.size(); ++i)
    // {
    //     boost::property_tree::ptree bin;
    //     bin.add("lower_bound", hist[i].first);
    //     bin.add("count", Utils::sround(hist[i].second*cnt));
    //     std::ostringstream binname;
    //     binname << "bin-" <<i;
    //     bins.add_child(binname.str(), bin);
    // }
    // tree.add_child("histogram", bins);

    std::ostringstream sample;
    for (std::vector<double>::size_type i =0; i < m_sample.size(); ++i)
    {
        sample << m_sample[i] << " ";
    };

    tree.add("sample", sample.str());


    if (m_doExact == true)
    {

        boost::property_tree::ptree counts;
        for (std::map<boost::int32_t, boost::uint32_t>::const_iterator i = m_counts.begin();
                i != m_counts.end(); ++i)
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


//---------------------------------------------------------------------------


void Stats::addMetadata()
{
    Metadata& metadata = getMetadataRef();
    metadata.addMetadata<boost::uint32_t>("sample_size",
                                          getOptions().getValueOrDefault<boost::uint32_t>("sample_size", 1000));
    metadata.addMetadata<boost::uint32_t>("seed",
                                          getOptions().getValueOrDefault<boost::uint32_t>("seed", 0));
    metadata.addMetadata<boost::uint32_t>("num_bins",
                                          getOptions().getValueOrDefault<boost::uint32_t>("num_bins", 20));
    metadata.addMetadata<boost::uint32_t>("stats_cache_size",
                                          getOptions().getValueOrDefault<boost::uint32_t>("num_bins", 20));
}

Stats::~Stats()
{

}

void Stats::initialize()
{
    Filter::initialize();




    return;
}


Options Stats::getDefaultOptions()
{
    Options options;
    Option sample_size("sample_size", 1000, "Number of points to return for uniform random 'sample'");
    Option num_bins("num_bins", 20, "Number of bins to use for histogram");
    Option stats_cache_size("stats_cache_size", 100000, "Number of points to use for histogram bin determination. Defaults to total number of points read if no option is specified.");
    Option seed("seed", 0, "Seed to use for repeatable random sample. A seed value of 0 means no seed is used");

    options.add(sample_size);
    options.add(num_bins);
    options.add(stats_cache_size);
    options.add(seed);
    return options;
}

pdal::StageSequentialIterator* Stats::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Stats(*this, buffer,
        log(), getPrevStage().getNumPoints(), getName(), getOptions());
}

namespace iterators
{
namespace sequential
{

Stats::Stats(const pdal::filters::Stats& filter, PointBuffer& buffer,
        LogPtr log, boost::uint64_t numPoints, const std::string& name,
        const Options& options)
    : pdal::FilterSequentialIterator(filter, buffer), m_log(log),
    m_numPoints(numPoints), m_name(name), m_options(options)
{}


boost::uint32_t Stats::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    for (boost::uint32_t pointIndex=0; pointIndex < numRead; pointIndex++)
    {
        std::multimap<DimensionPtr, stats::SummaryPtr>::const_iterator p;
        for (p = m_stats.begin(); p != m_stats.end(); ++p)
        {
            DimensionPtr d = p->first;
            stats::SummaryPtr c = p->second;

            double output = data.applyScaling(*d, pointIndex);
            c->insert(output);
        }
    }
    return numRead;
}

boost::uint64_t Stats::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Stats::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

void Stats::readBufferEndImpl(PointBuffer& buffer)
{
    pdal::Metadata& metadata = buffer.getMetadataRef();
    pdal::Metadata stats = toMetadata();
    stats.setName(m_name);
    metadata.setMetadata(stats);
}

void Stats::readBufferBeginImpl(PointBuffer& buffer)
{
    // We'll assume you're not changing the schema per-read call

    if (m_stats.size() == 0)
    {
        std::string exact =
            m_options.getValueOrDefault<std::string>("exact_dimensions", "");
        if (exact.size())
        {
            m_log->get(logDEBUG) << "Calculating histogram statistics for "
                "exact names '" << exact << "'"<<std::endl;
            boost::char_separator<char> seps(" ,");


            tokenizer parameters(exact, seps);
            for (tokenizer::iterator t = parameters.begin(); t != parameters.end(); ++t)
            {
                m_log->get(logDEBUG) << "adding '" << *t << "' as exact "
                    "dimension name to cumulate stats for" << std::endl;
                m_exact_dimension_names.push_back(*t);
                m_dimension_names.push_back(*t);

            }
        }


        std::map<std::string, bool> exact_dimensions;
        for (std::vector<std::string>::const_iterator i = m_exact_dimension_names.begin();
                i != m_exact_dimension_names.end(); ++i)
        {
            m_log->get(logDEBUG2) << "Using exact histogram counts for '" <<
                *i << "'" << std::endl;
            std::pair<std::string,bool> p(*i, true);
            exact_dimensions.insert(p);
        }

        std::string names =
            m_options.getValueOrDefault<std::string>("dimensions", "");
        if (names.size())
        {
            m_log->get(logDEBUG) << "Using explicit list of dimension names'" <<
                 names << "'"<< std::endl;
            boost::char_separator<char> seps(" ,");

            tokenizer parameters(names, seps);
            for (tokenizer::iterator t = parameters.begin(); t != parameters.end(); ++t)
            {
                m_log->get(logDEBUG) << "adding '" << *t <<
                    "' as dimension name to cumulate stats for" << std::endl;
                // Add to explicit dim list if not already there
                if (exact_dimensions.find(*t) == exact_dimensions.end())
                    m_dimension_names.push_back(*t);
            
            }
        }

        Schema const& schema = buffer.getSchema();

        boost::uint64_t numPoints = m_numPoints;
        boost::uint32_t stats_cache_size(1000);

        try
        {
            stats_cache_size =
                m_options.getValueOrThrow<boost::uint32_t>("stats_cache_size");
            m_log->get(logDEBUG2) << "Using " << stats_cache_size <<
                "for histogram cache size set from option" << std::endl;

        }
        catch (pdal::option_not_found const&)
        {
            if (numPoints != 0)
            {
                if (numPoints > (std::numeric_limits<boost::uint32_t>::max)())
                {
                    throw std::out_of_range("too many points for the histogram cache");
                }
                stats_cache_size = static_cast<boost::uint32_t>(numPoints);
                m_log->get(logDEBUG2) << "Using point count, " << numPoints <<
                    ", for histogram cache size" << std::endl;

            }
        }

        boost::uint32_t sample_size =
            m_options.getValueOrDefault<boost::uint32_t>("sample_size", 100000);
        boost::uint32_t seed =
            m_options.getValueOrDefault<boost::uint32_t>("seed", 0);

        m_log->get(logDEBUG2) << "Using " << sample_size <<
            " for sample size" << std::endl;
        m_log->get(logDEBUG2) << "Using " << seed << " for sample seed" <<
            std::endl;

        boost::uint32_t bin_count =
            m_options.getValueOrDefault<boost::uint32_t>("num_bins", 20);
        
        if (m_dimension_names.size())
        {
            m_log->get(logDEBUG2) << "Explicit dimension size:" <<
                m_dimension_names.size() << std::endl;
            
            for (std::vector<std::string>::const_iterator  i = m_dimension_names.begin(); i != m_dimension_names.end(); i++)
            {
                std::string const& name = *i;
                m_log->get(logDEBUG2) << "Requested to cumulate stats for "
                    "dimension with name '" << name <<"'"<< std::endl;
                Dimension const& dim = schema.getDimension(name);
                m_log->get(logDEBUG2) << "Found dimension with name '" <<
                    dim.getName() << "' and namespace '" <<
                    dim.getNamespace() << "'"<< std::endl;

                DimensionPtr d =
                    boost::shared_ptr<Dimension>(new Dimension(dim));
                m_log->get(logDEBUG2) << "Cumulating stats for dimension " <<
                    d->getName() << " with namespace: " <<
                    d->getNamespace() << std::endl;

                std::map<std::string, bool>::const_iterator exact =
                    exact_dimensions.find(d->getName());
                bool doExact(false);
                if (exact != exact_dimensions.end())
                {
                    m_log->get(logDEBUG2) << "Cumulating exact stats for "
                        "dimension " << d->getName() << std::endl;
                    doExact = true;
                }
            
                bool doSample =
                    m_options.getValueOrDefault<bool>("do_sample", false);
                stats::SummaryPtr c = boost::shared_ptr<stats::Summary>(new stats::Summary(bin_count, sample_size, stats_cache_size, seed, doExact, doSample));

                std::pair<DimensionPtr, stats::SummaryPtr> p(d,c);
                m_dimensions.push_back(d);
                m_stats.insert(p);
            }
        } else {
            bool doSample =
                m_options.getValueOrDefault<bool>("do_sample", true);
            schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();
            for (schema::index_by_index::const_iterator iter = dims.begin(); iter != dims.end(); ++iter)
            {
                DimensionPtr d = boost::shared_ptr<Dimension>(new Dimension(*iter));
                m_log->get(logDEBUG2) << "Cumulating stats for dimension " <<
                    d->getName() << " with namespace: " << d->getNamespace() <<
                    std::endl;

                std::map<std::string, bool>::const_iterator exact =
                    exact_dimensions.find(d->getName());
                bool doExact(false);
                if (exact != exact_dimensions.end())
                {
                    m_log->get(logDEBUG2) << "Cumulating exact stats for "
                        "dimension " << d->getName() << std::endl;
                    doExact = true;
                }
                stats::SummaryPtr c = boost::shared_ptr<stats::Summary>(new stats::Summary(bin_count, sample_size, stats_cache_size, seed, doExact, doSample));

                std::pair<DimensionPtr, stats::SummaryPtr> p(d,c);
                m_dimensions.push_back(d);
                m_stats.insert(p);
            }
        }

    }

}

pdal::Metadata Stats::toMetadata() const
{
    pdal::Metadata output;

    std::vector<DimensionPtr>::const_iterator p;
    boost::uint32_t position(0);
    for (p = m_dimensions.begin(); p != m_dimensions.end(); ++p)
    {
        std::multimap<DimensionPtr, stats::SummaryPtr>::const_iterator i;
        DimensionPtr d = *p;
        i = m_stats.find(d);
        if (i == m_stats.end())
            throw pdal_error("unable to find dimension in summary!");
        const stats::SummaryPtr stat = i->second;

        pdal::Metadata sub = stat->toMetadata();
        sub.setName(d->getName());
        sub.addMetadata("namespace", d->getNamespace());
        sub.addMetadata("position", position);
        output.addMetadata(sub);
        position++;
    }

    return output;
}
boost::property_tree::ptree Stats::toPTree() const
{
    boost::property_tree::ptree tree;

    std::vector<DimensionPtr>::const_iterator p;
    boost::uint32_t position(0);
    for (p = m_dimensions.begin(); p != m_dimensions.end(); ++p)
    {
        std::multimap<DimensionPtr, stats::SummaryPtr>::const_iterator i;
        DimensionPtr d = *p;
        i = m_stats.find(d);
        if (i == m_stats.end())
            throw pdal_error("unable to find dimension in summary!");
        const stats::SummaryPtr stat = i->second;

        boost::property_tree::ptree subtree = stat->toPTree();
        subtree.add("position", position);
        tree.add_child(d->getName(), subtree);
        position++;
    }

    return tree;
}

stats::Summary const& Stats::getStats(Dimension const& dim) const
{
    // FIXME: do this smarter
    std::multimap<DimensionPtr, stats::SummaryPtr>::const_iterator p;
    for (p = m_stats.begin(); p != m_stats.end(); ++p)
    {
        if (dim == *p->first)
            return *p->second;
    }
    std::ostringstream oss;
    oss <<"Dimension with name '" << dim.getName() << "' not found";
    throw pdal_error(oss.str());
}

void Stats::reset()
{
    m_stats.clear();
}

}

} // sequential

namespace random
{

Stats::Stats(const pdal::filters::Stats& filter, PointBuffer& buffer)
    : pdal::FilterRandomIterator(filter, buffer)
{
    return;
}


Stats::~Stats()
{
    return;
}

boost::uint64_t Stats::seekImpl(boost::uint64_t count)
{

    return getPrevIterator().seek(count);
}


boost::uint32_t Stats::readBufferImpl(PointBuffer& data)
{
    return getPrevIterator().read(data);
}


} // random


} // iterators



} // namespaces
