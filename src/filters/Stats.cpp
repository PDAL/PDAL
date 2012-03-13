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

namespace pdal { namespace filters {

//---------------------------------------------------------------------------
namespace stats {
    

boost::property_tree::ptree Summary::toPTree() const
{
    boost::property_tree::ptree tree;
    
    boost::uint32_t cnt = static_cast<boost::uint32_t>(count());
    tree.put("count", cnt);
    tree.put("minimum", minimum());
    tree.put("maximum", maximum());
    tree.put("average", average());
    
    boost::property_tree::ptree bins;
    histogram_type hist = histogram();
    for(boost::int32_t i = 0; i < hist.size(); ++i)
    {
        boost::property_tree::ptree bin;
        bin.add("lower_bound", hist[i].first);
        bin.add("count", Utils::sround(hist[i].second*cnt));
        std::ostringstream binname;
        binname << "bin-" <<i;
        bins.add_child(binname.str(), bin );
    }
    tree.add_child("histogram", bins);
    
    std::ostringstream sample;
    for(std::vector<double>::size_type i =0; i < m_sample.size(); ++i)
    {
        sample << m_sample[i] << " ";
    };
    
    tree.add("sample", sample.str());
    return tree;
}


} // namespace stats


//---------------------------------------------------------------------------


Stats::Stats(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    return;
}


Stats::Stats(Stage& prevStage)
    : Filter(prevStage, Options::none())
{
    return;
}


Stats::~Stats()
{

}

void Stats::initialize()
{
    Filter::initialize();
    return;
}


const Options Stats::getDefaultOptions() const
{
    Options options;
    Option sample_size("sample_size", 1000, "Number of points to return for uniform random 'sample'");
    Option num_bins("num_bins", 20, "Number of bins to use for histogram");
    Option stats_cache_size("stats_cache_size", 1000, "Number of points to use for histogram bin determination. Defaults to total number of points read if no option is specified.");
    Option seed("seed", 0, "Seed to use for repeatable random sample. A seed value of 0 means no seed is used");
    
    options.add(sample_size);
    options.add(num_bins);
    options.add(stats_cache_size);
    options.add(seed);
    return options;
}

pdal::StageSequentialIterator* Stats::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Stats(*this, buffer);
}

namespace iterators { namespace sequential {

Stats::Stats(const pdal::filters::Stats& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_statsFilter(filter)
{
    return;
}


boost::uint32_t Stats::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    const boost::uint32_t numPoints = data.getNumPoints();
    
    for (boost::uint32_t pointIndex=0; pointIndex < numPoints; pointIndex++)
    {
        std::multimap<DimensionPtr, stats::SummaryPtr>::const_iterator p;
        for (p = m_stats.begin(); p != m_stats.end(); ++p)
        {
            
            DimensionPtr d = p->first;
            stats::SummaryPtr c = p->second;
            
            double output = getValue(data, *d, pointIndex);
            c->insert(output);
        }
    }
    return numRead;
}

double Stats::getValue(PointBuffer& data, Dimension& d, boost::uint32_t pointIndex)
{
        double output(0.0);
        
    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);
    
    switch (d.getInterpretation())
    {
        case dimension::SignedByte:
            i8 = data.getField<boost::int8_t>(d, pointIndex);
            output = d.applyScaling<boost::int8_t>(i8);
            break;
        case dimension::UnsignedByte:
            u8 = data.getField<boost::uint8_t>(d, pointIndex);
            output = d.applyScaling<boost::uint8_t>(u8);
            break;
        case dimension::Float:
            if (d.getByteSize() == 4)
            {
                flt = data.getField<float>(d, pointIndex);
                output = static_cast<double>(flt);
                break;
            } else if (d.getByteSize() == 8)
            {
                output = data.getField<double>(d, pointIndex);
                break;
            }
            else 
            {
                std::ostringstream oss;
                oss << "Unable to interpret Float of size '" << d.getByteSize() <<"'";
                throw pdal_error(oss.str());
            }
            
        case dimension::SignedInteger:
            if (d.getByteSize() == 1)
            {
                i8 = data.getField<boost::int8_t>(d, pointIndex);
                output = d.applyScaling<boost::int8_t>(i8);
                break;
            } else if (d.getByteSize() == 2)
            {
                i16 = data.getField<boost::int16_t>(d, pointIndex);
                output = d.applyScaling<boost::int16_t>(i16);
                break;
            } else if (d.getByteSize() == 4)
            {
                i32 = data.getField<boost::int32_t>(d, pointIndex);
                output = d.applyScaling<boost::int32_t>(i32);
                break;
            } else if (d.getByteSize() == 8)
            {
                i64 = data.getField<boost::int64_t>(d, pointIndex);
                output = d.applyScaling<boost::int64_t>(i64);
                break;
            }
            else 
            {
                std::ostringstream oss;
                oss << "Unable to interpret SignedInteger of size '" << d.getByteSize() <<"'";
                throw pdal_error(oss.str());
            }
        case dimension::UnsignedInteger:
            if (d.getByteSize() == 1)
            {
                u8 = data.getField<boost::uint8_t>(d, pointIndex);
                output = d.applyScaling<boost::uint8_t>(u8);
                break;
            } else if (d.getByteSize() == 2)
            {
                u16 = data.getField<boost::uint16_t>(d, pointIndex);
                output = d.applyScaling<boost::uint16_t>(u16);
                break;
            } else if (d.getByteSize() == 4)
            {
                u32 = data.getField<boost::uint32_t>(d, pointIndex);
                output = d.applyScaling<boost::uint32_t>(u32);
                break;
            } else if (d.getByteSize() == 8)
            {
                u64 = data.getField<boost::uint64_t>(d, pointIndex);
                output = d.applyScaling<boost::uint64_t>(u64);
                break;
            }
            else 
            {
                std::ostringstream oss;
                oss << "Unable to interpret UnsignedInteger of size '" << d.getByteSize() <<"'";
                throw pdal_error(oss.str());
            }

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be summarized");
    }    
    
    return output;
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

void Stats::readBufferBeginImpl(PointBuffer& buffer)
{
    // We'll assume you're not changing the schema per-read call

    if (m_stats.size() == 0)
    {
        Schema const& schema = buffer.getSchema();
    
        boost::uint64_t numPoints = getStage().getPrevStage().getNumPoints(); 
        boost::uint32_t stats_cache_size(1000);
   

        try 
        {
            stats_cache_size = getStage().getOptions().getValueOrThrow<boost::uint32_t>("stats_cache_size");
            getStage().log()->get(logDEBUG2) << "Using " << stats_cache_size << "for histogram cache size set from option" << std::endl;

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
                getStage().log()->get(logDEBUG2) << "Using point count, " << numPoints << ", for histogram cache size" << std::endl;

            }
        }

        boost::uint32_t sample_size = getStage().getOptions().getValueOrDefault<boost::uint32_t>("sample_size", 1000);
        boost::uint32_t seed = getStage().getOptions().getValueOrDefault<boost::uint32_t>("seed", 0);

        getStage().log()->get(logDEBUG2) << "Using " << sample_size << " for sample size" << std::endl;
        getStage().log()->get(logDEBUG2) << "Using " << seed << " for sample seed" << std::endl;
    
    
        boost::uint32_t bin_count = getStage().getOptions().getValueOrDefault<boost::uint32_t>("num_bins", 20);

        schema::index_by_index const& dims = schema.getDimensions().get<schema::index>(); 
        
        for (schema::index_by_index::const_iterator iter = dims.begin(); iter != dims.end(); ++iter)
        {
            DimensionPtr d = boost::shared_ptr<Dimension>(new Dimension( *iter));
            getStage().log()->get(logDEBUG2) << "Cumulating stats for dimension " << d->getName() << std::endl;
            stats::SummaryPtr c = boost::shared_ptr<stats::Summary>(new stats::Summary(bin_count, sample_size, stats_cache_size));
        
            std::pair<DimensionPtr, stats::SummaryPtr> p(d,c);
            m_dimensions.push_back(d);
            m_stats.insert(p);
        }
        
    }

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

} } // iterators::sequential

} } // namespaces
