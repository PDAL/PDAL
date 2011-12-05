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

#include <pdal/PointBuffer.hpp>
#include <boost/algorithm/string.hpp>

namespace pdal { namespace filters {

//---------------------------------------------------------------------------
namespace stats {
    

boost::property_tree::ptree Collector::toPTree() const
{
    boost::property_tree::ptree tree;

    tree.put("count", count());
    tree.put("minimum", minimum());
    tree.put("maximum", maximum());
    tree.put("average", average());

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
    // const std::vector<Dimension>& dims = getSchema().getDimensions();
    // schema::index_by_index const& dims = getSchema().getDimensions().get<schema::index>();    
    // for (schema::index_by_index::const_iterator iter = dims.begin(); iter != dims.end(); ++iter)
    // {
    //     const Dimension& dim = *iter;
    //     StatsCollector* stats = m_stats[dim.getId()];
    //     delete stats;
    //     m_stats.erase(dim.getId());
    // }
}

void Stats::initialize()
{
    Filter::initialize();

    // schema::index_by_index const& dims = getSchema().getDimensions().get<schema::index>();  
    // for (schema::index_by_index::const_iterator iter = dims.begin(); iter != dims.end(); ++iter)
    // {
    //     const Dimension& dim = *iter;
    //     m_stats[dim.getId()] = new StatsCollector();
    // }

    return;
}


const Options Stats::getDefaultOptions() const
{
    Options options;
    return options;
}



    

// const StatsCollector& Stats::getStats(DimensionId::Id field) const
// {
//     const StatsCollector* s = m_stats.find(field)->second;
//     return *s;
// }


void Stats::processBuffer(PointBuffer& data) const
{
    // const boost::uint32_t numPoints = data.getNumPoints();
    // 
    // const Schema& schema = data.getSchema();
    // 
    // // BUG: fix this!
    // const int indexXi = schema.getDimensionIndex(DimensionId::X_i32);
    // const int indexYi = schema.getDimensionIndex(DimensionId::Y_i32);
    // const int indexZi = schema.getDimensionIndex(DimensionId::Z_i32);
    // const int indexXd = schema.getDimensionIndex(DimensionId::X_f64);
    // const int indexYd = schema.getDimensionIndex(DimensionId::Y_f64);
    // const int indexZd = schema.getDimensionIndex(DimensionId::Z_f64);
    // 
    // StatsCollector& statsX = (indexXi!=-1) ? *(m_stats.find(DimensionId::X_i32)->second) : *(m_stats.find(DimensionId::X_f64)->second);
    // StatsCollector& statsY = (indexYi!=-1) ? *(m_stats.find(DimensionId::Y_i32)->second) : *(m_stats.find(DimensionId::Y_f64)->second);
    // StatsCollector& statsZ = (indexZi!=-1) ? *(m_stats.find(DimensionId::Z_i32)->second) : *(m_stats.find(DimensionId::Z_f64)->second);
    // 
    // for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    // {
    //     const double x = (indexXi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexXi) : data.getField<double>(pointIndex, indexXd);
    //     const double y = (indexYi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexYi) : data.getField<double>(pointIndex, indexYd);
    //     const double z = (indexZi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexZi) : data.getField<double>(pointIndex, indexZd);
    // 
    //     statsX.insert(x);
    //     statsY.insert(y);
    //     statsZ.insert(z);
    // 
    //     data.setNumPoints(pointIndex+1);
    // }

    return;
}


pdal::StageSequentialIterator* Stats::createSequentialIterator() const
{
    return new pdal::filters::iterators::sequential::Stats(*this);
}





namespace iterators { namespace sequential {


Stats::Stats(const pdal::filters::Stats& filter)
    : pdal::FilterSequentialIterator(filter)
    , m_statsFilter(filter)
{
    return;
}


boost::uint32_t Stats::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    m_statsFilter.processBuffer(data);

    const boost::uint32_t numPoints = data.getNumPoints();
    
    
    
    // 
    // // BUG: fix this!
    // const int indexXi = schema.getDimensionIndex(DimensionId::X_i32);
    // const int indexYi = schema.getDimensionIndex(DimensionId::Y_i32);
    // const int indexZi = schema.getDimensionIndex(DimensionId::Z_i32);
    // const int indexXd = schema.getDimensionIndex(DimensionId::X_f64);
    // const int indexYd = schema.getDimensionIndex(DimensionId::Y_f64);
    // const int indexZd = schema.getDimensionIndex(DimensionId::Z_f64);
    // 
    // StatsCollector& statsX = (indexXi!=-1) ? *(m_stats.find(DimensionId::X_i32)->second) : *(m_stats.find(DimensionId::X_f64)->second);
    // StatsCollector& statsY = (indexYi!=-1) ? *(m_stats.find(DimensionId::Y_i32)->second) : *(m_stats.find(DimensionId::Y_f64)->second);
    // StatsCollector& statsZ = (indexZi!=-1) ? *(m_stats.find(DimensionId::Z_i32)->second) : *(m_stats.find(DimensionId::Z_f64)->second);
    

    for (boost::uint32_t pointIndex=0; pointIndex < numPoints; pointIndex++)
    {
        std::multimap<DimensionPtr, stats::CollectorPtr>::const_iterator p;
        for (p = m_stats.begin(); p != m_stats.end(); ++p)
        {
            
            DimensionPtr d = p->first;
            stats::CollectorPtr c = p->second;
            
            double output = getValue(data, *d, pointIndex);
            c->insert(output);
            
        }
    //     const double x = (indexXi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexXi) : data.getField<double>(pointIndex, indexXd);
    //     const double y = (indexYi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexYi) : data.getField<double>(pointIndex, indexYd);
    //     const double z = (indexZi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexZi) : data.getField<double>(pointIndex, indexZd);
    // 
    //     statsX.insert(x);
    //     statsY.insert(y);
    //     statsZ.insert(z);
    // 
    //     data.setNumPoints(pointIndex+1);
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
    
    switch (d.getDataType())
    {
        case Dimension::Float:
            flt = data.getField<float>(d, pointIndex);
            output = static_cast<double>(flt);
            break;
        case Dimension::Double:
            output = data.getField<double>(d, pointIndex);
            break;
        
        case Dimension::Int8:
            i8 = data.getField<boost::int8_t>(d, pointIndex);
            output = d.applyScaling<boost::int8_t>(i8);
            break;
        case Dimension::Uint8:
            u8 = data.getField<boost::uint8_t>(d, pointIndex);
            output = d.applyScaling<boost::uint8_t>(u8);
            break;
        case Dimension::Int16:
            i16 = data.getField<boost::int16_t>(d, pointIndex);
            output = d.applyScaling<boost::int16_t>(i16);
            break;
        case Dimension::Uint16:
            u16 = data.getField<boost::uint16_t>(d, pointIndex);
            output = d.applyScaling<boost::uint16_t>(u16);
            break;
        case Dimension::Int32:
            i32 = data.getField<boost::int32_t>(d, pointIndex);
            output = d.applyScaling<boost::int32_t>(i32);
            break;
        case Dimension::Uint32:
            u32 = data.getField<boost::uint32_t>(d, pointIndex);
            output = d.applyScaling<boost::uint32_t>(u32);
            break;
        case Dimension::Int64:
            i64 = data.getField<boost::int64_t>(d, pointIndex);
            output = d.applyScaling<boost::int64_t>(i64);
            break;
        case Dimension::Uint64:
            u64 = data.getField<boost::uint64_t>(d, pointIndex);
            output = d.applyScaling<boost::uint64_t>(u64);
            break;
        case Dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case Dimension::Undefined:
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
    Schema const& schema = buffer.getSchema();
    
    if (m_stats.size() == 0)
    {
        schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();  
        for (schema::index_by_index::const_iterator iter = dims.begin(); iter != dims.end(); ++iter)
        {
            DimensionPtr d = boost::shared_ptr<Dimension>(new Dimension( *iter));
            stats::CollectorPtr c = boost::shared_ptr<stats::Collector>(new stats::Collector);
        
            std::pair<DimensionPtr, stats::CollectorPtr> p(d,c);
            m_stats.insert(p);
        }
        
    }

} 

boost::property_tree::ptree Stats::toPTree() const
{
    boost::property_tree::ptree tree;

    std::multimap<DimensionPtr, stats::CollectorPtr>::const_iterator p;
    for (p = m_stats.begin(); p != m_stats.end(); ++p)
    {
        
        const stats::CollectorPtr stat = p->second;
        boost::property_tree::ptree subtree = stat->toPTree();
    
        tree.add_child(p->first->getName(), subtree);

    }

    return tree;
}

stats::Collector const& Stats::getStats(Dimension const& dim) const
{
    // FIXME: do this smarter
    std::multimap<DimensionPtr, stats::CollectorPtr>::const_iterator p;
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
