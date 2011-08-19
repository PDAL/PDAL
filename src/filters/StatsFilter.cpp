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

#include <pdal/filters/StatsFilter.hpp>

#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>
#include <pdal/exceptions.hpp>
#include <pdal/Color.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/StatsFilterIterator.hpp>

namespace pdal { namespace filters {

//---------------------------------------------------------------------------

StatsCollector::StatsCollector()
    : m_count(0)
    , m_minimum(0.0)
    , m_maximum(0.0)
    , m_sum(0.0)
{
    return;
}


void StatsCollector::reset()
{
    m_count = 0;
    m_minimum = 0.0;
    m_maximum = 0.0;
    m_sum = 0.0;
    return;
}


void StatsCollector::insert(double value)
{
    if (m_count==0)
    {
        m_minimum = value;
        m_maximum = value;
        m_sum = value;
    }
    else
    {
        m_minimum = std::min(m_minimum, value);
        m_maximum = std::max(m_maximum, value);
        m_sum += value;
    }

    ++m_count;

    return;
}


boost::property_tree::ptree StatsCollector::toPTree() const
{
    boost::property_tree::ptree tree;

    tree.put("count", count());
    tree.put("minimum", minimum());
    tree.put("maximum", maximum());
    tree.put("average", average());

    return tree;
}


//---------------------------------------------------------------------------


StatsFilter::StatsFilter(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    return;
}


StatsFilter::StatsFilter(Stage& prevStage)
    : Filter(prevStage, Options::none())
{
    return;
}


StatsFilter::~StatsFilter()
{
    Schema::Dimensions dims = getSchema().getDimensions();
    for (Schema::DimensionsIter iter = dims.begin(); iter != dims.end(); ++iter)
    {
        const Dimension& dim = *iter;
        StatsCollector* stats = m_stats[dim.getField()];
        delete stats;
        m_stats.erase(dim.getField());
    }
}

void StatsFilter::initialize()
{
    Filter::initialize();

    const Schema::Dimensions dims = getSchema().getDimensions();
    for (Schema::DimensionsCIter iter = dims.begin(); iter != dims.end(); ++iter)
    {
        const Dimension& dim = *iter;
        m_stats[dim.getField()] = new StatsCollector();
    }

    return;
}


const Options StatsFilter::getDefaultOptions() const
{
    Options options;
    return options;
}


void StatsFilter::reset()
{
    const Schema::Dimensions dims = getSchema().getDimensions();
    for (Schema::DimensionsCIter iter = dims.begin(); iter != dims.end(); ++iter)
    {
        const Dimension& dim = *iter;
        m_stats[dim.getField()]->reset();
    }
}
    

const StatsCollector& StatsFilter::getStats(Dimension::Field field) const
{
    const StatsCollector* s = m_stats.find(field)->second;
    return *s;
}


void StatsFilter::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    // BUG: fix this!
    const int indexXi = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int indexYi = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int indexZi = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const int indexXd = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    const int indexYd = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    const int indexZd = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);

    StatsCollector& statsX = *(m_stats.find(Dimension::Field_X)->second);
    StatsCollector& statsY = *(m_stats.find(Dimension::Field_Y)->second);
    StatsCollector& statsZ = *(m_stats.find(Dimension::Field_Z)->second);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        const double x = (indexXi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexXi) : data.getField<double>(pointIndex, indexXd);
        const double y = (indexYi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexYi) : data.getField<double>(pointIndex, indexYd);
        const double z = (indexZi!=-1) ? data.getField<boost::int32_t>(pointIndex, indexZi) : data.getField<double>(pointIndex, indexZd);

        statsX.insert(x);
        statsY.insert(y);
        statsZ.insert(z);

        data.setNumPoints(pointIndex+1);
    }

    return;
}


pdal::StageSequentialIterator* StatsFilter::createSequentialIterator() const
{
    return new StatsFilterSequentialIterator(*this);
}


boost::property_tree::ptree StatsFilter::toStatsPTree() const
{
    boost::property_tree::ptree tree;

    for (std::map<Dimension::Field, StatsCollector*>::const_iterator iter = m_stats.cbegin();
         iter != m_stats.cend();
         ++iter)
    {
        const StatsCollector* stat = iter->second;
        boost::property_tree::ptree subtree = stat->toPTree();

        // BUG: make this work for all fields
        if (iter->first == Dimension::Field_X || iter->first == Dimension::Field_Y ||iter->first == Dimension::Field_Z)
        {
            const std::string key = Dimension::getFieldName(iter->first);
            tree.add_child(key, subtree);
        }
    }

    return tree;
}


} } // namespaces
