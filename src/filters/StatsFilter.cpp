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


void StatsFilter::initialize()
{
    Filter::initialize();

    reset();

    return;
}


const Options StatsFilter::getDefaultOptions() const
{
    Options options;
    return options;
}


void StatsFilter::reset()
{
    m_totalPoints = 0;

    m_minimumX = 0.0;
    m_minimumY = 0.0;
    m_minimumZ = 0.0;
    
    m_maximumX = 0.0;
    m_maximumY = 0.0;
    m_maximumZ = 0.0;
    
    m_sumX = 0.0;
    m_sumY = 0;
    m_sumZ = 0;

    return;
}


void StatsFilter::getData(boost::uint64_t& count, 
                          double& minx, double& miny, double& minz, 
                          double& maxx, double& maxy, double& maxz,
                          double& avgx, double& avgy, double& avgz) const
{
    minx = m_minimumX;
    miny = m_minimumY;
    minz = m_minimumZ;
    maxx = m_maximumX;
    maxy = m_maximumY;
    maxz = m_maximumZ;

    avgx = m_sumX / (double)m_totalPoints;
    avgy = m_sumY / (double)m_totalPoints;
    avgz = m_sumZ / (double)m_totalPoints;

    count = m_totalPoints;

    return;
}


void StatsFilter::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        const double x = data.getField<double>(pointIndex, indexX);
        const double y = data.getField<double>(pointIndex, indexY);
        const double z = data.getField<double>(pointIndex, indexZ);

        if (m_totalPoints==0)
        {
            m_minimumX = x;
            m_minimumY = y;
            m_minimumZ = z;

            m_maximumX = x;
            m_maximumY = y;
            m_maximumZ = z;

            m_sumX = x;
            m_sumY = y;
            m_sumZ = z;
        }
        else
        {
            m_minimumX = std::min(m_minimumX, x);
            m_minimumY = std::min(m_minimumY, y);
            m_minimumZ = std::min(m_minimumZ, z);

            m_maximumX = std::max(m_maximumX, x);
            m_maximumY = std::max(m_maximumY, y);
            m_maximumZ = std::max(m_maximumZ, z);

            m_sumX += x;
            m_sumY += y;
            m_sumZ += z;
        }

        ++m_totalPoints;

        data.setNumPoints(pointIndex+1);
    }

    return;
}


pdal::StageSequentialIterator* StatsFilter::createSequentialIterator() const
{
    return new StatsFilterSequentialIterator(*this);
}

} } // namespaces
