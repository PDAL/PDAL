/******************************************************************************
* Copyright (c) 2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <iostream>

#include <pdal/PCLConversions.hpp>
#include <pdal/filters/PCLBlock.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <pdal/PointBuffer.hpp>
#include <pdal/GlobalEnvironment.hpp>

#ifdef PDAL_HAVE_PCL
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/pipeline/pipeline.h>
#include <pcl/io/pcd_io.h>
#endif

namespace pdal
{
namespace filters
{

/** \brief This method processes the PointBuffer through the given pipeline. */
boost::uint32_t
PCLBlock::processBuffer(PointBuffer& srcData, std::string& filename, PointBuffer& dstData) const
{
    boost::uint32_t numPointsAfterFiltering = srcData.getNumPoints();

#ifdef PDAL_HAVE_PCL
    bool logOutput = log()->getLevel() > logDEBUG1;
    if (logOutput)
        log()->floatPrecision(8);

    Schema const& schema = srcData.getSchema();

    Dimension const& dX = schema.getDimension("X");
    Dimension const& dY = schema.getDimension("Y");
    Dimension const& dZ = schema.getDimension("Z");

    log()->get(logDEBUG2) << srcData.applyScaling(dX, 0) << ", " << srcData.applyScaling(dY, 0) << ", " << srcData.applyScaling(dZ, 0) << std::endl;

    log()->get(logDEBUG2) << "Process PCLBlock..." << std::endl;

    // convert PointBuffer to PointNormal
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pdal::PDALtoPCD(srcData, *cloud);

    log()->get(logDEBUG2) << cloud->points[0].x << ", " << cloud->points[0].y << ", " << cloud->points[0].z << std::endl;

    // create PointCloud for results
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_f(new pcl::PointCloud<pcl::PointNormal>);

    int level = log()->getLevel();
    switch (level)
    {
        case 0:
            pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
            break;

        case 1:
            pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
            break;

        case 2:
            pcl::console::setVerbosityLevel(pcl::console::L_WARN);
            break;

        case 3:
            pcl::console::setVerbosityLevel(pcl::console::L_INFO);
            break;

        case 4:
            pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
            break;

        default:
            pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
            break;
    }

    pcl::Pipeline<pcl::PointNormal> pipeline;
    pipeline.setInputCloud(cloud);
    pipeline.setFilename(filename);
    pipeline.setOffsets(dX.getNumericOffset(), dY.getNumericOffset(), dZ.getNumericOffset());
    pipeline.filter(*cloud_f);

    if (cloud_f->points.size() > 0)
    {
        pdal::PCDtoPDAL(*cloud_f, dstData);

        log()->get(logDEBUG2) << cloud->points.size() << " before, " << cloud_f->points.size() << " after" << std::endl;

        log()->get(logDEBUG2) << dstData.getNumPoints() << std::endl;

        log()->get(logDEBUG2) << dstData.applyScaling(dX, 0) << ", " << dstData.applyScaling(dY, 0) << ", " << dstData.applyScaling(dZ, 0) << std::endl;
    }
    else
    {
        log()->get(logDEBUG2) << "Filtered cloud has no points!" << std::endl;
    }

    numPointsAfterFiltering = cloud_f->points.size();
#endif

    return numPointsAfterFiltering;
}


pdal::StageSequentialIterator* PCLBlock::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::PCLBlock(*this, buffer);
}


namespace iterators
{
namespace sequential
{


PCLBlock::PCLBlock(const pdal::filters::PCLBlock& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
{
    return;
}

boost::uint32_t PCLBlock::readBufferImpl(PointBuffer& buffer)
{
#ifdef PDAL_HAVE_PCL
    std::string filename =
        m_pclblockFilter.getOptions().getValueOrThrow<std::string>("filename");
    m_pclblockFilter.log()->get(logDEBUG2) << "Using " << filename << std::endl;

    boost::uint32_t originalCapacity = buffer.getCapacity();
    boost::int64_t numPointsNeeded = static_cast<boost::int64_t>(buffer.getCapacity());

    PointBuffer outputData(buffer.getSchema(), originalCapacity);
    PointBuffer tmpData(buffer.getSchema(), originalCapacity);

    m_pclblockFilter.log()->get(logDEBUG2) << "Fetching for block of size: " << numPointsNeeded << std::endl;

    while (numPointsNeeded > 0)
    {
        if (getPrevIterator().atEnd())
        {
            m_pclblockFilter.log()->get(logDEBUG2) << "previous iterator is .atEnd, stopping"
                                                   << std::endl;
            break;
        }

        buffer.resize(static_cast<boost::uint32_t>(numPointsNeeded));

        const boost::uint32_t numSrcPointsRead = getPrevIterator().read(buffer);

        m_pclblockFilter.log()->get(logDEBUG3) << "Fetched "
                                               << numSrcPointsRead << " from previous iterator."
                                               << std::endl;

        assert(numSrcPointsRead <= numPointsNeeded);

        const boost::uint32_t numPointsProcessed = m_pclblockFilter.processBuffer(buffer, filename, tmpData);
        m_pclblockFilter.log()->get(logDEBUG3) << "Processed " << numPointsProcessed << " in PCL block filter" << std::endl;

        m_pclblockFilter.log()->get(logDEBUG3) << tmpData.getNumPoints() << " passed PCL block filter" << std::endl;

        if (tmpData.getNumPoints() > 0)
        {
            outputData.copyPointsFast(outputData.getNumPoints(), 0, tmpData, tmpData.getNumPoints());
            outputData.setNumPoints(outputData.getNumPoints() + tmpData.getNumPoints());
        }

        numPointsNeeded -= numSrcPointsRead;
        m_pclblockFilter.log()->get(logDEBUG3) << numPointsNeeded << " left to read this block" << std::endl;

    }

    const boost::uint32_t numPointsAchieved = outputData.getNumPoints();

    buffer.resize(originalCapacity);
    buffer.setNumPoints(0);
    buffer.copyPointsFast(0, 0, outputData, outputData.getNumPoints());
    buffer.setNumPoints(outputData.getNumPoints());
    m_pclblockFilter.log()->get(logDEBUG2) << "Copying " << outputData.getNumPoints() << " at end of readBufferImpl" << std::endl;

    return numPointsAchieved;
#else
    (void)m_pclblockFilter;
    return buffer.getNumPoints();
#endif
}


boost::uint64_t PCLBlock::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool PCLBlock::atEndImpl() const
{
    return getPrevIterator().atEnd();
}


} // sequential
} // iterators

} // filters
} // pdal

