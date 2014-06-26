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

#include <pdal/PCLConversions.hpp>
#include <pdal/filters/PCLBlock.hpp>

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

void PCLBlock::processOptions(const Options& options)
{
    m_filename = options.getValueOrThrow<std::string>("filename");
}


void PCLBlock::ready(PointContext ctx)
{
    m_iDim = ctx.schema()->getDimensionPtr("Intensity");
    m_xDim = ctx.schema()->getDimensionPtr("X");
    m_yDim = ctx.schema()->getDimensionPtr("Y");
    m_zDim = ctx.schema()->getDimensionPtr("Z");
}


PointBufferSet PCLBlock::run(PointBufferPtr input)
{
    PointBufferPtr output(new PointBuffer(input->context()));
    PointBufferSet pbSet;
    pbSet.insert(output);

#ifdef PDAL_HAVE_PCL
    bool logOutput = log()->getLevel() > logDEBUG1;
    if (logOutput)
        log()->floatPrecision(8);

    log()->get(logDEBUG2) << input->getFieldAs<double>(*m_xDim, 0) << ", " <<
        input->getFieldAs<double>(*m_yDim, 0) << ", " <<
        input->getFieldAs<double>(*m_zDim, 0) << std::endl;
    log()->get(logDEBUG2) << "Process PCLBlock..." << std::endl;

    // convert PointBuffer to PointNormal
    typedef pcl::PointCloud<pcl::PointNormal> Cloud;
    Cloud::Ptr cloud(new Cloud);
    PDALtoPCD(*input, *cloud, m_xDim, m_yDim, m_zDim, m_iDim);

    log()->get(logDEBUG2) << cloud->points[0].x << ", " <<
        cloud->points[0].y << ", " << cloud->points[0].z << std::endl;

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
    pipeline.setFilename(m_filename);
    pipeline.setOffsets(m_xDim->getNumericOffset(), m_yDim->getNumericOffset(),
        m_zDim->getNumericOffset());

    // create PointCloud for results
    Cloud::Ptr cloud_f(new Cloud);
    pipeline.filter(*cloud_f);

    if (cloud_f->points.empty())
    {
        log()->get(logDEBUG2) << "Filtered cloud has no points!" << std::endl;
        return pbSet;
    }

    PCDtoPDAL(*cloud_f, *output, m_xDim, m_yDim, m_zDim, m_iDim);

    log()->get(logDEBUG2) << cloud->points.size() << " before, " <<
        cloud_f->points.size() << " after" << std::endl;
    log()->get(logDEBUG2) << output->size() << std::endl;
    log()->get(logDEBUG2) << output->getFieldAs<double>(*m_xDim, 0) << ", " <<
        output->getFieldAs<double>(*m_yDim, 0) << ", " << 
        output->getFieldAs<double>(*m_zDim, 0) << std::endl;
#endif
    return pbSet;
}

} // filters
} // pdal

