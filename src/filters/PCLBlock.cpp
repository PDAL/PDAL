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

#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/pipeline/pipeline.h>
#include <pcl/io/pcd_io.h>

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

}


PointBufferSet PCLBlock::run(PointBufferPtr input)
{
    PointBufferPtr output = input->makeNew();
    PointBufferSet pbSet;
    pbSet.insert(output);

    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);

    log()->get(LogLevel::Debug2) << input->getFieldAs<double>(Dimension::Id::X, 0) << ", " <<
        input->getFieldAs<double>(Dimension::Id::Y, 0) << ", " <<
        input->getFieldAs<double>(Dimension::Id::Z, 0) << std::endl;
    log()->get(LogLevel::Debug2) << "Process PCLBlock..." << std::endl;

    Bounds<double> const& buffer_bounds = input->calculateBounds();

    // convert PointBuffer to PointNormal
    typedef pcl::PointCloud<pcl::PointNormal> Cloud;
    Cloud::Ptr cloud(new Cloud);
    PDALtoPCD(*input, *cloud, buffer_bounds);

    log()->get(LogLevel::Debug2) << cloud->points[0].x << ", " <<
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
    // PDALtoPCD subtracts min values in each XYZ dimension to prevent rounding
    // errors in conversion to float. These offsets need to be conveyed to the
    // pipeline to offset any bounds entered as part of a PassThrough filter.
    pipeline.setOffsets(buffer_bounds.getMinimum(0), buffer_bounds.getMinimum(1), buffer_bounds.getMinimum(2));

    // create PointCloud for results
    Cloud::Ptr cloud_f(new Cloud);
    pipeline.filter(*cloud_f);

    if (cloud_f->points.empty())
    {
        log()->get(LogLevel::Debug2) << "Filtered cloud has no points!" << std::endl;
        return pbSet;
    }

    PCDtoPDAL(*cloud_f, *output, buffer_bounds);

    log()->get(LogLevel::Debug2) << cloud->points.size() << " before, " <<
        cloud_f->points.size() << " after" << std::endl;
    log()->get(LogLevel::Debug2) << output->size() << std::endl;
    log()->get(LogLevel::Debug2) << output->getFieldAs<double>(Dimension::Id::X, 0) << ", " <<
        output->getFieldAs<double>(Dimension::Id::Y, 0) << ", " << 
        output->getFieldAs<double>(Dimension::Id::Z, 0) << std::endl;
    return pbSet;
}

} // filters
} // pdal

