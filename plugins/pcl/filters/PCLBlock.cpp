/******************************************************************************
* Copyright (c) 2013-2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "PCLBlock.hpp"

#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "../PCLConversions.hpp"
#include "../pipeline/PCLPipeline.h"

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.pclblock", "PCL Block implementation",
               "http://pdal.io/stages/filters.pclblock.html");

CREATE_SHARED_PLUGIN(1, 0, PCLBlock, Filter, s_info)

std::string PCLBlock::getName() const
{
    return s_info.name;
}

void PCLBlock::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename);
    args.add("methods", "methods", m_methods);
}

PointViewSet PCLBlock::run(PointViewPtr input)
{
    using namespace Dimension;

    PointViewPtr output = input->makeNew();
    PointViewSet viewSet;
    viewSet.insert(output);

    log()->get(LogLevel::Debug2) << "Process PCLBlock..." << std::endl;

    BOX3D bounds;
    input->calculateBounds(bounds);

    // convert PointView to PointNormal
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    Cloud::Ptr cloud(new Cloud);
    pclsupport::PDALtoPCD(input, *cloud, bounds);

    pclsupport::setLogLevel(log()->getLevel());

    pcl::Pipeline<pcl::PointXYZ> pipeline;
    pipeline.setInputCloud(cloud);
    if (!m_filename.empty())
        pipeline.setFilename(m_filename);
    else if (!m_methods.empty())
        pipeline.setMethods(m_methods);
    else
        throwError("No PCL pipeline specified.");
    // PDALtoPCD subtracts min values in each XYZ dimension to prevent rounding
    // errors in conversion to float. These offsets need to be conveyed to the
    // pipeline to offset any bounds entered as part of a PassThrough filter.
    pipeline.setOffsets(bounds.minx, bounds.miny, bounds.minz);

    // create PointCloud for results
    Cloud::Ptr cloud_f(new Cloud);
    pipeline.filter(*cloud_f);

    if (cloud_f->points.empty())
    {
        log()->get(LogLevel::Debug2) << "Filtered cloud has no points!\n";
        return viewSet;
    }

    pclsupport::PCDtoPDAL(*cloud_f, output, bounds);

    return viewSet;
}

} // namespace pdal
