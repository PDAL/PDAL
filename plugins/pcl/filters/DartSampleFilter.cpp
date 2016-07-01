/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "DartSampleFilter.hpp"

#include "dart_sample.h"
#include "PCLConversions.hpp"
#include "PCLPipeline.h"

#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.dartsample", "Dart sample filter",
               "http://pdal.io/stages/filters.dartsample.html");

CREATE_SHARED_PLUGIN(1, 0, DartSampleFilter, Filter, s_info)

std::string DartSampleFilter::getName() const
{
    return s_info.name;
}

void DartSampleFilter::addArgs(ProgramArgs& args)
{
    args.add("radius", "Minimum distance criterion", m_radius, 1.0);
}

PointViewSet DartSampleFilter::run(PointViewPtr input)
{
    PointViewSet viewSet;
    PointViewPtr output = input->makeNew();

    log()->floatPrecision(2);
    log()->get(LogLevel::Info) << "DartSampleFilter (radius="
                               << m_radius << ")\n";

    BOX3D buffer_bounds;
    input->calculateBounds(buffer_bounds);

    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    Cloud::Ptr cloud(new Cloud);
    pclsupport::PDALtoPCD(input, *cloud, buffer_bounds);

    pclsupport::setLogLevel(log()->getLevel());

    pcl::DartSample<pcl::PointXYZ> ds;
    ds.setInputCloud(cloud);
    ds.setRadius(m_radius);

    std::vector<int> samples;
    ds.filter(samples);

    if (samples.empty())
    {
        log()->get(LogLevel::Warning) << "Filtered cloud has no points!\n";
        return viewSet;
    }

    for (const auto& i : samples)
        output->appendPoint(*input, i);

    double frac = (double)samples.size() / (double)cloud->size();
    log()->get(LogLevel::Info) << "Retaining " << samples.size() << " of "
                               << cloud->size() << " points ("
                               <<  100*frac
                               << "%)\n";

    viewSet.insert(output);
    return viewSet;
}

} // namespace pdal
