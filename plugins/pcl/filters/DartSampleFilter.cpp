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

Options DartSampleFilter::getDefaultOptions()
{
    Options options;
    options.add("radius", 1.0, "Minimum distance criterion");
    return options;
}

void DartSampleFilter::processOptions(const Options& options)
{
    m_radius = options.getValueOrDefault<double>("radius", 1.0);
}

PointViewSet DartSampleFilter::run(PointViewPtr input)
{
    PointViewPtr output = input->makeNew();
    PointViewSet viewSet;
    viewSet.insert(output);

    log()->floatPrecision(2);
    log()->get(LogLevel::Info) << "DartSampleFilter (radius="
                               << m_radius << ")\n";

    BOX3D buffer_bounds;
    input->calculateBounds(buffer_bounds);

    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    Cloud::Ptr cloud(new Cloud);
    pclsupport::PDALtoPCD(input, *cloud, buffer_bounds);

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

    return viewSet;
}

} // namespace pdal
