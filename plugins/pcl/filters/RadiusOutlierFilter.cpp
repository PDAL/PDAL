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

#include "RadiusOutlierFilter.hpp"

#include "PCLConversions.hpp"

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.radiusoutlier", "Radius outlier removal",
               "http://pdal.io/stages/filters.radiusoutlier.html");

CREATE_SHARED_PLUGIN(1, 0, RadiusOutlierFilter, Filter, s_info)

std::string RadiusOutlierFilter::getName() const
{
    return s_info.name;
}

Options RadiusOutlierFilter::getDefaultOptions()
{
    Options options;
    options.add("min_neighbors", 2, "Minimum number of neighbors in radius");
    options.add("radius", 1, "Radius");
    options.add("classify", true, "Apply classification labels?");
    options.add("extract", false, "Extract ground returns?");
    return options;
}

void RadiusOutlierFilter::processOptions(const Options& options)
{
    m_min_neighbors = options.getValueOrDefault<int>("min_neighbors", 2);
    m_radius = options.getValueOrDefault<double>("radius", 1);
    m_classify = options.getValueOrDefault<bool>("classify", true);
    m_extract = options.getValueOrDefault<bool>("extract", false);
}

void RadiusOutlierFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}

PointViewSet RadiusOutlierFilter::run(PointViewPtr input)
{
    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);
    log()->get(LogLevel::Debug2) << "Process RadiusOutlierFilter...\n";

    // convert PointView to PointXYZ
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    Cloud::Ptr cloud(new Cloud);
    BOX3D bounds;
    input->calculateBounds(bounds);
    pclsupport::PDALtoPCD(input, *cloud, bounds);

    // PCL should provide console output at similar verbosity level as PDAL
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

    // setup the outlier filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror(true);
    ror.setInputCloud(cloud);
    ror.setMinNeighborsInRadius(m_min_neighbors);
    ror.setRadiusSearch(m_radius);

    pcl::PointCloud<pcl::PointXYZ> output;
    ror.setNegative(true);
    ror.filter(output);

    // filtered to return inliers
    pcl::PointIndicesPtr inliers(new pcl::PointIndices);
    ror.getRemovedIndices(*inliers);

    // inverse are the outliers
    std::vector<int> outliers(input->size()-inliers->indices.size());
    for (PointId i = 0, j = 0, k = 0; i < input->size(); ++i)
    {
        if (i == (PointId)inliers->indices[j])
        {
            j++;
            continue;
        }
        outliers[k++] = i;
    }

    PointViewSet viewSet;
    if (!outliers.empty() && (m_classify || m_extract))
    {

        if (m_classify)
        {
            log()->get(LogLevel::Debug2) << "Labeled " << outliers.size() << " outliers as noise!\n";

            // set the classification label of outlier returns as 18
            // (corresponding to ASPRS LAS specification for high noise)
            for (const auto& i : outliers)
            {
                input->setField(Dimension::Id::Classification, i, 18);
            }

            viewSet.insert(input);
        }

        if (m_extract)
        {
            log()->get(LogLevel::Debug2) << "Extracted " << inliers->indices.size() << " inliers!\n";

            // create new PointView containing only outliers
            PointViewPtr output = input->makeNew();
            for (const auto& i : inliers->indices)
            {
                output->appendPoint(*input, i);
            }

            viewSet.erase(input);
            viewSet.insert(output);
        }
    }
    else
    {
        if (outliers.empty())
            log()->get(LogLevel::Debug2) << "Filtered cloud has no outliers!\n";

        if (!(m_classify || m_extract))
            log()->get(LogLevel::Debug2) << "Must choose --classify or --extract\n";

        // return the input buffer unchanged
        viewSet.insert(input);
    }

    return viewSet;
}

} // namespace pdal
