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

#include "GroundFilter.hpp"

#include "PCLConversions.hpp"

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.ground",
    "Progressive morphological filter",
    "http://pdal.io/stages/filters.ground.html" );

CREATE_SHARED_PLUGIN(1, 0, GroundFilter, Filter, s_info)

std::string GroundFilter::getName() const { return s_info.name; }

void GroundFilter::processOptions(const Options& options)
{
    m_maxWindowSize = options.getValueOrDefault<double>("maxWindowSize", 33);
    m_slope = options.getValueOrDefault<double>("slope", 1);
    m_maxDistance = options.getValueOrDefault<double>("maxDistance", 2.5);
    m_initialDistance = options.getValueOrDefault<double>("initialDistance", 0.15);
    m_cellSize = options.getValueOrDefault<double>("cellSize", 1);
    m_classify = options.getValueOrDefault<bool>("classify", true);
    m_extract = options.getValueOrDefault<bool>("extract", false);
    m_approximate = options.getValueOrDefault<bool>("approximate", false);
}

void GroundFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}

PointViewSet GroundFilter::run(PointViewPtr input)
{
    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);
    log()->get(LogLevel::Debug2) << "Process GroundFilter...\n";

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

    // setup the PMF filter
    pcl::PointIndicesPtr idx(new pcl::PointIndices);
    if (!m_approximate)
    {

        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud(cloud);
        pmf.setMaxWindowSize(m_maxWindowSize);
        pmf.setSlope(m_slope);
        pmf.setMaxDistance(m_maxDistance);
        pmf.setInitialDistance(m_initialDistance);
        pmf.setCellSize(m_cellSize);

        // run the PMF filter, grabbing indices of ground returns
        pmf.extract(idx->indices);
    } else
    {
        pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud(cloud);
        pmf.setMaxWindowSize(m_maxWindowSize);
        pmf.setSlope(m_slope);
        pmf.setMaxDistance(m_maxDistance);
        pmf.setInitialDistance(m_initialDistance);
        pmf.setCellSize(m_cellSize);

        // run the PMF filter, grabbing indices of ground returns
        pmf.extract(idx->indices);

    }

    PointViewSet viewSet;
    if (!idx->indices.empty() && (m_classify || m_extract))
    {

        if (m_classify)
        {
            log()->get(LogLevel::Debug2) << "Labeled " << idx->indices.size() << " ground returns!\n";

            // set the classification label of ground returns as 2
            // (corresponding to ASPRS LAS specification)
            for (const auto& i : idx->indices)
            { input->setField(Dimension::Id::Classification, i, 2); }

            viewSet.insert(input);
        }

        if (m_extract)
        {
            log()->get(LogLevel::Debug2) << "Extracted " << idx->indices.size() << " ground returns!\n";

            // create new PointView containing only ground returns
            PointViewPtr output = input->makeNew();
            for (const auto& i : idx->indices)
            {
                output->appendPoint(*input, i);
            }

            viewSet.erase(input);
            viewSet.insert(output);
        }
    }
    else
    {
        if (idx->indices.empty())
            log()->get(LogLevel::Debug2) << "Filtered cloud has no ground returns!\n";

        if (!(m_classify || m_extract))
            log()->get(LogLevel::Debug2) << "Must choose --classify or --extract\n";

        // return the input buffer unchanged
        viewSet.insert(input);
    }

    return viewSet;
}

} // namespace pdal

