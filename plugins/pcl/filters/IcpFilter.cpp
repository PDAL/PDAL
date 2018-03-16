/******************************************************************************
 * Copyright (c) 2017, Peter J. Gadomski <pete@gadom.ski>
 *
 * All rights reserved
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

#include "../PCLConversions.hpp"
#include "IcpFilter.hpp"
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pdal/EigenUtils.hpp>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.icp", "Iterative Closest Point (ICP) filter",
               "http://pdal.io/stages/filters.icp.html");

CREATE_SHARED_PLUGIN(1, 0, IcpFilter, Filter, s_info)

std::string IcpFilter::getName() const
{
    return s_info.name;
}

PointViewSet IcpFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    if (this->m_fixed)
    {
        log()->get(LogLevel::Debug2) << "Calculating ICP\n";
        PointViewPtr result = this->icp(this->m_fixed, view);
        viewSet.insert(result);
        log()->get(LogLevel::Debug2) << "ICP complete\n";
        this->m_complete = true;
    }
    else
    {
        log()->get(LogLevel::Debug2) << "Adding fixed points\n";
        this->m_fixed = view;
    }
    return viewSet;
}

void IcpFilter::done(PointTableRef _)
{
    if (!this->m_complete)
    {
        throw pdal_error(
            "filters.icp must have two point view inputs, no more, no less");
    }
}

PointViewPtr IcpFilter::icp(PointViewPtr fixed, PointViewPtr moving) const
{
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> Cloud;
    Cloud::Ptr fixedCloud(new Cloud());
    pclsupport::PDALtoPCD(fixed, *fixedCloud);
    Cloud::Ptr movingCloud(new Cloud());
    pclsupport::PDALtoPCD(moving, *movingCloud);
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputCloud(movingCloud);
    icp.setInputTarget(fixedCloud);
    Cloud result;
    icp.align(result);

    MetadataNode root = getMetadata();
    // I couldn't figure out the template-fu to get
    // `MetadataNodeImpl::setValue` to work for all Eigen matrices with one
    // function, so I'm just brute-forcing the cast for now.
    root.add("transform",
             Eigen::MatrixXd(icp.getFinalTransformation().cast<double>()));
    root.add("converged", icp.hasConverged());
    root.add("fitness", icp.getFitnessScore());

    assert(moving->size() == result.points.size());
    for (PointId i = 0; i < moving->size(); ++i)
    {
        moving->setField(Dimension::Id::X, i, result.points[i].x);
        moving->setField(Dimension::Id::Y, i, result.points[i].y);
        moving->setField(Dimension::Id::Z, i, result.points[i].z);
    }
    return moving;
}
}
