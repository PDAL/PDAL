/******************************************************************************
* Copyright (c) 2012-2014, Bradley J Chambers (brad.chambers@gmail.com)
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
*
* Inspired, and partially borrowed from VTK_PCL_Conversions
* https://github.com/daviddoria/VTK_PCL_Conversions
****************************************************************************/

#pragma once

#include <pdal/PointView.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/for_each_type.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

namespace pdal
{
namespace pclsupport
{

template<typename CLOUDFETCH>
void setValues(PointViewPtr view, Dimension::Id::Enum dim, size_t numPts,
    CLOUDFETCH fetcher)
{
    for (size_t i = 0; i < numPts; ++i)
        view->setField(dim, i, fetcher(i));
}

/**
 * \brief Convert PCD point cloud to PDAL.
 *
 * Converts PCD data to PDAL format.
 */
template <typename CloudT>
void PCDtoPDAL(CloudT &cloud, PointViewPtr view, BOX3D const& bounds)
{
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type
        FieldList;

    if (pcl::traits::has_xyz<typename CloudT::PointType>::value)
    {
        auto getX = [&cloud, &bounds](size_t i)
            { return cloud.points[i].x + bounds.minx; };
        auto getY = [&cloud, &bounds](size_t i)
            { return cloud.points[i].y + bounds.miny; };
        auto getZ = [&cloud, &bounds](size_t i)
            { return cloud.points[i].z + bounds.minz; };
        setValues(view, Dimension::Id::X, cloud.points.size(), getX);
        setValues(view, Dimension::Id::Y, cloud.points.size(), getY);
        setValues(view, Dimension::Id::Z, cloud.points.size(), getZ);
    }

    if (pcl::traits::has_intensity<typename CloudT::PointType>::value)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            float f;
            bool hasIntensity = true;

            typename CloudT::PointType p = cloud.points[i];
            pcl::for_each_type<FieldList>
                (pcl::CopyIfFieldExists<typename CloudT::PointType, float>
                    (p, "intensity", hasIntensity, f));
            view->setField(Dimension::Id::Intensity, i, f);
        }
    }

    if (pcl::traits::has_color<typename CloudT::PointType>::value)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            uint32_t v;

            typename CloudT::PointType p = cloud.points[i];
            pcl::for_each_type<FieldList>
               (pcl::CopyIfFieldExists<typename CloudT::PointType, uint32_t>
                   (p, "rgba", v));
            view->setField<uint8_t>(Dimension::Id::Red, i, (v & 0x00FF0000) >> 16);
            view->setField<uint8_t>(Dimension::Id::Green, i, (v & 0x0000FF00) >> 8);
            view->setField<uint8_t>(Dimension::Id::Blue, i, (v & 0x000000FF));
        }
    }
}


template <typename CloudT>
void PCDtoPDAL(CloudT &cloud, PointViewPtr view)
{
    BOX3D buffer_bounds(0,0,0,0,0,0);
    pdal::pclsupport::PCDtoPDAL(cloud, view, buffer_bounds);
}


/**
 * \brief Convert PDAL point cloud to PCD.
 *
 * Converts PDAL data to PCD format.
 */
template <typename CloudT>
void PDALtoPCD(PointViewPtr view, CloudT &cloud, BOX3D const& bounds)
{
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type
        FieldList;

    cloud.width = view->size();
    cloud.height = 1;  // unorganized point cloud
    cloud.is_dense = false;
    cloud.points.resize(cloud.width);

    if (pcl::traits::has_xyz<typename CloudT::PointType>::value)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            double xd = view->getFieldAs<double>(Dimension::Id::X, i) - bounds.minx;
            double yd = view->getFieldAs<double>(Dimension::Id::Y, i) - bounds.miny;
            double zd = view->getFieldAs<double>(Dimension::Id::Z, i) - bounds.minz;

            typename CloudT::PointType p = cloud.points[i];
            p.x = (float)xd;
            p.y = (float)yd;
            p.z = (float)zd;
            cloud.points[i] = p;
        }
    }

    if (pcl::traits::has_intensity<typename CloudT::PointType>::value)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            typename CloudT::PointType p = cloud.points[i];

            float f = view->getFieldAs<float>(Dimension::Id::Intensity, i);
            pcl::for_each_type<FieldList>
                (pcl::SetIfFieldExists<typename CloudT::PointType, float>
                    (p, "intensity", f));
            cloud.points[i] = p;
        }
    }

    if (pcl::traits::has_color<typename CloudT::PointType>::value)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            typename CloudT::PointType p = cloud.points[i];

            uint8_t r = view->getFieldAs<uint8_t>(Dimension::Id::Red, i);
            uint8_t g = view->getFieldAs<uint8_t>(Dimension::Id::Green, i);
            uint8_t b = view->getFieldAs<uint8_t>(Dimension::Id::Blue, i);
            pcl::for_each_type<FieldList> (
                pcl::SetIfFieldExists<typename CloudT::PointType, uint32_t> (
                    p, "rgba", ((uint8_t)r) << 16 | ((uint8_t)g) << 8 | ((uint8_t)b)
                )
            );
            cloud.points[i] = p;
        }
    }
}


template <typename CloudT>
void PDALtoPCD(PointViewPtr view, CloudT &cloud)
{
    BOX3D buffer_bounds(0,0,0,0,0,0);
    PDALtoPCD(view, cloud, buffer_bounds);
}


}  // namespace pcl
}  // namespace pdal
