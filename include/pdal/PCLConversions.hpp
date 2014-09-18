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

#include <pdal/PointBuffer.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/for_each_type.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

namespace
{

using namespace pdal;

template<typename CLOUDFETCH>
void setValues(PointBuffer& buf, Dimension::Id::Enum dim, size_t numPts,
    CLOUDFETCH fetcher)
{
    for (size_t i = 0; i < numPts; ++i)
        buf.setField(dim, i, fetcher(i));
}

} //namespace

namespace pdal
{
/**
 * \brief Convert PCD point cloud to PDAL.
 *
 * Converts PCD data to PDAL format.
 */
template <typename CloudT>
void PCDtoPDAL(CloudT &cloud, PointBuffer& buf)
{
    Bounds<double> buffer_bounds(0,0,0,0,0,0);
    PCDtoPDAL(cloud, buf, buffer_bounds);
}

template <typename CloudT>
void PCDtoPDAL(CloudT &cloud, PointBuffer& buf, Bounds<double> const& bounds)
{
#ifdef PDAL_HAVE_PCL
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type
        FieldList;

    if (pcl::traits::has_xyz<typename CloudT::PointType>::value)
    {
        auto getX = [&cloud, &bounds](size_t i)
            { return cloud.points[i].x + bounds.getMinimum(0); };
        auto getY = [&cloud, &bounds](size_t i)
            { return cloud.points[i].y + bounds.getMinimum(1); };
        auto getZ = [&cloud, &bounds](size_t i)
            { return cloud.points[i].z + bounds.getMinimum(2); };
        setValues(buf, Dimension::Id::X, cloud.points.size(), getX);
        setValues(buf, Dimension::Id::Y, cloud.points.size(), getY);
        setValues(buf, Dimension::Id::Z, cloud.points.size(), getZ);
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
            buf.setField(Dimension::Id::Intensity, i, f);
        }
    }

    if (pcl::traits::has_color<typename CloudT::PointType>::value)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            boost::uint32_t v;
            
            typename CloudT::PointType p = cloud.points[i];
            pcl::for_each_type<FieldList>
               (pcl::CopyIfFieldExists<typename CloudT::PointType, boost::uint32_t>
                   (p, "rgba", v));
            buf.setField<boost::uint8_t>(Dimension::Id::Red, i, (v & 0x00FF0000) >> 16);
            buf.setField<boost::uint8_t>(Dimension::Id::Green, i, (v & 0x0000FF00) >> 8);
            buf.setField<boost::uint8_t>(Dimension::Id::Blue, i, (v & 0x000000FF));
        }
    }
#endif // PDAL_HAVE_PCL
}


/**
 * \brief Convert PDAL point cloud to PCD.
 *
 * Converts PDAL data to PCD format.
 */
template <typename CloudT>
void PDALtoPCD(const PointBuffer& data, CloudT &cloud)
{
    Bounds<double> buffer_bounds(0,0,0,0,0,0);
    PDALtoPCD(const_cast<PointBuffer&>(data), cloud, buffer_bounds);
}

template <typename CloudT>
void PDALtoPCD(PointBuffer& data, CloudT &cloud, Bounds<double> const& bounds)
{
#ifdef PDAL_HAVE_PCL
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type
        FieldList;

    cloud.width = data.size();
    cloud.height = 1;  // unorganized point cloud
    cloud.is_dense = false;
    cloud.points.resize(cloud.width);

    if (pcl::traits::has_xyz<typename CloudT::PointType>::value)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            double xd = data.getFieldAs<double>(Dimension::Id::X, i) - bounds.getMinimum(0);
            double yd = data.getFieldAs<double>(Dimension::Id::Y, i) - bounds.getMinimum(1);
            double zd = data.getFieldAs<double>(Dimension::Id::Z, i) - bounds.getMinimum(2);

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

            float f = data.getFieldAs<float>(Dimension::Id::Intensity, i);
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

            boost::uint8_t r = data.getFieldAs<boost::uint8_t>(Dimension::Id::Red, i);
            boost::uint8_t g = data.getFieldAs<boost::uint8_t>(Dimension::Id::Green, i);
            boost::uint8_t b = data.getFieldAs<boost::uint8_t>(Dimension::Id::Blue, i);
            pcl::for_each_type<FieldList> (
                pcl::SetIfFieldExists<typename CloudT::PointType, boost::uint32_t> (
                    p, "rgba", ((int)r) << 16 | ((int)g) << 8 | ((int)b)
                )
            );
            cloud.points[i] = p;
        }
    }
#endif // PDAL_HAVE_PCL
}

}  // namespace pdal

