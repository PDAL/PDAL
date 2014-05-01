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

#ifndef INCLUDED_PCL_CONVERSIONS_HPP
#define INCLUDED_PCL_CONVERSIONS_HPP

#include <sstream>

#include <pdal/Dimension.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Schema.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/for_each_type.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

namespace pdal
{
typedef boost::optional<Dimension const &> ConstDimOptional;

/**
 * \brief Helper function to set PCD data in PDAL PointBuffer
 *
 * PCD will always convert from Float to SignedInteger. For PCD's internal
 * representation, we only scale the data, but do not apply offsets. Therefore,
 * only the scale is removed when converting back to PDAL.
 */
inline void setPDAL(PointBuffer& buffer, Dimension dim, uint32_t idx, float val)
{
    try
    {
        float scale = boost::numeric_cast<float>(dim.getNumericScale());
        int32_t ival = boost::numeric_cast<int32_t>(val / scale);
        buffer.setField<int32_t>(dim, idx, ival);
    }
    catch (boost::numeric::bad_numeric_cast& e)
    {
        std::stringstream oss;
        oss << "Unable to cast from Float to SignedInteger in PCDtoPDAL\n";
        throw pdal_error(oss.str());
    }
}

/**
 * \brief Helper function to set PDAL PointBuffer data in PCD
 *
 * PDAL data is always converted from SignedInteger to Float for PCD. Data is
 * scaled, but not offset.
 */
inline float setPCL(const PointBuffer& buffer, Dimension dim, uint32_t idx)
{
    float val;
    try
    {
        float scale = boost::numeric_cast<float>(dim.getNumericScale());
        int32_t ival = buffer.getFieldAs<int32_t>(dim, idx, false);
        val = boost::numeric_cast<float>(ival * scale);
    }
    catch (boost::numeric::bad_numeric_cast& e)
    {
        std::stringstream oss;
        oss << "Unable to cast from SignedInteger to Float in PDALtoPCD\n";
        throw pdal_error(oss.str());
    }

    return val;
}

/**
 * \brief Convert PCD point cloud to PDAL.
 *
 * Converts PCD data to PDAL format.
 */
template <typename CloudT>
void PCDtoPDAL(CloudT &cloud, PointBuffer& buffer)
{
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    const Schema &schema = buffer.getSchema();
    buffer.setNumPoints(cloud.points.size());

    // Begin by setting XYZ dimension data
    const Dimension &dimX = schema.getDimension("X");
    const Dimension &dimY = schema.getDimension("Y");
    const Dimension &dimZ = schema.getDimension("Z");
    if (pcl::traits::has_xyz<typename CloudT::PointType>::value)
    {
        uint32_t i = 0;
        typename CloudT::const_iterator it = cloud.begin();
        for (; it != cloud.end(); ++it, ++i)
        {
            setPDAL(buffer, dimX, i, (*it).x);
            setPDAL(buffer, dimY, i, (*it).y);
            setPDAL(buffer, dimZ, i, (*it).z);
        }
    }

    // Continue with optional Intensity dimension
    ConstDimOptional dI = schema.getDimensionOptional("Intensity");
    if (dI && pcl::traits::has_intensity<typename CloudT::PointType>::value)
    {
        uint32_t i = 0;
        typename CloudT::const_iterator it = cloud.begin();
        for (; it != cloud.end(); ++it, ++i)
        {
            float val;
            pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*it), "intensity", val));
            buffer.setField<float>(*dI, i, val);
        }
    }

    // Conclude with optional RGB dimensions
    ConstDimOptional dR = schema.getDimensionOptional("Red");
    ConstDimOptional dG = schema.getDimensionOptional("Green");
    ConstDimOptional dB = schema.getDimensionOptional("Blue");
    if (dR && dG && dB && pcl::traits::has_color<typename CloudT::PointType>::value)
    {
        uint32_t i = 0;
        typename CloudT::const_iterator it = cloud.begin();
        for (; it != cloud.end(); ++it, ++i)
        {
            uint32_t val;
            pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, uint32_t> ((*it), "rgba", val));
            buffer.setField<uint16_t>(*dR, i, (val & 0x00FF0000) >> 16);
            buffer.setField<uint16_t>(*dG, i, (val & 0x0000FF00) >> 8);
            buffer.setField<uint16_t>(*dB, i, (val & 0x000000FF));
        }
    }
}

/**
 * \brief Convert PDAL point cloud to PCD.
 *
 * Converts PDAL data to PCD format.
 */
template <typename CloudT>
void PDALtoPCD(const PointBuffer& buffer, CloudT &cloud)
{
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    const Schema &schema = buffer.getSchema();

    cloud.width = buffer.getNumPoints();
    cloud.height = 1;  // unorganized point cloud
    cloud.is_dense = false;
    cloud.points.resize(cloud.width);

    // Begin by getting XYZ dimension data
    const Dimension &dimX = schema.getDimension("X");
    const Dimension &dimY = schema.getDimension("Y");
    const Dimension &dimZ = schema.getDimension("Z");
    if (pcl::traits::has_xyz<typename CloudT::PointType>::value)
    {
        uint32_t i = 0;
        typename CloudT::iterator it = cloud.begin();
        for (; it != cloud.end(); ++it, ++i)
        {
            (*it).x = setPCL(buffer, dimX, i);
            (*it).y = setPCL(buffer, dimY, i);
            (*it).z = setPCL(buffer, dimZ, i);

            cloud.points[i] = (*it);
        }
    }

    // Continue with optional Intensity dimension
    ConstDimOptional dI = schema.getDimensionOptional("Intensity");
    if (dI && pcl::traits::has_intensity<typename CloudT::PointType>::value)
    {
        uint32_t i = 0;
        typename CloudT::iterator it = cloud.begin();
        for (; it != cloud.end(); ++it, ++i)
        {
            float val = buffer.getFieldAs<float>(*dI, i);
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> ((*it), "intensity", val));
            cloud.points[i] = (*it);
        }
    }

    // Conclude with optional RGB dimensions
    ConstDimOptional dR = schema.getDimensionOptional("Red");
    ConstDimOptional dG = schema.getDimensionOptional("Green");
    ConstDimOptional dB = schema.getDimensionOptional("Blue");
    if (dR && dG && dB && pcl::traits::has_color<typename CloudT::PointType>::value)
    {
        uint32_t i = 0;
        typename CloudT::iterator it = cloud.begin();
        for (; it != cloud.end(); ++it, ++i)
        {
            uint16_t r = buffer.getFieldAs<uint16_t>(*dR, i);
            uint16_t g = buffer.getFieldAs<uint16_t>(*dG, i);
            uint16_t b = buffer.getFieldAs<uint16_t>(*dB, i);
            uint32_t val = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, uint32_t> ((*it), "rgba", val));
            cloud.points[i] = (*it);
        }
    }
}
}  // pdal

#endif  // INCLUDED_PCL_CONVERSIONS_HPP

