/******************************************************************************
* Copyright (c) 2012-2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <fstream>
#include <string>

#include <boost/filesystem.hpp>

#include <pdal/Dimension.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Schema.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/StageIterator.hpp>

#ifdef PDAL_HAVE_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/for_each_type.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#endif

namespace pdal
{
/**
 * \brief Convert PCD point cloud to PDAL.
 *
 * Converts PCD data to PDAL format.
 */
template <typename CloudT>
void PCDtoPDAL(CloudT &cloud, PointBuffer& data)
{
#ifdef PDAL_HAVE_PCL
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    const pdal::Schema &buffer_schema = data.getSchema();

    data.setNumPoints(cloud.points.size());

    typename CloudT::PointType testPoint = cloud.points[0];

    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    float x_val = 0.0f;
    float y_val = 0.0f;
    float z_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "x", has_x, x_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "y", has_y, y_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "z", has_z, z_val));

    const pdal::Dimension &dX = buffer_schema.getDimension("X");
    const pdal::Dimension &dY = buffer_schema.getDimension("Y");
    const pdal::Dimension &dZ = buffer_schema.getDimension("Z");

    if (has_x && has_y && has_z)
    {
        boost::uint32_t size = dX.getByteSize();
        switch (dX.getInterpretation())
        {
            case dimension::Float:
                for (size_t i = 0; i < cloud.points.size(); ++i)
                    data.setField<float>(dX, i, dX.removeScaling<float>(cloud.points[i].x));
                break;

            case dimension::SignedInteger:
                if (size == 1)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int8_t>(dX, i, dX.removeScaling<boost::int8_t>(cloud.points[i].x));
                if (size == 2)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int16_t>(dX, i, dX.removeScaling<boost::int16_t>(cloud.points[i].x));
                if (size == 4)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int32_t>(dX, i, dX.removeScaling<boost::int32_t>(cloud.points[i].x));
                if (size == 8)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int64_t>(dX, i, dX.removeScaling<boost::int64_t>(cloud.points[i].x));
                break;

            case dimension::UnsignedInteger:
                if (size == 1)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint8_t>(dX, i, dX.removeScaling<boost::uint8_t>(cloud.points[i].x));
                if (size == 2)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint16_t>(dX, i, dX.removeScaling<boost::uint16_t>(cloud.points[i].x));
                if (size == 4)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint32_t>(dX, i, dX.removeScaling<boost::uint32_t>(cloud.points[i].x));
                if (size == 8)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint64_t>(dX, i, dX.removeScaling<boost::uint64_t>(cloud.points[i].x));
                break;

            case dimension::RawByte:
            case dimension::Pointer:
            case dimension::Undefined:
                throw pdal_error("Dimension data type unable to be scaled in conversion from PCL to PDAL");
        }

        switch (dY.getInterpretation())
        {
            case dimension::Float:
                for (size_t i = 0; i < cloud.points.size(); ++i)
                    data.setField<float>(dY, i, dY.removeScaling<float>(cloud.points[i].y));
                break;

            case dimension::SignedInteger:
                if (size == 1)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int8_t>(dY, i, dY.removeScaling<boost::int8_t>(cloud.points[i].y));
                if (size == 2)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int16_t>(dY, i, dY.removeScaling<boost::int16_t>(cloud.points[i].y));
                if (size == 4)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int32_t>(dY, i, dY.removeScaling<boost::int32_t>(cloud.points[i].y));
                if (size == 8)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int64_t>(dY, i, dY.removeScaling<boost::int64_t>(cloud.points[i].y));
                break;

            case dimension::UnsignedInteger:
                if (size == 1)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint8_t>(dY, i, dY.removeScaling<boost::uint8_t>(cloud.points[i].y));
                if (size == 2)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint16_t>(dY, i, dY.removeScaling<boost::uint16_t>(cloud.points[i].y));
                if (size == 4)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint32_t>(dY, i, dY.removeScaling<boost::uint32_t>(cloud.points[i].y));
                if (size == 8)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint64_t>(dY, i, dY.removeScaling<boost::uint64_t>(cloud.points[i].y));
                break;

            case dimension::RawByte:
            case dimension::Pointer:
            case dimension::Undefined:
                throw pdal_error("Dimension data type unable to be scaled in conversion from PCL to PDAL");
        }

        switch (dZ.getInterpretation())
        {
            case dimension::Float:
                for (size_t i = 0; i < cloud.points.size(); ++i)
                    data.setField<float>(dZ, i, dZ.removeScaling<float>(cloud.points[i].z));
                break;

            case dimension::SignedInteger:
                if (size == 1)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int8_t>(dZ, i, dZ.removeScaling<boost::int8_t>(cloud.points[i].z));
                if (size == 2)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int16_t>(dZ, i, dZ.removeScaling<boost::int16_t>(cloud.points[i].z));
                if (size == 4)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int32_t>(dZ, i, dZ.removeScaling<boost::int32_t>(cloud.points[i].z));
                if (size == 8)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::int64_t>(dZ, i, dZ.removeScaling<boost::int64_t>(cloud.points[i].z));
                break;

            case dimension::UnsignedInteger:
                if (size == 1)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint8_t>(dZ, i, dZ.removeScaling<boost::uint8_t>(cloud.points[i].z));
                if (size == 2)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint16_t>(dZ, i, dZ.removeScaling<boost::uint16_t>(cloud.points[i].z));
                if (size == 4)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint32_t>(dZ, i, dZ.removeScaling<boost::uint32_t>(cloud.points[i].z));
                if (size == 8)
                    for (size_t i = 0; i < cloud.points.size(); ++i)
                        data.setField<boost::uint64_t>(dZ, i, dZ.removeScaling<boost::uint64_t>(cloud.points[i].z));
                break;

            case dimension::RawByte:
            case dimension::Pointer:
            case dimension::Undefined:
                throw pdal_error("Dimension data type unable to be scaled in conversion from PCL to PDAL");
        }
    }

    boost::optional<pdal::Dimension const &> dI = buffer_schema.getDimensionOptional("Intensity");

    bool has_i = false;
    float i_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "intensity", has_i, i_val));

    if (has_i && dI)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            float v;

            typename CloudT::PointType p = cloud.points[i];
            pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "intensity", has_i, v));

            data.setField<float>(*dI, i, v);
        }
    }
#endif
}

/**
 * \brief Convert PDAL point cloud to PCD.
 *
 * Converts PDAL data to PCD format.
 */
template <typename CloudT>
void PDALtoPCD(PointBuffer& data, CloudT &cloud)
{
#ifdef PDAL_HAVE_PCL
    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    const pdal::Schema &buffer_schema = data.getSchema();

    cloud.width = data.getNumPoints();
    cloud.height = 1;  // unorganized point cloud
    cloud.is_dense = false;
    cloud.points.resize(cloud.width);

    typename CloudT::PointType testPoint = cloud.points[0];

    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    float x_val = 0.0f;
    float y_val = 0.0f;
    float z_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "x", has_x, x_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "y", has_y, y_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "z", has_z, z_val));

    const pdal::Dimension &dX = buffer_schema.getDimension("X");
    const pdal::Dimension &dY = buffer_schema.getDimension("Y");
    const pdal::Dimension &dZ = buffer_schema.getDimension("Z");

    if (has_x && has_y && has_z)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            float xd = data.applyScaling(dX, i);
            float yd = data.applyScaling(dY, i);
            float zd = data.applyScaling(dZ, i);

            typename CloudT::PointType p = cloud.points[i];
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", xd));
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", yd));
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", zd));
            cloud.points[i] = p;
        }
    }

    boost::optional<pdal::Dimension const &> dI = buffer_schema.getDimensionOptional("Intensity");

    bool has_i = false;
    float i_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "intensity", has_i, i_val));

    if (has_i && dI)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            boost::int32_t vi = data.getField<boost::int32_t>(*dI, i);

            float vd = dI->applyScaling(vi);

            typename CloudT::PointType p = cloud.points[i];
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "intensity", vd));
            cloud.points[i] = p;
        }
    }
#endif
}
}  // pdal

#endif  // INCLUDED_PCL_CONVERSIONS_HPP

