/******************************************************************************
* Copyright (c) 2018, RIEGL Laser Measurement Systems GmbH (support@riegl.com)
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
*     * Neither the name of Hobu, Inc., Flaxen Geo Consulting or RIEGL
*       Laser Measurement Systems GmbH nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#pragma once

#include <vector>
#include <Eigen/Dense>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <riegl/rdb.hpp>

namespace pdal
{

//______________________________________________________________________________
/*!
 * Core RIEGL RDB point cloud reader class
 */
class PDAL_DLL RdbPointcloud
{
public:
    RdbPointcloud(
        const std::string& location, //!< database filename
        const std::string& filter,   //!< optional filter string
        const bool         extras    //!< true: all, false: PDAL attributes only
    );
    virtual ~RdbPointcloud();

    void addDimensions(PointLayoutPtr layout);

    point_count_t read(PointViewPtr view, const point_count_t count);
    bool read(PointRef& point);

    riegl::rdb::Pointcloud& pointcloud()
    {
        return m_pointcloud;
    }

    const riegl::rdb::Pointcloud& pointcloud() const
    {
        return m_pointcloud;
    }

    std::string crsWKT() const
    {
        return m_crs_wkt;
    }

    int crsEPSG() const
    {
        return m_crs_epsg;
    }

    Eigen::Matrix4d crsPose() const
    {
        if (m_crs_pose)
             return *m_crs_pose;
        else return Eigen::Matrix4d::Identity();
    }

    /*!
     * Reads the axis-aligned 3D bounding box of the point cloud in the
     * coordinate system defined by the metadata object "riegl.geo_tag".
     * If the RDB file does not contain coordinates ("riegl.xyz"), then
     * the function returns false.
     */
    bool getBoundingBox(
        Eigen::Vector4d& minimum, //!< [out] corner closest to the origin
        Eigen::Vector4d& maximum  //!< [out] corner farthest from the origin
    );

private:
    struct AttributeBuffer
    {
        typedef std::shared_ptr<AttributeBuffer> Ptr;

        pdal::Dimension::Id   id;   //!< PDAL dimension ID
        pdal::Dimension::Type type; //!< PDAL dimension type
        std::string           name; //!< PDAL dimension name
        std::vector<uint8_t>  data; //!< attribute data buffer

        AttributeBuffer(
            const pdal::Dimension::Id   id,   //!< PDAL dimension ID
            const pdal::Dimension::Type type, //!< PDAL dimension type
            const std::string           name, //!< PDAL dimension name
            const size_t                size  //!< number of points
        );

        inline void resize(const size_t size);

        inline void* at(const size_t index);

        static size_t typeSize(
            const pdal::Dimension::Type type
        );

        static riegl::rdb::pointcloud::DataType typeRDB(
            const pdal::Dimension::Type type
        );

        static pdal::Dimension::Type typePDAL(
            const riegl::rdb::pointcloud::DataType type
        );

    private:
        const size_t m_typeSize;
    };

    // point reader implementation
    template <typename TargetAdapter>
    point_count_t readPoints(
        const TargetAdapter target,
        const point_count_t count
    );

private:
    riegl::rdb::Context                 m_context;
    riegl::rdb::Pointcloud              m_pointcloud;
    std::string                         m_crs_wkt;
    int                                 m_crs_epsg;
    std::shared_ptr<Eigen::Matrix4d>    m_crs_pose;
    std::vector<AttributeBuffer::Ptr>   m_select_buffer;
    riegl::rdb::pointcloud::QuerySelect m_select_query;
    uint32_t                            m_select_index;
    uint32_t                            m_select_count;
    const uint32_t                      m_buffer_size;
    AttributeBuffer::Ptr                m_buffer_px;
    AttributeBuffer::Ptr                m_buffer_py;
    AttributeBuffer::Ptr                m_buffer_pz;
    AttributeBuffer::Ptr                m_buffer_nx;
    AttributeBuffer::Ptr                m_buffer_ny;
    AttributeBuffer::Ptr                m_buffer_nz;
};

} // namespace pdal
