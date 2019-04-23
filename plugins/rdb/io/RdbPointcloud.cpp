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

#include <sstream>
#include <array>
#include <nlohmann/json.hpp>
#include "RdbPointcloud.hpp"

namespace pdal
{


//---< RdbPointcloud::AttributeBuffer::PUBLIC >---------------------------------


RdbPointcloud::AttributeBuffer::AttributeBuffer(
    const pdal::Dimension::Id   id,
    const pdal::Dimension::Type type,
    const std::string           name,
    const size_t                size
):
    id  (id),
    type(type),
    name(name),
    data(), // see resize()
    m_typeSize(typeSize(type))
{
    resize(size);
}


void RdbPointcloud::AttributeBuffer::resize(const size_t size)
{
    data.resize(m_typeSize * size);
}


void* RdbPointcloud::AttributeBuffer::at(const size_t index)
{
    return &data[m_typeSize * index];
}


size_t RdbPointcloud::AttributeBuffer::typeSize(
    const pdal::Dimension::Type type
)
{
    using namespace pdal::Dimension;
    switch (type)
    {
        case Type::Unsigned8:  return sizeof(uint8_t);
        case Type::Unsigned16: return sizeof(uint16_t);
        case Type::Unsigned32: return sizeof(uint32_t);
        case Type::Unsigned64: return sizeof(uint64_t);
        case Type::Signed8:    return sizeof(int8_t);
        case Type::Signed16:   return sizeof(int16_t);
        case Type::Signed32:   return sizeof(int32_t);
        case Type::Signed64:   return sizeof(int64_t);
        case Type::Float:      return sizeof(float);
        case Type::Double:     return sizeof(double);
        default:               return 0;
    }
}


riegl::rdb::pointcloud::DataType RdbPointcloud::AttributeBuffer::typeRDB(
    const pdal::Dimension::Type type
)
{
    using namespace pdal::Dimension;
    using namespace riegl::rdb::pointcloud;
    switch (type)
    {
        case Type::Unsigned8:  return DataType::UINT8;
        case Type::Unsigned16: return DataType::UINT16;
        case Type::Unsigned32: return DataType::UINT32;
        case Type::Unsigned64: return DataType::UINT64;
        case Type::Signed8:    return DataType::INT8;
        case Type::Signed16:   return DataType::INT16;
        case Type::Signed32:   return DataType::INT32;
        case Type::Signed64:   return DataType::INT64;
        case Type::Float:      return DataType::SINGLE;
        case Type::Double:     return DataType::DOUBLE;
        default:               return DataType::NONE;
    }
}


pdal::Dimension::Type RdbPointcloud::AttributeBuffer::typePDAL(
    const riegl::rdb::pointcloud::DataType type
)
{
    using namespace pdal::Dimension;
    using namespace riegl::rdb::pointcloud;
    switch (type)
    {
        case DataType::UINT8:  return Type::Unsigned8;
        case DataType::UINT16: return Type::Unsigned16;
        case DataType::UINT32: return Type::Unsigned32;
        case DataType::UINT64: return Type::Unsigned64;
        case DataType::INT8:   return Type::Signed8;
        case DataType::INT16:  return Type::Signed16;
        case DataType::INT32:  return Type::Signed32;
        case DataType::INT64:  return Type::Signed64;
        case DataType::SINGLE: return Type::Float;
        case DataType::DOUBLE: return Type::Double;
        default:               return Type::None;
    }
}


//---< RdbPointcloud::PUBLIC >--------------------------------------------------


RdbPointcloud::RdbPointcloud(
    const std::string& location,
    const std::string& filter,
    const bool         extras
):
    m_context(),
    m_pointcloud(m_context),
    m_crs_wkt(),
    m_crs_epsg(0),
    m_crs_pose(),
    m_select_buffer(),
    m_select_query(),
    m_select_index(0),
    m_select_count(0),
    m_buffer_size(100*1000),
    m_buffer_px(),
    m_buffer_py(),
    m_buffer_pz(),
    m_buffer_nx(),
    m_buffer_ny(),
    m_buffer_nz()
{
    using namespace pdal::Dimension;
    using namespace riegl::rdb::pointcloud;

    // open database
    OpenSettings settings(m_context);
    settings.cacheSize = 0; // no cache required, because we only read once
    m_pointcloud.open(location, settings);

    // query spatial reference system
    if (m_pointcloud.metaData().exists("riegl.geo_tag"))
    {
        NL::json node;

        try
        {
            std::string s = m_pointcloud.metaData().get("riegl.geo_tag");
            node = NL::json::parse(s);
            if (node["crs"]["epsg"].is_number_integer())
                m_crs_epsg = node["crs"]["epsg"].get<int>();
            if (node["crs"]["wkt"].is_string())
                m_crs_wkt = node["crs"]["wkt"].get<std::string>();
            if (node["pose"].is_array())
            {
                const NL::json pose = node["pose"];
                if ( (pose.size() == 4) &&
                    (pose[0].size() == 4) &&
                    (pose[1].size() == 4) &&
                    (pose[2].size() == 4) &&
                    (pose[3].size() == 4)
                )
                {
                    Eigen::Matrix4d matrix;
                    for (int row = 0; row < 4; ++row)
                    for (int col = 0; col < 4; ++col)
                    {
                        matrix(row, col) = pose[row][col].get<double>();
                    }
                    m_crs_pose = std::make_shared<Eigen::Matrix4d>(matrix);
                }
            }
        }
        catch (const NL::json::parse_error&)
        {}
    }

    // setup read query and attribute buffers
    m_select_query = m_pointcloud.select(filter);
    const auto attributes = m_pointcloud.pointAttribute().list();
    for (const auto& attribute: attributes)
    {
        const auto bindDefaultBuffer = [&](
            const pdal::Dimension::Id id,  // pdal dimension ID
            const std::string&        name // rdb attribute name
        ) -> AttributeBuffer::Ptr
        {
            const auto buffer = std::make_shared<AttributeBuffer>(
                id,
                pdal::Dimension::defaultType(id),
                pdal::Dimension::name(id),
                m_buffer_size
            );
            this->m_select_query.bind(
                name,
                AttributeBuffer::typeRDB(buffer->type),
                buffer->data.data()
            );
            m_select_buffer.push_back(buffer);
            return buffer;
        };

        if (attribute == "riegl.id")
        {
            bindDefaultBuffer(Id::PointId, "riegl.id");
        }
        else if (attribute == "riegl.source_cloud_id")
        {
            bindDefaultBuffer(Id::OriginId, "riegl.source_cloud_id");
        }
        else if (attribute == "riegl.timestamp")
        {
            // @@@ or rather 'Id::GpsTime'?
            bindDefaultBuffer(Id::InternalTime, "riegl.timestamp");
        }
        else if (attribute == "riegl.xyz")
        {
            m_buffer_px = bindDefaultBuffer(Id::X, "riegl.xyz[0]");
            m_buffer_py = bindDefaultBuffer(Id::Y, "riegl.xyz[1]");
            m_buffer_pz = bindDefaultBuffer(Id::Z, "riegl.xyz[2]");
            assert(m_buffer_px->type == pdal::Dimension::Type::Double);
            assert(m_buffer_py->type == pdal::Dimension::Type::Double);
            assert(m_buffer_pz->type == pdal::Dimension::Type::Double);
        }
        else if (attribute == "riegl.intensity")
        {
            bindDefaultBuffer(Id::Intensity, "riegl.intensity");
        }
        else if (attribute == "riegl.amplitude")
        {
            bindDefaultBuffer(Id::Amplitude, "riegl.amplitude");
        }
        else if (attribute == "riegl.reflectance")
        {
            bindDefaultBuffer(Id::Reflectance, "riegl.reflectance");
        }
        else if (attribute == "riegl.deviation")
        {
            bindDefaultBuffer(Id::Deviation, "riegl.deviation");
        }
        else if (attribute == "riegl.pulse_width")
        {
            bindDefaultBuffer(Id::PulseWidth, "riegl.pulse_width");
        }
        else if (attribute == "riegl.background_radiation")
        {
            bindDefaultBuffer(Id::BackgroundRadiation, "riegl.background_radiation");
        }
        else if (attribute == "riegl.target_index")
        {
            bindDefaultBuffer(Id::ReturnNumber, "riegl.target_index");
        }
        else if (attribute == "riegl.target_count")
        {
            bindDefaultBuffer(Id::NumberOfReturns, "riegl.target_count");
        }
        else if (attribute == "riegl.scan_direction")
        {
            bindDefaultBuffer(Id::ScanDirectionFlag, "riegl.scan_direction");
        }
        else if (attribute == "riegl.scan_angle")
        {
            bindDefaultBuffer(Id::ScanAngleRank, "riegl.scan_angle");
        }
        else if (attribute == "riegl.class")
        {
            bindDefaultBuffer(Id::Classification, "riegl.class");
        }
        else if (attribute == "riegl.rgba")
        {
            bindDefaultBuffer(Id::Red,   "riegl.rgba[0]");
            bindDefaultBuffer(Id::Green, "riegl.rgba[1]");
            bindDefaultBuffer(Id::Blue,  "riegl.rgba[2]");
        }
        else if (attribute == "riegl.surface_normal")
        {
            m_buffer_nx = bindDefaultBuffer(Id::NormalX, "riegl.surface_normal[0]");
            m_buffer_ny = bindDefaultBuffer(Id::NormalY, "riegl.surface_normal[1]");
            m_buffer_nz = bindDefaultBuffer(Id::NormalZ, "riegl.surface_normal[2]");
            assert(m_buffer_nx->type == pdal::Dimension::Type::Double);
            assert(m_buffer_ny->type == pdal::Dimension::Type::Double);
            assert(m_buffer_nz->type == pdal::Dimension::Type::Double);
        }
        else if (extras) // then handle custom point attributes
        {
            const auto details  = m_pointcloud.pointAttribute().get(attribute);
            const auto typeRDB  = details.dataType(); // suggested data type
            const auto typePDAL = AttributeBuffer::typePDAL(typeRDB);
            for (uint8_t i = 0; i < details.length; ++i)
            {
                std::stringstream nameRDB;  nameRDB  << details.name;
                std::stringstream namePDAL; namePDAL << details.name;
                if (details.length > 1) // vector attribute
                {
                    nameRDB  << "[" << int(i) << "]";
                    namePDAL << "." << int(i);
                }
                const auto buffer = std::make_shared<AttributeBuffer>(
                    pdal::Dimension::Id::Unknown, // see addDimensions()
                    typePDAL,
                    namePDAL.str(),
                    m_buffer_size
                );
                m_select_query.bind(
                    nameRDB.str(),
                    typeRDB,
                    buffer->data.data()
                );
                m_select_buffer.push_back(buffer);
            }
        }
    }
}


RdbPointcloud::~RdbPointcloud()
{
    m_select_query.close();
    m_pointcloud  .close();
}


void RdbPointcloud::addDimensions(PointLayoutPtr layout)
{
    for (auto& buffer: m_select_buffer)
    {
        buffer->id = layout->registerOrAssignDim(buffer->name, buffer->type);
    }
}


point_count_t RdbPointcloud::read(PointViewPtr view, const point_count_t count)
{
    struct PointViewAdapter
    {
        PointViewAdapter(PointViewPtr view): view(view) { }
        point_count_t size() const { return view->size(); }
        void setField(
            const Dimension::Id   dimension,
            const Dimension::Type type,
            const PointId         index,
            const void*           value
        ) const
        {
            view->setField(dimension, type, index, value);
        }
        PointViewPtr view;
    };
    return readPoints(PointViewAdapter(view), count);
}


bool RdbPointcloud::read(PointRef& point)
{
    struct PointRefAdapter
    {
        PointRefAdapter(PointRef& point): point(point) { }
        point_count_t size() const { return 0; }
        void setField(
            const Dimension::Id   dimension,
            const Dimension::Type type,
            const PointId       /*index*/,
            const void*           value
        ) const
        {
            point.setField(dimension, type, value);
        }
        PointRef& point;
    };
    return readPoints(PointRefAdapter(point), 1) == 1;
}


bool RdbPointcloud::getBoundingBox(
    Eigen::Vector4d& minimum,
    Eigen::Vector4d& maximum
)
{
    if (m_pointcloud.pointAttribute().exists("riegl.xyz"))
    {
        using namespace riegl::rdb::pointcloud;
        QueryStat stat = m_pointcloud.stat();
        GraphNode root = stat.index();

        std::array<std::array<double, 3>, 2> limits; // 0=min, 1=max
        stat.minimum(root.id, "riegl.xyz", DOUBLE, limits[0].data());
        stat.maximum(root.id, "riegl.xyz", DOUBLE, limits[1].data());

        const double inf = std::numeric_limits<double>::infinity();
        minimum = Eigen::Vector4d(+inf, +inf, +inf, 1.0);
        maximum = Eigen::Vector4d(-inf, -inf, -inf, 1.0);
        const Eigen::Matrix4d pose = crsPose();

        for (int i = 0; i < 8; ++i) // transform box corners to crs...
        {
            const Eigen::Vector4d corner = pose * Eigen::Vector4d(
                limits[(i >> 0) & 1][0],
                limits[(i >> 1) & 1][1],
                limits[(i >> 2) & 1][2],
                1.0
            );
            minimum = minimum.cwiseMin(corner); // ...and update
            maximum = maximum.cwiseMax(corner); // bounding box!
        }
        return true;
    }
    return false;
}


//---< RdbPointcloud::PRIVATE >-------------------------------------------------


template <typename TargetAdapter>
point_count_t RdbPointcloud::readPoints(
    const TargetAdapter target,
    const point_count_t count
)
{
    point_count_t total = 0;
    point_count_t index = target.size();
    while (total < count) // copy points from our buffer to the PDAL-view
    {
        if (m_select_index >= m_select_count) // then read next block
        {
            m_select_index = 0; // rewind internal buffer
            m_select_count = m_select_query.next(m_buffer_size);
            if (m_select_count == 0) // then we reached end-of-file
            {
                return total;
            }
            if (m_crs_pose) // then transform points
            {
                const auto transform = [&](
                    const AttributeBuffer::Ptr& x,
                    const AttributeBuffer::Ptr& y,
                    const AttributeBuffer::Ptr& z,
                    const double                w
                )
                {
                    auto px = static_cast<double*>(x->at(0));
                    auto py = static_cast<double*>(y->at(0));
                    auto pz = static_cast<double*>(z->at(0));
                    const auto end = px + m_select_count;
                    for (; px < end; ++px, ++py, ++pz)
                    {
                        const Eigen::Vector4d xyz(
                            *m_crs_pose * Eigen::Vector4d(*px, *py, *pz, w)
                        );
                        *px = xyz[0];
                        *py = xyz[1];
                        *pz = xyz[2];
                    }
                };
                if (m_buffer_px && m_buffer_py && m_buffer_pz) // coordinates
                {
                    transform(m_buffer_px, m_buffer_py, m_buffer_pz, 1.0);
                }
                if (m_buffer_nx && m_buffer_ny && m_buffer_nz) // normals
                {
                    transform(m_buffer_nx, m_buffer_ny, m_buffer_nz, 0.0);
                }
            }
        }
        while ((total < count) && (m_select_index < m_select_count))
        {
            for (const auto& buffer: m_select_buffer)
            {
                target.setField(
                    buffer->id,
                    buffer->type,
                    index,
                    buffer->at(m_select_index)
                );
            }
            m_select_index++;
            index++;
            total++;
        }
    }
    return total;
}

}
