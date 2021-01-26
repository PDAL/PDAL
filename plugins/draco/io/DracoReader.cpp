/******************************************************************************
* Copyright (c) 2020 Hobu, Inc. (info@hobu.co)
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

#include <algorithm>
#include <pdal/util/FileUtils.hpp>

#include "DracoReader.hpp"

#include <draco/io/point_cloud_io.h>
#include <draco/compression/decode.h>

namespace pdal {

static PluginInfo const s_info
{
        "readers.draco",
        "Read data from a Draco array.",
        "http://pdal.io/stages/readers.draco.html"
};

CREATE_SHARED_STAGE(DracoReader, s_info)
std::string DracoReader::getName() const { return s_info.name; }

// Returns the attribute data in |attr| as an array of type T.
template <typename T>
bool CopyAttributeData(point_count_t num_points,
                  const draco::PointAttribute *attr,
                  std::vector<T>& data)
{
    const int num_components = attr->num_components();
    data.resize(num_points * num_components, T(0));

    for (draco::PointIndex i(0); i < num_points; ++i) {
        const draco::AttributeValueIndex val_index = attr->mapped_index(i);
        bool got_data = false;
        switch (num_components) {
            case 1:
                got_data = attr->ConvertValue<T, 1>(val_index,
                                        data.data() + i.value() * num_components);
                break;
            case 2:
                got_data = attr->ConvertValue<T, 2>(val_index,
                                        data.data() + i.value() * num_components);
                break;
            case 3:
                got_data = attr->ConvertValue<T, 3>(val_index,
                                        data.data() + i.value() * num_components);
                break;
            case 4:
                got_data = attr->ConvertValue<T, 4>(val_index,
                                        data.data() + i.value() * num_components);
                break;
            default:
            break;
    }
    if (!got_data)
       throw pdal_error("CopyAttributeData unable to read data for point ");
    }

    return true;
}

Dimension::Type getPdalType(draco::DataType t)
{
    using namespace draco;
    switch (t)
    {
        case DT_INT8:
            return Dimension::Type::Signed8;
        case DT_UINT8:
            return Dimension::Type::Unsigned8;
        case DT_INT16:
            return Dimension::Type::Signed16;
        case DT_UINT16:
            return Dimension::Type::Unsigned16;
        case DT_INT32:
            return Dimension::Type::Signed32;
        case DT_UINT32:
            return Dimension::Type::Unsigned32;
        case DT_INT64:
            return Dimension::Type::Signed64;
        case DT_UINT64:
            return Dimension::Type::Unsigned64;
        case DT_FLOAT32:
            return Dimension::Type::Float;
        case DT_FLOAT64:
            return Dimension::Type::Double;

        default:
            // Not supported draco domain types
            throw pdal_error("Invalid Dim type from Draco");
    }
}

void DracoReader::addArgs(ProgramArgs& args)
{
}

void DracoReader::prepared(PointTableRef table)
{
    if (m_filename.empty())
        throwError("Required argument 'filename' (Draco array name) "
            "not provided.");
}

void DracoReader::initialize()
{
    if (!FileUtils::fileExists(m_filename))
        throwError("File '" + m_filename + "' does not exist");

    m_istreamPtr = Utils::openFile(m_filename, true);
    if (!m_istreamPtr)
        throwError("Unable to open file '" + m_filename + "' ");

    m_data.assign(std::istreambuf_iterator<char>(*m_istreamPtr),
                std::istreambuf_iterator<char>());
    Utils::closeFile(m_istreamPtr);

    draco::DecoderBuffer draco_buffer;
    draco_buffer.Init(m_data.data(), m_data.size());

    draco::Decoder decoder;

    auto geom_status = draco::Decoder::GetEncodedGeometryType(&draco_buffer);
    if (!geom_status.ok())
    {
        return throwError(geom_status.status().error_msg());
    }
    const draco::EncodedGeometryType geom_type = geom_status.value();
    draco::StatusOr<std::unique_ptr<draco::PointCloud>> pc_status = decoder.DecodePointCloudFromBuffer(&draco_buffer);

    if (!pc_status.ok())
    {
        return throwError(pc_status.status().error_msg());
    }

    m_pc = std::move(pc_status).value();

    m_count = m_pc->num_points();
}

void DracoReader::addOneDimension(Dimension::Id id, const draco::PointAttribute* attr, PointLayoutPtr layout, int index, int attNum)
{
    draco::DataType dt = attr->data_type();
    draco::GeometryAttribute::Type dracoAtt = attr->attribute_type();
    Dimension::Type pdalType = getPdalType(dt);
    int offset = draco::DataTypeLength(dt);
    const uint8_t *address = attr->GetAddressOfMappedIndex(draco::PointIndex(0));
    layout->registerDim(id);

    const DimensionInfo dimInfo = { id, dracoAtt, pdalType, index, offset, attNum, address };
    m_dimensions.push_back(dimInfo);

}

void DracoReader::addDimensions(PointLayoutPtr layout)
{
    using namespace draco;

    log()->get(LogLevel::Debug) << "draco pc num_attributes: "
                                << m_pc->num_attributes() << std::endl;;

    const GeometryMetadata *metadata = m_pc->GetMetadata();
    for (int i=0; i < m_pc->num_attributes(); ++i)
    {
        const PointAttribute* attr = m_pc->GetAttributeByUniqueId(i);
        if (attr == nullptr)
            throw new pdal_error("Invalid draco attribute configuration");
        const AttributeMetadata* attr_metadata = m_pc->GetAttributeMetadataByAttributeId(i);
        DataType dt = attr->data_type();
        GeometryAttribute::Type at = attr->attribute_type();

        int8_t nc = attr->num_components();
        log()->get(LogLevel::Debug) << "named component: "
                                    << GeometryAttribute::TypeToString(at)
                                    << " subcomponents: " << (int)nc << std::endl;;
        std::string name;
        if (attr_metadata)
        {
            log()->get(LogLevel::Debug) << "number of metadata entries: "
                                        << attr_metadata->num_entries()
                                        << std::endl;
            attr_metadata->GetEntryString("name", &name);
            log()->get(LogLevel::Debug) << "metadata name: "
                                        << name
                                        << std::endl;
        }
        switch (at)
        {
            case GeometryAttribute::POSITION:
            {
                addOneDimension(Dimension::Id::X, attr, layout, i, 0);
                addOneDimension(Dimension::Id::Y, attr, layout, i, 1);
                addOneDimension(Dimension::Id::Z, attr, layout, i, 2);
                break;
            }
            case GeometryAttribute::NORMAL:
            {
                addOneDimension(Dimension::Id::NormalX, attr, layout, i, 0);
                addOneDimension(Dimension::Id::NormalY, attr, layout, i, 1);
                addOneDimension(Dimension::Id::NormalZ, attr, layout, i, 2);
                break;
            }
            case GeometryAttribute::COLOR:
            {
                addOneDimension(Dimension::Id::Red, attr, layout, i, 0);
                addOneDimension(Dimension::Id::Green, attr, layout, i, 1);
                addOneDimension(Dimension::Id::Blue, attr, layout, i, 2);
            }
            case GeometryAttribute::TEX_COORD:
            {
                addOneDimension(Dimension::Id::TextureU, attr, layout, i, 0);
                addOneDimension(Dimension::Id::TextureV, attr, layout, i, 1);
                if (nc == 3)
                {
                    m_textureW = true;
                    addOneDimension(Dimension::Id::TextureW, attr, layout, i, 2);
                }
                break;
            }

            case GeometryAttribute::GENERIC:
            {
                Dimension::Id id = pdal::Dimension::id(name);
                if (id != Dimension::Id::Unknown) {
                    addOneDimension(id, attr, layout, i, 0);
                }
                break;
            }

            default:
                // Not supported draco domain types
                std::string attStr = GeometryAttribute::TypeToString(at);
                throw pdal_error("Gometry Attribute " +
                                    attStr + " is not a valid Draco attribute");
                break;
        }
    }
}


void DracoReader::ready(PointTableRef)
{
    // using namespace draco;
    // for (int at_id=0; at_id < m_pc->num_attributes(); ++at_id)
    // {
    //     const PointAttribute* attr = m_pc->GetAttributeByUniqueId(at_id);
    //     draco::DataType dt = attr->data_type();
    //     draco::GeometryAttribute::Type at = attr->attribute_type();

    //     std::string name;
    //     const AttributeMetadata* attr_metadata = m_pc->GetAttributeMetadataByAttributeId(at_id);
    //     if (attr_metadata)
    //     {
    //         attr_metadata->GetEntryString("name", &name);
    //     }

    //     switch (at)
    //     {
    //         case GeometryAttribute::POSITION:
    //             CopyAttributeData<double>(m_count, attr, m_positions);
    //             break;
    //         case GeometryAttribute::NORMAL:
    //             CopyAttributeData<double>(m_count, attr, m_normals);
    //             break;
    //         case GeometryAttribute::COLOR:
    //             CopyAttributeData<uint16_t>(m_count, attr, m_colors);
    //         case GeometryAttribute::TEX_COORD:
    //             CopyAttributeData<double>(m_count, attr, m_textures);
    //             break;

    //         case GeometryAttribute::GENERIC:
    //             if (name.length() > 0)
    //             {
    //                 Dimension::Id id = Dimension::id(name);
    //                 CopyAttributeData<double>(1, attr, m_generics.at(id));
    //             }

    //             break;

    //         default:
    //             // Unsupported draco domain types
    //             throw pdal_error("Unknown Geometry Attribute Type");
    //     }
    // }
}


bool DracoReader::processOne(PointViewPtr view, point_count_t pid)
{

    // point_count_t pid = point.pointId();
    for (auto& dim: m_dimensions)
    {
        //should get the attribute regardless of if it's a multi-dim attribute
        const int offset = dim.typeLength * (pid + dim.attNum);
        view->setField(dim.pdalId, dim.pdalType, pid, dim.address+offset);
    }

    return true;
}


point_count_t DracoReader::read(PointViewPtr view, point_count_t count)
{
    PointRef point = view->point(0);
    PointId id;
    for (id = 0; id < count; ++id)
    {
        point.setPointId(id);
        if (!processOne(view, point.pointId()))
            break;
    }
    return id;
}

void DracoReader::done(pdal::BasePointTable &table)
{
}

} // namespace pdal
