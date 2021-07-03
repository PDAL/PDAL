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
    std::vector<char> data;
    data.assign(std::istreambuf_iterator<char>(*m_istreamPtr),
                std::istreambuf_iterator<char>());
    Utils::closeFile(m_istreamPtr);

    draco::DecoderBuffer draco_buffer;
    draco_buffer.Init(data.data(), data.size());

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
}

void DracoReader::addOneDimension(Dimension::Id id, const draco::PointAttribute* attr, PointLayoutPtr layout, int index, int attNum)
{
    //find corresponding pdal data type
    draco::DataType dt = attr->data_type();
    Dimension::Type pdalType = getPdalType(dt);

    //add to pdal layout and dim vector
    layout->registerDim(id);
    const DimensionInfo dimInfo = { id, attr, pdalType, attNum };
    m_dimensions.push_back(dimInfo);
}

void DracoReader::addDimensions(PointLayoutPtr layout)
{
    using namespace draco;

    log()->get(LogLevel::Debug) << "Number of attributes in point cloud: "
                                << m_pc->num_attributes() << std::endl;;

    //iterate and collect information on draco attributes
    for (int i=0; i < m_pc->num_attributes(); ++i)
    {
        const PointAttribute* attr = m_pc->GetAttributeByUniqueId(i);
        if (attr == nullptr)
            throw new pdal_error("Invalid draco attribute configuration");
        const AttributeMetadata* attr_metadata = m_pc->GetAttributeMetadataByAttributeId(i);

        //get types
        DataType dt = attr->data_type();
        Dimension::Type pt = getPdalType(dt);
        GeometryAttribute::Type at = attr->attribute_type();

        int8_t nc = attr->num_components();
        log()->get(LogLevel::Debug) << "Dimension read: "
                                    << GeometryAttribute::TypeToString(at)
                                    << ", Data type: " << Dimension::interpretationName(pt)
                                    << std::endl;;
        std::string name;
        if (attr_metadata)
        {
            attr_metadata->GetEntryString("name", &name);
            log()->get(LogLevel::Debug) << "  Generic type name: "
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
                break;
            }
            case GeometryAttribute::TEX_COORD:
            {
                addOneDimension(Dimension::Id::TextureU, attr, layout, i, 0);
                addOneDimension(Dimension::Id::TextureV, attr, layout, i, 1);
                if (nc == 3)
                {
                    addOneDimension(Dimension::Id::TextureW, attr, layout, i, 2);
                }
                break;
            }

            case GeometryAttribute::GENERIC:
            {
                Dimension::Id id = pdal::Dimension::id(name);
                if (id != Dimension::Id::Unknown) {
                    addOneDimension(id, attr, layout, i, 0);
                } else {
                    layout->assignDim(name, pt);
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
}

point_count_t DracoReader::read(PointViewPtr view, point_count_t count)
{
    PointId id = view->size();
    point_count_t numRead = 0;

    count = (std::min)(count, (point_count_t)m_pc->num_points());
    while (numRead < count)
    {
        for (auto& dim: m_dimensions)
        {
            const uint8_t *src = dim.attr->GetAddressOfMappedIndex(draco::PointIndex((uint32_t)numRead)) +
                draco::DataTypeLength(dim.attr->data_type()) * dim.attNum;
            view->setField(dim.pdalId, dim.pdalType, id, src);
        }
        ++id;
        ++numRead;
    }
    return id;
}

void DracoReader::done(pdal::BasePointTable &table)
{
}

} // namespace pdal
