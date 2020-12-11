/******************************************************************************
* Copyright (c) 2020, Hobu, Inc.
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

#include <string.h>
#include <cctype>
#include <limits>

#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>

#include "DracoWriter.hpp"


namespace pdal {

static PluginInfo const s_info
{
    "writers.draco",
    "Write data using Draco.",
    "http://pdal.io/stages/writers.draco.html"
};

struct DracoWriter::Args
{
    std::string m_filename;
    std::map<std::string, std::string> m_dimMap;
};


CREATE_SHARED_STAGE(DracoWriter, s_info)

// void writeAttributeValue(DracoWriter::DimBuffer& dim,
    // PointRef& point, size_t idx)
// {
//     Everything e;

//     switch (dim.m_type)
//     {
//     case Dimension::Type::Double:
//         e.d = point.getFieldAs<double>(dim.m_id);
//         break;
//     case Dimension::Type::Float:
//         e.f = point.getFieldAs<float>(dim.m_id);
//         break;
//     case Dimension::Type::Signed8:
//         e.s8 = point.getFieldAs<int8_t>(dim.m_id);
//         break;
//     case Dimension::Type::Signed16:
//         e.s16 = point.getFieldAs<int16_t>(dim.m_id);
//         break;
//     case Dimension::Type::Signed32:
//         e.s32 = point.getFieldAs<int32_t>(dim.m_id);
//         break;
//     case Dimension::Type::Signed64:
//         e.s64 = point.getFieldAs<int64_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned8:
//         e.u8 = point.getFieldAs<uint8_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned16:
//         e.u16 = point.getFieldAs<uint16_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned32:
//         e.u32 = point.getFieldAs<uint32_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned64:
//         e.u64 = point.getFieldAs<uint64_t>(dim.m_id);
//         break;
//     default:
//         throw pdal_error("Unsupported attribute type for " + dim.m_name);
//     }

//     size_t size = Dimension::size(dim.m_type);
//     memcpy(dim.m_buffer.data() + (idx * size), &e, size);
// }


DracoWriter::DracoWriter():
    m_args(new DracoWriter::Args)
{
//     m_args->m_defaults = NL::json::parse(attributeDefaults);
}


DracoWriter::~DracoWriter(){}


std::string DracoWriter::getName() const { return s_info.name; }

void DracoWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    // TODO add dimension map as an argument
    // args.add("dimensions", "Map of pdal to draco dimensions ", m_dimensions);
    // TODO add quantization as an argument
    // args.add("precision", "Degree of precision on float and double values", m_precision);
}

struct FileStreamDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
        {
            ptr->flush();
            Utils::closeFile(ptr);
        }
    }
};

// std::vector<std::string> DracoWriter::getHeaders(PointTableRef table) {
//     const PointLayoutPtr layout(table.layout());
//     for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
//     {
//         if (di != m_dims.begin())
//             *m_stream << m_delimiter;

//         if (m_quoteHeader)
//             *m_stream << "\"" << layout->dimName(di->id) << "\"";
//         else
//             *m_stream << layout->dimName(di->id);
//     }
//     *m_stream << m_newline;

// }

void DracoWriter::initialize(PointTableRef table)
{
    m_stream = FileStreamPtr(Utils::createFile(m_filename, true),
        FileStreamDeleter());
    if (!m_stream)
        throwError("Couldn't open '" + m_filename + "' for output.");

}


void DracoWriter::ready(pdal::BasePointTable &table)
{
    *m_stream << std::fixed;
    auto layout = table.layout();

    //pointcloud initialization
    //   void Init(Type attribute_type, DataBuffer *buffer, int8_t num_components,
    //             DataType data_type, bool normalized, int64_t byte_stride,
    //             int64_t byte_offset);
    auto dimensions = layout->dims();
    const auto numComponents = dimensions.size();

    for (auto& dim : dimensions)
    {
        //add point_attributes to draco pointcloud
        //For known dimensions, add them to the draco geometry attribute vector
        //For generics, directly add them to the draco point_attribute so that
        // we can add the name at the same time. After this loop we'll remove
        // duplicates of the known attributes and then add those to the
        // point_attribute.
        switch (dim)
        {
            case Dimension::Id::X:
                m_dims.push_back(draco::GeometryAttribute::POSITION);
                break;
            case Dimension::Id::Y:
                m_dims.push_back(draco::GeometryAttribute::POSITION);
                break;
            case Dimension::Id::Z:
                m_dims.push_back(draco::GeometryAttribute::POSITION);
                break;
            case Dimension::Id::NormalX:
                m_dims.push_back(draco::GeometryAttribute::NORMAL);
                break;
            case Dimension::Id::NormalY:
                m_dims.push_back(draco::GeometryAttribute::NORMAL);
                break;
            case Dimension::Id::NormalZ:
                m_dims.push_back(draco::GeometryAttribute::NORMAL);
                break;
            case Dimension::Id::Red:
                m_dims.push_back(draco::GeometryAttribute::COLOR);
                break;
            case Dimension::Id::Green:
                m_dims.push_back(draco::GeometryAttribute::COLOR);
                break;
            case Dimension::Id::Blue:
                m_dims.push_back(draco::GeometryAttribute::COLOR);
                break;
            //TODO Account for possibily that it's just TextureU and TextureV
            case Dimension::Id::TextureU:
                m_dims.push_back(draco::GeometryAttribute::TEX_COORD);
                break;
            case Dimension::Id::TextureV:
                m_dims.push_back(draco::GeometryAttribute::TEX_COORD);
                break;
            case Dimension::Id::TextureW:
                m_dims.push_back(draco::GeometryAttribute::TEX_COORD);
                break;
            //for generic attributes, add them to the point_attribute now
            default:
                m_genericDims.push_back(Dimension::name(dim));
                break;
        }
    }

    m_dims.erase( std::unique( m_dims.begin(), m_dims.end() ), m_dims.end() );
    //each point_attribute needs to be added to the pointcloud
    m_current_idx = 0;
}


void DracoWriter::write(const PointViewPtr view)
{
    //testing just writing position
    //create our point_attribute with just POSITION(x,y,z)
    draco::PointAttribute pa;
    draco::PointCloudBuilder pc_builder;

    pc_builder.Start(view->size());
    const int32_t posId = pc_builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    // pa.Init(draco::GeometryAttribute::POSITION, 1, draco::DT_FLOAT32, false, view->size());

    PointRef point(*view, 0);
    std::vector<float> position(view->size()*3, 0.f);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        const auto pointId = draco::PointIndex(idx);
        position[idx * 3 + 0] = point.getFieldAs<float>(Dimension::Id::X);
        position[idx * 3 + 1] = point.getFieldAs<float>(Dimension::Id::Y);
        position[idx * 3 + 2] = point.getFieldAs<float>(Dimension::Id::Z);
    }
    pc_builder.SetAttributeValuesForAllPoints(posId, position.data(), 0);
    std::unique_ptr<draco::PointCloud> pc = pc_builder.Finalize(false);

    draco::EncoderBuffer buffer;
    draco::Encoder encoder;

    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 11);
    const auto status = encoder.EncodePointCloudToBuffer(*pc, &buffer);

    //TODO add error message from draco status?
    if (!status.ok())
        throw pdal_error("Error encoding draco pointcloud");

    const auto bufferSize = buffer.size();
    std::vector<char> *output = buffer.buffer();
    for (auto &i : *output)
        *m_stream << i;
}


void DracoWriter::done(PointTableRef table)
{

}



} // namespace pdal
