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
    // args.add("quantization", "Amount of quantization during draco encoding", m_quantization);
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
    pdal::Dimension::IdList dimensions = layout->dims();

    for (auto& dim : dimensions)
    {
        //add point_attributes to draco pointcloud
        //For known dimensions, add them to the draco geometry attribute vector
        //For generics, directly add them to the draco point_attribute so that
        // we can add the name at the same time. After this loop we'll remove
        // duplicates of the known attributes and then add those to the
        // point_attribute.
        int numComponents(0);

        //run through each one dimension, if it's a part of a group then check
        //for the others and remove them if they already exist
        //num components - eg. Position incorporates X, Y, Z. So 3 components.
        switch (dim)
        {
            case Dimension::Id::X:
                m_dims[draco::GeometryAttribute::POSITION] = components(dimensions,
                    Dimension::IdList{ Dimension::Id::Y, Dimension::Id::Z });
                addAttribute(draco::GeometryAttribute::POSITION, numComponents);
                break;
            case Dimension::Id::Y:
                numComponents = components(dimensions,
                    Dimension::IdList{ Dimension::Id::X, Dimension::Id::Z });
                addAttribute(draco::GeometryAttribute::POSITION, numComponents);
                break;
            case Dimension::Id::Z:
                numComponents = components(dimensions,
                    Dimension::IdList{ Dimension::Id::X, Dimension::Id::Y });
                addAttribute(draco::GeometryAttribute::POSITION, numComponents);
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
                m_genericDims.push_back(dim);
                break;
        }
    }
}

int DracoWriter::components(Dimension::IdList &list, Dimension::IdList typeVector)
{
    int numComponents(0);
    for (auto &t: typeVector)
    {
        auto it = std::find(list.begin(), list.end(), t);
        if (it != list.end())
        {
            list.erase(it);
            ++numComponents;
        }
    }
    return numComponents;
}

void DracoWriter::addAttribute(draco::GeometryAttribute::Type t, Dimension::Id pt, int n) {
    //get pdal dimension data type
    Dimension::Type pdalDataType = Dimension::defaultType(pt);
    //get corresponding draco data type
    draco::DataType dracoDataType = m_dracoTypeMap[pdalDataType];
    //add it to the pointcloud attributes
    m_pc.AddAttribute(t, n, dracoDataType);
}

void DracoWriter::addAttribute(draco::GeometryAttribute::Type t, int n) {
    //get straight lookup working first, then add lookup for datatype
    draco::DataType dataType = draco::DT_INVALID;
    if (t == draco::GeometryAttribute::POSITION) {
        dataType = draco::DT_FLOAT32;
    }
    if (t == draco::GeometryAttribute::NORMAL) {
        dataType = draco::DT_FLOAT32;
    }
    if (t == draco::GeometryAttribute::TEX_COORD) {
        dataType = draco::DT_FLOAT64;
    }
    if (t == draco::GeometryAttribute::COLOR) {
        dataType = draco::DT_FLOAT64;
    }
    m_pc.AddAttribute(t, n, dataType);
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
