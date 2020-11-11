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

#include "DracoReader.hpp"

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
//     args.addSynonym("filename", "array_name");
//     args.add("config_file", "Draco configuration file location",
//         m_cfgFileName);
//     args.add("chunk_size", "Draco read chunk size", m_chunkSize,
//         point_count_t(1000000));
//     args.add("stats", "Dump Draco query stats to stdout", m_stats, false);
//     args.add("bbox3d", "Bounding box subarray to read from Draco in format "
//         "([minx, maxx], [miny, maxy], [minz, maxz])", m_bbox);
}

void DracoReader::prepared(PointTableRef table)
{
    if (m_filename.empty())
        throwError("Required argument 'filename' (Draco array name) "
            "not provided.");
}


void DracoReader::initialize()
{

}

void DracoReader::addDimensions(PointLayoutPtr layout)
{

}

void DracoReader::ready(PointTableRef)
{
    m_istreamPtr = Utils::openFile(m_filename, true);
    m_data.assign(std::istreambuf_iterator<char>(*m_istreamPtr),
                std::istreambuf_iterator<char>());
    Utils::closeFile(m_istreamPtr);

    std::cerr << "size: " << m_data.size()  << std::endl;

    m_draco_buffer.Init(m_data.data(), m_data.size());


    draco::Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&m_draco_buffer);
    auto type_statusor = draco::Decoder::GetEncodedGeometryType(&m_draco_buffer);
    if (!type_statusor.ok())
    {
        return throwError(type_statusor.status().error_msg());
    }
    const draco::EncodedGeometryType geom_type = type_statusor.value();

}


// namespace
// {
//
// template<typename DracoEngine>
// size_t addFields(DracoEngine& engine, const DimTypeList& dims)
// {
//     using namespace Dimension;
//     using namespace draco;
//
//     size_t pointSize = 0;
//     for (auto di = dims.begin(); di != dims.end(); ++di)
//     {
//         switch (di->m_type)
//         {
//         case Type::Double:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_FLOAT64);
//             break;
//         case Type::Float:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_FLOAT32);
//             break;
//         case Type::Signed64:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT64);
//             break;
//         case Type::Signed32:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT32);
//             break;
//         case Type::Signed16:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT16);
//             break;
//         case Type::Signed8:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT8);
//             break;
//         case Type::Unsigned64:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT64);
//             break;
//         case Type::Unsigned32:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT32);
//             break;
//         case Type::Unsigned16:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT16);
//             break;
//         case Type::Unsigned8:
//             engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT8);
//             break;
//         default:
//             return 0;
//         }
//         pointSize += Dimension::size(di->m_type);
//     }
//     return pointSize;
// }
//
// } // anonymous namespace
//


bool DracoReader::processOne(PointRef& point)
{
    return false;
}


point_count_t DracoReader::read(PointViewPtr view, point_count_t count)
{
    PointRef point = view->point(0);
    PointId id;
    for (id = 0; id < count; ++id)
    {
        point.setPointId(id);
        if (!processOne(point))
            break;
    }
    return id;
}

void DracoReader::done(pdal::BasePointTable &table)
{
//     m_array->close();
}

} // namespace pdal
