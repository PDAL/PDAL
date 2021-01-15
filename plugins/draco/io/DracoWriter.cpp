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
};


CREATE_SHARED_STAGE(DracoWriter, s_info)

DracoWriter::DracoWriter():
    m_args(new DracoWriter::Args)
{
}


DracoWriter::~DracoWriter(){}


std::string DracoWriter::getName() const { return s_info.name; }

void DracoWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("dimensions", "Map of pdal to draco dimensions ", m_userDimJson);
    args.add("quantization", "Amount of quantization during draco encoding", m_userQuant);
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

    parseQuants();
}

void DracoWriter::parseQuants() {
    if (!m_userQuant.is_object()) {
        if (std::string(m_userQuant.type_name()) == "null") return;
        throw pdal_error("Option 'quantization' must be a JSON object, not a " +
            std::string(m_userQuant.type_name()));
    }
    //Quantization levels are available for POSITION, NORMAL, TEX_COORD, COLOR,
    //and GENERIC draco types
    for (auto& entry : m_userQuant.items()) {
        const std::string attribute = entry.key();
        const int quant = entry.value().get<int>();
        if (m_quant.find(attribute) == m_quant.end())
            throw pdal_error("Quantization attribute " + attribute +
                " is not a valid Draco Goemetry Attribute");
        m_quant[attribute] = quant;
    }

}

void DracoWriter::parseDimensions()
{
    if(!m_userDimJson.is_object()) {
        if (std::string(m_userQuant.type_name()) == "null") return;
        throw pdal_error("Option 'dimensions' must be a JSON object, not a " +
            std::string(m_userDimJson.type_name()));
    }

    for(auto& entry : m_userDimJson.items()) {
        std::string dimString = entry.key();
        auto datasetName = entry.value();

        if(!datasetName.is_string()) {
            throw pdal_error("Every value in 'dimensions' must be a string. Key '"
                + dimString + "' has value with type '"
                + std::string(datasetName.type_name()) + "'");
        } else {
            log()->get(LogLevel::Info) << "Key: " << dimString << ", Value: "
                << datasetName << std::endl;

            Dimension::Id dimType = Dimension::id(dimString);
            m_userDimMap[dimType] = datasetName.get<std::string>();
        }
    }
    //account for possible errors in dimensions
    //x, y, z must all be specified if one of them is
    if (m_userDimMap.find(Dimension::Id::X) != m_userDimMap.end())
    {
        std::string x, y, z;
        try {
            x = m_userDimMap.at(Dimension::Id::X);
            y = m_userDimMap.at(Dimension::Id::Y);
            z = m_userDimMap.at(Dimension::Id::Z);
        } catch(std::out_of_range e) {
            throw pdal_error("X, Y, and Z dimensions must all be specified if one is.");
        }
        if ( x != y || y != z || x != z ) {
            throw pdal_error("X, Y, and Z dimensions must be of the same type");
        }
    }
    //red, green, and blue must all be specified if one of them is
    if (m_userDimMap.find(Dimension::Id::Red) != m_userDimMap.end())
    {
        std::string r, g, b;
        try {
            r = m_userDimMap.at(Dimension::Id::Red);
            g = m_userDimMap.at(Dimension::Id::Green);
            b = m_userDimMap.at(Dimension::Id::Blue);
        } catch(std::out_of_range e) {
            throw pdal_error("Red, Green, and Blue dimensions must all be specified if one is.");
        }
        if ( r != g || g != b || r != b ) {
            throw pdal_error("Red, Green, and Blue dimensions must be of the same type");
        }
    }
    //normals x, y, z must all be specified if one of them is
    if (m_userDimMap.find(Dimension::Id::NormalX) != m_userDimMap.end())
    {
        std::string nx, ny, nz;
        try {
            nx = m_userDimMap.at(Dimension::Id::NormalX);
            ny = m_userDimMap.at(Dimension::Id::NormalY);
            nz = m_userDimMap.at(Dimension::Id::NormalZ);
        } catch(std::out_of_range e) {
            throw pdal_error("NormalX, Y, and Z dimensions must all be specified if one is.");
        }
        if ( nx != ny || ny != nz || nx != nz ) {
            throw pdal_error("NormalX, Y, and Z dimensions must be of the same type");
        }
    }
    //make sure textures u, v, w are all the same, but it's possible there are
    //only u and v, so make sure we know how many dimensions are in the file first
    if (m_userDimMap.find(Dimension::Id::TextureU) != m_userDimMap.end())
    {
        std::string tu, tv, tw;
        int texNum = m_dims.at(draco::GeometryAttribute::TEX_COORD);
        try {
            tu = m_userDimMap.at(Dimension::Id::TextureU);
            tv = m_userDimMap.at(Dimension::Id::TextureV);
            if (texNum == 3) tw = m_userDimMap.at(Dimension::Id::TextureW);
            else tw = tv;
        } catch(std::out_of_range e) {
            throw pdal_error("TextureU, V, and W dimensions don't match the file.");
        }
        if ( tu != tv || tv != tw || tu != tw ) {
            throw pdal_error("TextureU, V, and W dimensions must be of the same type");
        }
    }
}

void DracoWriter::ready(pdal::BasePointTable &table)
{
    *m_stream << std::fixed;
    auto layout = table.layout();

    pdal::Dimension::IdList dimensions = layout->dims();

    for (auto& dim : dimensions)
    {
        //create list of dimensions needed
        int numComponents(1);
        const auto it = dimMap.find(dim);
        if (it != dimMap.end())
        {
            //get draco type associated with the current dimension in loop
            draco::GeometryAttribute::Type dracoType = it->second;
            if (!m_dims.count(dracoType)) m_dims[dracoType] = 1;
            else ++m_dims.at(dracoType);
        }
        else
        {
            m_genericDims.push_back(dim);
        }
    }
}

void DracoWriter::addGeneric(Dimension::Id pt, int n) {
    //get pdal dimension data type
    Dimension::Type pdalDataType = Dimension::defaultType(pt);
    //get corresponding draco data type
    draco::DataType dracoDataType = typeMap.at(pdalDataType);

    //add generic type to the pointcloud builder
    draco::GeometryAttribute ga;
    ga.Init(draco::GeometryAttribute::GENERIC, nullptr, n, dracoDataType,
            false, draco::DataTypeLength(dracoDataType) * n, 0);
    auto attId = m_pc->AddAttribute(ga, true, m_pc->num_points());

    //add generic attribute to map with its id
    m_genericMap[pt] = attId;

    //create metadata object and add dimension string as name
    draco::Metadata metadata;
    const std::string attName = "name";
    const std::string name = Dimension::name(pt);
    metadata.AddEntryString(attName, name);
    std::unique_ptr<draco::AttributeMetadata> metaPtr(new draco::AttributeMetadata(metadata));

    //attach metadata to generic attribute in pointcloud
    m_pc->AddAttributeMetadata(attId, std::move(metaPtr));
}

void DracoWriter::addAttribute(draco::GeometryAttribute::Type t, int n) {
    // - iterate over values in dimMap, which are draco dimensions
    // - use the pdal type associated with it to get the pdal data type
    // - use map of pdal types to draco types to get the correct draco type
    // - add it to the point cloud and add attribute id to attMap with dim as key
    const auto it = std::find_if(
        dimMap.begin(),
        dimMap.end(),
        [t](const std::pair<Dimension::Id, draco::GeometryAttribute::Type>& one)
        {
            return one.second == t;
        });

    pdal::Dimension::Type pdalType = Dimension::defaultType(it->first);
    draco::DataType dataType = draco::DT_INVALID;
    if (it != dimMap.end())
        dataType = typeMap.at(pdalType);

    //create geometry attribute and add to pointcloud
    draco::GeometryAttribute ga;
    ga.Init(t, nullptr, n, dataType, false, draco::DataTypeLength(dataType) * n, 0);
    m_attMap[t] = m_pc->AddAttribute(ga, true, m_pc->num_points());
}

void DracoWriter::initPointCloud(point_count_t size)
{
    //begin initialization of the point cloud with the point count
    m_pc->set_num_points(size);
    //go through known draco attributes and add to the pointcloud
    for (auto &dim: m_dims)
    {
        addAttribute(dim.first, dim.second);
    }
    for (auto &dim: m_genericDims)
    {
        addGeneric(dim, 1);
    }
}

void DracoWriter::addPoint(int attId, Dimension::IdList idList, PointRef &point, PointId idx)
{
    //get first id in list. All ids in any given list should be the same datatype
    Dimension::Id dim = idList[0];
    Dimension::Type type;

    if (m_userDimMap.find(dim) != m_userDimMap.end()) {
        std::string typeStr = m_userDimMap.at(dim);
        type = Dimension::type(typeStr);
    } else {
        type = Dimension::defaultType(dim);
    }

    //call addToPointCloud with appropriate datatype
    switch (type) {
        case Dimension::Type::Signed8:
            addToPointCloud<int8_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Unsigned8:
            addToPointCloud<uint8_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Signed16:
            addToPointCloud<int16_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Unsigned16:
            addToPointCloud<uint16_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Signed32:
            addToPointCloud<int32_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Unsigned32:
            addToPointCloud<uint32_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Signed64:
            addToPointCloud<int64_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Unsigned64:
            addToPointCloud<uint64_t>(attId, idList, point, idx);
            break;
        case Dimension::Type::Float:
            addToPointCloud<float>(attId, idList, point, idx);
            break;
        case Dimension::Type::Double:
            addToPointCloud<double>(attId, idList, point, idx);
            break;
        default:
            throw pdal_error("Invalid type for pdal dimension " + Dimension::name(dim));
            break;
    }

}

void DracoWriter::write(const PointViewPtr view)
{
    //initialize pointcloud builder
    initPointCloud(view->size());
    //we don't know how many texture dimensions are available until here
    parseDimensions();

    PointRef point(*view, 0);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        const auto pointId = draco::PointIndex(idx);

        if (m_attMap.find(draco::GeometryAttribute::POSITION) != m_attMap.end())
        {
            const int id = m_attMap[draco::GeometryAttribute::POSITION];
            Dimension::IdList idList {
                Dimension::Id::X,
                Dimension::Id::Y,
                Dimension::Id::Z
            };
            addPoint(id, idList, point, idx);
        }

        if (m_attMap.find(draco::GeometryAttribute::COLOR) != m_attMap.end())
        {
            const int id = m_attMap[draco::GeometryAttribute::COLOR];
            Dimension::IdList idList {
                Dimension::Id::Red,
                Dimension::Id::Green,
                Dimension::Id::Blue
            };
            addPoint(id, idList, point, idx);
        }

        if (m_attMap.find(draco::GeometryAttribute::TEX_COORD) != m_attMap.end())
        {
            const int n = m_dims[draco::GeometryAttribute::TEX_COORD];
            const int id = m_attMap[draco::GeometryAttribute::TEX_COORD];
            Dimension::IdList idList {
                Dimension::Id::TextureU,
                Dimension::Id::TextureV
            };
            if (n > 2)
                idList.push_back(Dimension::Id::TextureW);
            addPoint(id, idList, point, idx);
        }

        if (m_attMap.find(draco::GeometryAttribute::NORMAL) != m_attMap.end())
        {
            const int id = m_attMap[draco::GeometryAttribute::NORMAL];
            Dimension::IdList idList {
                Dimension::Id::NormalX,
                Dimension::Id::NormalY,
                Dimension::Id::NormalZ
            };
            addPoint(id, idList, point, idx);
        }

        //go through list of added dimensions and get the associated data
        for (auto& dim: m_genericMap)
        {
            const int id = dim.second;
            Dimension::IdList idList {
                dim.first
            };

            addPoint(id, idList, point, idx);
        }

    }

    draco::EncoderBuffer buffer;
    draco::Encoder encoder;
    encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);

    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, m_quant.at("POSITION"));
    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, m_quant.at("NORMAL"));
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, m_quant.at("TEX_COORD"));
    encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, m_quant.at("COLOR"));
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, m_quant.at("GENERIC"));
    const auto status = encoder.EncodePointCloudToBuffer(*m_pc, &buffer);

    if (!status.ok()) {
        const std::string err = status.error_msg();
        throw pdal_error("Error encoding draco pointcloud. " + err);
    }

    const auto bufferSize = buffer.size();
    std::vector<char> *output = buffer.buffer();

    for (auto &i : *output)
        *m_stream << i;
}


void DracoWriter::done(PointTableRef table)
{
    m_stream.reset();
}

} // namespace pdal
