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

const std::map<pdal::Dimension::Type, draco::DataType> typeMap =
{
    { pdal::Dimension::Type::Double,      draco::DataType::DT_FLOAT64 },
    { pdal::Dimension::Type::Float,       draco::DataType::DT_FLOAT32 },
    { pdal::Dimension::Type::Signed8,     draco::DataType::DT_INT8 },
    { pdal::Dimension::Type::Unsigned8,   draco::DataType::DT_UINT8 },
    { pdal::Dimension::Type::Signed16,    draco::DataType::DT_INT16 },
    { pdal::Dimension::Type::Unsigned16,  draco::DataType::DT_UINT16 },
    { pdal::Dimension::Type::Signed32,    draco::DataType::DT_INT32 },
    { pdal::Dimension::Type::Unsigned32,  draco::DataType::DT_UINT32 },
    { pdal::Dimension::Type::Signed64,    draco::DataType::DT_INT64 },
    { pdal::Dimension::Type::Unsigned64,  draco::DataType::DT_UINT64 },
};

const std::map<pdal::Dimension::Id, draco::GeometryAttribute::Type> dimMap =
{
    { pdal::Dimension::Id::X,         draco::GeometryAttribute::POSITION },
    { pdal::Dimension::Id::Y,         draco::GeometryAttribute::POSITION },
    { pdal::Dimension::Id::Z,         draco::GeometryAttribute::POSITION },
    { pdal::Dimension::Id::NormalX,   draco::GeometryAttribute::NORMAL },
    { pdal::Dimension::Id::NormalY,   draco::GeometryAttribute::NORMAL },
    { pdal::Dimension::Id::NormalZ,   draco::GeometryAttribute::NORMAL },
    { pdal::Dimension::Id::Red,       draco::GeometryAttribute::COLOR },
    { pdal::Dimension::Id::Green,     draco::GeometryAttribute::COLOR },
    { pdal::Dimension::Id::Blue,      draco::GeometryAttribute::COLOR },
    { pdal::Dimension::Id::TextureU,  draco::GeometryAttribute::TEX_COORD },
    { pdal::Dimension::Id::TextureV,  draco::GeometryAttribute::TEX_COORD },
    { pdal::Dimension::Id::TextureW,  draco::GeometryAttribute::TEX_COORD }
};

static PluginInfo const s_info
{
    "writers.draco",
    "Write data using Draco.",
    "http://pdal.io/stages/writers.draco.html"
};

CREATE_SHARED_STAGE(DracoWriter, s_info)

DracoWriter::DracoWriter()
{
}

DracoWriter::~DracoWriter(){}

std::string DracoWriter::getName() const { return s_info.name; }

void DracoWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("dimensions", "Json mapping of pdal dimensions to desired data types", m_userDimJson);
    args.add("quantization", "Json mapping of Draco Attributes to desired quantization level", m_userQuant);
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
                " is not a valid Draco Geometry Attribute");
        m_quant[attribute] = quant;
    }
}


DracoWriter::DimensionInfo *DracoWriter::findDimInfo(draco::GeometryAttribute::Type dt) {
    for (auto &dim: m_dims) {
        if (dim.dracoAtt == dt) return &dim;
    }
    throw pdal_error("Draco attribute " + draco::GeometryAttribute::TypeToString(dt) +
        " doesn't exist in this file.");
}

DracoWriter::DimensionInfo *DracoWriter::findDimInfo(Dimension::Id pt)
{
    for (auto &dim: m_dims) {
        for (auto &dimType: dim.pdalDims) {
            if (dimType.m_id == pt) return &dim;
        }
    }
    throw pdal_error("Dimension Id " + Dimension::name(pt) +
        " doesn't exist in this file.");
}

Dimension::IdList DracoWriter::getDimensions(draco::GeometryAttribute::Type dt)
{
    Dimension::IdList returnList;
    for (auto &dim: dimMap) {
        if (dim.second == dt) returnList.push_back(dim.first);
    }
    return returnList;
}

void DracoWriter::parseDimensions(BasePointTable &table)
{
    PointLayoutPtr layout = table.layout();
    pdal::Dimension::IdList dimensions = layout->dims();
    for (auto& dim : dimensions)
    {
        std::string dimString = Dimension::name(dim);
        //if it doesn't exist in the json, then skip it
        if (!m_userDimJson.contains(dimString)) continue;
        //else add it to the list
        auto dataType = m_userDimJson[dimString].get<std::string>();

        log()->get(LogLevel::Info) << "Key: " << dimString << ", Value: "
            << dataType << std::endl;
        const auto it = dimMap.find(dim);
        Dimension::Type pdalType = Dimension::type(dataType);
        if (it != dimMap.end())
        {
            //get draco type associated with the current dimension in loop
            draco::GeometryAttribute::Type dracoType = it->second;
            //search through DimInfo vector to see if we already have the draco attribute
            bool found = false;
            for (auto &dimInfo: m_dims) {
                if (dimInfo.dracoAtt == dracoType) {
                    dimInfo.pdalDims.push_back(DimType(dim, pdalType));
                    found = true;
                    break;
                }
            }
            //if it's not there, add it to the vector
            if (!found) {
                DimensionInfo d = {
                    dracoType,
                    -1, //default attribute id to -1
                    { DimType(dim, pdalType) }
                };
                m_dims.push_back(d);
            }
        }
        else //not a "known" draco type
        {
            //add generic dimension to DimensionInfo vector
            DimensionInfo d = {
                draco::GeometryAttribute::GENERIC,
                -1, //default attribute id to -1
                { DimType(dim, pdalType) }
            };
            m_dims.push_back(d);
        }

    }

    //go through types of m_dims pdal ids and make sure they're consistent
    for (auto &dimInfo: m_dims) {
        int numDims = dimInfo.pdalDims.size();
        //start with first type
        DimType preType = dimInfo.pdalDims[0];
        //go through following types and make sure they match
        for (int i = 1; i < numDims; ++i) {
            DimType curType = dimInfo.pdalDims[i];
            std::string curName = Dimension::name(curType.m_id);
            std::string preName = Dimension::name(preType.m_id);

            if (curType.m_type != preType.m_type)
                throw pdal_error("Ids " + curName + " and " + preName +
                    " are not matching types, but are part of the same draco attribute.");
            preType = dimInfo.pdalDims[i];
        }
    }

    //add dimensions that should be there (eg POSITION = X, Y, Z) with zero fill
    //If a dimension isn't specified, but is necessary for multi-dimensional
    //Geometry Attributes, fill it with zeros
    for (auto &dimInfo: m_dims) {
        int numDims = dimInfo.pdalDims.size();
        auto idList = getDimensions(dimInfo.dracoAtt);
        //we can assume that all dimensions here have the same type
        //and that the first entry will be filled
        Dimension::Type pdalType = dimInfo.pdalDims[0].m_type;

        for (size_t i = numDims; i < idList.size(); ++i) {
            DimType d(Dimension::Id::Unknown, pdalType);
            dimInfo.pdalDims.push_back(d);
        }
    }

}

void DracoWriter::createDims(BasePointTable &table) {
    PointLayoutPtr layout = table.layout();
    pdal::Dimension::IdList dimensions = layout->dims();
    for (auto& dim : dimensions)
    {
        //check that dimension exists in the dimension argument
        std::string dimStr = Dimension::name(dim);
        Dimension::Type pdalType = layout->dimType(dim);
        //create list of dimensions needed int numComponents(1);
        const auto it = dimMap.find(dim);
        if (it != dimMap.end())
        {
            //get draco type associated with the current dimension in loop
            draco::GeometryAttribute::Type dracoType = it->second;
            //search through DimInfo vector to see if we already have the draco attribute
            bool found = false;
            for (auto &dimInfo: m_dims) {
                if (dimInfo.dracoAtt == dracoType) {
                    dimInfo.pdalDims.push_back(DimType(dim, pdalType));
                    found = true;
                    break;
                }
            }
            //if it's not there, add it to the vector
            if (!found) {
                DimensionInfo d = {
                    dracoType,
                    -1, //default attribute id to -1
                    { DimType(dim, pdalType) }
                };
                m_dims.push_back(d);
            }

        }
        else //not a "known" draco type
        {
            //add generic dimension to DimensionInfo vector
            DimensionInfo d = {
                draco::GeometryAttribute::GENERIC,
                -1, //default attribute id to -1
                { DimType(dim, pdalType) }
            };
            m_dims.push_back(d);
        }
    }
}

void DracoWriter::ready(pdal::BasePointTable &table)
{
    if(m_userDimJson.is_object()) parseDimensions(table);
    else createDims(table);
}

void DracoWriter::addGeneric(Dimension::Id pt)
{
    //get pdal dimension data type
    Dimension::Type pdalDataType = Dimension::defaultType(pt);
    //get corresponding draco data type
    draco::DataType dracoDataType = typeMap.at(pdalDataType);

    //add generic type to the pointcloud
    draco::GeometryAttribute ga;
    ga.Init(draco::GeometryAttribute::GENERIC, nullptr, 1, dracoDataType,
            false, draco::DataTypeLength(dracoDataType) * 1, 0);
    auto attId = m_pc->AddAttribute(ga, true, m_pc->num_points());

    //create metadata object and add dimension string as name
    draco::Metadata metadata;
    const std::string attName = "name";
    const std::string name = Dimension::name(pt);
    metadata.AddEntryString(attName, name);
    std::unique_ptr<draco::AttributeMetadata> metaPtr(new draco::AttributeMetadata(metadata));

    //attach metadata to generic attribute in pointcloud
    m_pc->AddAttributeMetadata(attId, std::move(metaPtr));

    //update attribute id
    DimensionInfo *dimInfo = findDimInfo(pt);
    dimInfo->attId = attId;
}

void DracoWriter::addAttribute(draco::GeometryAttribute::Type t, int n)
{
    // - iterate over values in dimMap, which are draco dimensions
    // - use the pdal dim associated with it to get the pdal data type
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
    int attId = m_pc->AddAttribute(ga, true, m_pc->num_points());

    //update attribute id
    DimensionInfo *dimInfo = findDimInfo(t);
    dimInfo->attId = attId;
}

void DracoWriter::initPointCloud(point_count_t size)
{
    //begin initialization of the point cloud with the point count
    m_pc->set_num_points((uint32_t)size);
    //go through known draco attributes and add to the pointcloud
    for (auto &dim: m_dims) {
        if (dim.dracoAtt == draco::GeometryAttribute::GENERIC) {
            addGeneric(dim.pdalDims[0].m_id);
        }
        else {
            addAttribute(dim.dracoAtt, dim.pdalDims.size());
        }
    }
}

void DracoWriter::addPoint(DimensionInfo dim, PointRef &point, PointId idx)
{
    const auto pointId = draco::PointIndex((uint32_t)idx);

    //find data type and create buffer
    Dimension::Type dataType = dim.pdalDims[0].m_type;
    size_t size = Dimension::size(dataType) * dim.pdalDims.size();

    std::vector<char> buffer(size, 0);
    //fill buffer
    point.getPackedData(dim.pdalDims, buffer.data());
    //add to draco pointcloud
    draco::PointAttribute *const att = m_pc->attribute(dim.attId);
    att->SetAttributeValue(att->mapped_index(pointId), buffer.data());
}

void DracoWriter::write(const PointViewPtr view)
{
    //initialize pointcloud builder
    initPointCloud(view->size());

    PointRef point(*view, 0);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        const auto pointId = draco::PointIndex((uint32_t)idx);
        for (auto &dim: m_dims) {
            addPoint(dim, point, idx);
        }
    }

    draco::EncoderBuffer buffer;
    draco::Encoder encoder;
    encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);

    //set quantization levels based on either defaults or user specifications
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

    std::vector<char> *output = buffer.buffer();

    for (auto &i : *output)
        *m_stream << i;
}


void DracoWriter::done(PointTableRef table)
{
    m_stream.reset();
}

} // namespace pdal
