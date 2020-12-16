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
    // std::map<std::string, std::string> m_dimMap;
};


CREATE_SHARED_STAGE(DracoWriter, s_info)

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

void DracoWriter::addAttribute(Dimension::Id pt, int n) {
    //get pdal dimension data type
    Dimension::Type pdalDataType = Dimension::defaultType(pt);
    //get corresponding draco data type
    draco::DataType dracoDataType = typeMap.at(pdalDataType);
    //add it to the pointcloud attributes
    m_pc.AddAttribute(draco::GeometryAttribute::GENERIC, n, dracoDataType);
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
    m_attMap[t] = m_pc.AddAttribute(t, n, dataType);
}

void DracoWriter::initPointCloud()
{
    //go through known draco attributes and add to the pointcloud
    for (auto &dim: m_dims)
    {
        addAttribute(dim.first, dim.second);
    }
}

void DracoWriter::write(const PointViewPtr view)
{
    //initialize pointcloud builder
    m_pc.Start(view->size());
    initPointCloud();
    // pc_builder.Start(view->size());
    // const int32_t posId = pc_builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    // pa.Init(draco::GeometryAttribute::POSITION, 1, draco::DT_FLOAT32, false, view->size());

    PointRef point(*view, 0);
    std::vector<double> position(view->size()*3, 0.f);
    std::vector<uint16_t> color(view->size()*3, 0.f);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        const auto pointId = draco::PointIndex(idx);

        if (m_attMap.find(draco::GeometryAttribute::POSITION) != m_attMap.end())
        {
            std::vector<double> pos(3, 0.f);
            const int id = m_attMap[draco::GeometryAttribute::POSITION];
            pos[0] = point.getFieldAs<double>(Dimension::Id::X);
            pos[1] = point.getFieldAs<double>(Dimension::Id::Y);
            pos[2] = point.getFieldAs<double>(Dimension::Id::Z);
            m_pc.SetAttributeValueForPoint(id, pointId, pos.data());
        }

        if (m_attMap.find(draco::GeometryAttribute::COLOR) != m_attMap.end())
        {
            std::vector<uint16_t> rgb(3, 0.f);
            const int id = m_attMap[draco::GeometryAttribute::COLOR];
            rgb[0] = point.getFieldAs<uint16_t>(Dimension::Id::Red);
            rgb[1] = point.getFieldAs<uint16_t>(Dimension::Id::Green);
            rgb[2] = point.getFieldAs<uint16_t>(Dimension::Id::Blue);
            m_pc.SetAttributeValueForPoint(id, pointId, rgb.data());
        }

        if (m_attMap.find(draco::GeometryAttribute::TEX_COORD) != m_attMap.end())
        {
            std::vector<double> tex(3, 0.f);
            const int id = m_attMap[draco::GeometryAttribute::TEX_COORD];
            tex[0] = point.getFieldAs<double>(Dimension::Id::TextureU);
            tex[1] = point.getFieldAs<double>(Dimension::Id::TextureV);
            tex[2] = point.getFieldAs<double>(Dimension::Id::TextureW);
            m_pc.SetAttributeValueForPoint(id, pointId, tex.data());
        }

        if (m_attMap.find(draco::GeometryAttribute::NORMAL) != m_attMap.end())
        {
            std::vector<double> norm(3, 0.f);
            const int id = m_attMap[draco::GeometryAttribute::NORMAL];
            norm[0] = point.getFieldAs<double>(Dimension::Id::NormalX);
            norm[1] = point.getFieldAs<double>(Dimension::Id::NormalY);
            norm[2] = point.getFieldAs<double>(Dimension::Id::NormalZ);
            m_pc.SetAttributeValueForPoint(id, pointId, norm.data());
        }
    }
    // m_pc.SetAttributeValuesForAllPoints(0, position.data(), 0);
    // m_pc.SetAttributeValuesForAllPoints(1, color.data(), 0);
    std::unique_ptr<draco::PointCloud> pc = m_pc.Finalize(false);

    draco::EncoderBuffer buffer;
    draco::Encoder encoder;

    //TODO make encoding method a writer argument?
    encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 11);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, 8);
    const auto status = encoder.EncodePointCloudToBuffer(*pc, &buffer);

    //TODO add error message from draco status?
    if (!status.ok()) {
        std::cout << "Error: " << status.error_msg() << std::endl;
        throw pdal_error("Error encoding draco pointcloud");
    }

    const auto bufferSize = buffer.size();
    std::vector<char> *output = buffer.buffer();
    for (auto &i : *output)
        *m_stream << i;
}


void DracoWriter::done(PointTableRef table)
{

}



} // namespace pdal
