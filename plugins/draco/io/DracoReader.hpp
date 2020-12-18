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

#pragma once

#define NOMINMAX

#include <iostream>

#include <pdal/Reader.hpp>
#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/decode.h>

namespace {
    const std::map<draco::DataType, pdal::Dimension::Type> dracoTypeMap =
    {
        { draco::DataType::DT_FLOAT64, pdal::Dimension::Type::Double },
        { draco::DataType::DT_FLOAT32, pdal::Dimension::Type::Float },
        { draco::DataType::DT_INT8, pdal::Dimension::Type::Signed8 },
        { draco::DataType::DT_INT16, pdal::Dimension::Type::Signed16 },
        { draco::DataType::DT_UINT16, pdal::Dimension::Type::Unsigned16 },
        { draco::DataType::DT_INT32, pdal::Dimension::Type::Signed32 },
        { draco::DataType::DT_UINT32, pdal::Dimension::Type::Unsigned32 },
        { draco::DataType::DT_INT64, pdal::Dimension::Type::Signed64 },
        { draco::DataType::DT_UINT64, pdal::Dimension::Type::Unsigned64 }
    };
};

namespace pdal
{

class PDAL_DLL DracoReader : public Reader
{
public:

    DracoReader() = default;
    std::string getName() const;
private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void prepared(PointTableRef);
    virtual void ready(PointTableRef);
    virtual bool processOne(PointRef& point);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);

    DracoReader(const DracoReader&) = delete;
    DracoReader& operator=(const DracoReader&) = delete;

    std::istream* m_istreamPtr;
    std::vector<char> m_data;
    draco::DecoderBuffer m_draco_buffer;
    std::unique_ptr<draco::PointCloud> m_pc;
    std::vector<double> m_positions;
    std::vector<uint16_t> m_colors;
    std::vector<double> m_normals;
    std::vector<double> m_textures;
    std::map<Dimension::Id, std::vector<double>> m_generics;

};

} // namespace pdal
