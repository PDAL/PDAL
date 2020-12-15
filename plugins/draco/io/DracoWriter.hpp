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

#pragma once

#define NOMINMAX

#include <pdal/Writer.hpp>

#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/encode.h>
#include <draco/attributes/geometry_attribute.h>
#include <draco/attributes/point_attribute.h>
#include "draco/point_cloud/point_cloud_builder.h"
#include <draco/core/vector_d.h>
#include "draco/compression/decode.h"
#include "draco/compression/expert_encode.h"
#include "draco/compression/encode.h"
#include "draco/attributes/attribute_quantization_transform.h"

namespace pdal
{
    typedef std::shared_ptr<std::ostream> FileStreamPtr;

class PDAL_DLL DracoWriter : public Writer/*, public Streamable*/
{
public:

    DracoWriter();
    ~DracoWriter();
    std::string getName() const;
private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table);
    // virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    // virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);

    bool flushCache(size_t size);
    int components(Dimension::IdList &list, Dimension::IdList typeVector);
    void addAttribute(draco::GeometryAttribute::Type t, int n);
    void addAttribute(draco::GeometryAttribute::Type t, Dimension::Id pt, int n);
    Dimension::IdList searchDims(draco::GeometryAttribute::Type t);

    std::map<draco::GeometryAttribute::Type, int> m_quantDefaults =
    {
        { draco::GeometryAttribute::POSITION,  11 },
        { draco::GeometryAttribute::NORMAL,     7 },
        { draco::GeometryAttribute::TEX_COORD, 10 },
        { draco::GeometryAttribute::COLOR,      8 },
        { draco::GeometryAttribute::GENERIC,    8 }
    };

    std::map<Dimension::Type, draco::DataType> m_typeMap =
    {
        { Dimension::Type::Double, draco::DataType::DT_FLOAT64 },
        { Dimension::Type::Float, draco::DataType::DT_FLOAT32 },
        { Dimension::Type::Signed8, draco::DataType::DT_INT8 },
        { Dimension::Type::Unsigned8, draco::DataType::DT_UINT8 },
        { Dimension::Type::Signed16, draco::DataType::DT_INT16 },
        { Dimension::Type::Unsigned16, draco::DataType::DT_UINT16 },
        { Dimension::Type::Signed32, draco::DataType::DT_INT32 },
        { Dimension::Type::Unsigned32, draco::DataType::DT_UINT32 },
        { Dimension::Type::Signed64, draco::DataType::DT_INT64 },
        { Dimension::Type::Unsigned64, draco::DataType::DT_UINT64 },
    };

    std::map<Dimension::Id, draco::GeometryAttribute::Type> m_dimMap =
    {
        { Dimension::Id::X,         draco::GeometryAttribute::POSITION },
        { Dimension::Id::Y,         draco::GeometryAttribute::POSITION },
        { Dimension::Id::Z,         draco::GeometryAttribute::POSITION },
        { Dimension::Id::NormalX,   draco::GeometryAttribute::NORMAL },
        { Dimension::Id::NormalY,   draco::GeometryAttribute::NORMAL },
        { Dimension::Id::NormalZ,   draco::GeometryAttribute::NORMAL },
        { Dimension::Id::Red,       draco::GeometryAttribute::COLOR },
        { Dimension::Id::Green,     draco::GeometryAttribute::COLOR },
        { Dimension::Id::Blue,      draco::GeometryAttribute::COLOR },
        { Dimension::Id::TextureU,  draco::GeometryAttribute::TEX_COORD },
        { Dimension::Id::TextureV,  draco::GeometryAttribute::TEX_COORD },
        { Dimension::Id::TextureW,  draco::GeometryAttribute::TEX_COORD }
    };

    struct Args;
    std::unique_ptr<DracoWriter::Args> m_args;

    std::string m_filename;
    FileStreamPtr m_stream;

    std::map<draco::GeometryAttribute::Type, int> m_dims;
    std::map<draco::GeometryAttribute::Type, int> m_attMap;
    Dimension::IdList m_genericDims;

    draco::PointCloudBuilder m_pc;
    int m_precision;

    DracoWriter(const DracoWriter&) = delete;
    DracoWriter& operator=(const DracoWriter&) = delete;
};

} // namespace pdal
