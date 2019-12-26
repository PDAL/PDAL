/******************************************************************************
* Copyright (c) 2019 TileDB, Inc
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
#include <pdal/Streamable.hpp>
#include <tiledb/tiledb>

namespace pdal
{

class PDAL_DLL TileDBReader : public Reader, public Streamable
{
public:
    struct Buffer
    {
        size_t m_count;  // Number of instances of the type.
        std::vector<uint8_t> m_data;

        Buffer(tiledb_datatype_t type, size_t count) : m_count(count),
            m_data(count * tiledb_datatype_size(type))
        {}
        size_t count() const
        { return m_count; }

        template<typename T>
        T *get()
        { return reinterpret_cast<T*>(m_data.data()); }
    };

    enum class DimCategory
    {
        Dimension,
        Attribute
    };

    struct DimInfo
    {
        Buffer *m_buffer;
        DimCategory m_dimCategory;
        size_t m_span;
        size_t m_offset;
        tiledb_datatype_t m_tileType;
        Dimension::Type m_type;
        Dimension::Id m_id;
        std::string m_name;
    };

    TileDBReader() = default;
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
    void localReady();
    bool processPoint(PointRef& point);

    std::string m_cfgFileName;
    point_count_t m_chunkSize;
    point_count_t m_offset;
    point_count_t m_resultSize;
    bool m_complete;
    bool m_stats;
    BOX3D m_bbox;
    std::vector<std::unique_ptr<Buffer>> m_buffers;
    std::vector<DimInfo> m_dims;

    std::unique_ptr<tiledb::Context> m_ctx;
    std::unique_ptr<tiledb::Array> m_array;
    std::unique_ptr<tiledb::Query> m_query;

    TileDBReader(const TileDBReader&) = delete;
    TileDBReader& operator=(const TileDBReader&) = delete;

    template<typename T>
    void setQueryBuffer(const DimInfo& di);
    void setQueryBuffer(const DimInfo& di);
};

} // namespace pdal
