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

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

#include <tiledb/tiledb>

namespace pdal
{

class PDAL_DLL TileDBReader : public Reader, public Streamable
{
public:
    class DimBuffer
    {
    public:
        virtual ~DimBuffer() = default;
        virtual void setQuery(tiledb::Query* query, size_t count) = 0;
        virtual void setFields(PointRef& point, size_t bufOffset) = 0;
    };

    TileDBReader();
    ~TileDBReader();

    std::string getName() const override;

private:
    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize() override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void prepared(PointTableRef) override;
    virtual void ready(PointTableRef) override;
    virtual bool processOne(PointRef& point) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual void done(PointTableRef table) override;

    void addDim(PointLayoutPtr layout, const std::string& name,
                tiledb_datatype_t typeTileDB, bool required);
    void localReady();
    bool processPoint(PointRef& point);

    struct Args;
    std::unique_ptr<TileDBReader::Args> m_args;

    point_count_t m_offset;
    point_count_t m_resultSize;
    bool m_complete;
    std::vector<std::unique_ptr<DimBuffer>> m_dims;

    std::unique_ptr<tiledb::Context> m_ctx;
    std::unique_ptr<tiledb::Array> m_array;
    std::unique_ptr<tiledb::Query> m_query;

    TileDBReader(const TileDBReader&) = delete;
    TileDBReader& operator=(const TileDBReader&) = delete;
};

} // namespace pdal
