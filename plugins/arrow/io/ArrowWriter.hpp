/******************************************************************************
* Copyright (c) 2023, Howard Butler (howard@hobu.co)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <pdal/pdal_features.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Streamable.hpp>

#include "ArrowCommon.hpp"

#include <arrow/type_fwd.h>
#include <arrow/io/type_fwd.h>
#include <parquet/type_fwd.h>

namespace pdal
{

    typedef std::map<pdal::Dimension::Id, std::unique_ptr<arrow::ArrayBuilder> > DimBuilderMap;

class PDAL_DLL ArrowWriter  : public Writer, public Streamable
{
public:
    ArrowWriter();
    ArrowWriter& operator=(const ArrowWriter&) = delete;
    ArrowWriter(const ArrowWriter&) = delete;
    ~ArrowWriter();

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void addDimensions(PointLayoutPtr layout);

    void setupParquet(std::vector<std::shared_ptr<arrow::Array>> const& arrays,
        PointTableRef table);
    void setupFeather(std::vector<std::shared_ptr<arrow::Array>> const& arrays,
        PointTableRef table);
    void gatherParquetGeoMetadata(std::shared_ptr<arrow::KeyValueMetadata>& input,
        SpatialReference& ref);
    void createBuilders(PointTableRef table);
    void FlushBatch(PointTableRef table);

    std::string m_formatString;
    arrowsupport::ArrowFormatType m_formatType;

    std::shared_ptr<arrow::Table> m_table;
    std::shared_ptr<arrow::Schema> m_schema;
    std::vector<std::shared_ptr<arrow::Array>> m_arrays;

    std::map<pdal::Dimension::Id, std::unique_ptr<arrow::ArrayBuilder> > m_builders;
    std::vector<pdal::Dimension::Id> m_dimIds;
    arrow::MemoryPool* m_pool;
    int m_batchSize;
    std::string m_geoParquetVersion;

    std::shared_ptr<arrow::io::FileOutputStream> m_file;
    std::unique_ptr<parquet::arrow::FileWriter> m_parquetFileWriter;
    std::shared_ptr<arrow::ipc::RecordBatchWriter> m_arrowFileWriter;

    std::shared_ptr<arrow::KeyValueMetadata> m_poKeyValueMetadata;

    std::string m_geoArrowDimensionName;
    point_count_t m_batchIndex;
    bool m_writePipelineMetadata;
    pdal::Dimension::Id m_wkbDimId;
    pdal::Dimension::Id m_geoArrowDimId;

    PointTable* m_pointTablePtr;
};

} // namespace pdal
