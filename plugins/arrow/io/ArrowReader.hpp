/******************************************************************************
* Copyright (c) 2023, Howard Butler (howard@hobu.co)*
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
*
*
****************************************************************************/

#pragma once

#include <memory>

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <arrow/type_fwd.h>
#include <arrow/io/type_fwd.h>
#include <arrow/ipc/type_fwd.h>

namespace pdal
{


enum ArrowFormatType {
    Feather = 0,
    ORC,
    Parquet
};


class PDAL_DLL ArrowReader : public pdal::Reader, public pdal::Streamable
{
public:
    ArrowReader();
    ~ArrowReader() {};

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t num);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);

    bool readNextBatchHeaders();
    bool readNextBatchData();
    bool fillPoint(PointRef& point);

    std::shared_ptr<arrow::io::ReadableFile> m_file;
    std::shared_ptr<arrow::ipc::RecordBatchFileReader> m_ipcReader;
    std::shared_ptr<::arrow::RecordBatchReader> m_parquetReader;

    std::shared_ptr<::arrow::RecordBatchReader> rb_reader;
    std::shared_ptr<arrow::RecordBatch> m_currentBatch;

    ArrowFormatType m_formatType;

    std::map<int, pdal::Dimension::Id> m_arrayIds;
    std::map<pdal::Dimension::Id, std::shared_ptr<arrow::Array> > m_arrays;

    arrow::MemoryPool* m_pool;
    int m_batchCount;
    int m_currentBatchIndex;
    int64_t m_currentBatchPointIndex;
    bool m_readMetadata;

};


} // namespace pdal
