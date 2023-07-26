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
****************************************************************************/

#include "ArrowReader.hpp"
#include "ArrowCommon.hpp"

#include <memory>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/PDALUtils.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.arrow",
    "Arrow Reader",
    "http://pdal.io/stages/readers.arrow.html"
};

CREATE_SHARED_STAGE(ArrowReader, s_info)

std::string ArrowReader::getName() const { return s_info.name; }


ArrowReader::ArrowReader()
    : pdal::Reader()
    , pdal::Streamable()
    , m_file(nullptr)
    , m_ipcReader(nullptr)
    , m_parquetReader(nullptr)
    , m_currentBatch(nullptr)
    , m_formatType(Feather)
    , m_pool(arrow::default_memory_pool())
    , m_batchCount(0)
    , m_currentBatchIndex(0)
    , m_currentBatchPointIndex(0)
    , m_readMetadata(false)

{}



void ArrowReader::addArgs(ProgramArgs& args)
{
    args.add("metadata", "", m_readMetadata, false);
}

void ArrowReader::initialize()
{

    if (pdal::Utils::isRemote(m_filename))
        m_filename = pdal::Utils::fetchRemote(m_filename);

    if (Utils::iequals(FileUtils::extension(m_filename), ".feather"))
    {
        m_formatType = Feather;
    }
    else if (Utils::iequals(FileUtils::extension(m_filename), ".parquet"))
    {
        m_formatType = Parquet;
    }

    auto result = arrow::io::ReadableFile::Open(m_filename);
    if (result.ok())
        m_file = result.ValueOrDie();
    else
    {
        std::stringstream msg;
        msg << "Unable to open '" << m_filename << "' for to read data!";
        throwError(msg.str());
    }

    if (m_formatType == Feather)
    {
        auto status = arrow::ipc::RecordBatchFileReader::Open(m_file);
        if (!status.ok())
        {
            std::stringstream msg;
            msg << "Unable to create RecordBatchFileReader for file '" << m_filename << "'";
            throwError(msg.str());
        }

        m_ipcReader = status.ValueOrDie();
        m_batchCount = m_ipcReader->num_record_batches();

        m_currentBatchIndex = 0;


        // Gather up a point count
        while (readNextBatchHeaders())
        {
            m_count = m_count + m_currentBatch->num_rows();
            m_currentBatchIndex++;
        }

        // add 1 to count
        m_count++;

        m_currentBatchIndex = 0;

        // Read our first batch
        readNextBatchHeaders();

    }
    if (m_formatType == Parquet)
    {
        auto arrow_reader_props = parquet::ArrowReaderProperties();
        arrow_reader_props.set_batch_size(128 * 1024);  // default 64 * 1024
        auto reader_properties = parquet::ReaderProperties(m_pool);
        parquet::arrow::FileReaderBuilder reader_builder;
        reader_builder.memory_pool(m_pool);
        reader_builder.properties(arrow_reader_props);

        std::unique_ptr<parquet::arrow::FileReader> arrow_reader;

        auto pOpenStatus = parquet::arrow::OpenFile(m_file, m_pool, &arrow_reader);

        auto batchOpenStatus = arrow_reader->GetRecordBatchReader(&m_parquetReader);
        if (!batchOpenStatus.ok())
        {
            std::stringstream msg;
            msg << "Unable to create parquet RecordBatchFileReader for file '" << m_filename << "'";
            throwError(msg.str());
        }

        for (arrow::Result<std::shared_ptr<arrow::RecordBatch>> maybe_batch : *m_parquetReader) {

            m_batchCount++;
        }
        auto closeStatus = m_parquetReader->Close();

        batchOpenStatus = arrow_reader->GetRecordBatchReader(&m_parquetReader);
        if (!batchOpenStatus.ok())
        {
            std::stringstream msg;
            msg << "Unable to create parquet RecordBatchFileReader for file '" << m_filename << "'";
            throwError(msg.str());
        }


        auto batchIterator = m_parquetReader->begin();
        auto result = *batchIterator;
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to read first batch for file '" << m_filename << "'";
            throwError(msg.str());
        }
        m_currentBatch = result.ValueOrDie();
        if (!m_currentBatch)
        {
            std::stringstream msg;
            msg << "Batch was null for file '" << m_filename << "'";
            throwError(msg.str());
        }
    }



}


void ArrowReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;
    Dimension::IdList ids;

    // We take the schema of the first batch. If the rest of the
    // batches don't match the schema, we're f'd

    std::shared_ptr<arrow::Schema> schema = m_currentBatch->schema();
    int fieldPosition(0);
    for(auto& f: schema->fields())
    {
        std::string name = f->name();
        auto& dt = f->type();
        arrow::Type::type t = dt->id();

        pdal::Dimension::Id id = layout->registerOrAssignDim(name, computePDALTypeFromArrow(t));
        m_arrayIds.insert({fieldPosition, id});
        fieldPosition++;
    }
}


void ArrowReader::ready(PointTableRef table)
{
    // gather dimensions from file
}


point_count_t ArrowReader::read(PointViewPtr view, point_count_t num)
{
    point_count_t numRead = 0;
    PointRef point(view->point(0));
    bool didRead(true);
    while (numRead < num && didRead ) {
        point.setPointId(numRead);
        didRead = processOne(point);
        ++numRead;
    }
    return numRead;
}


bool ArrowReader::readNextBatchHeaders()
{
    if (m_currentBatchIndex == m_batchCount)
        return false;
    auto readResult = m_ipcReader->ReadRecordBatch(m_currentBatchIndex);
    if (!readResult.ok())
    {
        std::stringstream msg;
        msg << "Unable to read RecordBatch " << m_currentBatchIndex << " for file '" << m_filename << "'";
        throwError(msg.str());
    }
    m_currentBatch = readResult.ValueOrDie();
    return true;
}


bool ArrowReader::readNextBatchData()
{

    for(int columnNum = 0; columnNum < m_currentBatch->num_columns(); ++columnNum)
    {
        // https://arrow.apache.org/docs/cpp/api/array.html#_CPPv4N5arrow5ArrayE
        std::shared_ptr<arrow::Array> array = m_currentBatch->column(columnNum);
        m_arrays[m_arrayIds[columnNum]] = array;
    }
    return true;
}

bool ArrowReader::fillPoint(PointRef& point)
{

    for(int columnNum = 0; columnNum < m_currentBatch->num_columns(); ++columnNum)
    {
        // https://arrow.apache.org/docs/cpp/api/array.html#_CPPv4N5arrow5ArrayE
        std::shared_ptr<arrow::Array> array = m_currentBatch->column(columnNum);
        arrow::DoubleArray* dArray = dynamic_cast<arrow::DoubleArray*>(array.get());
        if (dArray)
        {
            point.setField<double>(m_arrayIds[columnNum], dArray->Value(point.pointId()));
            continue;
        }
        arrow::FloatArray* fArray = dynamic_cast<arrow::FloatArray*>(array.get());
        if (fArray)
        {
            point.setField<float>(m_arrayIds[columnNum], fArray->Value(point.pointId()));
            continue;
        }
        arrow::Int8Array* int8Array = dynamic_cast<arrow::Int8Array*>(array.get());
        if (int8Array)
        {
            point.setField<int8_t>(m_arrayIds[columnNum], int8Array->Value(point.pointId()));
            continue;
        }
        arrow::UInt8Array* uint8Array = dynamic_cast<arrow::UInt8Array*>(array.get());
        if (uint8Array)
        {
            point.setField<uint8_t>(m_arrayIds[columnNum], uint8Array->Value(point.pointId()));
            continue;
        }
        arrow::Int16Array* int16Array = dynamic_cast<arrow::Int16Array*>(array.get());
        if (int16Array)
        {
            point.setField<int16_t>(m_arrayIds[columnNum], int16Array->Value(point.pointId()));
            continue;
        }
        arrow::UInt16Array* uint16Array = dynamic_cast<arrow::UInt16Array*>(array.get());
        if (uint16Array)
        {
            point.setField<uint16_t>(m_arrayIds[columnNum], uint16Array->Value(point.pointId()));
            continue;
        }
        arrow::Int32Array* int32Array = dynamic_cast<arrow::Int32Array*>(array.get());
        if (int32Array)
        {
            point.setField<int32_t>(m_arrayIds[columnNum], int32Array->Value(point.pointId()));
            continue;
        }
        arrow::UInt32Array* uint32Array = dynamic_cast<arrow::UInt32Array*>(array.get());
        if (uint32Array)
        {
            point.setField<uint32_t>(m_arrayIds[columnNum], uint32Array->Value(point.pointId()));
            continue;
        }
        arrow::Int64Array* int64Array = dynamic_cast<arrow::Int64Array*>(array.get());
        if (int64Array)
        {
            point.setField<int64_t>(m_arrayIds[columnNum], int64Array->Value(point.pointId()));
            continue;
        }
        arrow::UInt64Array* uint64Array = dynamic_cast<arrow::UInt64Array*>(array.get());
        if (uint64Array)
        {
            point.setField<uint64_t>(m_arrayIds[columnNum], uint64Array->Value(point.pointId()));
            continue;
        }

        throwError("Unable to convert Arrow Datatype!");
    }
    return true;
}


bool ArrowReader::processOne(PointRef& point)
{

    if (m_currentBatchPointIndex == m_currentBatch->num_rows())
    {
        // go read a new batch
        m_currentBatchIndex++;

        bool nextBatch = readNextBatchHeaders();
        if (!nextBatch) return false; // we're done

        m_currentBatchPointIndex = 0;

        // go read data for next batch
        readNextBatchData();
    }

    m_currentBatchPointIndex++;
    return fillPoint(point);


}


void ArrowReader::done(PointTableRef table)
{
    auto result = m_file->Close();

}


} // namespace pdal
