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

#include <pdal/Geometry.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>

#include <nlohmann/json.hpp>
#include <ogr_geometry.h>

#include <arrow/record_batch.h>
#include <arrow/io/api.h>
#include <arrow/ipc/api.h>

namespace pdal
{

using namespace arrowsupport;

static PluginInfo const s_info
{
    "readers.arrow",
    "Arrow Reader",
    "http://pdal.org/stages/readers.arrow.html"
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
    , m_formatType(arrowsupport::Unknown)
    , m_pool(arrow::default_memory_pool())
    , m_batchCount(0)
    , m_currentBatchIndex(0)
    , m_currentBatchPointIndex(0)
    , m_readMetadata(false)

{}



void ArrowReader::addArgs(ProgramArgs& args)
{
    args.add("metadata", "", m_readMetadata, false);
    args.add("geoarrow_dimension_name", "", m_geoArrowDimName, "xyz");
    args.add("format", "", m_formatTypeString, "");
}


void ArrowReader::loadArrowGeoMetadata(const std::shared_ptr<const arrow::KeyValueMetadata> &kv_metadata)
{
    if (!kv_metadata)
        return;
    auto geo = kv_metadata->Get("ARROW:extension:metadata");
    NL::json metadata;

    if (geo.ok())
    {
        // load up the JSON and set our stuff
        try
        {
            metadata = NL::json::parse(*geo);
            } catch (NL::json::parse_error& e)
            {
                log()->get(LogLevel::Warning) << "unable to parse GeoArrow metadata with error '"
                                              << e.what() << "'" << std::endl;
                return;
            }


            // std::string dimType = metadata["ARROW:extension:name"];
            // if (!Utils::iequals(dimType, "geoarrow.point"))
            // {
            //     std::stringstream oss;
            //     oss << "GeoArrow metadata does not contain 'geoarrow.point' data. It contains '"
            //                                   << dimType << "' data." << std::endl;
            //     throwError(oss.str());
            // }

            if (!metadata.contains("crs"))
            {
                log()->get(LogLevel::Warning) << "GeoArrow metadata does not contain 'crs' entry"
                                              << std::endl;
                return;
            }
            NL::json crs = metadata["crs"];

            SpatialReference ref;
            if (crs.is_object())
            {
                ref.set(crs.dump());
            }
            else if (crs.is_string())
            {

                ref.set(crs.get<std::string>());
            }


            setSpatialReference(ref);
    } else
    {
        log()->get(LogLevel::Warning) << "No GeoArrow metadata available for this column" <<std::endl;
    }

}

void ArrowReader::loadParquetGeoMetadata(const std::shared_ptr<const arrow::KeyValueMetadata> &kv_metadata)
{
    if (!kv_metadata)
        return;
    auto geo = kv_metadata->Get("geo");
    NL::json metadata;

    if (geo.ok())
    {
        // load up the JSON and set our stuff
        try
        {
            metadata = NL::json::parse(*geo);
            } catch (NL::json::parse_error& e)
            {
                log()->get(LogLevel::Warning) << "unable to parse GeoParquet 'geo' metadata with error '"
                                              << e.what() << "'" << std::endl;
                return;
            }

            if (!metadata.contains("primary_column"))
            {
                log()->get(LogLevel::Warning) << "GeoParquet metadata does not contain 'primary_column' entry"
                                              << std::endl;
                return;

            }

            if (!metadata.contains("columns"))
            {
                log()->get(LogLevel::Warning) << "GeoParquet metadata does not contain 'columns' entry"
                                              << std::endl;
                return;
            }
            NL::json columns = metadata["columns"];

            std::string primary_column = metadata["primary_column"];
            NL::json column = metadata["columns"][primary_column];

            log()->get(LogLevel::Info) << "primary column is " << primary_column << std::endl;

            if (!column.contains("crs"))
            {
                log()->get(LogLevel::Warning) << "no 'crs' key available to fetch spatial reference information, setting to 4326" << std::endl;
                setSpatialReference("EPSG:4326");
                return;
            }

            SpatialReference ref;
            NL::json crs = column["crs"];
            if (crs.is_object())
            {
                ref.set(crs.dump());
            }
            else if (crs.is_string())
            {

                ref.set(column["crs"].get<std::string>());
            }

            if (column.contains("epoch"))
            {

                ref.setEpoch(column["epoch"].get<double>());
            }

            setSpatialReference(ref);
    } else
    {
        log()->get(LogLevel::Warning) << "unable to fetch GeoArrow metadata with error '"
                                      << geo.status().ToString() << "'" << std::endl;
    }

}

void ArrowReader::initialize()
{
    if (Utils::iequals(FileUtils::extension(m_filename), ".feather"))
    {
        m_formatType = arrowsupport::Feather;
    }
    else if (Utils::iequals(FileUtils::extension(m_filename), ".parquet"))
    {
        m_formatType = arrowsupport::Parquet;
    }
    if (m_formatTypeString.size())
    {

        if (Utils::iequals(m_formatTypeString, "geoarrow"))
            m_formatType = arrowsupport::Feather;
        else if (Utils::iequals(m_formatTypeString, "geoparquet"))
            m_formatType = arrowsupport::Parquet;
        else
            throwError("Unknown format type " + m_formatTypeString);

    }

    auto result = arrow::io::ReadableFile::Open(m_filename);
    if (result.ok())
        m_file = result.ValueOrDie();
    else
    {
        std::stringstream msg;
        msg << "Unable to open '" << m_filename << "' for to read data with message '"
            << result.status().ToString() <<"'";
        throwError(msg.str());
    }

    if (m_formatType == arrowsupport::Feather)
    {
        auto status = arrow::ipc::RecordBatchFileReader::Open(m_file);
        if (!status.ok())
        {
            std::stringstream msg;
            msg << "Unable to create RecordBatchFileReader for file  '" << m_filename << "' with message '"
                << result.status().ToString() <<"'";
            throwError(msg.str());
        }

        m_ipcReader = status.ValueOrDie();
        m_batchCount = m_ipcReader->num_record_batches();

        const auto fields = m_ipcReader->schema()->fields();

        for (const auto& field: fields)
        {
            auto metadata = field->metadata();
            if (metadata)
                if (metadata->Contains("ARROW:extension:metadata"))
                    loadArrowGeoMetadata(metadata);
        }

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
    if (m_formatType == arrowsupport::Parquet)
    {
        auto arrow_reader_props = parquet::ArrowReaderProperties();
        arrow_reader_props.set_batch_size(128 * 1024);  // default 64 * 1024
        auto reader_properties = parquet::ReaderProperties(m_pool);
        parquet::arrow::FileReaderBuilder reader_builder;
        reader_builder.memory_pool(m_pool);
        reader_builder.properties(arrow_reader_props);



#if ARROW_VERSION_MAJOR >= 21
        auto reader_result = parquet::arrow::OpenFile(m_file, m_pool);
        if (!reader_result.ok())
        {
            std::stringstream msg;
            msg << "Unable to open file '" << m_filename << "' with message '"
                << reader_result.status().ToString() << "'";
            throwError(msg.str());
        }
        m_arrow_reader = std::move(reader_result).ValueOrDie();
#else
        auto pOpenStatus = parquet::arrow::OpenFile(m_file, m_pool, &m_arrow_reader);
        if (!pOpenStatus.ok())
        {
            std::stringstream msg;
            msg << "Unable to open file '" << m_filename << "' with message '"
                << pOpenStatus.ToString() <<"'";
            throwError(msg.str());
        }
#endif


        const auto metadata = m_arrow_reader->parquet_reader()->metadata();
        loadParquetGeoMetadata(metadata->key_value_metadata());

        auto batchOpenStatus = m_arrow_reader->GetRecordBatchReader({0},&m_parquetReader);
        if (!batchOpenStatus.ok())
        {
            std::stringstream msg;
            msg << "Unable to create parquet RecordBatchFileReader for file '" << m_filename << "' with message '"
                << result.status().ToString() <<"'";
            throwError(msg.str());
        }

        for (arrow::Result<std::shared_ptr<arrow::RecordBatch>> maybe_batch : *m_parquetReader) {

            m_batchCount++;
        }
        auto closeStatus = m_parquetReader->Close();

        batchOpenStatus = m_arrow_reader->GetRecordBatchReader({0}, &m_parquetReader);
        if (!batchOpenStatus.ok())
        {
            std::stringstream msg;
            msg << "Unable to create parquet RecordBatchFileReader for file '" << m_filename << "' with message '"
                << result.status().ToString() <<"'";
            throwError(msg.str());
        }


        auto batchIterator = m_parquetReader->begin();
        auto result = *batchIterator;
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to read first batch for file '" << m_filename << "' with message '"
                << result.status().ToString() <<"'";
            throwError(msg.str());
        }
        m_currentBatch = result.ValueOrDie();
        if (!m_currentBatch)
        {
            std::stringstream msg;
            msg << "Batch was null for file '" << m_filename << "' with message '"
                << result.status().ToString() <<"'";
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

        if (t == arrow::Type::FIXED_SIZE_LIST)
        {
            if (Utils::iequals(name, m_geoArrowDimName))
            {
                layout->registerDim(pdal::Dimension::Id::X);
                layout->registerDim(pdal::Dimension::Id::Y);
                layout->registerDim(pdal::Dimension::Id::Z);
            }

        }

        pdal::Dimension::Type pt = pdalType(t);

        // If we're not a known Type, we're not adding the dimension to the
        // layout
        if (pt != pdal::Dimension::Type::None)
        {
            pdal::Dimension::Id id = layout->registerOrAssignDim(name, pt);
            m_arrayIds.insert({fieldPosition, id});
        }
        fieldPosition++;
    }
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

    if (m_formatType == arrowsupport::Feather){

        auto readResult = m_ipcReader->ReadRecordBatch(m_currentBatchIndex);
        if (!readResult.ok())
        {
            std::stringstream msg;
            msg << "Unable to read RecordBatch " << m_currentBatchIndex << " for file '" << m_filename << "' with message '"
                    << readResult.status().ToString() <<"'";
            throwError(msg.str());
        }
        m_currentBatch = readResult.ValueOrDie();

    } else if (m_formatType == arrowsupport::Parquet)
    {
        if (!(m_parquetReader.get()))
        {
            std::stringstream msg;
            msg << "Reader is null!" << std::endl;
            throwError(msg.str());
        }

        auto result = m_parquetReader->Next();
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to read next batch for file '" << m_filename << "' with message '"
                << result.status().ToString() <<"'";
            throwError(msg.str());
        }
        m_currentBatch = result.ValueOrDie();

    }

    return true;
}


bool ArrowReader::readNextBatchData()
{

    m_arrays.clear();
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

        pdal::Dimension::Id pDimId = m_arrayIds[columnNum];
        switch (array->type_id())
        {
            case arrow::Type::DOUBLE:
            {
                const auto castArray = static_cast<const arrow::DoubleArray*>(array.get());
                point.setField<double>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::FLOAT:
            {
                const auto castArray = static_cast<const arrow::FloatArray*>(array.get());
                point.setField<float>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::INT8:
            {
                const auto castArray = static_cast<const arrow::Int8Array*>(array.get());
                point.setField<int8_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::UINT8:
            {
                const auto castArray = static_cast<const arrow::UInt8Array*>(array.get());
                point.setField<uint8_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::INT16:
            {
                const auto castArray = static_cast<const arrow::Int16Array*>(array.get());
                point.setField<int16_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::UINT16:
            {
                const auto castArray = static_cast<const arrow::UInt16Array*>(array.get());
                point.setField<uint16_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::INT32:
            {
                const auto castArray = static_cast<const arrow::Int32Array*>(array.get());
                point.setField<int32_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::UINT32:
            {
                const auto castArray = static_cast<const arrow::UInt32Array*>(array.get());
                point.setField<uint32_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::INT64:
            {
                const auto castArray = static_cast<const arrow::Int64Array*>(array.get());
                point.setField<int64_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::UINT64:
            {
                const auto castArray = static_cast<const arrow::UInt64Array*>(array.get());
                point.setField<uint64_t>(pDimId, castArray->Value(m_currentBatchPointIndex));
                break;
            }
            case arrow::Type::BINARY:
            {
                // We assume any binary arrays are WKB. If they aren't we are throwing
                // an error
                const auto castArray = static_cast<const arrow::BinaryArray*>(array.get());
                std::string_view wkb = castArray->Value(m_currentBatchPointIndex);
                pdal::Geometry pt = pdal::Geometry(std::string(wkb));
                OGRGeometry* g = (OGRGeometry*) pt.getOGRHandle();
                OGRPoint* p = dynamic_cast<OGRPoint*>(g->toPoint());
                if (p)
                {
                    point.setField<double>(Dimension::Id::X, p->getX());
                    point.setField<double>(Dimension::Id::Y, p->getY());
                    point.setField<double>(Dimension::Id::Z, p->getZ());
                } else
                {
                    throwError("BinaryArray field was not WKB of type point!");
                }
                break;
            }
            case arrow::Type::FIXED_SIZE_LIST:
            case arrow::Type::LIST:
            {
                const auto listArray = static_cast<const arrow::FixedSizeListArray*>(array.get());
                assert(listArray->values()->type_id() == arrow::Type::DOUBLE);
                const auto pointValues =
                    std::static_pointer_cast<arrow::DoubleArray>(listArray->values());

                int nDim(3); // only xyz for now

                point.setField<double>(Dimension::Id::X, pointValues->Value((nDim * m_currentBatchPointIndex)));
                point.setField<double>(Dimension::Id::Y, pointValues->Value((nDim * m_currentBatchPointIndex) + 1));
                point.setField<double>(Dimension::Id::Z, pointValues->Value((nDim * m_currentBatchPointIndex) + 2));
                break;
            }
            case arrow::Type::STRING:
            case arrow::Type::STRUCT:
            {
                // don't do anything for these
                continue;
            }
            default:
                throw pdal_error("Unrecognized PDAL dimension type for dimension");


        }

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

    bool retval = fillPoint(point);
    m_currentBatchPointIndex++;
    return retval;


}


void ArrowReader::done(PointTableRef table)
{
    if (m_formatType == arrowsupport::Feather)
    {

    }
    else if (m_formatType == arrowsupport::Parquet)
    {

        auto result = m_parquetReader->Close();
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to read next batch for file '" << m_filename << "' with message '"
                << result.ToString() <<"'";
            throwError(msg.str());
        }

    }

    auto result = m_file->Close();




}


} // namespace pdal
