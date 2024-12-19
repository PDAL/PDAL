/****************************************************************************
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

#include <memory>
#include <vector>

#include "ArrowWriter.hpp"
#include "ArrowCommon.hpp"

#include <pdal/PointView.hpp>
#include <pdal/pdal_config.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <io/private/las/Header.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <random>

namespace pdal
{

static PluginInfo const s_info
{
    "writers.arrow",
    "Arrow Writer",
    "http://pdal.io/stages/writers.arrow.html"
};

CREATE_SHARED_STAGE(ArrowWriter, s_info)

class BaseDimHandler
{
public:
    virtual ~BaseDimHandler()
    {}

    virtual Utils::StatusWithReason operator()(const PointRef& point) = 0;
    Utils::StatusWithReason finish(std::shared_ptr<arrow::Array>& array)
    {
        arrow::Status status = builder().Finish(&array);
        if (!status.ok())
            return { -1, status.message() };
        return true;
    }

private:
    virtual arrow::ArrayBuilder& builder() = 0;
};

template<typename DT, typename BT>  // Dimension type, Builder type
class DimHandler : public BaseDimHandler
{
public:
    DimHandler(arrow::MemoryPool *pool, Dimension::Id id) : m_builder(pool), m_id(id)
    {}

    Utils::StatusWithReason operator()(const PointRef& point) override
    {
        arrow::Status status = m_builder.Append(point.getFieldAs<DT>(m_id));
        if (!status.ok())
            return { -1, status.message() };
        return true;
    }

private:
    arrow::ArrayBuilder& builder() override
    { return m_builder; }

private:
    BT m_builder;
    Dimension::Id m_id;
};

class XYZHandler : public BaseDimHandler
{
public:
    XYZHandler(arrow::MemoryPool *pool) :
        m_doubleBuilder(std::make_shared<arrow::DoubleBuilder>(pool)),
        m_builder(pool, m_doubleBuilder, 3)
    {
    }

    Utils::StatusWithReason operator()(const PointRef& point) override
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);
        double z = point.getFieldAs<double>(Dimension::Id::Z);

        arrow::Status status = m_builder.Append() &
            m_doubleBuilder->Append(x) &
            m_doubleBuilder->Append(y) &
            m_doubleBuilder->Append(z);
        if (!status.ok())
            return { -1, status.message() };
        return true;
    }

private:
    arrow::ArrayBuilder& builder() override
    { return m_builder; }

private:
    std::shared_ptr<arrow::DoubleBuilder> m_doubleBuilder;
    arrow::FixedSizeListBuilder m_builder;
};

class WkbHandler : public BaseDimHandler
{
public:
    WkbHandler(arrow::MemoryPool *pool) : m_builder(pool)
    {}

    // Write XYZ as little-endian encoded well-known binary.
    Utils::StatusWithReason operator()(const PointRef& point) override
    {
        auto tole = [](double d)
        {
            uint64_t *u = reinterpret_cast<uint64_t *>(&d);
            *u = htole64(*u);
            d = *(reinterpret_cast<double *>(u));
            return d;
        };

        double x = tole(point.getFieldAs<double>(Dimension::Id::X));
        double y = tole(point.getFieldAs<double>(Dimension::Id::Y));
        double z = tole(point.getFieldAs<double>(Dimension::Id::Z));

        // The first five bytes in the buffer is the magic code for a
        // little-endian encoded XYZ point.
        // Finding this in any document these days is nigh impossible. See the
        // GDAL source code. :(
        static uint8_t buf[5 + 3 * sizeof(double)] { 0x01, 0x80, 0x00, 0x00, 0x01 };
        static uint8_t * const xpos = buf + 5;
        static uint8_t * const ypos = xpos + sizeof(x);
        static uint8_t * const zpos = ypos + sizeof(y);

        memcpy(xpos, &x, sizeof(x));
        memcpy(ypos, &y, sizeof(y));
        memcpy(zpos, &z, sizeof(z));

        arrow::Status status = m_builder.Append(buf, sizeof(buf));
        if (!status.ok())
            return { -1, status.message() };
        return true;
    }

    arrow::ArrayBuilder& builder() override
    { return m_builder; }

private:
    arrow::BinaryBuilder m_builder;
};


std::string ArrowWriter::getName() const { return s_info.name; }


ArrowWriter::ArrowWriter() :
    m_formatType(arrowsupport::Unknown),
    m_pool(arrow::default_memory_pool()),
    m_batchIndex(0),
    m_pointTablePtr(nullptr)
{
}

ArrowWriter::~ArrowWriter()
{}

void ArrowWriter::initialize()
{
    std::string ext = Utils::tolower(FileUtils::extension(filename()));
    if (Utils::iequals("feather", m_formatString) || ext == ".feather")
        m_formatType = arrowsupport::Feather;
    if (Utils::iequals("parquet", m_formatString) || ext == ".parquet")
        m_formatType = arrowsupport::Parquet;

    if (m_formatType == arrowsupport::Unknown)
    {
        std::stringstream msg;
        msg << "Unknown format '" << m_formatString <<
               "' provided. Unable to write array";
        throwError(msg.str());
    }

    auto result = arrow::io::FileOutputStream::Open(filename(), /*append=*/false);
    if (result.ok())
        m_file = result.ValueOrDie();
    else
    {
        std::stringstream msg;
        msg << "Unable to open '" << filename() << "' for arrow output with error " <<
            result.status().ToString();
        throwError(msg.str());
    }

    m_ogrPoint.reset(new pdal::Geometry(0.0, 0.0, 0.0, getSpatialReference()));
}


bool ArrowWriter::processOne(PointRef& point)
{
    for (auto& handler : m_dimHandlers)
        (*handler)(point);

    if (++m_batchIndex == (point_count_t)m_batchSize)
    {
        flushBatch();
        m_batchIndex = 0;
    }
    return true;
}

NL::json getPROJJSON(const pdal::SpatialReference& ref)
{

    NL::json projjson;
    NL::json column;
    try
    {
        projjson  = NL::json::parse(ref.getPROJJSON());
    } catch (NL::json::parse_error& e)
    {
        column["error"] = e.what();
    }
    column["crs"] = projjson;
    column["edges"] = ref.isGeographic() ? "spherical" : "planar";
    return column;
}


void ArrowWriter::addArgs(ProgramArgs& args)
{
    args.add("format", "Output format ('feather','parquet','geoparquet')", m_formatString,
        "feather");
    args.add("geoarrow_dimension_name", "Dimension name for GeoArrow xyz struct",
        m_geoArrowDimensionName, "xyz");
    args.add("batch_size", "Arrow batch size", m_batchSize, 65536 * 4);
    args.add("write_pipeline_metadata", "Write PDAL metadata to schema",
        m_writePipelineMetadata, true);
    args.add("geoparquet_version", "GeoParquet version string", m_geoParquetVersion, "1.0.0");
}

void ArrowWriter::prepared(PointTableRef table)
{
    using namespace Dimension;

    // Eliminate worries about copying arrow classes.
    m_dimHandlers.reserve(table.layout()->dims().size());

    //Always do XYZ.
    m_dimHandlers.push_back(std::make_unique<XYZHandler>(m_pool));
    if (m_formatType == arrowsupport::Parquet)
        m_dimHandlers.push_back(std::make_unique<WkbHandler>(m_pool));
    for (Id id : table.layout()->dims())
    {
        // Aready taken care of.
        if (id == Id::X || id == Id::Y || id == Id::Z)
            continue;
        switch (table.layout()->dimType(id))
        {
        case Type::Unsigned8:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<uint8_t, arrow::UInt8Builder>>(m_pool, id));
            break;
        case Type::Unsigned16:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<uint16_t, arrow::UInt16Builder>>(m_pool, id));
            break;
        case Type::Unsigned32:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<uint32_t, arrow::UInt32Builder>>(m_pool, id));
            break;
        case Type::Unsigned64:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<uint64_t, arrow::UInt64Builder>>(m_pool, id));
            break;
        case Type::Signed8:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<int8_t, arrow::Int8Builder>>(m_pool, id));
            break;
        case Type::Signed16:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<int16_t, arrow::Int16Builder>>(m_pool, id));
            break;
        case Type::Signed32:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<int32_t, arrow::Int32Builder>>(m_pool, id));
            break;
        case Type::Signed64:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<int64_t, arrow::Int64Builder>>(m_pool, id));
            break;
        case Type::Float:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<float, arrow::FloatBuilder>>(m_pool, id));
            break;
        case Type::Double:
            m_dimHandlers.push_back(
                std::make_unique<DimHandler<double, arrow::DoubleBuilder>>(m_pool, id));
            break;
        default:
            throwError("Invalid type found for dimension '" + table.layout()->dimName(id) +
                "'.");
        }
    }
}

void ArrowWriter::ready(PointTableRef table)
{
    m_pointTablePtr = static_cast<PointTable*>(&table);

    std::vector<std::shared_ptr<arrow::Field>> fields;

    bool bAddedStruct(false);
    const auto& layout = table.layout();
    auto dims = layout->dims();
    for (auto& id : dims)
    {
        if ((id == pdal::Dimension::Id::X ||
             id == pdal::Dimension::Id::Y ||
             id == pdal::Dimension::Id::Z ||
             id == m_geoArrowDimId ))
        {
            // Use the struct field instead of adding XYZ dimensions
            // to the output
            if (bAddedStruct)
                continue ; // only add once

            auto dimensionField = arrow::field("dimension", arrow::float64(), false);
            std::shared_ptr<arrow::DataType> dt = arrow::fixed_size_list(dimensionField, 3);
            auto field = arrow::field(m_geoArrowDimensionName, dt, false);

            auto kvMetadata = field->metadata()
                               ? field->metadata()->Copy()
                               : std::make_shared<arrow::KeyValueMetadata>();
            kvMetadata->Append("ARROW:extension:name", "geoarrow.point");
            kvMetadata->Append("ARROW:extension:metadata", getPROJJSON(table.spatialReference()).dump(-1));

            NL::json dimDetail;
            dimDetail["name"] = layout->dimName(id);
            dimDetail["description"] = pdal::Dimension::description(id);
            dimDetail["interpretation"] = pdal::Dimension::interpretationName(layout->dimType(id));
            dimDetail["size"] = layout->dimSize(id);

            if (m_writePipelineMetadata)
            {
                std::stringstream m;
                Utils::toJSON(table.metadata(), m);
                kvMetadata->Append("PDAL:pipeline:metadata", m.str());
            }

            kvMetadata->Append("PDAL:dimension:metadata", dimDetail.dump(-1));

            field = field->WithMetadata(kvMetadata);
            fields.push_back(field);

            m_dimensionOutputNames.push_back(m_geoArrowDimensionName);

            bAddedStruct = true;
            log()->get(LogLevel::Info) << "Adding GeoArrow point struct" << std::endl;
        }
        else
        {
            auto dimType = layout->dimType(id);
            std::string name = layout->dimName(id);
            std::shared_ptr<arrow::Field> field = toArrowType(name, dimType);
            auto kvMetadata = field->metadata()
                                ? field->metadata()->Copy()
                                : std::make_shared<arrow::KeyValueMetadata>();
            if (Utils::iequals("wkb", name))
            {
                kvMetadata->Append("ARROW:extension:name", "geoarrow.wkb");
            }

            NL::json dimDetail;
            dimDetail["name"] = layout->dimName(id);
            dimDetail["description"] = pdal::Dimension::description(id);
            dimDetail["interpretation"] = pdal::Dimension::interpretationName(layout->dimType(id));
            dimDetail["size"] = layout->dimSize(id);
            kvMetadata->Append("PDAL:dimension:metadata", dimDetail.dump(-1));

            field = field->WithMetadata(kvMetadata);
            fields.push_back(field);
            m_dimensionOutputNames.push_back(name);
            log()->get(LogLevel::Info) << "Adding dimension '" << name << "'" << std::endl;
        }

    }

    m_schema.reset(new arrow::Schema(fields));

    if ((int)m_dimensionOutputNames.size() != m_schema->num_fields())
        throwError("Arrow schema size does not match PDAL schema size!");

    m_table = arrow::Table::Make(m_schema, m_arrays);

    if (m_formatType == arrowsupport::Parquet)
    {
        setupParquet(m_arrays, table);
    }

    if (m_formatType == arrowsupport::Feather)
    {
        setupFeather(m_arrays, table);
    }
}

void ArrowWriter::write(const PointViewPtr view)
{
    for (PointRef point : *view)
        processOne(point);
}

void ArrowWriter::gatherParquetGeoMetadata(std::shared_ptr<arrow::KeyValueMetadata>& input, const SpatialReference& ref)
{

    NL::json metadata;
    NL::json version;
    arrow::BuildInfo info = arrow::GetBuildInfo();
    version["arrow"] = info.version_string;
    version["pdal"] = pdal::Config::fullVersionString();

    NL::json column;
    column["encoding"] = "WKB";
    column["geometry_types"] = std::vector<std::string> {"Point"};

    column.update(getPROJJSON(ref.empty() ? SpatialReference("EPSG:4326") : ref));

    NL::json wkb;
    wkb["wkb"] = column;

    NL::json geo;
    geo["version"] = m_geoParquetVersion; // GeoParquet version
    geo["primary_column"] = "wkb";
    geo["columns"] = wkb;

    input->Append("geo", geo.dump(-1));

}


void ArrowWriter::setupParquet(std::vector<std::shared_ptr<arrow::Array>> const& arrays,
                               PointTableRef table)
{

    parquet::WriterProperties::Builder m_oWriterPropertiesBuilder{};

    m_oWriterPropertiesBuilder.max_row_group_length(m_batchSize);
    m_oWriterPropertiesBuilder.created_by("pdal "+pdal::Config::fullVersionString());
    m_oWriterPropertiesBuilder.version(parquet::ParquetVersion::PARQUET_2_6);
    m_oWriterPropertiesBuilder.data_page_version(parquet::ParquetDataPageVersion::V2);
    m_oWriterPropertiesBuilder.compression(parquet::Compression::SNAPPY);

    std::shared_ptr<parquet::ArrowWriterProperties> arrowWriterProperties =
        parquet::ArrowWriterProperties::Builder().store_schema()->build();

    std::shared_ptr<parquet::SchemaDescriptor> parquet_schema;
    auto result = parquet::arrow::ToParquetSchema( m_schema.get(),
                                                   *m_oWriterPropertiesBuilder.build(),
                                                   *arrowWriterProperties,
                                                   &parquet_schema);
    if (!result.ok())
    {
        std::stringstream msg;
        msg << "Unable to convert ToParquetSchema with error '" << result.ToString() << "'";
        throwError(msg.str());
    }




    auto schema_node = std::static_pointer_cast<parquet::schema::GroupNode>(
        parquet_schema->schema_root());

    m_poKeyValueMetadata = m_schema->metadata()
                         ? m_schema->metadata()->Copy()
                         : std::make_shared<arrow::KeyValueMetadata>();

    auto status =
        ::arrow::ipc::SerializeSchema(*m_schema, m_pool);
    if (status.ok())
    {
        // The serialized schema is not UTF-8, which is required for
        // Thrift
        const std::string schema_as_string = (*status)->ToString();
        const std::string schema_base64 =
            ::arrow::util::base64_encode(schema_as_string);
        static const std::string kArrowSchemaKey = "ARROW:schema";
        const_cast<arrow::KeyValueMetadata *>(
            m_poKeyValueMetadata.get())
            ->Append(kArrowSchemaKey, schema_base64);
    }

    gatherParquetGeoMetadata(m_poKeyValueMetadata, table.spatialReference());
    m_schema = m_schema->WithMetadata(m_poKeyValueMetadata);
    m_poKeyValueMetadata = m_schema->metadata()->Copy();

    log()->get(LogLevel::Info) << m_poKeyValueMetadata->ToString() << std::endl;

    auto base_writer = parquet::ParquetFileWriter::Open(
                             m_file, std::move(schema_node),
                             m_oWriterPropertiesBuilder.build(), m_poKeyValueMetadata);
    if (!result.ok())
    {
        std::stringstream msg;
        msg << "Unable to convert open ParquetFileWriter with error '"
            << result.ToString() << "'";
        throwError(msg.str());
    }

    auto schema_ptr = std::make_shared<::arrow::Schema>(*m_schema);

    result = parquet::arrow::FileWriter::Make(m_pool,
                                              std::move(base_writer),
                                              m_schema,
                                              arrowWriterProperties,
                                              &m_parquetFileWriter);
    if (!result.ok())
    {
        std::stringstream msg;
        msg << "Unable to make parquet::arrow::FileWriter "
            << result.ToString();
        throwError(msg.str());
    }
}


void ArrowWriter::setupFeather(std::vector<std::shared_ptr<arrow::Array>> const& arrays,
                               PointTableRef table)
{
    arrow::ipc::IpcWriteOptions writeOptions;
    auto result = arrow::ipc::MakeFileWriter(m_file.get(),
                                               m_schema,
                                               writeOptions,
                                               m_poKeyValueMetadata);
    if (result.ok())
        m_arrowFileWriter = result.ValueOrDie();
    else
    {
        std::stringstream msg;
        msg << "Unable to open '" << filename() << "' for arrow output with error " <<
            result.status().ToString();
        throwError(msg.str());
    }
}


void ArrowWriter::flushBatch()
{
    // Wipe off our arrays we're making a new batch
    std::vector<std::shared_ptr<arrow::Array>> arrays;

    for (auto& handler : m_dimHandlers)
    {
        std::shared_ptr<arrow::Array> array;
        auto status = handler->finish(array);
        if (!status)
            throwError(status.what());
        arrays.push_back(array);
    }

    if (m_formatType == arrowsupport::Parquet)
    {
        auto result = m_parquetFileWriter->NewRowGroup(m_batchSize);
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to make NewRowGroup" << result.ToString();
            throwError(msg.str());
        }

        for(auto& array: arrays)
        {
            result = m_parquetFileWriter->WriteColumnChunk(*array);
            if (!result.ok())
            {
                std::stringstream msg;
                msg << "Unable to make WriteColumnChunk" << result.ToString();
                throwError(msg.str());
            }
        }

    }
    else  // Feather
    {
        std::shared_ptr<arrow::RecordBatch> batch =
            arrow::RecordBatch::Make(m_schema, m_batchIndex, arrays);

        auto result = m_arrowFileWriter->WriteRecordBatch(*batch);
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to write arrow batch" << result.ToString();
            throwError(msg.str());
        }
    }
}


void ArrowWriter::done(PointTableRef table)
{
    // flush our final batch
    flushBatch();

    if (m_formatType == arrowsupport::Feather)
    {
        auto result = m_arrowFileWriter->Close();
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to open to close file writer for file '" << filename() <<
                "' for with error " << result.ToString();
            throwError(msg.str());
        }
        result = m_file->Close();
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to open to write feather table for file '" << filename() <<
                "' for with error " << result.ToString();
            throwError(msg.str());
        }
    }

    if (m_formatType == arrowsupport::Parquet)
    {
        auto result = m_parquetFileWriter->Close();
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to close FileWriter" << result.ToString();
            throwError(msg.str());
        }

        result = m_file->Close();
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to close File" << result.ToString();
            throwError(msg.str());
        }

    }

    log()->get(LogLevel::Debug) << "total memory allocated "
                                << m_pool->bytes_allocated() << std::endl;

}

} // namespaces
