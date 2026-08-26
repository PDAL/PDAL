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

#include <arrow/io/api.h>
#include <arrow/ipc/writer.h>
#include <arrow/util/base64.h>
#include <parquet/arrow/schema.h>
#include <parquet/arrow/writer.h>
#include <parquet/file_writer.h>

namespace pdal
{

using namespace arrowsupport;

static PluginInfo const s_info
{
    "writers.arrow",
    "Arrow Writer",
    "https://pdal.org/stages/writers.arrow.html"
};

CREATE_SHARED_STAGE(ArrowWriter, s_info)

class BaseDimHandler
{
public:
    virtual ~BaseDimHandler()
    {}

    // Get an arrow field approriate for handling a dimension.
    virtual std::shared_ptr<arrow::Field> field() = 0;
    // Append a the dimension data from a point to an array builder.
    virtual Utils::StatusWithReason append(const PointRef& point) = 0;
    // Finish building of point data for a builder. The builder is reused for the next
    // data block.
    virtual Utils::StatusWithReason finish(std::shared_ptr<arrow::Array>& array)
    {
        arrow::Status status = builder().Finish(&array);
        if (!status.ok())
            return { -1, status.message() };
        return true;
    }

private:
    // Get a builder appropriate for the data type.
    virtual arrow::ArrayBuilder& builder() = 0;
};

template<typename DT>  // DT - Dimension type.
class DimHandler : public BaseDimHandler
{
public:
    DimHandler(arrow::MemoryPool *pool, Dimension::Id id, const std::string& name) :
        m_builder(pool), m_id(id), m_name(name)
    {}

    FieldPtr field() override
    {
        auto kvMetadata = std::make_shared<arrow::KeyValueMetadata>();

        // Note that field-level metadata is not stored in Parquet.
        NL::json metadata {
                { "name", m_name },
                { "description", Dimension::description(m_id) },
                { "interpretation", Dimension::interpretationName(Dimension::type<DT>()) },
                { "size", sizeof(DT) }
        };
        kvMetadata->Append("PDAL:dimension:metadata", metadata.dump(-1));

        return arrow::field(m_name, TypeTraits<DT>::dataType(), kvMetadata);
    }

    Utils::StatusWithReason append(const PointRef& point) override
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
    arrow::NumericBuilder<typename TypeTraits<DT>::TypeClass> m_builder;
    Dimension::Id m_id;
    std::string m_name;
};


// Handler for WKB-encoded XYZ data per GeoParquet specification.
class WkbHandler : public BaseDimHandler
{
public:
    WkbHandler(arrow::MemoryPool *pool, const std::string& dimName, 
        const std::string& pipelineMetadata = std::string()) :
        m_pipelineMetadata(pipelineMetadata), m_dimName(dimName),
        m_builder(arrow::fixed_size_binary(29), pool)
    {}

    FieldPtr field() override
    {
        NL::json metadata {
            { "name", m_dimName },
            { "description", "WKB points" },
            { "interpretation", "binary" },
            { "size", 29 }
        };

        auto kvMetadata = std::make_shared<arrow::KeyValueMetadata>();
        kvMetadata->Append("ARROW:extension:name", "geoarrow.wkb");
        kvMetadata->Append("PDAL:dimension:metadata", metadata.dump(-1));

        // Must be binary instead of fixed-length binary to conform to GeoParquet.
        return arrow::field(m_dimName, arrow::binary(), kvMetadata);
    }

    // Write XYZ as little-endian encoded well-known binary.
    Utils::StatusWithReason append(const PointRef& point) override
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
        // little-endian encoded XYZ 2.5d point. The first byte is the little-endian
        // code (0x01). The remaining bytes specify the geometry type.
        // Finding this in any document these days is nigh impossible. See the
        // GDAL source code. :(
        static uint8_t buf[5 + 3 * sizeof(double)] { 0x01, 0x01, 0x00, 0x00, 0x80 };
        static uint8_t * const xpos = buf + 5;
        static uint8_t * const ypos = xpos + sizeof(x);
        static uint8_t * const zpos = ypos + sizeof(y);

        memcpy(xpos, &x, sizeof(x));
        memcpy(ypos, &y, sizeof(y));
        memcpy(zpos, &z, sizeof(z));

        arrow::Status status = m_builder.Append(buf, 29);
        if (!status.ok())
            return { -1, status.message() };
        return true;
    }

    arrow::ArrayBuilder& builder() override
    { return m_builder; }

private:
    std::string m_pipelineMetadata;
    std::string m_dimName;
    arrow::BinaryBuilder m_builder;
};


std::string ArrowWriter::getName() const { return s_info.name; }


ArrowWriter::ArrowWriter() :
    m_pool(arrow::default_memory_pool()),
    m_batchIndex(0)
{
}

ArrowWriter::~ArrowWriter()
{}

void ArrowWriter::initialize()
{
    using namespace arrowsupport;

    StringList versionNums = Utils::split(m_geoParquetVersionString, '.');
    int majorVersion = std::stoi(versionNums[0]);
    int minorVersion = std::stoi(versionNums[1]);

    if (majorVersion == 1 && minorVersion == 0)
        m_version = ParquetVersion::GeoParquet10;
    else if (majorVersion == 1 && minorVersion == 1)
        m_version = ParquetVersion::GeoParquet11;
    else if (majorVersion == 2)
        m_version = ParquetVersion::GeoParquet20;
    else
        throwError("Invalid GeoParquet version string: '" +
            m_geoParquetVersionString + "'. Expected 1.0.0, 1.1.0 or 2.0.0");

    auto result = arrow::io::FileOutputStream::Open(filename(), /*append=*/false);
    if (result.ok())
        m_file = result.ValueOrDie();
    else
        throwError("Unable to open '" + filename() + "' for arrow output with error " +
            result.status().ToString());
}


bool ArrowWriter::processOne(PointRef& point)
{
    for (auto& handler : m_dimHandlers)
    {
        auto ok = handler->append(point);
        if (!ok)
            throwError("Unable to append point data to arrow array: " + ok.what() + ".");
    }

    if (++m_batchIndex == (point_count_t)m_batchSize)
        flushBatch();
    return true;
}

NL::json getPROJJSON(const pdal::SpatialReference& ref)
{
    NL::json column;
    try
    {
        column["crs"] = NL::json::parse(ref.getPROJJSON());
    } catch (NL::json::parse_error& e)
    {
        column["error"] = e.what();
    }
    column["edges"] = ref.isGeographic() ? "spherical" : "planar";
    return column;
}


void ArrowWriter::addArgs(ProgramArgs& args)
{
    //args.add("format", "Output format ('feather','parquet','geoparquet')", m_formatString,
    //    "feather");
    args.add("geometry_name", "Dimension name for Parquet geometry column",
        m_geoDimensionName, "geometry");
    args.addSynonym("geometry_name", "geoarrow_dimension_name");
    args.add("batch_size", "Arrow batch size", m_batchSize, 65536 * 4);
    args.add("write_pipeline_metadata", "Write PDAL metadata to file metadata",
        m_writePipelineMetadata, true);
    args.add("geoparquet_version", "GeoParquet version string", m_geoParquetVersionString, "1.1.0");
}

void ArrowWriter::prepared(PointTableRef table)
{
    using namespace Dimension;

    // Eliminate worries about copying arrow classes.
    m_dimHandlers.reserve(table.layout()->dims().size());

    std::string pipelineMetadata;
    if (m_writePipelineMetadata)
        pipelineMetadata = Utils::toJSON(table.metadata());
    // For Parquet, the WKB column is the GeoParquet-spec geometry column. The 
    // packed GeoArrow xyz struct is no longer written
    m_dimHandlers.push_back(std::make_unique<WkbHandler>(m_pool, m_geoDimensionName,
        pipelineMetadata));
    for (Id id : table.layout()->dims())
    {
        // Aready taken care of.
        if (id == Id::X || id == Id::Y || id == Id::Z)
            continue;
        std::string name = table.layout()->dimName(id);
        switch (table.layout()->dimType(id))
        {
        case Type::Unsigned8:
            m_dimHandlers.push_back(std::make_unique<DimHandler<uint8_t>>(m_pool, id, name));
            break;
        case Type::Unsigned16:
            m_dimHandlers.push_back(std::make_unique<DimHandler<uint16_t>>(m_pool, id, name));
            break;
        case Type::Unsigned32:
            m_dimHandlers.push_back(std::make_unique<DimHandler<uint32_t>>(m_pool, id, name));
            break;
        case Type::Unsigned64:
            m_dimHandlers.push_back(std::make_unique<DimHandler<uint64_t>>(m_pool, id, name));
            break;
        case Type::Signed8:
            m_dimHandlers.push_back(std::make_unique<DimHandler<int8_t>>(m_pool, id, name));
            break;
        case Type::Signed16:
            m_dimHandlers.push_back(std::make_unique<DimHandler<int16_t>>(m_pool, id, name));
            break;
        case Type::Signed32:
            m_dimHandlers.push_back(std::make_unique<DimHandler<int32_t>>(m_pool, id, name));
            break;
        case Type::Signed64:
            m_dimHandlers.push_back(std::make_unique<DimHandler<int64_t>>(m_pool, id, name));
            break;
        case Type::Float:
            m_dimHandlers.push_back(std::make_unique<DimHandler<float>>(m_pool, id, name));
            break;
        case Type::Double:
            m_dimHandlers.push_back(std::make_unique<DimHandler<double>>(m_pool, id, name));
            break;
        default:
            throwError("Invalid type found for dimension '" + name + "'.");
        }
    }
}

void ArrowWriter::ready(PointTableRef table)
{
    std::vector<std::shared_ptr<arrow::Field>> fields;
    for (auto& h : m_dimHandlers)
        fields.push_back(h->field());

    m_schema.reset(new arrow::Schema(fields));

    setupParquet(table);
}

void ArrowWriter::write(const PointViewPtr view)
{
    for (PointRef point : *view)
        processOne(point);
}

void ArrowWriter::gatherParquetGeoMetadata(std::shared_ptr<arrow::KeyValueMetadata>& input,
    const SpatialReference& ref, const std::string& pipelineMetadata)
{
    NL::json column = {
        { "encoding", "WKB" },
        { "geometry_types", { "Point Z" } }
    };

    // null SRS should be null in metadata; srid:0 in schema
    NL::json crs;
    if (ref.empty())
        column.update({{ "crs", nullptr }});
    else
        column.update(getPROJJSON(ref));

    NL::json wkb;
    wkb[m_geoDimensionName] = column;

    NL::json geo {
        { "version", m_geoParquetVersionString },
        { "primary_column", m_geoDimensionName },
        { "columns", wkb }
    };

    input->Append("geo", geo.dump(-1));
    //!! add to geo or stay separate?
    if (pipelineMetadata.size())
        input->Append("PDAL:pipeline:metadata", pipelineMetadata);
}

// Rebuilds the parquet schema, assigning the geometry/geography parquet logical
// type to the geom field
arrowsupport::GroupNodePtr ArrowWriter::applyGeoType(
    const arrowsupport::GroupNodePtr& root, const SpatialReference& ref)
{
    parquet::schema::NodeVector fields;
    for (int i = 0; i < root->field_count(); ++i)
    {
        auto child = root->field(i);
        if (child->name() != m_geoDimensionName)
        {
            fields.push_back(child);
            continue;
        }

        // Parquet standard for unknown CRS. I don't think we have any reason to
        // write the CRS84 default (crs=null).
        std::string crs = ref.empty() ? "srid:0" : getPROJJSON(ref)["crs"].dump(-1);

        std::shared_ptr<const parquet::LogicalType> logicalType = ref.isGeographic()
            ? parquet::LogicalType::Geography(crs) : parquet::LogicalType::Geometry(crs);

        fields.push_back(parquet::schema::PrimitiveNode::Make(
            child->name(), child->repetition(), logicalType, parquet::Type::BYTE_ARRAY));
    }

    return std::static_pointer_cast<parquet::schema::GroupNode>(
        parquet::schema::GroupNode::Make(root->name(), root->repetition(), fields));
}

void ArrowWriter::setupParquet(PointTableRef table)
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
    auto result = parquet::arrow::ToParquetSchema(m_schema.get(),
                                                  *m_oWriterPropertiesBuilder.build(),
                                                  *arrowWriterProperties,
                                                  &parquet_schema);
    if (!result.ok())
        throwError("Unable to convert ToParquetSchema with error: "+ result.ToString());

    auto schema_node = std::static_pointer_cast<parquet::schema::GroupNode>(
        parquet_schema->schema_root());
    // If it's a geoparquet 2.0 file, we need to rebuild the schema node to mark
    // the output as geometry type
    if (m_version == arrowsupport::ParquetVersion::GeoParquet20)
        schema_node = applyGeoType(schema_node, table.spatialReference());

    m_poKeyValueMetadata = m_schema->metadata()
                         ? m_schema->metadata()->Copy()
                         : std::make_shared<arrow::KeyValueMetadata>();

    auto status = ::arrow::ipc::SerializeSchema(*m_schema, m_pool);
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

    std::string pipelineMetadata;
    if (m_writePipelineMetadata)
        pipelineMetadata = Utils::toJSON(table.metadata());

    gatherParquetGeoMetadata(m_poKeyValueMetadata, table.spatialReference(),
        pipelineMetadata);
    m_schema = m_schema->WithMetadata(m_poKeyValueMetadata);
    m_poKeyValueMetadata = m_schema->metadata()->Copy();

    log()->get(LogLevel::Info) << m_poKeyValueMetadata->ToString() << std::endl;

    auto base_writer = parquet::ParquetFileWriter::Open(
                             m_file, std::move(schema_node),
                             m_oWriterPropertiesBuilder.build(), m_poKeyValueMetadata);
    if (!result.ok())
        throwError("Unable to convert open ParquetFileWriter with error: " + result.ToString());

    result = parquet::arrow::FileWriter::Make(m_pool,
                                              std::move(base_writer),
                                              m_schema,
                                              arrowWriterProperties,
                                              &m_parquetFileWriter);
    if (!result.ok())
        throwError("Unable to make parquet::arrow::FileWriter: " + result.ToString());
}


void ArrowWriter::flushBatch()
{
    std::vector<std::shared_ptr<arrow::Array>> arrays;

    // Get the data from the builders into arrays.
    for (auto& handler : m_dimHandlers)
    {
        std::shared_ptr<arrow::Array> array;
        auto status = handler->finish(array);
        if (!status)
            throwError(status.what());
        arrays.push_back(std::move(array));
    }

    auto result = m_parquetFileWriter->NewRowGroup();

    if (!result.ok())
        throwError("Unable to make NewRowGroup: " + result.ToString());

    for (auto& array: arrays)
    {
        result = m_parquetFileWriter->WriteColumnChunk(*array);
        if (!result.ok())
            throwError("Unable to make WriteColumnChunk: " + result.ToString());
    }

    m_batchIndex = 0;
}


void ArrowWriter::done(PointTableRef table)
{
    // flush our final batch
    flushBatch();

    auto result = m_parquetFileWriter->Close();
    if (!result.ok())
        throwError("Unable to close FileWriter: " + result.ToString());

    result = m_file->Close();
    if (!result.ok())
        throwError("Unable to close file: " + result.ToString());

    log()->get(LogLevel::Debug) << "total memory allocated "
                                << m_pool->bytes_allocated() << std::endl;
}

} // namespaces
