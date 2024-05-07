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

void ArrowWriter::createBuilders(PointTableRef table)
{

    using namespace pdal;

    m_builders.clear();
    const auto& layout = table.layout();

    for (auto& field : m_schema->fields())
    {
        auto id = layout->findDim(field->name());
        std::string name = layout->dimName(id);

        std::unique_ptr<arrow::ArrayBuilder> builder;
        arrow::Status status = arrow::MakeBuilder(m_pool, field->type(), &builder);
        if (!status.ok())
        {
            std::stringstream msg;
            msg << "Unable to create builder for '" << name << "'";
            throwError(msg.str());
        }
        auto found = m_builders.find(id);
        if (found != m_builders.end())
            throwError("Map already contains id");
        m_builders.insert({id, std::move(builder)});
    }
}


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
    bool bAddedStruct(false);
    for (auto& id: m_dimIds)
    {
        arrow::ArrayBuilder* builder = m_builders[id].get();
        if (!builder)
        {
            throwError("unable to fetch builder for dimension!");
        }

        if ((id == pdal::Dimension::Id::X ||
             id == pdal::Dimension::Id::Y ||
             id == pdal::Dimension::Id::Z ||
             id == m_geoArrowDimId))
        {
            // Use the struct field instead
            if (bAddedStruct)
                continue ; // only add once
            arrow::ArrayBuilder* builder = m_builders[m_geoArrowDimId].get();
            writeGeoArrow(point, builder);
            bAddedStruct = true;
        }
        else
        {
            arrow::Type::type at = builder->type()->id();
            pdal::Dimension::Type t = computePDALTypeFromArrow(at);
            writePointData(point, id, t, builder);
        }

    }
    if (m_formatType == arrowsupport::Parquet)
    {
        arrow::ArrayBuilder* builder = m_builders[m_wkbDimId].get();
        writeWkb(point, *m_ogrPoint.get(), builder);
    }

    m_batchIndex++;

    if (m_batchIndex == (point_count_t)m_batchSize)
    {
        FlushBatch(*m_pointTablePtr);
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

            m_dimIds.push_back(m_geoArrowDimId);

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
            m_dimIds.push_back(id);
            log()->get(LogLevel::Info) << "Adding dimension '" << name << "'" << std::endl;
        }

    }

    m_schema.reset(new arrow::Schema(fields));

    if ((int)m_dimIds.size() != m_schema->num_fields())
        throwError("Arrow schema size does not match PDAL schema size!");

    createBuilders(*m_pointTablePtr);

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

void ArrowWriter::addDimensions(PointLayoutPtr layout)
{
    if (m_formatType == arrowsupport::Parquet)
        m_wkbDimId = layout->assignDim("wkb", Dimension::Type::None);

    m_geoArrowDimId = layout->assignDim("xyz", Dimension::Type::None);
}

void ArrowWriter::write(const PointViewPtr view)
{
    PointRef point(*view, 0);

    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}

void ArrowWriter::gatherParquetGeoMetadata(std::shared_ptr<arrow::KeyValueMetadata>& input, SpatialReference& ref)
{

    NL::json metadata;
    NL::json version;
    arrow::BuildInfo info = arrow::GetBuildInfo();
    version["arrow"] = info.version_string;
    version["pdal"] = pdal::Config::fullVersionString();

    NL::json column;
    column["encoding"] = "WKB";
    column["geometry_types"] = std::vector<std::string> {"Point"};

    if (ref.empty())
        ref = SpatialReference("EPSG:4326");
    column.update(getPROJJSON(ref));

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
    SpatialReference ref = m_pointTablePtr->spatialReference();
    gatherParquetGeoMetadata(m_poKeyValueMetadata, ref);
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


void ArrowWriter::FlushBatch(PointTableRef table)
{

    // Wipe off our arrays we're making a new batch
    m_arrays.clear();
    for (auto& id: m_dimIds)
    {
        arrow::ArrayBuilder* builder = m_builders[id].get();
        std::shared_ptr<arrow::Array> array;
        auto ok = builder->Finish(&array);
        if (!ok.ok())
        {
            std::stringstream msg;
            msg << "Unable to finish array";
            throwError(msg.str());
        }
        m_arrays.push_back(array);
    }

    if (m_formatType == arrowsupport::Parquet)
    {
        auto builder = m_builders.begin()->second.get();
        if (!builder)
            throwError("Unable to get first dimension builder!");
        int length = builder->length();
        auto result = m_parquetFileWriter->NewRowGroup(length);
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to make NewRowGroup" << result.ToString();
            throwError(msg.str());
        }

        for(auto& array: m_arrays)
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

    if (m_formatType == arrowsupport::Feather)
    {
        std::shared_ptr<arrow::RecordBatch> batch = arrow::RecordBatch::Make(m_schema,
                                                                             m_batchIndex,
                                                                             m_arrays);

        auto result = m_arrowFileWriter->WriteRecordBatch(*batch);
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to write arrow batch" << result.ToString();
            throwError(msg.str());
        }
    }

    m_builders.clear();
    createBuilders(*m_pointTablePtr);

}


void ArrowWriter::done(PointTableRef table)
{
    // flush our final batch
    FlushBatch(table);

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
