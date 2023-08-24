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
    m_writeGeoParquet(false)
{
}

ArrowWriter::~ArrowWriter()
{}


void ArrowWriter::computeArrowSchema(pdal::PointTableRef table)
{
    using namespace pdal;

    const auto& layout = table.layout();
    std::vector<std::shared_ptr<arrow::Field>> fields;
    int count = 0;

    auto dims = layout->dims();
    auto rng = std::default_random_engine {};
    std::shuffle(std::begin(dims), std::end(dims), rng);



    for (auto& id : dims)
    {
        auto dimType = layout->dimType(id);
        std::string name = layout->dimName(id);
        std::shared_ptr<arrow::Field> field = toArrowType(name, dimType);

        std::unique_ptr<arrow::ArrayBuilder> builder;
        arrow::Status status = arrow::MakeBuilder(m_pool, field->type(), &builder);
        if (!status.ok())
        {
            std::stringstream msg;
            msg << "Unable to create builder for '" << name << "'";
            throwError(msg.str());
        }

        m_builders.insert({id, std::move(builder)});

        fields.push_back(field);
        m_dimIds.push_back(id);
    }


    m_schema.reset(new arrow::Schema(fields));

    int num_fields = m_schema->num_fields();
    if ((int)layout->dims().size() != (int)num_fields)
        throwError("Arrow schema size does not match PDAL schema size!");
}

void ArrowWriter::initialize()
{

    std::string ext = Utils::tolower(FileUtils::extension(m_filename));
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

    auto result = arrow::io::FileOutputStream::Open(m_filename, /*append=*/false);
    if (result.ok())
        m_file = result.ValueOrDie();
    else
    {
        std::stringstream msg;
        msg << "Unable to open '" << m_filename << "' for arrow output with error " << result.status().ToString();
        throwError(msg.str());
    }
}


bool ArrowWriter::processOne(PointRef& point)
{
    for (auto& id: m_dimIds)
    {
        arrow::ArrayBuilder* builder = m_builders[id].get();
        arrow::Type::type at = builder->type()->id();
        pdal::Dimension::Type t = computePDALTypeFromArrow(at);

        writePointData(point, id, t, builder);
    }
    if (m_writeGeoParquet)
    {
        arrow::ArrayBuilder* builder = m_builders[m_wkbDimId].get();
        writeWkb(point, builder);

    }
    return true;
}

void ArrowWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("format", "Output format ('feature','parquet','geoparquet','orc')", m_formatString, "feather");
    args.add("geoparquet", "Write geoparquet when writing parquet?", m_writeGeoParquet, false);
}

void ArrowWriter::ready(PointTableRef table)
{
    computeArrowSchema(table);
}

void ArrowWriter::addDimensions(PointLayoutPtr layout)
{
    if (m_writeGeoParquet)
        m_wkbDimId = layout->registerOrAssignDim("wkb", Dimension::Type::None);
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

void ArrowWriter::gatherGeoMetadata(std::shared_ptr<arrow::KeyValueMetadata>& input, SpatialReference& ref)
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
    column["edges"] = ref.isGeographic() ? "spherical" : "planar";
    
    
    NL::json projjson;
    try
    {
        projjson  = NL::json::parse(ref.getPROJJSON());
    } catch (NL::json::parse_error& e)
    {
        log()->get(LogLevel::Warning) << "unable to parse projjson" << std::endl;
    }
    column["crs"] = projjson;

    NL::json wkb;
    wkb["wkb"] = column;

    NL::json geo;
    geo["version"] = "1.0.0-dev"; // GeoParquet version
    geo["primary_column"] = "wkb";
    geo["columns"] = wkb;

    input->Append("geo", geo.dump(-1));




}
void ArrowWriter::writeParquet(std::vector<std::shared_ptr<arrow::Array>> const& arrays,
                               PointTableRef table)
{



    parquet::WriterProperties::Builder m_oWriterPropertiesBuilder{};
    std::unique_ptr<parquet::arrow::FileWriter> m_poFileWriter{};

    m_oWriterPropertiesBuilder = parquet::WriterProperties::Builder();
    m_oWriterPropertiesBuilder.max_row_group_length(64 * 1024);
    m_oWriterPropertiesBuilder.created_by(pdal::Config::fullVersionString());
    m_oWriterPropertiesBuilder.version(parquet::ParquetVersion::PARQUET_2_6);
    m_oWriterPropertiesBuilder.data_page_version(parquet::ParquetDataPageVersion::V2);
    m_oWriterPropertiesBuilder.compression(parquet::Compression::SNAPPY);
    m_oWriterPropertiesBuilder.build();

    std::shared_ptr<parquet::ArrowWriterProperties> arrowWriterProperties =
        parquet::ArrowWriterProperties::Builder().store_schema()->build();

    std::shared_ptr<parquet::SchemaDescriptor> parquet_schema;
    auto result = parquet::arrow::ToParquetSchema( &(*m_table->schema()), 
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

    std::shared_ptr<arrow::KeyValueMetadata> m_poKeyValueMetadata;

    m_poKeyValueMetadata = m_table->schema()->metadata()
                           ? m_table->schema()->metadata()->Copy()
                           : std::make_shared<arrow::KeyValueMetadata>();


    std::unique_ptr<parquet::ParquetFileWriter> base_writer;
    base_writer = parquet::ParquetFileWriter::Open(
                             m_file, schema_node,
                             m_oWriterPropertiesBuilder.build(), m_poKeyValueMetadata);
    if (!result.ok()) 
    {
        std::stringstream msg;
        msg << "Unable to convert open ParquetFileWriter with error '" << result.ToString() << "'";
        throwError(msg.str());
    }

    SpatialReference ref = table.spatialReference();
    gatherGeoMetadata(m_poKeyValueMetadata, ref);


    auto schema_ptr = std::make_shared<::arrow::Schema>(*m_table->schema());

    result = parquet::arrow::FileWriter::Make(
                m_pool, std::move(base_writer), std::move(schema_ptr),
                arrowWriterProperties, &m_poFileWriter);
    if (!result.ok()) 
    {
        std::stringstream msg;
        msg << "Unable to make parquet::arrow::FileWriter " << result.ToString();
        throwError(msg.str());
    }

    result = m_poFileWriter->NewRowGroup(m_builders[pdal::Dimension::Id::X]->length());
    if (!result.ok())
    {
        std::stringstream msg;
        msg << "Unable to make NewRowGroup" << result.ToString();
        throwError(msg.str());
    }

    for(auto& array: arrays)
    {
        result = m_poFileWriter->WriteColumnChunk(*array);
        if (!result.ok())
        {
            std::stringstream msg;
            msg << "Unable to make WriteColumnChunk" << result.ToString();
            throwError(msg.str());
        }

    }


    result = m_poFileWriter->Close();
    if (!result.ok())
    {
        std::stringstream msg;
        msg << "Unable to close FileWriter" << result.ToString();
        throwError(msg.str());
    }

    result = m_file->Close();

}


void ArrowWriter::done(PointTableRef table)
{

    std::vector<std::shared_ptr<arrow::Array>> arrays;

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
        arrays.push_back(array);
    }

    if ( (int)arrays.size() != m_schema->num_fields())
    {
        std::stringstream msg;
        msg << "Arrow schema size does not match PDAL schema size!";
        throwError(msg.str());
    }

    m_table = arrow::Table::Make(m_schema, arrays);


    if (m_formatType == arrowsupport::Feather)
    {


        std::shared_ptr<arrow::KeyValueMetadata> m_poKeyValueMetadata;

        m_poKeyValueMetadata = m_table->schema()->metadata()
                            ? m_table->schema()->metadata()->Copy()
                            : std::make_shared<arrow::KeyValueMetadata>();

        SpatialReference ref = table.spatialReference();
        gatherGeoMetadata(m_poKeyValueMetadata, ref);

        m_table = m_table->ReplaceSchemaMetadata(m_poKeyValueMetadata);
        auto result = arrow::ipc::feather::WriteTable(*m_table, m_file.get());
        result = m_file->Close();
        if (!result.ok()) 
        {
            std::stringstream msg;
            msg << "Unable to open to write feather table for file '" << m_filename << "' for with error " << result.ToString();
            throwError(msg.str());
        }
    }

    if (m_formatType == arrowsupport::Parquet)
    {

        writeParquet(arrays, table);
    }


    log()->get(LogLevel::Debug) << "total memory allocated " << m_pool->bytes_allocated() << std::endl;

}

} // namespaces
