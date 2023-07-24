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
#include <pdal/private/gdal/GDALUtils.hpp>
#include <io/private/las/Header.hpp>

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
    m_pool(arrow::default_memory_pool())
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
    auto result = arrow::io::FileOutputStream::Open(m_filename, /*append=*/false);
    if (result.ok())
        m_file = result.ValueOrDie();
    else
    {
        std::stringstream msg;
        msg << "Unable to open '" << m_filename << "' for arrow output!";
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

    return true;
}

void ArrowWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("format", "Output format ('feature','parquet','orc')", m_format, "feather");
}

void ArrowWriter::ready(PointTableRef table)
{
    computeArrowSchema(table);


    if (!(Utils::iequals("feather", m_format) ||
          Utils::iequals("orc", m_format) ||
          Utils::iequals("parquet", m_format)))
    {
        std::stringstream msg;
        msg << "Unknown format '" << m_format <<
               "' provided. Unable to write array";
        throwError(msg.str());
    }

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

    if (Utils::iequals(m_format, "feather"))
    {
        auto result = arrow::ipc::feather::WriteTable(*m_table, m_file.get());
        result = m_file->Close();
        if (!result.ok()) {
            throwError("Unable to close feather writer");
        }
    }

    if (Utils::iequals(m_format, "parquet"))
    {
        // https://arrow.apache.org/docs/cpp/parquet.html
        // Choose compression
        std::shared_ptr<parquet::WriterProperties> props =
            parquet::WriterProperties::Builder().compression(arrow::Compression::SNAPPY)->build();

        // Opt to store Arrow schema for easier reads back into Arrow
        std::shared_ptr<parquet::ArrowWriterProperties> arrow_props =
            parquet::ArrowWriterProperties::Builder().store_schema()->build();


        auto result = parquet::arrow::WriteTable(*m_table.get(),
                                                  m_pool, m_file,
                                                  /*chunk_size=*/3, props, arrow_props);
        if (!result.ok()) {
            throwError("Unable to write parquet table");
        }
        result = m_file->Close();
    }

    if (Utils::iequals(m_format, "orc"))
    {
        // https://arrow.apache.org/docs/cpp/orc.html
        auto writer_options = arrow::adapters::orc::WriteOptions();
        auto status = arrow::adapters::orc::ORCFileWriter::Open(m_file.get(), writer_options);
        if (!status.ok()) {
           throwError("Unable to instantiate ORC writer");
        }
        std::unique_ptr<arrow::adapters::orc::ORCFileWriter> writer = std::move(status.ValueOrDie());
        if (!(writer->Write(*m_table.get())).ok()) {
            throwError("Unable to write ORC data");
        }
        if (!(writer->Close()).ok()) {
            throwError("Unable to close ORC writer");
        }
    }


    log()->get(LogLevel::Debug) << "total memory allocated " << m_pool->bytes_allocated() << std::endl;

}

} // namespaces
