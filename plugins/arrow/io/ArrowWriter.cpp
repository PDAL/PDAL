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


#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/ipc/feather.h>
#include <parquet/arrow/writer.h>
#include <parquet/exception.h>

#include <arrow/util/type_fwd.h>
#include <pdal/PointView.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <io/private/las/Header.hpp>


std::shared_ptr<arrow::Field> toArrowType(std::string name, pdal::Dimension::Type t)
{
    using namespace pdal;
    std::shared_ptr<arrow::Field> field;
    switch (t)
    {

    case Dimension::Type::Unsigned8:
        field = arrow::field(name, arrow::uint8());
        break;
    case Dimension::Type::Signed8:
        field = arrow::field(name, arrow::int8());
        break;
    case Dimension::Type::Unsigned16:
        field = arrow::field(name, arrow::uint16());
        break;
    case Dimension::Type::Signed16:
        field = arrow::field(name, arrow::int16());
        break;
    case Dimension::Type::Unsigned32:
        field = arrow::field(name, arrow::uint32());
        break;
    case Dimension::Type::Signed32:
        field = arrow::field(name, arrow::int32());
        break;
    case Dimension::Type::Float:
        field = arrow::field(name, arrow::float32());
        break;
    case Dimension::Type::Double:
        field = arrow::field(name, arrow::float64());
        break;
    case Dimension::Type::Unsigned64:
        field = arrow::field(name, arrow::uint64());
        break;
    case Dimension::Type::Signed64:
        field = arrow::field(name, arrow::int64());
        break;
    case Dimension::Type::None:
        throw pdal_error("PDAL 'none' type unsupported for dimension " + name);
    default:
        throw pdal_error("Unrecognized PDAL dimension type for dimension " + name);

    }

    return field;

}




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

    DimBuilderMap builders;
    const auto& layout = table.layout();
    Dimension::IdList all = layout->dims();
    std::vector<std::shared_ptr<arrow::Field>> fields;
    for (auto& id : all)
    {
        auto dimType = layout->dimType(id);
        std::string name = layout->dimName(id);
        std::shared_ptr<arrow::Field> field = toArrowType(name, dimType);

        std::unique_ptr<arrow::ArrayBuilder> builder;
        arrow::Status status = arrow::MakeBuilder(m_pool, field->type(), &builder);

        m_builders.insert({id, std::move(builder)});


        fields.push_back(field);
    }

    m_schema.reset(new arrow::Schema(fields));
    int num_fields = m_schema->num_fields();
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

void AppendValue(pdal::PointRef& point,
                 pdal::Dimension::Id id,
                 pdal::Dimension::Type t,
                 arrow::ArrayBuilder* builder)
{

    switch (t)
    {

    case Dimension::Type::Unsigned8:
        {
            arrow::UInt8Builder* uint8b = dynamic_cast<arrow::UInt8Builder*>(builder);
            if (uint8b)
            {
                uint8_t value = point.getFieldAs<uint8_t>(id);
                auto ok = uint8b->Append(value);
            };
            break;
        }
    case Dimension::Type::Signed8:
        {
            arrow::Int8Builder* int8b = dynamic_cast<arrow::Int8Builder*>(builder);
            if (int8b)
            {
                int8_t value = point.getFieldAs<int8_t>(id);
                auto ok = int8b->Append(value);
            };
            break;
        }
    case Dimension::Type::Unsigned16:
        {
            arrow::UInt16Builder* uint16b = dynamic_cast<arrow::UInt16Builder*>(builder);
            if (uint16b)
            {
                uint16_t value = point.getFieldAs<uint16_t>(id);
                auto ok = uint16b->Append(value);
            };
            break;
        }
    case Dimension::Type::Signed16:
        {
            arrow::Int16Builder* int16b = dynamic_cast<arrow::Int16Builder*>(builder);
            if (int16b)
            {
                int16_t value = point.getFieldAs<int16_t>(id);
                auto ok = int16b->Append(value);
            };
            break;
        }
    case Dimension::Type::Unsigned32:
        {
            arrow::UInt32Builder* uint32b = dynamic_cast<arrow::UInt32Builder*>(builder);
            if (uint32b)
            {
                uint32_t value = point.getFieldAs<uint32_t>(id);
                auto ok = uint32b->Append(value);
            };
            break;
        }
    case Dimension::Type::Signed32:
        {
            arrow::Int32Builder* int32b = dynamic_cast<arrow::Int32Builder*>(builder);
            if (int32b)
            {
                int32_t value = point.getFieldAs<int32_t>(id);
                auto ok = int32b->Append(value);
            }
            break;
        }
    case Dimension::Type::Float:
        {
            arrow::FloatBuilder* float32b = dynamic_cast<arrow::FloatBuilder*>(builder);
            if (float32b)
            {
                float value = point.getFieldAs<float>(id);
                auto ok = float32b->Append(value);
            }
            break;
        }
    case Dimension::Type::Double:
        {
            arrow::DoubleBuilder* float64b = dynamic_cast<arrow::DoubleBuilder*>(builder);
            if (float64b)
            {
                double value = point.getFieldAs<double>(id);
                auto ok = float64b->Append(value);
            }
            break;
        }
    case Dimension::Type::Unsigned64:
        {
            arrow::UInt64Builder* uint64b = dynamic_cast<arrow::UInt64Builder*>(builder);
            if (uint64b)
            {
                uint64_t value = point.getFieldAs<uint64_t>(id);
                auto ok = uint64b->Append(value);
            }

            break;
        }
    case Dimension::Type::Signed64:
        {
            arrow::Int64Builder* int64b = dynamic_cast<arrow::Int64Builder*>(builder);
            if (int64b)
            {
                int64_t value = point.getFieldAs<int64_t>(id);
                auto ok = int64b->Append(value);
            }
            break;
        }
    case Dimension::Type::None:
        throw pdal_error("PDAL 'none' type unsupported for dimension" );
        break;
    default:
        throw pdal_error("Unrecognized PDAL dimension type for dimension");

    }
}

pdal::Dimension::Type computePDALTypeFromArrow(arrow::Type::type t)
{

    switch (t)
    {

        case arrow::Type::UINT8:
            return Dimension::Type::Unsigned8;
        case arrow::Type::UINT16:
            return Dimension::Type::Unsigned16;
        case arrow::Type::UINT32:
            return Dimension::Type::Unsigned32;
        case arrow::Type::UINT64:
            return Dimension::Type::Unsigned64;
        case arrow::Type::INT8:
            return Dimension::Type::Signed8;
        case arrow::Type::INT16:
            return Dimension::Type::Signed16;
        case arrow::Type::INT32:
            return Dimension::Type::Signed32;
        case arrow::Type::INT64:
            return Dimension::Type::Signed64;
        case arrow::Type::FLOAT:
            return Dimension::Type::Float;
        case arrow::Type::DOUBLE:
            return Dimension::Type::Double;
        default:
            throw pdal_error("Unrecognized Arrow dimension type for dimension ");

    }

    return Dimension::Type::None;

}

bool ArrowWriter::processOne(PointRef& point)
{
    for (auto& ibd: m_builders)
    {
        pdal::Dimension::Id id = ibd.first;
        arrow::ArrayBuilder* builder = ibd.second.get();
        arrow::Type::type at = builder->type()->id();
        pdal::Dimension::Type t = computePDALTypeFromArrow(at);
        AppendValue(point, id, t, builder);
    }

    return true;
}

void ArrowWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("format", "Output format ('feature','parquet')", m_format, "feather");
}

void ArrowWriter::ready(PointTableRef table)
{
     computeArrowSchema(table);
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

    for (auto& ibd: m_builders)
    {
        arrow::ArrayBuilder* builder = ibd.second.get();
        std::shared_ptr<arrow::Array> array;
        auto ok = builder->Finish(&array);
        arrays.push_back(array);
    }

    m_table = arrow::Table::Make(m_schema, arrays);

    if (Utils::iequals(m_format, "feather"))
    {
        auto result = arrow::ipc::feather::WriteTable(*m_table, m_file.get());
        result = m_file->Close();
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

        result = m_file->Close();
    }
}

} // namespaces
