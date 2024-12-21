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

#include <arrow/api.h>
/**
#include <arrow/io/api.h>
#include <arrow/ipc/api.h>
#include <arrow/ipc/reader.h>
#include <arrow/adapters/orc/adapter.h>
#include <arrow/util/key_value_metadata.h>
#include <arrow/util/base64.h>

#include <parquet/arrow/reader.h>
#include <parquet/exception.h>
#include <parquet/arrow/writer.h>
#include <parquet/types.h>
#include <parquet/metadata.h>
#include <parquet/file_writer.h>
#include <parquet/schema.h>
#include <parquet/arrow/writer.h>
#include <parquet/arrow/schema.h>

#include <pdal/pdal_types.hpp>
**/

namespace pdal
{

namespace arrowsupport
{
enum ArrowFormatType {
    Feather = 0,
    Parquet,
    Unknown = 256
};

}
}

pdal::Dimension::Type computePDALTypeFromArrow(arrow::Type::type t)
{
    using namespace pdal;

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
            return Dimension::Type::None;

    }

    return Dimension::Type::None;

}


std::shared_ptr<arrow::Field> toArrowType(std::string name, pdal::Dimension::Type t)
{
    using namespace pdal;
    std::shared_ptr<arrow::Field> field;
    switch (t)
    {

    case Dimension::Type::Unsigned8:
        field = arrow::field(name, arrow::uint8(), false);
        break;
    case Dimension::Type::Signed8:
        field = arrow::field(name, arrow::int8(), false);
        break;
    case Dimension::Type::Unsigned16:
        field = arrow::field(name, arrow::uint16(), false);
        break;
    case Dimension::Type::Signed16:
        field = arrow::field(name, arrow::int16(), false);
        break;
    case Dimension::Type::Unsigned32:
        field = arrow::field(name, arrow::uint32(), false);
        break;
    case Dimension::Type::Signed32:
        field = arrow::field(name, arrow::int32(), false);
        break;
    case Dimension::Type::Float:
        field = arrow::field(name, arrow::float32(), false);
        break;
    case Dimension::Type::Double:
        field = arrow::field(name, arrow::float64(), false);
        break;
    case Dimension::Type::Unsigned64:
        field = arrow::field(name, arrow::uint64(), false);
        break;
    case Dimension::Type::Signed64:
        field = arrow::field(name, arrow::int64(), false);
        break;
    case Dimension::Type::None:
        field = arrow::field(name, arrow::binary(), false);
        break;
    default:
        throw pdal_error("Unrecognized PDAL dimension type for dimension " + name);

    }

    return field;

}
