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
#include <arrow/io/api.h>
#include <arrow/ipc/api.h>
#include <arrow/ipc/reader.h>
#include <arrow/adapters/orc/adapter.h>

#include <parquet/arrow/writer.h>
#include <parquet/exception.h>
#include <parquet/arrow/writer.h>

#include <pdal/pdal_types.hpp>

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
            throw pdal_error("Unrecognized Arrow dimension type for dimension ");

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
        throw pdal_error("PDAL 'none' type unsupported for dimension " + name);
    default:
        throw pdal_error("Unrecognized PDAL dimension type for dimension " + name);

    }

    return field;

}



void writePointData(pdal::PointRef& point,
                    pdal::Dimension::Id id,
                    pdal::Dimension::Type t,
                    arrow::ArrayBuilder* builder)
{
    using namespace pdal;

    switch (t)
    {

    case Dimension::Type::Unsigned8:
        {
            arrow::UInt8Builder* uint8b = dynamic_cast<arrow::UInt8Builder*>(builder);
            if (uint8b)
            {
                uint8_t value = point.getFieldAs<uint8_t>(id);
                auto ok = uint8b->Append(value);
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write uint8_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write int8_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write uint16_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write int16_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write uint32_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write int32_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write float32 value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write float64 value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write uint64_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
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
                if (!ok.ok())
                {
                    std::stringstream msg;
                    msg << "Unable to write int64_t value '" << value << "' to file";
                    throw pdal_error(msg.str());
                }
            }
            break;
        }
    case Dimension::Type::None:
        throw pdal_error("PDAL 'none' type unsupported for dimension" );
    default:
        throw pdal_error("Unrecognized PDAL dimension type for dimension");

    }
}

