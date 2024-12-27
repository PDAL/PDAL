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
#include <arrow/type_fwd.h>

#include <pdal/Dimension.hpp>

namespace pdal
{

namespace arrowsupport
{
enum ArrowFormatType {
    Feather = 0,
    Parquet,
    Unknown = 256
};

Dimension::Type pdalType(arrow::Type::type t)
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
            return Dimension::Type::None;
    }
}

template <typename T>
struct TypeTraits {};

using DataTypePtr = std::shared_ptr<arrow::DataType>;
using FieldPtr = std::shared_ptr<arrow::Field>;

template<>
struct TypeTraits<uint8_t>
{
    using TypeClass = arrow::UInt8Type;

    static DataTypePtr dataType()
    { return arrow::uint8(); }  // Same as std::make_shared<TypeClass>(TypeClass);
};

template<>
struct TypeTraits<uint16_t>
{
    using TypeClass = arrow::UInt16Type;

    static DataTypePtr dataType()
    { return arrow::uint16(); }
};

template<>
struct TypeTraits<uint32_t>
{
    using TypeClass = arrow::UInt32Type;

    static DataTypePtr dataType()
    { return arrow::uint32(); }
};

template<>
struct TypeTraits<uint64_t>
{
    using TypeClass = arrow::UInt64Type;

    static DataTypePtr dataType()
    { return arrow::uint64(); }
};

template<>
struct TypeTraits<int8_t>
{
    using TypeClass = arrow::Int8Type;

    static DataTypePtr dataType()
    { return arrow::int8(); }
};

template<>
struct TypeTraits<int16_t>
{
    using TypeClass = arrow::Int16Type;

    static DataTypePtr dataType()
    { return arrow::int16(); }
};

template<>
struct TypeTraits<int32_t>
{
    using TypeClass = arrow::Int32Type;

    static DataTypePtr dataType()
    { return arrow::int32(); }
};

template<>
struct TypeTraits<int64_t>
{
    using TypeClass = arrow::Int64Type;

    static DataTypePtr dataType()
    { return arrow::int64(); }
};

template<>
struct TypeTraits<float>
{
    using TypeClass = arrow::FloatType;

    static DataTypePtr dataType()
    { return arrow::float32(); }
};

template<>
struct TypeTraits<double>
{
    using TypeClass = arrow::DoubleType;

    static DataTypePtr dataType()
    { return arrow::float64(); }
};

} // namespace arrowsupport
} // namespace pdal
