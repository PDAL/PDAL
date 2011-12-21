/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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
****************************************************************************/

#include <pdal/plang/AstUtils.hpp>

#include <string>
#include <iostream>


namespace pdal { namespace plang {

class my_visitor : public ::boost::static_visitor<DataType>
{
public:
    DataType operator()(boost::uint8_t) const  { return DataType_Uint8; }
    DataType operator()(boost::uint16_t) const { return DataType_Uint16; }
    DataType operator()(boost::uint32_t) const { return DataType_Uint32; }
    DataType operator()(boost::uint64_t) const { return DataType_Uint64; }
    DataType operator()(boost::int8_t) const   { return DataType_Int8; }
    DataType operator()(boost::int16_t) const  { return DataType_Int16; }
    DataType operator()(boost::int32_t) const  { return DataType_Int32; }
    DataType operator()(boost::int64_t) const  { return DataType_Int64; }
    DataType operator()(float) const           { return DataType_Float32; }
    DataType operator()(double) const          { return DataType_Float64; }
    DataType operator()(bool) const            { return DataType_Bool; }
};

DataType AstUtils::inferType(variant_t value)
{


    return ::boost::apply_visitor(my_visitor(), value);
}


bool AstUtils::convert(DataType srcEnum, variant_t srcValue, DataType dstEnum, variant_t& dstValue)
{
    switch (srcEnum)
    {
#define CASE_BODY(tname) \
        { \
            const tname v = boost::get<tname>(srcValue); \
            switch (dstEnum) \
            { \
            case DataType_Uint8:   dstValue = (boost::uint8_t)v; break; \
            case DataType_Uint16:  dstValue = (boost::uint16_t)v; break; \
            case DataType_Uint32:  dstValue = (boost::uint32_t)v; break; \
            case DataType_Uint64:  dstValue = (boost::uint64_t)v; break; \
            case DataType_Int8:    dstValue = (boost::int8_t)v; break; \
            case DataType_Int16:   dstValue = (boost::int16_t)v; break; \
            case DataType_Int32:   dstValue = (boost::int32_t)v; break; \
            case DataType_Int64:   dstValue = (boost::int64_t)v; break; \
            case DataType_Float32: dstValue = (float)v; break; \
            case DataType_Float64: dstValue = (double)v; break; \
            case DataType_Bool: assert(0); break; \
            default: assert(0); \
            } \
        } \
        break
    case DataType_Uint8:   CASE_BODY(boost::uint8_t);
    case DataType_Uint16:  CASE_BODY(boost::uint16_t);
    case DataType_Uint32:  CASE_BODY(boost::uint32_t);
    case DataType_Uint64:  CASE_BODY(boost::uint64_t);
    case DataType_Int8:    CASE_BODY(boost::int8_t);
    case DataType_Int16:   CASE_BODY(boost::int16_t);
    case DataType_Int32:   CASE_BODY(boost::int32_t);
    case DataType_Int64:   CASE_BODY(boost::int64_t);
    case DataType_Float32: CASE_BODY(float);
    case DataType_Float64: CASE_BODY(double);
    case DataType_Bool:    assert(0); break;
#undef CASE_BODY
    default:
        assert(0);
    }

    return true;
}


template<typename T>
static void do_arithmetic(variant_t leftValue, variant_t rightValue, variant_t& dstValue, NodeType nodetype)
{
    const T left = boost::get<T>(leftValue);
    const T right = boost::get<T>(rightValue);
    T dst;
    
    switch (nodetype)
    {
    case NodeType_Add: dst = left + right; break;
    case NodeType_Subtract: dst = left - right; break;
    case NodeType_Multiply: dst = left * right; break;
    case NodeType_Divide: dst = left / right; break;
    default: assert(0);
    }

    dstValue = dst;

    return;
}


template<typename T>
static void do_relational(variant_t leftValue, variant_t rightValue, variant_t& dstValue, NodeType nodetype)
{
    const T left = boost::get<T>(leftValue);
    const T right = boost::get<T>(rightValue);
    bool dst;
    
    switch (nodetype)
    {
    case NodeType_Greater:   dst = left > right; break;
    case NodeType_GreaterEq: dst = left >= right; break;
    case NodeType_Less:      dst = left < right; break;
    case NodeType_LessEq:    dst = left <= right; break;
    default: assert(0);
    }

    dstValue = dst;

    return;
}


template<typename T>
static void do_equality(variant_t leftValue, variant_t rightValue, variant_t& dstValue, NodeType nodetype)
{
    const T left = boost::get<T>(leftValue);
    const T right = boost::get<T>(rightValue);
    bool dst;
    
    switch (nodetype)
    {
    case NodeType_Equal:     dst = left == right; break;
    case NodeType_NotEqual:  dst = left != right; break;
    default: assert(0);
    }

    dstValue = dst;

    return;
}


template<typename T>
static void do_bit_arithmetic(variant_t leftValue, variant_t rightValue, variant_t& dstValue, NodeType nodetype)
{
    const T left = boost::get<T>(leftValue);
    const T right = boost::get<T>(rightValue);
    T dst;
    
    switch (nodetype)
    {
    case NodeType_ArithXor:     dst = left ^ right; break;
    case NodeType_ArithOr:      dst = left | right; break;
    case NodeType_ArithAnd:     dst = left & right; break;
    default: assert(0);
    }

    dstValue = dst;

    return;
}


template<typename T>
static void do_bit_logical(variant_t leftValue, variant_t rightValue, variant_t& dstValue, NodeType nodetype)
{
    const T left = boost::get<T>(leftValue);
    const T right = boost::get<T>(rightValue);
    T dst;
    
    switch (nodetype)
    {
    case NodeType_LogicalAnd:   dst = left && right; break;
    case NodeType_LogicalOr:    dst = left || right; break;
    default: assert(0);
    }

    dstValue = dst;

    return;
}

bool AstUtils::apply_binop(NodeType nodetype, DataType datatype, variant_t leftValue, variant_t rightValue, variant_t& dstValue)
{
    switch (nodetype)
    {
    case NodeType_Add:
    case NodeType_Subtract:
    case NodeType_Multiply:
    case NodeType_Divide:
        switch (datatype)
        {
        case DataType_Uint8:   do_arithmetic<boost::uint8_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint16:  do_arithmetic<boost::uint16_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint32:  do_arithmetic<boost::uint32_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint64:  do_arithmetic<boost::uint64_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int8:    do_arithmetic<boost::int8_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int16:   do_arithmetic<boost::int16_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int32:   do_arithmetic<boost::int32_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int64:   do_arithmetic<boost::int64_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Float32: do_arithmetic<float>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Float64: do_arithmetic<double>(leftValue, rightValue, dstValue, nodetype); break;
        default: assert(0);
        }
        break;

    case NodeType_Greater:
    case NodeType_GreaterEq:
    case NodeType_Less:
    case NodeType_LessEq:
        switch (datatype)
        {
        case DataType_Uint8:   do_relational<boost::uint8_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint16:  do_relational<boost::uint16_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint32:  do_relational<boost::uint32_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint64:  do_relational<boost::uint64_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int8:    do_relational<boost::int8_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int16:   do_relational<boost::int16_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int32:   do_relational<boost::int32_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int64:   do_relational<boost::int64_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Float32: do_relational<float>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Float64: do_relational<double>(leftValue, rightValue, dstValue, nodetype); break;
        default: assert(0);
        }
        break;

    case NodeType_Equal:
    case NodeType_NotEqual:
        switch (datatype)
        {
        case DataType_Uint8:   do_equality<boost::uint8_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint16:  do_equality<boost::uint16_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint32:  do_equality<boost::uint32_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint64:  do_equality<boost::uint64_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int8:    do_equality<boost::int8_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int16:   do_equality<boost::int16_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int32:   do_equality<boost::int32_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Int64:   do_equality<boost::int64_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Float32: do_equality<float>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Float64: do_equality<double>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Bool:    do_equality<bool>(leftValue, rightValue, dstValue, nodetype); break;
        default: assert(0);
        }
        break;

    case NodeType_ArithXor:
    case NodeType_ArithAnd:
    case NodeType_ArithOr:
        switch (datatype)
        {
        case DataType_Uint8:   do_bit_arithmetic<boost::uint8_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint16:  do_bit_arithmetic<boost::uint16_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint32:  do_bit_arithmetic<boost::uint32_t>(leftValue, rightValue, dstValue, nodetype); break;
        case DataType_Uint64:  do_bit_arithmetic<boost::uint64_t>(leftValue, rightValue, dstValue, nodetype); break;
        default: assert(0);
        }
        break;

    case NodeType_LogicalAnd:
    case NodeType_LogicalOr:
        switch (datatype)
        {
        case DataType_Bool:    do_bit_logical<bool>(leftValue, rightValue, dstValue, nodetype); break;
        default: assert(0);
        }
        break;

    default:
        assert(0);
        break;
    }

    return true;
}


bool AstUtils::apply_negate(DataType datatype, variant_t srcValue, variant_t& dstValue)
{
    switch (datatype)
    {
#define CASE_BODY(tname) \
    { \
        const tname v = boost::get<tname>(srcValue); \
        const tname vv = -v; \
        dstValue = vv; \
    } \
    break
    case DataType_Uint8:
    case DataType_Uint16:
    case DataType_Uint32:
    case DataType_Uint64:
        // illegal to negate an unsigned value
        assert(0);
        break;
    case DataType_Int8:    CASE_BODY(boost::int8_t);
    case DataType_Int16:   CASE_BODY(boost::int16_t);
    case DataType_Int32:   CASE_BODY(boost::int32_t);
    case DataType_Int64:   CASE_BODY(boost::int64_t);
    case DataType_Float32: CASE_BODY(float);
    case DataType_Float64: CASE_BODY(double);
    case DataType_Bool:    assert(0); break;
    default:
        assert(0);
#undef CASE_BODY
    }

    return true;
}


std::string AstUtils::getName(DataType datatype)
{
    switch (datatype)
    {
    case DataType_Unknown: return "Unknown";
    case DataType_Int8: return "I8";
    case DataType_Int16: return "I168";
    case DataType_Int32: return "I32";
    case DataType_Int64: return "I64";
    case DataType_Uint8: return "U8";
    case DataType_Uint16: return "U168";
    case DataType_Uint32: return "U32";
    case DataType_Uint64: return "U64";
    case DataType_Float32: return "F32";
    case DataType_Float64: return "F64";
    case DataType_Bool: return "Bool";
    }

    assert(0);
    return "";
}


std::string AstUtils::getName(NodeType nodetype)
{
    switch (nodetype)
    {
    case NodeType_Add1:      return "Add1";
    case NodeType_Subtract1: return "Subtract1";
    case NodeType_Multiply1: return "Multiply1";
    case NodeType_Divide1:   return "Divide1";

    case NodeType_Add:       return "Addition";
    case NodeType_Subtract:  return "Subtract";
    case NodeType_Multiply:  return "Multiply";
    case NodeType_Divide:    return "Divide";

    case NodeType_Negate:    return "Negate";

    case NodeType_Greater1:   return "Greater1";
    case NodeType_GreaterEq1: return "GreaterEq1";
    case NodeType_Less1:      return "Less1";
    case NodeType_LessEq1:    return "LessEq1";

    case NodeType_Greater:   return "Greater";
    case NodeType_GreaterEq: return "GreaterEq";
    case NodeType_Less:      return "Less";
    case NodeType_LessEq:    return "LessEq";

    case NodeType_Equal1:     return "Equal1";
    case NodeType_NotEqual1:  return "NotEqual1";

    case NodeType_Equal:     return "Equal";
    case NodeType_NotEqual:  return "NotEqual";

    case NodeType_ArithXor1:     return "ArithXor1";
    case NodeType_ArithOr1:      return "ArithOr1";
    case NodeType_ArithAnd1:     return "ArithAnd1";
    case NodeType_LogicalOr1:    return "LogicalOr1";
    case NodeType_LogicalAnd1:   return "LogicalAnd1";

    case NodeType_ArithXor:     return "ArithXor";
    case NodeType_ArithOr:      return "ArithOr";
    case NodeType_ArithAnd:     return "ArithAnd";
    case NodeType_LogicalOr:    return "LogicalOr";
    case NodeType_LogicalAnd:   return "LogicalAnd";
    
    case NodeType_Convert:   return "Convert";
    case NodeType_Constant:  return "Constant";
    case NodeType_Variable:  return "Variable";
    case NodeType_NopV:      return "NopV";
    }

    assert(0);
    return "";
}


} } // namespaces
