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

#ifndef INCLUDED_TYPES_HPP
#define INCLUDED_TYPES_HPP

#include <stdint.h>

#include <pdal/pdal_internal.hpp>


namespace pdal
{

using point_count_t = uint32_t;
using PointId = uint32_t;

enum EndianType
{
    Endian_Little,
    Endian_Big,
    Endian_Unknown = 128
};


enum StageIteratorType
{
    StageIterator_Sequential = (1u << 0),
    StageIterator_Random = (1u << 1),
    StageIterator_Block = (1u << 2),
    StageIterator_Unknown = (1u << 8)
};



enum StageOperationType
{
    StageOperation_All = (1u << 0),
    StageOperation_Type = (1u << 1),
    STageOperation_Id = (1u << 2)
};

enum LogLevel
{
    logERROR = 0,
    logWARNING,
    logINFO,
    logDEBUG,
    logDEBUG1,
    logDEBUG2,
    logDEBUG3,
    logDEBUG4,
    logDEBUG5
};


} // namespace

#endif
