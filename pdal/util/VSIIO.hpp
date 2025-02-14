/******************************************************************************
* Copyright (c) 2025, Norman Barker (norman.barker@gmail.com)
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

#pragma once

#include <cstring>
#include <iostream>


namespace pdal
{
namespace VSI
{

class VSIStreamBuffer;

class VSIStream : public std::iostream
{
private:
    VSIStreamBuffer* pVsiBuf;

public:
    VSIStream(const std::string& filename, std::ios::openmode mode);
    virtual ~VSIStream();


    VSIStream(const VSIStream &) = delete;
    VSIStream &operator=(const VSIStream &) = delete;
    VSIStream(VSIStream &&) = delete;
    VSIStream &operator=(VSIStream &&) = delete;
    VSIStream& operator<<(uint8_t v);
    VSIStream& operator<<(int8_t v);
    VSIStream& operator<<(uint16_t v);
    VSIStream& operator<<(int16_t v);
    VSIStream& operator<<(uint32_t v);
    VSIStream& operator<<(int32_t v);
    VSIStream& operator<<(uint64_t v);
    VSIStream& operator<<(int64_t v);
    VSIStream& operator<<(float v);
    VSIStream& operator<<(double v);
    VSIStream& operator>>(uint8_t& v);
    VSIStream& operator>>(int8_t& v);
    VSIStream& operator>>(uint16_t& v);
    VSIStream& operator>>(int16_t& v);
    VSIStream& operator>>(uint32_t& v);
    VSIStream& operator>>(int32_t& v);
    VSIStream& operator>>(uint64_t& v);
    VSIStream& operator>>(int64_t& v);
    VSIStream& operator>>(float& v);
    VSIStream& operator>>(double& v);
};

} // namespace VSI
} // namespace pdal
