/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <string.h>

#include <string>
#include <ostream>

namespace pdal
{
namespace lasdump
{

struct Exception
{
    Exception(const std::string& text) : m_text(text)
        {}

    std::string m_text;
};

inline std::ostream& operator << (std::ostream& out, const Exception& ex)
{
    out << ex.m_text;
    return out;
}


inline int32_t cksum(const void *c, size_t size)
{
    int32_t sum = 0;

    const int32_t *p = static_cast<const int32_t *>(c);
    while (size)
    {
        int32_t val = 0;
        memcpy(&val, p, std::min(sizeof(int32_t), size));
        sum += val;
        p++;
        if (size <= sizeof(int32_t))
            break;
        size -= sizeof(int32_t);
    }
    return -sum;
}

inline int32_t cksum(const std::vector<char>& v)
{
    return cksum(static_cast<const void *>(v.data()), v.size());
}

inline int32_t cksum(const std::vector<unsigned char>& v)
{
    return cksum(static_cast<const void *>(v.data()), v.size());
}

} // namespace lasdump
} // namespace pdal

