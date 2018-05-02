/******************************************************************************
* Copyright (c) 2014, Hobu Inc., hobu@hobu.co
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
*     * Neither the name of Hobu, Inc. Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
 * Copyright (C) 1996, 1997 Theodore Ts'o.
 *
 * %Begin-Header%
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * %End-Header%
****************************************************************************/

// This is a C++ification of the libuuid code, less the code that actually
// creates UUIDs, which is most of it and we don't need at this time.

#pragma once

#include <cstdint>
#include <string>

#include "pdal_util_export.hpp"

namespace pdal
{

#pragma pack(push)
#pragma pack(1)
struct uuid
{
    uint32_t time_low;
    uint16_t time_mid;
    uint16_t time_hi_and_version;
    uint16_t clock_seq;
    uint8_t node[6];
};
#pragma pack(pop)

inline bool operator < (const uuid& u1, const uuid& u2)
{
    if (u1.time_low != u2.time_low)
        return u1.time_low < u2.time_low;
    if (u1.time_mid != u2.time_mid)
        return u1.time_mid < u2.time_mid;
    if (u1.time_hi_and_version != u2.time_hi_and_version)
        return u1.time_hi_and_version < u2.time_hi_and_version;
    for (size_t i = 0; i < sizeof(u1.node); ++i)
        if (u1.node[i] != u2.node[i])
            return u1.node[i] < u2.node[i];
    return false;
}

class PDAL_DLL Uuid
{
    friend bool operator < (const Uuid& u1, const Uuid& u2);
public:
    Uuid();
    Uuid(const char *c);
    Uuid(const std::string& s);

    void clear();
    void unpack(const char *c);
    void pack(char *c) const;
    bool parse(const std::string& s);
    std::string unparse() const;
    std::string toString() const;
    bool empty() const;
    bool isNull() const;

/**
    // Sadly, MS doesn't do constexpr.
    static constexpr size_t size()
        { return sizeof(m_data); }
**/
    static const int size = sizeof(uuid);

private:
    uuid m_data;
};

bool operator == (const Uuid& u1, const Uuid& u2);
bool operator < (const Uuid& u1, const Uuid& u2);
std::ostream& operator << (std::ostream& out, const Uuid& u);
std::istream& operator >> (std::istream& in, Uuid& u);

} // namespace pdal

