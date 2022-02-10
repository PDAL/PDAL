/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <cassert>
#include <climits>
#include <iostream>

namespace pdal
{
namespace copcwriter
{

class GridKey
{
public:
    GridKey(int i, int j, int k)
    {
        assert(i < (std::numeric_limits<uint8_t>::max)());
        assert(j < (std::numeric_limits<uint8_t>::max)());
        assert(k < (std::numeric_limits<uint8_t>::max)());
        m_key = (i << (2 * CHAR_BIT)) | (j << CHAR_BIT) | k;
    }

    int i() const
        { return (m_key >> (2 * CHAR_BIT)); }

    int j() const
        { return ((m_key >> (CHAR_BIT)) & 0xFF); }

    int k() const
        { return (m_key & 0xFF); }

    int key() const
        { return m_key; }

private:
    int m_key;
};

inline bool operator==(const GridKey& k1, const GridKey& k2)
{
    return k1.key() == k2.key();
}

inline std::ostream& operator<<(std::ostream& out, const GridKey& k)
{
    out << k.i() << "/" << k.j() << "/" << k.k();
    return out;
}

} // namespace copcwriter
} // namespace pdal

namespace std
{
    template<> struct hash<pdal::copcwriter::GridKey>
    {
        std::size_t operator()(const pdal::copcwriter::GridKey& k) const noexcept
        {
            return std::hash<int>()(k.key());
        }
    };
}
