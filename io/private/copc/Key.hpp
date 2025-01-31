/******************************************************************************
 * Copyright (c) 2018, Connor Manning
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

#include <functional>  // for hash

#include <pdal/pdal_types.hpp>

#include <pdal/util/Bounds.hpp>
#include <pdal/util/Extractor.hpp>

namespace pdal
{
namespace copc
{

class PDAL_EXPORT Key
{
    // A depth/X/Y/Z key representing a data node, as well as the bounds of the contained data.
public:
    Key()
    {}

    Key(const std::string& s)
    {
        fill(s);
    }

    bool valid() const
    {
        return d != -1;
    }

    bool fill(const std::string& s)
    {
        d = -1;
        const StringList tokens(Utils::split(s, '-'));
        if (tokens.size() != 4)
            return false;

        size_t cnt;
        d = std::stoi(tokens[0], &cnt);
        if (cnt != tokens[0].size())
            return false;
        x = std::stoi(tokens[1], &cnt);
        if (cnt != tokens[1].size())
            return false;
        y = std::stoi(tokens[2], &cnt);
        if (cnt != tokens[2].size())
            return false;
        z = std::stoi(tokens[3], &cnt);
        if (cnt != tokens[3].size())
            return false;
        return true;
    }

    int32_t d = 0;
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    std::string toString() const
    {
        return (std::string)(*this);
    }

    operator std::string() const
    {
        return std::to_string(d) + '-' + std::to_string(x) + '-' +
            std::to_string(y) + '-' + std::to_string(z);
    }

    Key child(int32_t dir) const
    {
        return Key(d + 1,
            (x << 1) | (dir & 0x1),
            (y << 1) | ((dir >> 1) & 0x1),
            (z << 1) | ((dir >> 2) & 0x1));
    }

    BOX3D bounds(const BOX3D& root) const
    {
        BOX3D cellBounds;

        int width = pow(2, d);
        double cellWidth = (root.maxx - root.minx) / width;
        // The test in each of these is to avoid unnecessary rounding errors when
        // we know the actual value.
        cellBounds.minx = (x == 0 ? root.minx : root.minx + (cellWidth * x));
        cellBounds.maxx = (x == (width - 1) ? root.maxx : root.minx + (cellWidth * (x + 1)));
        cellBounds.miny = (y == 0 ? root.miny : root.miny + (cellWidth * y));
        cellBounds.maxy = (y == (width - 1) ? root.maxy : root.miny + (cellWidth * (y + 1)));
        cellBounds.minz = (z == 0 ? root.minz : root.minz + (cellWidth * z));
        cellBounds.maxz = (z == (width - 1) ? root.maxz : root.minz + (cellWidth * (z + 1)));
        return cellBounds;
    }

    static Key invalid()
    {
        static Key badkey = Key(-1, 0, 0, 0);

        return badkey;
    }

private:
    Key(int d, int x, int y, int z) : d(d), x(x), y(y), z(z)
    {}
};

inline LeExtractor& operator>>(LeExtractor& in, Key& k)
{
    in >> k.d >> k.x >> k.y >> k.z;
    return in;
}

inline bool operator==(const Key& a, const Key& b)
{
    return a.d == b.d && a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool operator!=(const Key& a, const Key& b)
{
    return !(a == b);
}

inline bool operator<(const Key& a, const Key& b)
{
    if (a.d < b.d) return true;
    if (a.d > b.d) return false;

    if (a.x < b.x) return true;
    if (a.x > b.x) return false;

    if (a.y < b.y) return true;
    if (a.y > b.y) return false;

    if (a.z < b.z) return true;
    return false;
}

inline std::ostream& operator<<(std::ostream& out, const Key& k)
{
    out << k.toString();
    return out;
}

} // namespace copc
} // namespace pdal

namespace std
{
    template<>
    struct hash<pdal::copc::Key>
    {
        std::size_t operator()(pdal::copc::Key const& k) const noexcept
        {
            std::hash<uint64_t> h;

            uint64_t k1 = ((uint64_t)k.d << 32) | k.x;
            uint64_t k2 = ((uint64_t)k.y << 32) | k.z;
            return h(k1) ^ (h(k2) << 1);
        }
    };
}

