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

namespace pdal
{

class PDAL_DLL Key
{
    // An EPT key representation (see https://git.io/fAiBh).  A depth/X/Y/Z key
    // representing a data node, as well as the bounds of the contained data.
public:
    Key()
    {}

    Key(const std::string& s)
    {
        const StringList tokens(Utils::split(s, '-'));
        if (tokens.size() != 4)
            throw pdal_error("Invalid EPT KEY: " + s);
        d = std::stoi(tokens[0]);
        x = std::stoi(tokens[1]);
        y = std::stoi(tokens[2]);
        z = std::stoi(tokens[3]);
    }

    BOX3D b;

    uint32_t d = 0;
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;

    std::string toString() const
    {
        return std::to_string(d) + '-' + std::to_string(x) + '-' +
            std::to_string(y) + '-' + std::to_string(z);
    }

    double& operator[](uint64_t i)
    {
        switch (i)
        {
            case 0: return b.minx;
            case 1: return b.miny;
            case 2: return b.minz;
            case 3: return b.maxx;
            case 4: return b.maxy;
            case 5: return b.maxz;
            default: throw pdal_error("Invalid Key[] index");
        }
    }

    uint32_t& idAt(uint64_t i)
    {
        switch (i)
        {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw pdal_error("Invalid Key::idAt index");
        }
    }

    Key bisect(uint64_t direction) const
    {
        Key key(*this);
        ++key.d;

        auto step([&key, direction](uint8_t i)
        {
            key.idAt(i) *= 2;

            const double mid(key[i] + (key[i + 3] - key[i]) / 2.0);
            const bool positive(direction & (((uint64_t)1) << i));
            if (positive)
            {
                key[i] = mid;
                ++key.idAt(i);
            }
            else
            {
                key[i + 3] = mid;
            }
        });

        for (uint8_t i(0); i < 3; ++i)
            step(i);

        return key;
    }
};

inline bool operator==(const Key& a, const Key& b)
{
    return a.d == b.d && a.x == b.x && a.y == b.y && a.z == b.z;
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

} // namespace pdal

namespace std
{
    template<>
    struct hash<pdal::Key>
    {
        std::size_t operator()(pdal::Key const& k) const noexcept
        {
            std::hash<uint64_t> h;

            uint64_t k1 = ((uint64_t)k.d << 32) | k.x;
            uint64_t k2 = ((uint64_t)k.y << 32) | k.z;
            return h(k1) ^ (h(k2) << 1);
        }
    };
}

