/******************************************************************************
 * Copyright (c) 2021, Hobu Inc.
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
 *     * Neither the name of Hobu, Inc. nor the
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

#include <array>

#include <pdal/pdal_types.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/Uuid.hpp>

#pragma once

// Don't complain about DLL interface. If you want to use this out of the MS DLL, make sure
// everything is built consistently.
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable: 4251)
#endif

namespace pdal
{
namespace las
{

int baseCount(int format);

struct Header
{
    struct xyz
    {
        xyz() : x(0), y(0), z(0)
        {}

        xyz(double x, double y, double z) : x(x), y(y), z(z)
        {}

        double x;
        double y;
        double z;
    };

    static const int LegacyReturnCount {5};
    static const int ReturnCount {15};
    static const int Size {375};
    static const int Size12 {227};
    static const int Size13 {235};
    static const int Size14 {375};
    static const int FormatMask {0xF}; // Make as small as possible to improve? optimization.
    static const int CompressionMask {0x80};
    static const int WktMask {0x10};

    std::string magic { "LASF" };
    uint16_t fileSourceId {};
    uint16_t globalEncoding {};
    Uuid projectGuid;
    uint8_t versionMajor {1};
    uint8_t versionMinor {2};
    std::string systemId { "PDAL" };
    std::string softwareId;
    uint16_t creationDoy;
    uint16_t creationYear;
    uint16_t headerSize {};            // Same as VLR offset
    uint16_t& vlrOffset {headerSize};  // Synonym
    uint32_t pointOffset {};
    uint32_t vlrCount {};
    uint8_t pointFormatBits {};
    uint16_t pointSize {};
    uint32_t legacyPointCount {};
    std::array<uint32_t, LegacyReturnCount> legacyPointsByReturn;
    xyz scale;
    xyz offset;
    BOX3D bounds;
    uint64_t waveOffset {};
    uint64_t evlrOffset {};
    uint32_t evlrCount {};
    uint64_t ePointCount {};
    std::array<uint64_t, ReturnCount> ePointsByReturn;

    void fill(const char *buf, size_t bufsize);
    std::vector<char> data() const;
    StringList validate(uint64_t fileSize) const;

    int size() const
        { return versionMinor >= 4 ? Size14 : versionMinor == 3 ? Size13 : Size12; }
    int ebCount() const
    {
        int base = baseCount();
        return (base ? pointSize - base : 0);
    }
    int baseCount() const
        { return las::baseCount(pointFormat()); }
    int pointFormat() const
        { return pointFormatBits & FormatMask; }
    void setPointFormat(int format)
        { pointFormatBits = format | (pointFormatBits & CompressionMask); }
    bool dataCompressed() const
        { return pointFormatBits & CompressionMask; }
    void setDataCompressed()
        { pointFormatBits |= CompressionMask; }
    void setPointCount(uint64_t pointCount);
    void setPointsByReturn(int returnNum, uint64_t pointCount);
    bool useWkt() const
        { return globalEncoding & WktMask; }
    // It's an error if the WKT flag isn't set and the point format > 5, but we roll with it.
    bool mustUseWkt() const
        { return (useWkt() && versionMinor >= 4) || has14PointFormat(); }
    uint64_t pointCount() const
        { return versionMinor >= 4 ? ePointCount : legacyPointCount; }
    int maxReturnCount() const
        { return versionMinor >= 4 ? ReturnCount : LegacyReturnCount; }
    bool versionAtLeast(int major, int minor) const
        { return versionMinor >= minor; } // Major is always 1.
    bool has14PointFormat() const
        { return pointFormat() > 5; }
    bool hasTime() const
        { return pointFormat() == 1 || pointFormat() >= 3; }
    bool hasWave() const
    {
        int f = pointFormat();
        return f == 4 || f == 5 || f == 9 || f == 10;
    }
    bool hasColor() const
    {
        int f = pointFormat();
        return f == 2 || f == 3 || f == 5 || f == 7 || f == 8 || f == 10;
    }
    bool hasInfrared() const
        { return pointFormat() == 8; }
};

#ifdef _MSC_VER
#pragma warning (pop)
#endif

} // namespace las
} // namespace pdal

