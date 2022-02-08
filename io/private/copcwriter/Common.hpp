/*****************************************************************************
 *   Copyright (c) 2021, Hobu, Inc. (info@hobu.co)                           *
 *                                                                           *
 *   All rights reserved.                                                    *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 3 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
 ****************************************************************************/

#pragma once

#include <stdint.h>
#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <pdal/PointTable.hpp>
#include <pdal/Scaling.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

#include <filters/StatsFilter.hpp>
#include <io/HeaderVal.hpp>

#include <io/private/las/Utils.hpp>

#include <lazperf/vlr.hpp>

namespace pdal
{

namespace stats
{

enum class Index
{
    X = 0,
    Y,
    Z,
    GpsTime,
    ReturnNumber
};

} // namespace stats

namespace copcwriter
{

// These are hopes, not absolutes.
const int MaxPointsPerNode = 100000;
const int MinimumPoints = 100;
const int MinimumTotalPoints = 1500;
const int CellCount = 128;  // Number of cells in each direction for a voxel.

struct Options
{
    std::string filename;
    StringList forwardSpec;
    NumHeaderVal<uint16_t, 0, 65535> filesourceId;
    NumHeaderVal<uint16_t, 0, 31> globalEncoding;
    UuidHeaderVal projectId;
    StringHeaderVal<32> systemId;
    StringHeaderVal<32> softwareId;
    NumHeaderVal<uint16_t, 0, 366> creationDoy;
    NumHeaderVal<uint16_t, 0, 65535> creationYear;
    StringHeaderVal<0> scaleX;
    StringHeaderVal<0> scaleY;
    StringHeaderVal<0> scaleZ;
    StringHeaderVal<0> offsetX;
    StringHeaderVal<0> offsetY;
    StringHeaderVal<0> offsetZ;
    bool fixedSeed;
};

struct Evlr
{
    lazperf::evlr_header header;
    std::vector<char> data;
};

struct BaseInfo
{
    Options opts;
    BOX3D bounds;
    BOX3D trueBounds;
    size_t pointSize;
    int maxLevel;
    SpatialReference srs;
    int pointFormatId;
    las::ExtraDims extraDims;
    int numExtraBytes; 
    Scaling scaling;
    std::set<std::string> forwards;
    bool forwardVlrs = false;
    std::vector<Evlr> vlrs;

    std::array<stats::Summary, 5> stats
    {
        stats::Summary("X", stats::Summary::NoEnum),
        stats::Summary("Y", stats::Summary::NoEnum),
        stats::Summary("Z", stats::Summary::NoEnum),
        stats::Summary("GpsTime", stats::Summary::NoEnum),
        stats::Summary("ReturnNumber", stats::Summary::Enumerate),
    };
};

} // namespace copcwriter
} // namespace pdal
