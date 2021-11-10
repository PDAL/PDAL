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

#include <nlohmann/json.hpp>

#include <pdal/Polygon.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/SrsTransform.hpp>

#include "Tile.hpp"

namespace pdal
{

struct OctArgs
{
public:
    SrsBounds clip;
    std::size_t threads = 0;
    double resolution = 0;
    std::vector<Polygon> polys;
    NL::json ogr;
};

struct EptArgs
{
public:
    std::string origin;
    NL::json addons;
    NL::json query;
    NL::json headers;
};

struct PolyXform
{
    Polygon poly;
    SrsTransform xform;
};

struct BoxXform
{
    BOX3D box;
    SrsTransform xform;
};

struct OctPrivate
{
public:
    OctPrivate();
    ~OctPrivate();

    std::unique_ptr<Connector> connector;
    std::unique_ptr<ThreadPool> pool;
    TilePtr currentTile;
    std::unique_ptr<Hierarchy> hierarchy;
    //ABELL - This needs a better name.
    std::queue<TilePtr> contents;
    std::mutex mutex;
    std::condition_variable contentsCv;
    std::vector<PolyXform> polys;
    BoxXform clip;
    PointId tilePointNum = 0;
    int32_t remainingTiles = 0;
    int32_t depthEnd = 0;
};

struct EptPrivate
{
    EptPrivate();
    ~EptPrivate();

    // m_filename, adjusted for historic silliness. m_filename is maintained in case we need
    // it for user feedback in some case.
    std::string adjFilename;
    std::unique_ptr<EptInfo> info;
    AddonList addons;
    int64_t queryOriginId = -1;
    int32_t hierarchyStep = 0;
};

} // namespace pdal
