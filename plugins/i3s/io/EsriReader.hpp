/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include <vector>
#include <list>
#include <map>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <arbiter/arbiter.hpp>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Bounds.hpp>

#include "EsriUtil.hpp"
#include "PageManager.hpp"

namespace pdal
{

class SrsTransform;
class ThreadPool;

class PDAL_DLL EsriReader : public Reader, public Streamable
{
public:
    EsriReader();
    ~EsriReader();

protected:
    std::unique_ptr<arbiter::Arbiter> m_arbiter;

    virtual NL::json initInfo() = 0;
    virtual std::vector<char> fetchBinary(std::string url, std::string attNum,
            std::string ext) const = 0;
    virtual std::string fetchJson(std::string) = 0;

private:
    struct Args;
    struct DimData;
    class TileContents;

    std::unique_ptr<Args> m_args;
    NL::json m_info;
    int m_nodeCap;
    i3s::Version m_version;
    SpatialReference m_nativeSrs;
    std::unique_ptr<i3s::PageManager> m_pageManager;
    std::unique_ptr<SrsTransform> m_ecefTransform;
    std::unique_ptr<ThreadPool> m_pool;
    std::vector<DimData> m_esriDims;
    size_t m_extraDimCount;
    std::vector<int> m_nodes;
    size_t m_curNodeIdx;
    size_t m_tilesToProcess;
    PointId m_pointId;
    std::queue<TileContents, std::list<TileContents>> m_contents;
    std::unique_ptr<TileContents> m_currentTile;
    mutable std::mutex m_mutex;
    mutable std::condition_variable m_contentsCv;

    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize(PointTableRef table) override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual bool processOne(PointRef&) override;
    void createView(std::string localUrl, int nodeIndex,  PointView& view);
    void traverseTree(i3s::PagePtr page, int node);
    void load(int nodeId);
    TileContents loadPath(const std::string& url);
    void checkTile(const TileContents& tile);
    void process(PointViewPtr dstView, const TileContents& tile,
        point_count_t count);
    bool processPoint(PointRef& dst, const TileContents& tile);
};

} // namespace pdal

