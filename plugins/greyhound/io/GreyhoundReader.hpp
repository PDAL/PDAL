/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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

#include <functional>
#include <queue>
#include <vector>

#include <arbiter/arbiter.hpp>

#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Bounds.hpp>

#include "bounds.hpp"
#include "pool.hpp"

namespace pdal
{

namespace greyhound = entwine;

class PDAL_DLL GreyhoundReader : public pdal::Reader
{

public:
    GreyhoundReader();

    static void* create();
    static int32_t destroy(void*);
    std::string getName() const override;

private:
    std::unique_ptr<arbiter::Arbiter> m_arbiter;

    std::string m_url;
    std::string m_resource;
    std::string m_sessionId;
    BOX3D m_queryBox;
    greyhound::Bounds m_queryBounds;
    greyhound::Bounds m_fullBounds;
    std::size_t m_depthBegin;
    std::size_t m_depthEnd;
    std::size_t m_baseDepth;
    std::size_t m_sparseDepth;
    Json::Value m_info;
    Json::Value m_schema;
    std::unique_ptr<greyhound::Point> m_scale;
    std::unique_ptr<greyhound::Point> m_offset;

    mutable std::mutex m_mutex;
    point_count_t m_numPoints = 0;
    const std::size_t m_hierarchyStep = 6;
    std::size_t m_taskId = 0;
    std::queue<std::function<void()>> m_tasks;
    std::map<std::size_t, std::function<void()>> m_running;
    std::unique_ptr<std::string> m_error;

    void inc(point_count_t n)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_numPoints += n;
    }

    DimTypeList m_dims;

    uint32_t m_depthBeginArg;
    uint32_t m_depthEndArg;
    std::vector<std::string> m_pathsArg;
    Json::Value m_filter;
    int32_t m_threadsArg;

    virtual void initialize(PointTableRef table) override;
    virtual void addArgs(ProgramArgs& args) override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void prepared(PointTableRef table) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual QuickInfo inspect() override;
    virtual void done(PointTableRef table) override { }

    void launchPooledReads(
            PointView& view,
            const greyhound::Bounds& bounds,
            std::size_t depth,
            greyhound::Pool& pool);

    void read(
            PointView& view,
            Json::Value& hierarchy,
            const greyhound::Bounds& bounds,
            std::size_t startDepth,
            std::size_t depth);

    std::vector<point_count_t> fetchVerticalHierarchy(
            const greyhound::Bounds& bounds,
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    Json::Value fetchHierarchy(
            const greyhound::Bounds& bounds,
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    point_count_t fetchData(
            PointView& view,
            const greyhound::Bounds& bounds,
            std::size_t depthBegin,
            std::size_t depthEnd);
};

} // namespace pdal

