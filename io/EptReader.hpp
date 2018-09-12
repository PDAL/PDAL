/******************************************************************************
* Copyright (c) 2018, Connor Manning (connor@hobu.co)
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
*       the documentation and/or key materials provided
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

#include <cstddef>

#include "EptSupport.hpp"

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

#include <arbiter/arbiter.hpp>

namespace pdal
{

class PDAL_DLL EptReader : public Reader
{
public:
    std::string getName() const override;

    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize() override;
    virtual QuickInfo inspect() override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual PointViewSet run(PointViewPtr view) override;

private:
    // If argument "origin" is specified, this function will clip the query
    // bounds to the bounds of the specified origin and set m_queryOriginId to
    // the selected OriginId value.  If the selected origin is not found, throw.
    void handleOriginQuery();

    // Aggregate all EPT keys overlapping our query bounds and their number of
    // points from a walk through the hierarchy.  Each of these keys will be
    // downloaded during the 'read' section.
    void overlaps(Pool& pool, const Json::Value& heirarchy, const Key& key);

    void readOne(PointViewPtr view, const Key& key) const;

    std::string m_root;

    std::unique_ptr<arbiter::Arbiter> m_arbiter;
    std::unique_ptr<arbiter::Endpoint> m_ep;

    Json::Value m_info;
    BOX3D m_bounds;
    uint64_t m_hierarchyStep = 0;

    class Args
    {
    public:
        Bounds& boundsArg() { return m_bounds; }
        BOX3D bounds() const
        {
            if (m_bounds.is3d())
                return m_bounds.to3d();

            const BOX2D b(m_bounds.to2d());
            if (b.empty())
                return BOX3D();

            return BOX3D(
                    b.minx, b.miny, (std::numeric_limits<double>::lowest)(),
                    b.maxx, b.maxy, (std::numeric_limits<double>::max)());
        }

        std::string& originArg() { return m_originArg; }
        const std::string& origin() const { return m_originArg; }

        uint64_t& threadsArg() { return m_threads; }
        uint64_t threads() const { return std::min<uint64_t>(4, m_threads); }

    private:
        Bounds m_bounds;
        std::string m_originArg;
        uint64_t m_threads;
    };

    Args m_args;

    BOX3D m_queryBounds;
    std::unique_ptr<uint64_t> m_queryOriginId;

    mutable std::mutex m_mutex;

    std::set<Key> m_overlapKeys;
    uint64_t m_overlapPoints = 0;
};

} // namespace pdal

