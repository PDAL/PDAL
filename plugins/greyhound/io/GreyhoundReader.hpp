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

#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Bounds.hpp>
#include <arbiter.hpp>

#include "dir.hpp"
#include "bbox.hpp"

namespace pdal
{

class PDAL_DLL GreyhoundReader : public pdal::Reader
{

public:
    GreyhoundReader();
    ~GreyhoundReader();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

private:
    std::string m_url;
    std::string m_resource;
    std::string m_sessionId;
    point_count_t m_numPoints;
    point_count_t m_index;
    BOX3D m_queryBounds;
    BOX3D m_conformingBounds;
//     BOX3D m_bounds;
    uint32_t m_depthBegin;
    uint32_t m_depthEnd;
    uint32_t m_baseDepth;
    uint32_t m_stopSplittingDepth;
    uint32_t m_split;
    uint32_t m_retryCount;
    Json::Value m_resourceInfo;
    uint32_t m_timeout;
    point_count_t m_splitCountThreshold;

    virtual void initialize(PointTableRef table);
    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual bool eof() const;
    virtual QuickInfo inspect();
    virtual void done(PointTableRef table);

    Json::Value fetch(const std::string& url) const;
    DimTypeList getSchema(const Json::Value& jsondata) const;
    BOX3D getBounds(const Json::Value& jsondata, const std::string& memberName) const;
    point_count_t readLevel(PointViewPtr view, point_count_t count, BOX3D bounds, uint32_t readBegin, uint32_t readEnd);
//     BOX3D zoom(BOX3D bounds, BOX3D fullBox, int& split) const;

    Json::Value fetchHierarchy(BOX3D bounds, uint32_t depthBegin, uint32_t depthEnd)  const;

    point_count_t readDirection(const greyhound::BBox& currentBox,
                                            const greyhound::BBox& queryBox,
                                            uint32_t& depthBegin,
                                            uint32_t& depthEnd,
                                            point_count_t count,
                                            PointViewPtr view,
                                            const Json::Value& hierarchy);
    DimTypeList m_dimData;
};

} // namespace pdal

