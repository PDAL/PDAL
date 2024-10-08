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

#include <map>
#include <string>
#include <memory>
#include <unordered_set>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

using StringMap = std::map<std::string, std::string>;

namespace las
{
    struct Header;
    struct Vlr;
    class VlrCatalog;
    using VlrList = std::vector<Vlr>;
}

namespace copc
{
    class Connector;
    struct Info;
    class Key;
    class Tile;
    struct Entry;
    class Hierarchy;
    using HierarchyPage = Hierarchy;
}

class PDAL_DLL CopcReader : public Reader, public Streamable
{
public:
    CopcReader();
    virtual ~CopcReader();
    std::string getName() const override;

private:
    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize(PointTableRef table) override;
    virtual QuickInfo inspect() override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual bool processOne(PointRef& point) override;
    virtual void done(PointTableRef) override;

    void setForwards(StringMap& headers, StringMap& query);
    std::vector<char> fetch(uint64_t offset, int32_t size);
    void fetchHeader();
    las::VlrList fetchSrsVlrs(const las::VlrCatalog& catalog);
    las::Vlr fetchEbVlr(const las::VlrCatalog& catalog);
    void validateHeader(const las::Header& h);
    void validateVlrInfo(const las::Vlr& v, const copc::Info& i);
    void createSpatialFilters();

    void done();
    void loadHierarchy();
    void loadHierarchy(copc::Hierarchy& hierarchy, const copc::HierarchyPage& page,
        const copc::Entry& entry);
    bool hasSpatialFilter() const;
    bool passesFilter(const copc::Key& key) const;
    bool passesSpatialFilter(const copc::Key& key) const;
    void process(PointViewPtr dstView, const copc::Tile& tile, point_count_t count);
    bool processPoint(const char *inbuf, PointRef& dst);
    void load(const copc::Entry& entry);
    void checkTile(const copc::Tile& tile);

    struct Args;
    std::unique_ptr<Args> m_args;
    struct Private;
    std::unique_ptr<Private> m_p;
};

} // namespace pdal

