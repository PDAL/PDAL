/******************************************************************************
 * Copyright (c) 2020, Hobu Inc.
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

#include <pdal/PointView.hpp>
#include <pdal/pdal_types.hpp>

#include "Addon.hpp"
#include "Accessor.hpp"

namespace pdal
{

class EptInfo;
class Connector;
class BasePointTable;
using BasePointTablePtr = std::unique_ptr<BasePointTable>;

// Tile

class Tile
{
public:
    Tile(const Accessor& accessor) : m_accessor(accessor)
    {}
    virtual ~Tile()
    {}

    BasePointTable& table() const
        { return *m_table; }
    Key key() const
        { return m_accessor.key(); }
    //ABELL - Fix this so that it uses actual counts or something.
    //  If this is fixed, access can be replaced with key.
    point_count_t size() const
        { return m_accessor.pointCount(); }
    const std::string& error() const
        { return m_error; }
    virtual void read(const Connector& connector, const std::string& baseFile) = 0;

private:
    const Accessor& m_accessor;

protected:
    std::string m_error;
    // Table for the base point data.
    BasePointTablePtr m_table;
};

class EptTile : public Tile
{
public:
    EptTile(const EptAccessor& accessor, const EptInfo& info, const AddonList& addons) :
        Tile(m_eptAccessor), m_eptAccessor(accessor), m_info(info), m_addons(addons)
    {}

    point_count_t nodeId() const
        { return m_eptAccessor.nodeId(); }
    virtual void read(const Connector& connector, const std::string& baseFile);
    BasePointTable *addonTable(Dimension::Id id) const
    {
        auto it = m_addonTables.find(id);
        return (it == m_addonTables.end() ? nullptr : it->second.get());
    }

private:
    EptAccessor m_eptAccessor;
    const EptInfo& m_info;
    const AddonList& m_addons;
    // Tables for the add on data.
    using AddonTableMap = std::map<Dimension::Id, BasePointTablePtr>;
    AddonTableMap m_addonTables;

    void readLaszip(const Connector& connector, const std::string& baseFile);
    void readBinary(const Connector& connector, const std::string& baseFile);
    void readZstandard(const Connector& connector, const std::string& baseFile);
    void readAddon(const Connector& connector, const Addon& addon);
    void transform(bool skipxyz);
};

/**
class CopcTile : public Tile
{
public:
    CopcTile(const CopcAccessor& accessor, const EptInfo& info, const Connector& connector) :
        Tile(accessor, info, connector)
     {}

    virtual void read();
};
**/

using TilePtr = std::unique_ptr<Tile>;

} // namespace pdal
