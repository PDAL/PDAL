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
    Tile(const Accessor& accessor, const Connector& connector) :
        m_accessor(accessor), m_connector(connector)
    {}
    virtual ~Tile()
    {}

    BasePointTable& table() const
        { return *m_table; }
    Key key() const
        { return m_accessor.key(); }
    //ABELL - Fix this so that it uses actual counts or something.
    point_count_t size() const
        { return m_accessor.pointCount(); }
    const std::string& error() const
        { return m_error; }
    virtual void read() = 0;

private:
    const Accessor& m_accessor;

protected:
    const Connector& m_connector;
    std::string m_error;
    // Table for the base point data.
    BasePointTablePtr m_table;
};

class EptTile : public Tile
{
public:
    EptTile(const EptAccessor& accessor, const EptInfo& info, const Connector& connector,
        const AddonList& addons) : Tile(m_eptAccessor, connector),
        m_eptAccessor(accessor), m_addons(addons), m_info(info)
    {}

    BasePointTable *addonTable(Dimension::Id id) const
        { return const_cast<EptTile *>(this)->m_addonTables[id].get(); }
    point_count_t nodeId() const
        { return m_eptAccessor.nodeId(); }
    virtual void read();

private:
    EptAccessor m_eptAccessor;
    const AddonList& m_addons;
    // Tables for the add on data.
    std::map<Dimension::Id, BasePointTablePtr> m_addonTables;
    const EptInfo& m_info;

    void readLaszip();
    void readBinary();
    void readZstandard();
    void readAddon(const Addon& addon);
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
