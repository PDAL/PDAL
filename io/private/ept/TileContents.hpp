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
#include "Overlap.hpp"

namespace pdal
{

class EptInfo;
class Connector;
class BasePointTable;
using BasePointTablePtr = std::unique_ptr<BasePointTable>;

class TileContents
{
public:
    TileContents(const Overlap& overlap, const EptInfo& info,
            const Connector& connector, const AddonList& addons) :
        m_overlap(overlap), m_info(info), m_connector(connector),
        m_addons(addons)
    {}

    BasePointTable& table() const
        { return *m_table; }
    const Key& key() const
        { return m_overlap.m_key; }
    point_count_t nodeId() const
        { return m_overlap.m_nodeId; }
    //ABELL - This is bad. We're assuming that the actual number of points we have matches
    // what our index information told us. This may not be the case because of some
    // issue. Downstream handling may depend on this being the actual number of points
    // in the tile, rather than the number that were *supposed to be* in the tile.
    point_count_t size() const
        { return m_overlap.m_count; }
    const std::string& error() const
        { return m_error; }
    BasePointTable *addonTable(Dimension::Id id) const
        { return const_cast<TileContents *>(this)->m_addonTables[id].get(); }
    void read();

private:
    Overlap m_overlap;
    const EptInfo& m_info;
    const Connector& m_connector;
    const AddonList& m_addons;
    std::string m_error;
    // Table for the base point data.
    BasePointTablePtr m_table;
    // Tables for the add on data.
    std::map<Dimension::Id, BasePointTablePtr> m_addonTables;

    void readLaszip();
    void readBinary();
    void readZstandard();
    void readAddon(const Addon& addon);
    void transform();
};

} // namespace pdal

