/******************************************************************************
 * Copyright (c) 2018, Connor Manning
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

#include <list>
#include <string>

#include <pdal/JsonFwd.hpp>
#include <pdal/PointLayout.hpp>

#include "Overlap.hpp"

namespace pdal
{

class Addon;
class Connector;
using AddonList = std::vector<Addon>;

class Addon
{
public:
    Addon(const std::string& dimName, const std::string& filename,
            Dimension::Type type) :
        m_name(dimName), m_filename(filename), m_type(type)
    { m_localId = m_layout.registerOrAssignDim(dimName, type); }

    std::string name() const
        { return m_name; }
    std::string filename() const
        { return m_filename; }
    Dimension::Type type() const
        { return m_type; }
    // Id for the local (internal) layout
    Dimension::Id localId() const
        { return m_localId; }
    // Id for the layout to which we'll copy data (ultimate PDAL ID).
    Dimension::Id externalId() const
        { return m_externalId; }
    void setExternalId(Dimension::Id externalId)
        { m_externalId = externalId; }
    Hierarchy& hierarchy()
        { return m_hierarchy; }
    PointLayout& layout() const
        { return const_cast<PointLayout &>(m_layout); }
    point_count_t points(const Key& key) const;
    std::string dataDir() const;
    std::string hierarchyDir() const;
    static AddonList store(const Connector& connector, const NL::json& spec,
        const PointLayout& layout);        
    static AddonList load(const Connector& connector, const NL::json& spec);

private:
    std::string m_name;
    std::string m_filename;
    Dimension::Type m_type;
    Dimension::Id m_localId;
    Dimension::Id m_externalId;

    Hierarchy m_hierarchy;
    PointLayout m_layout;

    static Addon loadAddon(const Connector& connector,
        const std::string& dimName, const std::string& filename);
};

} // namespace pdal

