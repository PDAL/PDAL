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

#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>

#include "Addon.hpp"
#include "Connector.hpp"

namespace pdal
{

point_count_t Addon::points(const Key& key) const
{
    auto it = m_hierarchy.find(key);
    return (it == m_hierarchy.end() ? 0 : it->m_count);
}

std::string Addon::dataDir() const
{
    return FileUtils::getDirectory(m_filename) + "ept-data/";
}

std::string Addon::hierarchyDir() const
{
    return FileUtils::getDirectory(m_filename) + "ept-hierarchy/";
}

AddonList Addon::store(const Connector& connector, const NL::json& spec,
    const PointLayout& layout)
{
    AddonList addons;
    std::string filename;
    try
    {
        for (auto it : spec.items())
        {
            std::string filename = it.key();
            std::string dimName = it.value().get<std::string>();
            if (!Utils::endsWith(filename, "ept-addon.json"))
                filename += "/ept-addon.json";

            Dimension::Id id = layout.findDim(dimName);
            if (id == Dimension::Id::Unknown)
                throw pdal_error("Invalid dimension '" + dimName + "' in "
                    "addon specification. Does not exist in source data.");
            Dimension::Type type = layout.dimType(id);
            std::string typestring = Dimension::toName(Dimension::base(type));
            size_t size = Dimension::size(type);

            Addon addon(dimName, filename, type);

            NL::json meta;
            meta["type"] = typestring;
            meta["size"] = size;
            meta["version"] = "1.0.0";
            meta["dataType"] = "binary";

            connector.makeDir(FileUtils::getDirectory(filename));
            connector.put(filename, meta.dump());

            addons.emplace_back(dimName, filename, type);
            addons.back().setExternalId(id);
        }
    }
    catch (NL::json::parse_error&)
    {
        throw pdal_error("Unable to parse EPT addon file '" + filename + "'.");
    }
    return addons;
}

AddonList Addon::load(const Connector& connector, const NL::json& spec)
{
    AddonList addons;
    std::string filename;
    try
    {
        for (auto it : spec.items())
        {
            std::string dimName = it.key();
            std::string filename = it.value().get<std::string>();
            if (!Utils::endsWith(filename, "ept-addon.json"))
                filename += "/ept-addon.json";

            addons.push_back(loadAddon(connector, dimName, filename));
        }
    }
    catch (NL::json::parse_error&)
    {
        throw pdal_error("Unable to parse EPT addon file '" + filename + "'.");
    }
    return addons;
}


Addon Addon::loadAddon(const Connector& connector,
    const std::string& dimName, const std::string& filename)
{
    NL::json info = connector.getJson(filename);
    std::string typestring = info["type"].get<std::string>();
    uint64_t size = info["size"].get<uint64_t>();
    Dimension::Type type = Dimension::type(typestring, size);

    return Addon(dimName, filename, type);
}

} // namespace pdal

