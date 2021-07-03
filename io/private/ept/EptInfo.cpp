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

#include "Connector.hpp"
#include "EptInfo.hpp"
#include "EptSupport.hpp"

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

EptInfo::EptInfo(const std::string& info)
{
    try
    {
        m_info = NL::json::parse(info);
    }
    catch(NL::json::parse_error&)
    {
        throw pdal_error("Unable to parse EPT info as JSON.");
    }
    initialize();
}

EptInfo::EptInfo(const std::string& filename, const Connector& connector) :
    m_filename(filename)
{
    if (Utils::startsWith(m_filename, "ept://"))
    {
        m_filename = m_filename.substr(6);
        if (!Utils::endsWith(m_filename, "/ept.json"))
            m_filename += "/ept.json";
    }
    m_info = connector.getJson(m_filename);
    initialize();
}

void EptInfo::initialize()
{
    m_bounds = toBox3d(m_info.at("bounds"));
    m_boundsConforming = toBox3d(m_info.at("boundsConforming"));
    m_points = m_info.value<uint64_t>("points", 0);
    m_span = m_info.at("span").get<uint64_t>();

    auto iSrs = m_info.find("srs");
    if (iSrs != m_info.end() && iSrs->size())
    {
        std::string wkt;
        auto iWkt = iSrs->find("wkt");
        auto iAuthority = iSrs->find("authority");
        auto iHorizontal = iSrs->find("horizontal");
        auto iVertical = iSrs->find("vertical");

        if (iWkt != iSrs->end())
        {
            if (!iWkt->is_string())
                throw pdal_error("srs.wkt must be specified as a string. "
                        "Found '" + iWkt->dump() + "'.");
            wkt = iWkt->get<std::string>();
        }
        else
        {
            if (iAuthority == iSrs->end() || iHorizontal == iSrs->end())
                throw pdal_error("srs must be defined with at least one of "
                        "wkt or both authority and horizontal specifications.");
            if (!iAuthority->is_string())
                throw pdal_error("srs.authority must be specified as a "
                        "string.  Found '" + iAuthority->dump() + "'.");
            wkt = iAuthority->get<std::string>();

            std::string horiz;
            if (iHorizontal->is_number_unsigned())
                horiz = std::to_string(iHorizontal->get<uint64_t>());
            else if (iHorizontal->is_string())
                horiz = iHorizontal->get<std::string>();
            else
                throw pdal_error("srs.horizontal must be specified as a "
                    "non-negative integer or equivalent string. "
                    "Found '" + iHorizontal->dump() + "'.");
            wkt += ":" + horiz;

            if (iVertical != iSrs->end())
            {
                std::string vert;
                if (iVertical->is_number_unsigned())
                    vert = std::to_string(iVertical->get<uint64_t>());
                else if (iVertical->is_string())
                    vert = iVertical->get<std::string>();
                else
                    throw pdal_error("srs.vertical must be specified as a "
                            "non-negative integer or equivalent string. "
                            "Found '" + iVertical->dump() + "'.");
                wkt += "+" + vert;
            }
        }
        m_srs.set(wkt);
        if (!m_srs.valid())
            throw pdal_error("Invalid/unknown srs.wkt specification.");
    }

    const std::string dt = m_info.at("dataType").get<std::string>();
    if (dt == "laszip")
        m_dataType = DataType::Laszip;
    else if (dt == "binary")
        m_dataType = DataType::Binary;
    else if (dt == "zstandard")
        m_dataType = DataType::Zstandard;
    else
        throw pdal_error("Unrecognized EPT dataType: " + dt);

    NL::json& schema = m_info["schema"];
    for (NL::json element: schema)
    {
        std::string name = element["name"].get<std::string>();
        std::string typestring = element["type"].get<std::string>();
        uint64_t size = element["size"].get<uint64_t>();
        Dimension::Type type = Dimension::type(typestring, size);
        double scale = element.value("scale", 1.0);
        double offset = element.value("offset", 0);

        name = Dimension::fixName(name);
        Dimension::Id id = m_remoteLayout.registerOrAssignFixedDim(name, type);
        m_dims[name] = DimType(id, type, scale, offset);
    }
    m_remoteLayout.finalize();
}

DimType EptInfo::dimType(Dimension::Id id) const
{
    //ABELL - This is yuck.  The map of strings to DimType is bad.
    for (auto it = m_dims.begin(); it != m_dims.end(); ++it)
        if (it->second.m_id == id)
            return it->second;
    return DimType();
}

std::string EptInfo::dataDir() const
{
    return FileUtils::getDirectory(m_filename) + "ept-data/";
}

std::string EptInfo::hierarchyDir() const
{
    return FileUtils::getDirectory(m_filename) + "ept-hierarchy/";
}

std::string EptInfo::sourcesDir() const
{
    return FileUtils::getDirectory(m_filename) + "ept-sources/";
}

} // namespace pdal

