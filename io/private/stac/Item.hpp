/******************************************************************************
 * Copyright (c) 2022, Kyle Mann (kyle@hobu.co)
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

#include <pdal/Stage.hpp>
#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/Log.hpp>

#include "Utils.hpp"
#include "../connector/Connector.hpp"

namespace pdal
{

namespace stac
{


struct SchemaUrls
{
    std::string catalog;
    std::string collection;
    std::string item;
};

class Item
{

public:
    Item(const NL::json& json,
        const std::string& itemPath,
        const connector::Connector& connector,
        bool validate);

    ~Item();
    Item(const Item& item);

    struct Filters {
        std::vector<RegEx> ids;
        Polygon bounds;
        SpatialReference srs;
        NL::json properties;
        DatePairs datePairs;
        std::vector<std::string> assetNames;
        std::vector<RegEx> collections;
    };

    bool init(const Filters& filters, NL::json rawReaderArgs, SchemaUrls schemaUrls);

    std::string id();
    std::string driver();
    Options options();
    std::string assetPath();

private:

    const NL::json m_json;
    const std::string m_path;

    const connector::Connector& m_connector;
    bool m_validate;

    StageFactory m_factory;
    std::string m_driver;
    SchemaUrls m_schemaUrls;
    Options m_readerOptions;
    std::string m_assetPath;
    std::string m_id;

    std::string extractDriverFromItem(const NL::json& asset) const;
    Options setReaderOptions(const NL::json& readerArgs, const std::string& driver,
        const std::string& filename) const;

    NL::json handleReaderArgs(NL::json rawReaderArgs);
    void validate();

    bool filter(const Filters& filters);
    bool filterAssets(std::vector<std::string> assetNames);
    bool filterIds(std::vector<RegEx> ids);
    bool filterCol(std::vector<RegEx> ids);
    bool filterDates(DatePairs dates);
    bool filterProperties(const NL::json& filterProps);
    bool filterBounds(Polygon bounds);
};


}
}
