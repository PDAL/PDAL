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

#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/util/ThreadPool.hpp>
#include "../connector/Connector.hpp"
#include "Item.hpp"


namespace pdal
{

namespace stac
{

class Catalog
{

public:
    Catalog(const NL::json& json,
        const std::string& catPath,
        const connector::Connector& connector,
        ThreadPool& pool,
        const LogPtr& logPtr);
    ~Catalog();

    struct Filters {
        std::vector<RegEx> ids;
    };

    void init(std::vector<std::string> assetNames, NL::json rawReaderArgs);
    void validate();
    bool filter(Filters filters);

    std::vector<Item> items();
    // std::string assetPath();

private:

    const NL::json m_json;
    const std::string m_path;
    const connector::Connector& m_connector;
    std::mutex m_mutex;
    ThreadPool& m_pool;
    const LogPtr& m_log;

    std::deque<std::pair<std::string, std::string>> m_errors;

    std::vector<Item> m_itemList;
    std::string m_driver;
    std::string m_schemaUrl = "https://schemas.stacspec.org/v1.0.0/catalog-spec/json-schema/catalog.json";
    Options m_readerOptions;
    NL::json m_assets;
    std::string m_dataPath;

};

}
}