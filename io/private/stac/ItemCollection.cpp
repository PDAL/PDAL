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

#include "ItemCollection.hpp"
#include "Utils.hpp"

namespace pdal
{

namespace stac
{
    ItemCollection::ItemCollection(const NL::json& json,
            const std::string& icPath,
            const connector::Connector& connector,
            const LogPtr& log):
        m_json(json), m_path(icPath), m_connector(connector), m_log(log)
    {}


    ItemCollection::~ItemCollection()
    {}

    std::vector<Item> ItemCollection::items()
    {
        return m_itemList;
    }

    bool ItemCollection::init(Filters filters, NL::json rawReaderArgs)
    {
        if (!m_json.contains("features"))
            throw pdal_error("Missing required key 'features' in FeatureCollection.");

        NL::json itemList = m_json.at("features");
        for (NL::json& itemJson: itemList)
        {
            Item item(itemJson, m_path, m_connector, m_log);
            if (item.init(filters.itemFilters, rawReaderArgs))
            {
                m_itemList.push_back(item);
            }
        }
        if (m_json.contains("links"))
        {
            NL::json links = m_json.at("links");
            for (NL::json& link: links)
            {
                if (!link.contains("rel"))
                    throw pdal_error("Missing required key 'rel' in STAC Link object.");
                std::string target = link.at("rel").get<std::string>();
                if (target == "next")
                {
                    const std::string nextLinkPath = link.at("href").get<std::string>();
                    std::string nextAbsPath = stac::handleRelativePath(m_path, nextLinkPath);
                    NL::json nextJson = m_connector.getJson(nextAbsPath);

                    ItemCollection ic(nextJson, nextAbsPath, m_connector, m_log);

                    if (ic.init(filters, rawReaderArgs))
                        for (auto& item: ic.items())
                            m_itemList.push_back(item);
                }
            }
        }
        return true;
    }



}//stac
}//pdal