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

namespace pdal
{

namespace stac
{

ItemCollection::ItemCollection(const NL::json& json,
        const std::string& icPath,
        const connector::Connector& connector,
        bool validate):
    m_json(json), m_path(icPath), m_connector(connector),
    m_validate(validate)
{}


ItemCollection::~ItemCollection()
{}

ItemList ItemCollection::items()
{
    return m_itemList;
}

bool ItemCollection::init(const Filters& filters, NL::json rawReaderArgs,
    SchemaUrls schemaUrls)
{
    const NL::json itemList = StacUtils::stacValue(m_json, "features");
    for (const NL::json& itemJson: itemList)
    {
        Item item(itemJson, m_path, m_connector, m_validate);
        if (item.init(*filters.itemFilters, rawReaderArgs, schemaUrls))
        {
            m_itemList.push_back(item);
        }
    }
    if (m_json.contains("links"))
    {
        const NL::json links = StacUtils::stacValue(m_json, "links");
        for (const NL::json& link: links)
        {
            std::string target = StacUtils::stacValue<std::string>(
                link, "rel", m_json);
            if (target == "next")
            {
                std::string nextLinkPath = StacUtils::stacValue<std::string>(
                    link, "href", m_json);
                std::string nextAbsPath =
                    StacUtils::handleRelativePath(m_path, nextLinkPath);
                NL::json nextJson = m_connector.getJson(nextAbsPath);

                ItemCollection ic(nextJson, nextAbsPath, m_connector,
                    m_validate);

                if (ic.init(filters, rawReaderArgs, schemaUrls))
                    for (auto& item: ic.items())
                        m_itemList.push_back(item);
            }
        }
    }
    return true;
}

}//stac

}//pdal