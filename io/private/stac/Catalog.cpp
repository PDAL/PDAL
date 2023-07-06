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

#include "Catalog.hpp"
#include "Utils.hpp"
#include "Collection.hpp"

#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>

namespace pdal
{

namespace stac
{
    Catalog::Catalog(const NL::json& json,
            const std::string& catPath,
            const connector::Connector& connector,
            ThreadPool& pool,
            const LogPtr& logPtr,
            bool validate) :
        m_json(json), m_path(catPath), m_connector(connector),
        m_pool(pool), m_log(logPtr), m_validate(validate)
    {}

    Catalog::Catalog(Catalog &cat):
        m_json(cat.m_json), m_path(cat.m_path), m_connector(cat.m_connector),
        m_pool(cat.m_pool), m_log(cat.m_log), m_validate(cat.m_validate)
    {}

    Catalog::~Catalog()
    {}

    bool Catalog::init(Filters filters, NL::json rawReaderArgs,
            SchemaUrls schemaUrls, bool isRoot=false)
    {
        m_root = isRoot;
        if (!filter(filters))
            return false;

        m_schemaUrls = schemaUrls;
        if (m_validate)
            validate();


        std::string catalogId = m_json.at("id").get<std::string>();

        auto itemLinks = m_json.at("links");

        m_log->get(LogLevel::Debug) << "Filtering..." << std::endl;

        for (const auto& link: itemLinks)
        {

            if (!link.count("href") || !link.count("rel"))
                throw pdal::pdal_error("item does not contain 'href' or 'rel'");

            const std::string linkType = link.at("rel").get<std::string>();
            const std::string linkPath = link.at("href").get<std::string>();
            const std::string absLinkPath = handleRelativePath(m_path, linkPath);

            m_pool.add([this, linkType, absLinkPath, filters, rawReaderArgs]()
            {
                try
                {
                    if (linkType == "item")
                    {
                        NL::json itemJson = m_connector.getJson(absLinkPath);
                        Item item(itemJson, absLinkPath, m_connector, m_log,
                            m_validate);

                        bool valid = item.init(filters.itemFilters,
                            rawReaderArgs, m_schemaUrls);
                        if (valid)
                        {
                            std::lock_guard<std::mutex> lock(m_mutex);
                            m_itemList.push_back(item);
                        }
                    }
                    else if (linkType == "catalog")
                    {
                        NL::json catalogJson = m_connector.getJson(absLinkPath);
                        std::unique_ptr<Catalog> catalog(new Catalog(
                            catalogJson, absLinkPath, m_connector, m_pool,
                            m_log, m_validate));

                        bool valid = catalog->init(filters, rawReaderArgs,
                            m_schemaUrls);
                        if (valid)
                        {
                            std::lock_guard<std::mutex> lock(m_mutex);
                            m_subCatalogs.push_back(std::move(catalog));
                        }
                    }
                    else if (linkType == "collection")
                    {
                        NL::json collectionJson = m_connector.getJson(absLinkPath);
                        Collection* collection(new Collection(
                            collectionJson, absLinkPath, m_connector, m_pool,
                            m_log, m_validate));

                        bool valid = collection->init(filters, rawReaderArgs,
                            m_schemaUrls);
                        if (valid)
                        {
                            std::lock_guard<std::mutex> lock(m_mutex);
                            Catalog* cat = dynamic_cast<Catalog*>(collection);
                            if (cat) {
                                m_subCatalogs.push_back(
                                    std::unique_ptr<Catalog>(new Catalog(*cat))
                                );
                            }
                        }
                    }
                }
                catch (std::exception& e)
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    std::pair<std::string, std::string> p {absLinkPath, e.what()};
                    m_errors.push_back(p);
                }
                catch (...)
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_errors.push_back({absLinkPath, "Unknown error"});
                }
            });
        }

        if (m_errors.size())
        {
            for (auto& p: m_errors)
            {
                m_log->get(LogLevel::Error) << "Failure fetching '" << p.first
                    << "' with error '" << p.second << "'";
            }
        }

        if (isRoot)
        {
            m_pool.await();
            m_pool.join();
            handleNested();
        }
        return true;
    }

    // Wait for all nested catalogs to finish processing their items so they can
    // be added to the overarching itemlist
    void Catalog::handleNested()
    {
        for (auto& catalog: m_subCatalogs)
            for (auto& i: catalog->items())
                m_itemList.push_back(i);
    }

    std::vector<Item> Catalog::items()
    {
        return m_itemList;
    }

    void Catalog::validate()
    {
        std::function<void( const nlohmann::json_uri&, nlohmann::json&)> fetch = schemaFetch;

        nlohmann::json_schema::json_validator val(
            fetch,
            [](const std::string &, const std::string &) {}
        );

        NL::json schemaJson = m_connector.getJson(m_schemaUrls.catalog);
        val.set_root_schema(schemaJson);
        val.validate(m_json);
    }

    //if catalog matches filter requirements, return true
    bool Catalog::filter(Filters filters) {
        if (!m_json.contains("id"))
        {
            std::stringstream msg;
            msg << "Invalid catalog . It is missing key 'id'.";
            throw pdal_error(msg.str());
        }

        if (!filters.ids.empty() && !m_root)
        {
            std::string id = m_json.at("id").get<std::string>();
            bool pruneFlag = true;
            for (auto& i: filters.ids)
            {
                if (std::regex_match(id, i.regex()))
                {
                    pruneFlag = false;
                    break;
                }
            }
            if (pruneFlag)
                return false;
        }

        return true;
    }


}// stac

}// pdal