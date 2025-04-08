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
#include "Collection.hpp"

#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>

namespace pdal
{

namespace stac
{

using namespace StacUtils;

Catalog::Catalog(const nlohmann::json& json,
        const std::string& catPath,
        const connector::Connector& connector,
        ThreadPool& pool,
        bool validate,
        LogPtr log) :
    m_json(json), m_path(catPath), m_connector(connector),
    m_pool(pool), m_validate(validate), m_log(log)
{}

Catalog::~Catalog()
{}

bool Catalog::init(const Filters& filters, nlohmann::json rawReaderArgs,
        SchemaUrls schemaUrls, bool isRoot=false)
{
    m_root = isRoot;
    if (!filter(filters))
        return false;

    std::string type = stacValue<std::string>(m_json, "type");
    if (type == "Catalog")
    {
        m_log->get(LogLevel::Debug) << "Selected Catalog: " << id() << std::endl;
        m_type = GroupType::catalog;
    }
    if (type == "Collection")
    {
        m_log->get(LogLevel::Debug) << "Selected Collection: " << id() << std::endl;
        m_type = GroupType::collection;
    }

    m_schemaUrls = schemaUrls;
    if (m_validate)
        validate();

    nlohmann::json itemLinks = stacValue(m_json, "links");

    for (auto link: itemLinks)
    {
        m_pool.add([this, &filters, rawReaderArgs, link]()
        {
            const std::string linkType = stacValue<std::string>(
                link, "rel", m_json);
            const std::string linkPath = stacValue<std::string>(
                link, "href", m_json);
            const std::string absLinkPath = handleRelativePath(m_path, linkPath);
            try {
                if (linkType == "item")
                    handleItem(*filters.itemFilters, rawReaderArgs, absLinkPath);
                else if (linkType == "collection")
                    handleCol(filters, rawReaderArgs, absLinkPath);
                else if (linkType == "catalog")
                    handleCat(filters, rawReaderArgs, absLinkPath);
            }
            catch (std::exception& e)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                StacError p {absLinkPath, e.what()};
                m_errors.push_back(p);
            }
            catch (...)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                StacError p {absLinkPath, "Unknown error"};
                m_errors.push_back(p);
            }
        });
    }

    if (isRoot)
    {
        m_pool.await();
        m_pool.join();
        hoistNested();
        collectErrors();

        // if has no items exist after joining everything together, return false
        if (items().empty())
            return false;
    }

    return true;
}

//bring all nested catalogs and collections to the top layer for item extraction
void Catalog::hoistNested()
{
    for (auto& catalog: m_subCatalogs)
    {
        catalog->hoistNested();
        for (auto& cat: catalog->subs())
        {
            m_subCatalogs.push_back(std::move(cat));
        }
    }
}

void Catalog::collectErrors()
{
    for (auto& catalog: m_subCatalogs)
    {
        for (auto e: catalog->errors())
        {
            m_errors.push_back(e);
        }
    }
}

void Catalog::handleItem(const Item::Filters& f, nlohmann::json readerArgs, std::string path)
{
        nlohmann::json itemJson = m_connector.getJson(path);
        Item item(itemJson, path, m_connector, m_validate, m_log);

        bool valid = item.init(f, readerArgs, m_schemaUrls);
        if (valid)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_itemList.push_back(item);
        }
}

void Catalog::handleCol(const Filters& f, nlohmann::json readerArgs, std::string path)
{
    nlohmann::json collectionJson = m_connector.getJson(path);
    std::unique_ptr<Collection> collection(new Collection(
        collectionJson, path, m_connector, m_pool, m_validate, m_log));

    //init will return false if collection has no items or sub catalogs/collections
    bool passed = collection->init(f, readerArgs, m_schemaUrls);
    if (passed)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_subCatalogs.push_back(std::move(collection));
    }
}

void Catalog::handleCat(const Filters& f, nlohmann::json readerArgs, std::string path)
{
    nlohmann::json catalogJson = m_connector.getJson(path);
    std::unique_ptr<Catalog> catalog(new Catalog(
        catalogJson, path, m_connector, m_pool, m_validate, m_log));

    //init will return false if catalog has no items or sub catalogs/collections
    bool passed = catalog->init(f, readerArgs, m_schemaUrls);
    if (passed)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_subCatalogs.push_back(std::move(catalog));
    }
}

ItemList& Catalog::items()
{
    return m_itemList;
}

SubList& Catalog::subs()
{
    return m_subCatalogs;
}

ErrorList Catalog::errors()
{
    return m_errors;
}

GroupType Catalog::type()
{
    return m_type;
}

std::string Catalog::id()
{
    if (m_id.empty())
        m_id = stacId(m_json);
    return m_id;
}

void Catalog::validate()
{
    nlohmann::json_schema::json_validator val(
        [this](const nlohmann::json_uri& json_uri, nlohmann::json& json) {
            json = m_connector.getJson(json_uri.url());
        },
        [](const std::string &, const std::string &) {}
    );

    nlohmann::json schemaJson = m_connector.getJson(m_schemaUrls.catalog);
    val.set_root_schema(schemaJson);
    try {
        val.validate(m_json);
    }
    catch (std::exception& e)
    {
        throw stac_error(m_id, "catalog",
            "STAC schema validation Error in root schema: " +
            m_schemaUrls.catalog + ". \n\n" + e.what());
    }
}

//if catalog matches filter requirements, return true
bool Catalog::filter(Filters filters) {
    if (filters.ids.empty() || m_root)
        return true;

    m_id = stacId(m_json);
    for (auto& i: filters.ids)
        if (std::regex_match(m_id, i.regex()))
            return true;

    return false;
}


}// stac

}// pdal
