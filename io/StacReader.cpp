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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
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
#include "StacReader.hpp"

#include <pdal/Kernel.hpp>
#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/SrsBounds.hpp>
#include <arbiter/arbiter.hpp>
#include <pdal/PipelineManager.hpp>

#include <pdal/util/ThreadPool.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/util/FileUtils.hpp>

#include "private/stac/Catalog.hpp"
#include "private/stac/Utils.hpp"
// #include "private/stac/ItemCollection.hpp"

#include "private/connector/Connector.hpp"
#include <pdal/StageWrapper.hpp>


namespace pdal
{


struct StacReader::Private
{
public:
    std::unique_ptr<Args> m_args;
    std::unique_ptr<ThreadPool> m_pool;
    std::vector<Reader*> m_readerList;
    mutable std::mutex m_mutex;
    std::vector<std::string> m_idList;
    std::unique_ptr<connector::Connector> m_connector;
    std::deque <std::pair<std::string, std::string>> m_errors;
    stac::Item::Filters m_itemFilters;
    stac::Catalog::Filters m_catFilters;
};

struct StacReader::Args
{

    std::vector<RegEx> item_ids;
    std::vector<RegEx> catalog_ids;
    NL::json properties;
    NL::json readerArgs;
    NL::json rawReaderArgs;
    NL::json::array_t dates;
    SrsBounds bounds;
    std::vector<std::string> assetNames;
    std::string catalogSchemaUrl;
    std::string featureSchemaUrl;
    bool validateSchema;
    int threads;

    NL::json m_query;
    NL::json m_headers;
};

static PluginInfo const stacinfo
{
    "readers.stac",
    "STAC Reader",
    "http://pdal.io/stages/readers.stac.html"
};

CREATE_STATIC_STAGE(StacReader, stacinfo)

std::string StacReader::getName() const { return stacinfo.name; }

StacReader::StacReader():
    m_args(new StacReader::Args),
    m_p(new StacReader::Private)
{};

StacReader::~StacReader(){};

void StacReader::addArgs(ProgramArgs& args)
{
    m_args.reset(new StacReader::Args());

    args.add("asset_names", "List of asset names to look for in data consumption. Default: 'data'", m_args->assetNames, {"data"});
    args.add("date_ranges", "Date ranges to include in your search. "
        "Eg. dates'[[\"min1\",\"max1\"],...]'", m_args->dates);
    args.add("bounds", "Bounding box to select stac items by. This will "
        "propogate down through all readers being used.", m_args->bounds);
    args.add("item_ids", "List of ID regexes to select STAC items based on.", m_args->item_ids);
    args.add("catalog_ids", "List of ID regexes to select STAC items based on.", m_args->catalog_ids);
    args.add("validate_schema", "Use JSON schema to validate your STAC objects. Default: false", m_args->validateSchema, false);
    args.add("header", "Header fields to forward with HTTP requests", m_args->m_headers);
    args.add("query", "Query parameters to forward with HTTP requests", m_args->m_query);
    args.add("properties", "Map of STAC property names to regular expression "
        "values. ie. {\"pc:type\": \"(lidar|sonar)\"}. Selected items will "
        "match all properties.", m_args->properties);
    args.add("reader_args", "Map of reader arguments to their values to pass through.",
        m_args->rawReaderArgs);
    args.add("catalog_schema_url", "URL of catalog schema you'd like to use for"
        " JSON schema validation.", m_args->catalogSchemaUrl,
        "https://schemas.stacspec.org/v1.0.0/catalog-spec/json-schema/catalog.json");
    args.add("feature_schema_url", "URL of feature schema you'd like to use for"
        " JSON schema validation.", m_args->featureSchemaUrl,
        "https://schemas.stacspec.org/v1.0.0/item-spec/json-schema/item.json");
    args.add("requests", "Number of threads for fetching JSON files, Default: 8",
        m_args->threads, 8);
    args.addSynonym("requests", "threads");


}


void StacReader::addItem(stac::Item& item)
{
    std::string driver = item.driver();

    if (m_args->validateSchema)
        item.validate();

    Stage *stage = m_factory.createStage(driver);

    if (!stage)
    {
        std::stringstream msg;
        msg << "Unable to create driver '" << driver << "' for "
            << "for asset located at '" << item.assetPath() <<"'";
        throw pdal_error(msg.str());
    }
    Reader* reader = dynamic_cast<Reader*>(stage);
    reader->setOptions(item.options());

    std::lock_guard<std::mutex> lock(m_p->m_mutex);
    m_p->m_idList.push_back(item.id());
    reader->setLog(log());
    m_merge.setInput(*reader);
    m_p->m_readerList.push_back(reader);

}

void StacReader::handleItem(NL::json stacJson, std::string itemPath)
{
    stac::Item item(stacJson, m_filename, *m_p->m_connector);
    if (item.init(m_p->m_itemFilters, m_args->rawReaderArgs))
        addItem(item);
}


void StacReader::handleCatalog(NL::json stacJson, std::string catPath, bool isRoot)
{
    stac::Catalog catalog(stacJson, catPath, *m_p->m_connector, *m_p->m_pool, log());

    if (m_args->validateSchema)
        catalog.validate();

    catalog.init(m_p->m_catFilters, m_args->rawReaderArgs, true);
    // m_p->m_pool->await();
    // m_p->m_pool->join();

    std::vector<stac::Item> items = catalog.items();
    for (stac::Item& item: items)
    {
        addItem(item);
    }
}

void StacReader::handleItemCollection(NL::json stacJson, std::string icPath)
{
    if (!stacJson.contains("features"))
        throw pdal_error("Missing required key 'features' in FeatureCollection.");

    NL::json itemList = stacJson.at("features");
    for (NL::json& item: itemList)
    {
        handleItem(item, icPath);
    }
    if (stacJson.contains("links"))
    {
        NL::json links = stacJson.at("links");
        for (NL::json& link: links)
        {
            if (!link.contains("rel"))
                throw pdal_error("Missing required key 'rel' in STAC Link object.");
            std::string target = link.at("rel").get<std::string>();
            if (target == "next")
            {
                const std::string nextLinkPath = link.at("href").get<std::string>();
                std::string nextAbsPath = stac::handleRelativePath(icPath, nextLinkPath);
                NL::json nextJson = m_p->m_connector->getJson(nextAbsPath);
                handleItemCollection(nextJson, nextAbsPath);
            }
        }
    }
}

void StacReader::initializeArgs()
{
    if (!m_args->item_ids.empty())
    {
        log()->get(LogLevel::Debug) << "Selecting Items with ids: " << std::endl;
        for (auto& id: m_args->item_ids)
            log()->get(LogLevel::Debug) << "    " << id.m_str << std::endl;
        m_p->m_itemFilters.ids = m_args->item_ids;
    }

    if (!m_args->catalog_ids.empty())
    {
        log()->get(LogLevel::Debug) << "Selecting Catalogs with ids: " << std::endl;
        for (auto& id: m_args->catalog_ids)
            log()->get(LogLevel::Debug) << "    " << id.m_str << std::endl;
        m_p->m_catFilters.ids = m_args->catalog_ids;
    }

    if (!m_args->dates.empty())
    {
        //TODO validate supplied dates?
        log()->get(LogLevel::Debug) << "Dates selected: " << m_args->dates  << std::endl;
        m_p->m_itemFilters.dates = m_args->dates;
    }

    if (!m_args->properties.empty())
    {
        if (!m_args->properties.is_object())
            throw pdal_error("Properties argument must be a valid JSON object.");
        log()->get(LogLevel::Debug) << "Property Pruning: " <<
            m_args->properties.dump() << std::endl;
        m_p->m_itemFilters.properties = m_args->properties;
    }

    if (!m_args->bounds.empty())
    {
        if (!m_args->bounds.valid())
            throw pdal_error("Supplied bounds are not valid.");
        log()->get(LogLevel::Debug) << "Bounds: " << m_args->bounds << std::endl;
        m_p->m_itemFilters.bounds = m_args->bounds;
    }

    if (!m_args->assetNames.empty())
    {
        log()->get(LogLevel::Debug) << "STAC Reader will look in these asset keys: ";
        for (auto& name: m_args->assetNames)
            log()->get(LogLevel::Debug) << name << std::endl;
        m_p->m_itemFilters.assetNames = m_args->assetNames;
    }

    if (m_args->validateSchema)
        log()->get(LogLevel::Debug) <<
            "JSON Schema validation flag is set." << std::endl;
    m_p->m_catFilters.itemFilters = m_p->m_itemFilters;

}

void StacReader::setConnectionForwards(StringMap& headers, StringMap& query)
{
    try
    {
        if (!m_args->m_headers.is_null())
            headers = m_args->m_headers.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'headers': ") + err.what());
    }

    try
    {
        if (!m_args->m_query.is_null())
            query = m_args->m_query.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'query': ") + err.what());
    }
}

void StacReader::initialize()
{

    StringMap headers;
    StringMap query;
    setConnectionForwards(headers, query);
    m_p->m_connector.reset(new connector::Connector(headers, query));

    m_p->m_pool.reset(new ThreadPool(m_args->threads));

    initializeArgs();

    NL::json stacJson = m_p->m_connector->getJson(m_filename);

    std::string stacType = stacJson.at("type");
    if (stacType == "Feature")
        handleItem(stacJson, m_filename);
    else if (stacType == "Catalog")
        handleCatalog(stacJson, m_filename, true);
    else if (stacType == "FeatureCollection")
        handleItemCollection(stacJson, m_filename);
    else
        throw pdal_error("Could not initialize STAC object of type " + stacType);

    m_p->m_pool->await();
    m_p->m_pool->stop();

    if (m_p->m_readerList.empty())
        throw pdal_error("Reader list is empty after filtering.");

    setInput(m_merge);
}

QuickInfo StacReader::inspect()
{
    QuickInfo qi;

    initialize();

    for (auto& reader: m_p->m_readerList)
    {
        QuickInfo readerQi = reader->preview();
        qi.m_bounds.grow(readerQi.m_bounds);
        qi.m_pointCount += readerQi.m_pointCount;

        for (auto& readerDim: readerQi.m_dimNames)
        {
            bool exists = false;
            for (auto& dim: qi.m_dimNames)
                if (dim == readerDim)
                    exists = true;
            if (!exists)
                qi.m_dimNames.push_back(readerDim);
        }
    }

    //TODO Make list of catalog ids and item ids to pass to metadata
    NL::json metadata;
    metadata["ids"] = NL::json::array();
    for (auto& id: m_p->m_idList)
        metadata["ids"].push_back(id);


    if (metadata.contains("ids"))
    {
        std::string metadataStr = metadata["ids"].dump();
        qi.m_metadata.addWithType("stac_ids", metadataStr, "json", "STAC Reader ID List");
    }

    qi.m_valid = true;
    return qi;
}


point_count_t StacReader::read(PointViewPtr view, point_count_t num)
{
    point_count_t cnt(0);

    PointRef point(view->point(0));
    for (PointId idx = 0; idx < num; ++idx)
    {
        point.setPointId(idx);
        processOne(point);
        cnt++;
    }
    return cnt;
}


bool StacReader::processOne(PointRef& point)
{
    return true;
}


void StacReader::prepared(PointTableRef table)
{
    m_merge.prepare(table);
    m_merge.setLog(log());
}


void StacReader::ready(PointTableRef table)
{
    StageWrapper::ready(m_merge, table);
}


PointViewSet StacReader::run(PointViewPtr view)
{
    return StageWrapper::run(m_merge, view);
}


} //namespace pdal
