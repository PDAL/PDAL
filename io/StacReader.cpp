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
#include <pdal/Polygon.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/private/SrsTransform.hpp>
#include <pdal/private/OGRSpec.hpp>
#include <pdal/Polygon.hpp>

#include <arbiter/arbiter.hpp>
#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>

#include <pdal/util/ThreadPool.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/util/FileUtils.hpp>


#include "private/stac/Collection.hpp"
#include "private/stac/ItemCollection.hpp"

#include "private/connector/Connector.hpp"
#include <pdal/StageWrapper.hpp>


namespace pdal
{

using namespace stac;

namespace
{

struct Args
{
    std::vector<RegEx> items;
    std::vector<RegEx> catalogs;
    std::vector<RegEx> collections;

    NL::json properties;
    NL::json readerArgs;
    NL::json rawReaderArgs;

    NL::json::array_t dates;
    SrsBounds bounds;
    OGRSpec ogr;
    std::vector<std::string> assetNames;

    SchemaUrls schemaUrls;

    bool validateSchema;
    int threads;
};

} // unnamed namespace

struct StacReader::Private
{
public:
    Private() : m_args(std::make_unique<Args>())
    {}

    std::unique_ptr<Args> m_args;
    StageFactory m_factory;
    MergeFilter m_merge;
    LogPtr m_log;

    //processing
    std::unique_ptr<ThreadPool> m_pool;
    std::vector<Reader*> m_readerList;
    std::unique_ptr<connector::Connector> m_connector;
    mutable std::mutex m_mutex;

    //book keeping
    std::vector<std::string> m_itemList;
    std::vector<std::string> m_catList;
    std::vector<std::string> m_colList;
    std::deque <std::pair<std::string, std::string>> m_errors;

    //filters
    std::unique_ptr<Item::Filters> m_itemFilters;
    std::unique_ptr<Catalog::Filters> m_catFilters;
    std::unique_ptr<Collection::Filters> m_colFilters;
    std::unique_ptr<ItemCollection::Filters> m_icFilters;

    void addItem(stac::Item& item);
    void handleNested(stac::Catalog& c);
    void handleItem(NL::json stacJson, std::string filename);
    void handleCatalog(NL::json stacJson, std::string catPath);
    void handleCollection(NL::json stacJson, std::string catPath);
    void handleItemCollection(NL::json stacJson, std::string icPath);
    void initializeArgs();
    void printErrors(stac::Catalog& c);

    void setLog(LogPtr log)
    { m_log = log; }

    LogPtr log() const
    { return m_log; }
};

static PluginInfo const stacinfo
{
    "readers.stac",
    "STAC Reader",
    "http://pdal.io/stages/readers.stac.html",
};

namespace
{

} // unnamed namespace


CREATE_STATIC_STAGE(StacReader, stacinfo)

std::string StacReader::getName() const { return stacinfo.name; }

StacReader::StacReader():
    m_p(new StacReader::Private)
{};

StacReader::~StacReader(){};

void StacReader::addArgs(ProgramArgs& args)
{
    //Filter options
    args.add("asset_names", "List of asset names to look for in data"
        " consumption. Default: 'data'", m_p->m_args->assetNames, {"data"});
    args.add("date_ranges", "Date ranges to include in your search. Dates are"
        "formatted according to RFC 3339. Eg. dates'[[\"min1\",\"max1\"],...]'"
        , m_p->m_args->dates);
    args.add("bounds", "Bounding box to select stac items by. This will "
        "propogate down through all readers being used.", m_p->m_args->bounds);
    args.add("ogr", "OGR filter geometries to select stac items by.",
        m_p->m_args->ogr);
    args.add("properties", "Map of STAC property names to regular expression "
        "values. ie. {\"pc:type\": \"(lidar|sonar)\"}. Selected items will "
        "match all properties.", m_p->m_args->properties);
    args.add("items", "List of Item ID regexes to select STAC items based on.",
        m_p->m_args->items);
    args.add("catalogs", "List of Catalog ID regexes to select STAC items "
        "based on.", m_p->m_args->catalogs);
    args.add("collections", "List of Collection ID regexes to select STAC "
        "items based on.", m_p->m_args->collections);
    args.addSynonym("items", "item_ids");
    args.addSynonym("catalogs", "catalog_ids");
    args.addSynonym("collections", "collection_ids");

    //Reader options
    args.add("validate_schema", "Use JSON schema to validate your STAC objects."
        " Default: false", m_p->m_args->validateSchema, false);
    args.add("reader_args", "Map of reader arguments to their values to pass"
        " through.", m_p->m_args->rawReaderArgs);
    args.add("requests", "Number of threads for fetching JSON files, Default: 8",
        m_p->m_args->threads, 8);
    args.addSynonym("requests", "threads");

    //Schema options
    args.add("catalog_schema_url", "URL of catalog schema you'd like to use for"
        " JSON schema validation.", m_p->m_args->schemaUrls.catalog,
        "https://schemas.stacspec.org/v1.0.0/catalog-spec/json-schema/catalog.json");
    args.add("collection_schema_url", "URL of collection schema you'd like to use for"
        " JSON schema validation.", m_p->m_args->schemaUrls.collection,
        "https://schemas.stacspec.org/v1.0.0/collection-spec/json-schema/collection.json");
    args.add("feature_schema_url", "URL of feature schema you'd like to use for"
        " JSON schema validation.", m_p->m_args->schemaUrls.item,
        "https://schemas.stacspec.org/v1.0.0/item-spec/json-schema/item.json");
}


void StacReader::Private::addItem(Item& item)
{
    std::string driver = item.driver();

    Stage *stage = m_factory.createStage(driver);

    if (!stage)
    {
        std::stringstream msg;
        msg << "Unable to create driver '" << driver << "' for "
            << "for asset located at '" << item.assetPath() <<"'";
        throw pdal_error(msg.str());
    }
    Reader* reader = dynamic_cast<Reader*>(stage);
    if (!reader)
    {
        std::stringstream msg;
        msg << "Unable to cast stage of type '" << driver << "' to a reader";
        throw pdal_error(msg.str());
    }
    reader->setOptions(item.options());

    std::lock_guard<std::mutex> lock(m_mutex);
    m_itemList.push_back(item.id());
    reader->setLog(log());
    m_merge.setInput(*reader);
    m_readerList.push_back(reader);
}

void StacReader::Private::handleItem(NL::json stacJson, std::string filename)
{
    Item item(stacJson, filename, *m_connector, m_args->validateSchema);

    log()->get(LogLevel::Debug) << "Found STAC Item " << item.id() << ".";
    if (item.init(*m_itemFilters, m_args->rawReaderArgs, m_args->schemaUrls))
        addItem(item);
}

void StacReader::Private::handleNested(Catalog& c)
{
    auto& subs = c.subs();
    for (auto& sub: subs)
    {
        //add sub col/cat ids to list for metadata bookkeeping
        if (sub->type() == GroupType::catalog)
            m_catList.push_back(sub->id());
        else if (sub->type() == GroupType::collection)
            m_colList.push_back(sub->id());

        //collect items from sub catalogs
        for (Item& item: sub->items())
            addItem(item);
    }
}


void StacReader::Private::handleCatalog(NL::json stacJson, std::string catPath)
{
    Catalog c(stacJson, catPath, *m_connector, *m_pool, m_args->validateSchema, log());

    // if init returns false, the collection has no items in itself or in
    // any sub-catalogs/collections.
    if (c.init(*m_catFilters, m_args->rawReaderArgs, m_args->schemaUrls, true))
    {
        m_catList.push_back(c.id());
        //iteracted
        handleNested(c);
        //collect items from root
        for (Item& item: c.items())
            addItem(item);
    }

    printErrors(c);
}

void StacReader::Private::handleCollection(NL::json stacJson, std::string colPath)
{
    Collection c(stacJson, colPath, *m_connector, *m_pool, m_args->validateSchema, log());

    // if init returns false, the collection has no items in itself or in
    // any sub-catalogs/collections.
    if (c.init(*m_colFilters, m_args->rawReaderArgs, m_args->schemaUrls, true))
    {
        m_colList.push_back(c.id());
        handleNested(c);
        //collect items from root
        for (Item& item: c.items())
            addItem(item);
    }

    printErrors(c);
}

void StacReader::Private::handleItemCollection(NL::json stacJson, std::string icPath)
{
    ItemCollection ic(stacJson, icPath, *m_connector, m_args->validateSchema);

    if (ic.init(*m_icFilters, m_args->rawReaderArgs, m_args->schemaUrls))
    {
        for (auto& item: ic.items())
            addItem(item);
    }
}

void StacReader::Private::printErrors(Catalog& c)
{
    ErrorList errors = c.errors();
    if (errors.size())
    {
        for (auto& p: errors)
        {
            log()->get(LogLevel::Error) << "Failure fetching '" << p.first
                << "' with error '" << p.second << "'\n";
        }
        throw pdal_error("Errors found during initial processing of the Catalog.");
    }
}

std::string listStr(std::string key, std::vector<RegEx> ids)
{
    std::stringstream s;
    s << key << ": [";
    for (auto& id: ids)
        s << id.m_str << ", ";
    s.seekp(-2, std::ios_base::end);
    s << "]" << std::endl;
    return s.str();
}

void StacReader::Private::initializeArgs()
{
    m_itemFilters = std::make_unique<Item::Filters>();
    m_catFilters = std::make_unique<Catalog::Filters>();
    m_colFilters = std::make_unique<Catalog::Filters>();
    m_icFilters = std::make_unique<ItemCollection::Filters>();

    log()->get(LogLevel::Debug) << "Filters: " << std::endl;
    if (!m_args->items.empty())
    {
        std::string itIdStr = listStr("Item Ids", m_args->items);
        log()->get(LogLevel::Debug) << itIdStr;

        m_itemFilters->ids = m_args->items;
    }

    if (!m_args->catalogs.empty())
    {
        std::string caIdStr = listStr("Catalog Ids", m_args->catalogs);
        log()->get(LogLevel::Debug) << caIdStr;

        m_catFilters->ids = m_args->catalogs;
    }

    if (!m_args->collections.empty())
    {
        std::string coIdStr = listStr("Collection Ids", m_args->collections);
        log()->get(LogLevel::Debug) << coIdStr;

        m_itemFilters->collections = m_args->collections;
        m_colFilters->ids = m_args->collections;
    }


    if (!m_args->dates.empty())
    {
        log()->get(LogLevel::Debug) <<
            "Dates: " << m_args->dates << std::endl;

        for (auto& datepair: m_args->dates)
        {
            if (datepair.size() != 2 ||
                datepair.type() != NL::detail::value_t::array)
            {
                throw pdal_error("User defined dates (" + datepair.dump() +
                    ") must be a range of [min, max].");
            }

            try
            {
                std::string minDate(datepair[0].get<std::string>());
                std::string maxDate(datepair[1].get<std::string>());

                std::time_t minTime = StacUtils::getStacTime(minDate);
                std::time_t maxTime = StacUtils::getStacTime(maxDate);
                if (minTime > maxTime)
                    log()->get(LogLevel::Warning) << "Min date (" << minDate <<
                        ") is greater than Max date (" << maxDate << ").";
                m_itemFilters->datePairs.push_back({ minTime, maxTime });
            }
            catch(NL::detail::type_error&)
            {
                throw pdal_error("User defined date range ("+ datepair.dump() +
                    ") is invalid. It must be of type string and comply " +
                    "with  RFC 3339.");
            }
        }
    }

    if (!m_args->properties.empty())
    {
        if (!m_args->properties.is_object())
            throw pdal_error("Properties argument must be a valid JSON object.");
        log()->get(LogLevel::Debug) << "Property Pruning: " <<
            m_args->properties.dump() << std::endl;
        m_itemFilters->properties = m_args->properties;
    }

    // There should be a check if both are specified
    if (!m_args->bounds.empty())
    {

        if (!m_args->bounds.valid())
            throw pdal_error("Supplied bounds are not valid.");
        log()->get(LogLevel::Debug) << "Bounds: " << m_args->bounds << std::endl;

        Polygon boundsPoly(m_args->bounds.to2d());
        if (!m_args->bounds.spatialReference().empty())
            boundsPoly.setSpatialReference(m_args->bounds.spatialReference());
        m_itemFilters->bounds = boundsPoly;
    }
    else if (m_args->ogr.size())
        m_itemFilters->bounds = m_args->ogr.getPolygons()[0];

    if (!m_args->assetNames.empty())
    {
        std::stringstream s;
        s << "Asset Keys: [";
        for (auto& name: m_args->assetNames)
            s << name << ", ";
        s.seekp(-2, std::ios_base::end);
        s << "]" << std::endl;
        auto it = m_itemFilters->assetNames.begin();
        log()->get(LogLevel::Debug) << s.str();
        m_itemFilters->assetNames.insert(it, m_args->assetNames.begin(), m_args->assetNames.end());
    }

    if (m_args->validateSchema)
        log()->get(LogLevel::Debug) <<
            "JSON Schema validation flag is set." << std::endl;

    m_colFilters->itemFilters = m_itemFilters.get();
    m_catFilters->itemFilters = m_itemFilters.get();
    m_catFilters->colFilters = m_colFilters.get();
    m_icFilters->itemFilters = m_itemFilters.get();
}

void StacReader::addDimensions(PointLayoutPtr layout)
{
    StageWrapper::addDimensions(m_p->m_merge, layout);
}

void StacReader::initialize()
{
    m_p->m_connector.reset(new connector::Connector(m_filespec));

    m_p->m_pool.reset(new ThreadPool(m_p->m_args->threads));
    m_p->setLog(log());
    m_p->initializeArgs();

    NL::json stacJson = m_p->m_connector->getJson(m_filename);

    std::string stacType = Utils::jsonValue<std::string>(stacJson, "type");
    if (stacType == "Feature")
        m_p->handleItem(stacJson, m_filename);
    else if (stacType == "Catalog")
        m_p->handleCatalog(stacJson, m_filename);
    else if (stacType == "Collection")
        m_p->handleCollection(stacJson, m_filename);
    else if (stacType == "FeatureCollection")
        m_p->handleItemCollection(stacJson, m_filename);
    else
        throw pdal_error("Could not initialize STAC object of type " + stacType);

    m_p->m_pool->await();
    m_p->m_pool->stop();

    if (m_p->m_readerList.empty())
        throwError("Reader list is empty after filtering.");

    setInput(m_p->m_merge);
    StageWrapper::initialize(m_p->m_merge);
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

    for (auto& id: m_p->m_catList)
        m_metadata.addList("catalog_ids", id);

    for (auto& id: m_p->m_colList)
        m_metadata.addList("collection_ids", id);

    for (auto& id: m_p->m_itemList)
    {
        m_metadata.addList("ids", id);
        m_metadata.addList("item_ids", id);
    }

    qi.m_metadata = m_metadata;

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
    m_p->m_merge.prepare(table);
    m_p->m_merge.setLog(log());
}


void StacReader::ready(PointTableRef table)
{
    StageWrapper::ready(m_p->m_merge, table);
}


PointViewSet StacReader::run(PointViewPtr view)
{
    return StageWrapper::run(m_p->m_merge, view);
}

void StacReader::done(PointTableRef)
{
    m_p->m_pool->stop();
    m_p->m_connector.reset();
}


} //namespace pdal
