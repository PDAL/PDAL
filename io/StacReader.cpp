/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include <pdal/util/ThreadPool.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/Kernel.hpp>
#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/Polygon.hpp>

namespace pdal
{

static PluginInfo const stacinfo
{
    "readers.stac",
    "STAC Reader",
    "http://pdal.io/stages/readers.stac.html"
};

CREATE_STATIC_STAGE(StacReader, stacinfo)

std::string StacReader::getName() const { return stacinfo.name; }

StacReader::StacReader(){};
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
    args.add("properties", "Map of STAC property names to regular expression "
        "values. ie. {\"pc:type\": \"(lidar|sonar)\"}. Selected items will "
        "match all properties.", m_args->properties);
    args.add("reader_args", "Map of reader arguments to their values to pass through.",
        m_args->readerArgs);
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


void schemaFetch(const nlohmann::json_uri& json_uri, nlohmann::json& json)
{
    std::unique_ptr<arbiter::Arbiter> fetcher;
    fetcher.reset(new arbiter::Arbiter());
    std::string jsonStr = fetcher->get(json_uri.url());
    json = nlohmann::json::parse(jsonStr);
}

void StacReader::validateSchema(NL::json stacJson)
{
    std::function<void(const nlohmann::json_uri&, nlohmann::json&)> fetch = schemaFetch;
    nlohmann::json_schema::json_validator val(
        fetch,
        [](const std::string &, const std::string &) {}
    );
    if (!stacJson.contains("type"))
        throw pdal_error("Invalid STAC json");
    std::string type = stacJson["type"].get<std::string>();
    std::string schemaUrl;

    if (type == "Feature")
    {
        schemaUrl = m_args->featureSchemaUrl;
        for (auto& extSchemaUrl: stacJson["stac_extensions"])
        {
            log()->get(LogLevel::Debug) << "Processing extension " << extSchemaUrl << std::endl;
            std::string schemaStr = m_arbiter->get(extSchemaUrl);
            NL::json schemaJson = NL::json::parse(schemaStr);
            val.set_root_schema(schemaJson);
            val.validate(stacJson);
        }
    }
    else if (type == "Catalog")
        schemaUrl = m_args->catalogSchemaUrl;
    else
        throw pdal_error("Invalid STAC type for PDAL consumption");

    std::string schemaStr = m_arbiter->get(schemaUrl);
    NL::json schemaJson = NL::json::parse(schemaStr);
    val.set_root_schema(schemaJson);
    val.validate(stacJson);
}

void StacReader::handleReaderArgs()
{
    for (NL::json& readerPipeline: m_args->readerArgs)
    {
        if (!readerPipeline.contains("type"))
            throw pdal_error("No \"type\" key found in supplied reader arguments.");

        std::string driver = readerPipeline["type"].get<std::string>();
        if (m_readerArgs.contains(driver))
            throw pdal_error("Multiple instances of the same driver in supplie reader arguments.");
        m_readerArgs[driver] = { };

        for (auto& arg: readerPipeline.items())
        {
            if (arg.key() == "type")
                continue;
            m_readerArgs[driver][arg.key()] = arg.value();
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
    }

    if (!m_args->catalog_ids.empty())
    {
        log()->get(LogLevel::Debug) << "Selecting Catalogs with ids: " << std::endl;
        for (auto& id: m_args->catalog_ids)
            log()->get(LogLevel::Debug) << "    " << id.m_str << std::endl;
    }

    if (!m_args->dates.empty())
    {
        //TODO validate supplied dates?
        log()->get(LogLevel::Debug) << "Dates selected: " << m_args->dates  << std::endl;
    }

    if (!m_args->properties.empty())
    {
        if (!m_args->properties.is_object())
            throw pdal_error("Properties argument must be a valid JSON object.");
        log()->get(LogLevel::Debug) << "Property Pruning: " <<
            m_args->properties.dump() << std::endl;
    }

    if (!m_args->bounds.empty())
    {
        if (!m_args->bounds.valid())
            throw pdal_error("Supplied bounds are not valid.");
        log()->get(LogLevel::Debug) << "Bounds: " << m_args->bounds << std::endl;
    }

    if (!m_args->readerArgs.empty())
    {
        if (m_args->readerArgs.is_object())
        {
            NL::json array_args = NL::json::array();
            array_args.push_back(m_args->readerArgs);
            m_args->readerArgs = array_args;
        }
        for (auto& opts: m_args->readerArgs)
            if (!opts.is_object())
                throw pdal_error("Reader Args must be a valid JSON object");

        log()->get(LogLevel::Debug) << "Reader Args: " << m_args->readerArgs.dump() << std::endl;
        handleReaderArgs();
    }

    if (!m_args->assetNames.empty())
    {
        log()->get(LogLevel::Debug) << "STAC Reader will look in these asset keys: ";
        for (auto& name: m_args->assetNames)
            log()->get(LogLevel::Debug) << name << std::endl;
    }

    if (m_args->validateSchema)
        log()->get(LogLevel::Debug) <<
            "JSON Schema validation flag is set." << std::endl;

}

void StacReader::initializeItem(NL::json stacJson)
{
    if (prune(stacJson))
        return;

    if (m_args->validateSchema)
        validateSchema(stacJson);

    bool assetExists = false;
    std::string assetName;
    for (auto& name: m_args->assetNames)
    {
        if (stacJson["assets"].contains(name))
        {
            assetName = name;
            assetExists = true;
        }
    }
    if (!assetExists)
        throw pdal_error("None of the asset names supplied exist in the STAC object.");

    std::string dataUrl = stacJson["assets"][assetName]["href"].get<std::string>();
    std::string driver = m_factory.inferReaderDriver(dataUrl);

    log()->get(LogLevel::Debug) << "Using driver " << driver <<
        " for file " << dataUrl << std::endl;

    Stage *reader = PluginManager<Stage>::createObject(driver);

    if (!reader)
        throwError("Unable to create reader for file '" + dataUrl + "'.");
    else
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        Options readerOptions;
        // add reader options defined in reader args to their respective readers
        if (m_readerArgs.contains(driver)) {
            NL::json args = m_readerArgs[driver].get<NL::json>();
            for (auto& arg : args.items()) {
                NL::detail::value_t type = m_readerArgs[driver][arg.key()].type();
                switch(type)
                {
                    case NL::detail::value_t::string:
                    {
                        std::string val = arg.value().get<std::string>();
                        readerOptions.add(arg.key(), arg.value().get<std::string>());
                        break;
                    }
                    case NL::detail::value_t::number_float:
                    {
                        readerOptions.add(arg.key(), arg.value().get<float>());
                        break;
                    }
                    case NL::detail::value_t::number_integer:
                    {
                        readerOptions.add(arg.key(), arg.value().get<int>());
                        break;
                    }
                    case NL::detail::value_t::boolean:
                    {
                        readerOptions.add(arg.key(), arg.value().get<bool>());
                        break;
                    }
                    default:
                    {
                        readerOptions.add(arg.key(), arg.value());
                        break;
                    }
                }
            }
        }

        readerOptions.add("filename", dataUrl);
        reader->setOptions(readerOptions);

        if (m_readerList.size() > 0)
            reader->setInput(*m_readerList.back());

        m_readerList.push_back(std::unique_ptr<Stage>(reader));
    }
}

void StacReader::initializeCatalog(NL::json stacJson, bool root = false)
{
    if (!stacJson.contains("id"))
        throw pdal_error("Invalid catalog. Missing key 'id'");
    std::string catalogId = stacJson.at("id").get<std::string>();

    if (root) {}
    else if (!m_args->catalog_ids.empty())
    {
        bool pruneFlag = true;
        for (auto& id: m_args->catalog_ids)
        {
            if (catalogId == id)
            {
                pruneFlag = false;
                break;
            }
        }
        if (pruneFlag)
            return;
    }


    if (m_args->validateSchema)
        validateSchema(stacJson);
    auto itemLinks = stacJson.at("links");
    log()->get(LogLevel::Debug) << "Filtering..." << std::endl;

    std::deque <std::pair<std::string, std::string>> errors;
    for (const auto& link: itemLinks)
    {
        if (!link.count("href") || !link.count("rel"))
        {
            // TODO: Remove debug.
            std::cout << "Failing " << link.dump() << std::endl;
            continue;
        }

        std::string linkType = link.at("rel").get<std::string>();
        std::string linkPath = link.at("href").get<std::string>();

        m_pool->add([this, link]()
        {
            try
            {
                if (linkType == "item")
                {
                    NL::json itemJson = NL::json::parse(m_arbiter->get(linkPath));
                    initializeItem(itemJson);
                }
                else if (linkType == "catalog")
                {
                    NL::json catalogJson = NL::json::parse(m_arbiter->get(linkPath));
                    initializeCatalog(catalogJson);
                }
            }
            catch (std::exception& e)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                std::pair<std::string, std::string> p {linkPath, e.what()};
                errors.push_back(p);
            }
            catch (...)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                errors.push_back({linkPath, "Unknown error"});
            }
        });
    }

    if (errors.size())
    {
        for (auto& p: errors)
        {
            log()->get(LogLevel::Error) << "Failure fetching '" << p.first << "' with error '"
                << p.second << "'";
        }
    }
}

void StacReader::initialize()
{
    m_pool.reset(new ThreadPool(m_args->threads));
    m_arbiter.reset(new arbiter::Arbiter());

    initializeArgs();

    std::string stacStr = m_arbiter->get(m_filename);
    NL::json stacJson = NL::json::parse(stacStr);

    if (!stacJson.contains("type"))
        throw pdal_error("Invalid STAC object provided.");

    std::string stacType = stacJson["type"];
    if (stacType == "Feature")
        initializeItem(stacJson);
    else if (stacType == "Catalog")
        initializeCatalog(stacJson, true);
    else
        throw pdal_error("Could not initialize STAC object of type " + stacType);

    m_pool->await();
    m_pool->stop();

    if (m_readerList.empty())
        throw pdal_error("Reader list is empty after filtering.");
}

// returns true if property matches
bool matchProperty(std::string key, NL::json val, NL::json properties, NL::detail::value_t type)
{
    switch (type)
    {
        case NL::detail::value_t::string:
        {
            std::string desired = val.get<std::string>();
            // std::regex desired(d);
            std::string value = properties[key].get<std::string>();
            if (value != desired)
                return false;
            // if (!std::regex_match(value, desired))
            //     return false;
            break;
        }
        case NL::detail::value_t::number_unsigned:
        {
            uint64_t value = properties[key].get<uint64_t>();
            uint64_t desired = val.get<uint64_t>();
            if (value != desired)
                return false;
            break;
        }
        case NL::detail::value_t::number_integer:
        {
            int value = properties[key].get<int>();
            int desired = val.get<int>();
            if (value != desired)
                return false;
            break;
        }
        case NL::detail::value_t::number_float:
        {
            int value = properties[key].get<double>();
            int desired = val.get<double>();
            if (value != desired)
                return false;
            break;
        }
        case NL::detail::value_t::boolean:
        {
            bool value = properties[key].get<bool>();
            bool desired = val.get<bool>();
            if (value != desired)
                return false;
            break;
        }
        default:
        {
            throw pdal_error("Data type of " + key + " is not supported for pruning.");
        }
    }
    return true;
}

void validateForPrune(NL::json stacJson)
{

    // TODO none of these error messages tell us
    // *which* itemId is invalid. Someone who is given
    // any of these errors is being told they're f'd right off and
    // given no information to do anything about it
    if (!stacJson.contains("id"))
        throw pdal_error("JSON object does not contain required key 'id'");

    if (!stacJson.contains("properties"))
        throw pdal_error("JSON object does not contain required key 'properties'");

    if (!stacJson.contains("geometry") || !stacJson.contains("bbox"))
        throw pdal_error("JSON object does not contain one of 'geometry' or 'bbox'");

    NL::json prop = stacJson["properties"];
    if (
        !prop.contains("datetime") &&
        (!prop.contains("start_datetime") && !prop.contains("end_datetime"))
    )
        throw pdal_error("JSON object properties value not contain required key"
            "'datetime' or 'start_datetime' and 'end_datetime'");

    // TODO validate the date ranges and other validation-type stuff
    // that's going on in `prune`
    //





}

// Returns true if item should be removed, false if it should stay
bool StacReader::prune(NL::json stacJson)
{
    validateForPrune(stacJson);

    // TODO compute a struct that tells you what kind of filtering
    // we're going to be doing. Do not make this part of any public StacReader API
    //
    // filters = whichFilters
    // filters.date, filters.bbox, filters.polygon, filters.date
    //
    // - write a filter function for each filter type that takes in a stacItem
    // - have this prune method switch on the filters and those methods, then
    //   there will only be one place to go to update or enhance this stuff
    //   and it will be easier to read in pieces


    // ID
    // If STAC ID matches *any* ID in supplied list, it will not be pruned.
    std::string itemId = stacJson["id"];
    bool idFlag = true;
    if (!m_args->item_ids.empty())
    {
        for (auto& id: m_args->item_ids)
        {
            if (std::regex_match(itemId, id.regex()))
            {
                idFlag = false;
            }
        }
    }
    else
        idFlag = false;

    if (idFlag)
        return true;

    NL::json properties = stacJson["properties"];

    // DateTime
    // If STAC datetime fits in *any* of the supplied ranges, it will not be pruned
    if (properties.contains("datetime"))
    {
        std::string stacDate = properties["datetime"];

        // TODO This would remove three lines of stuff and
        // be much clearer
        //
        // bool haveDateFlag = !m_args->dates.empty()
        bool dateFlag = true;
        if (m_args->dates.empty())
            dateFlag = false;
        for (auto& range: m_args->dates)
        {
            //If the extracted item date fits into any of the dates provided by
            //the user, then do not prune this item based on dates.
            //
            // TODO check that range.size() >= 2
            if (
                stacDate >= range[0].get<std::string>() &&
                stacDate <= range[1].get<std::string>()
            )
                dateFlag = false;
        }
        if (dateFlag)
            return true;
    } else if (properties.contains("start_datetime") && properties.contains("end_datetime"))
    {
        // Handle if STAC object has start and end datetimes instead of one
        std::string stacStartDate = properties["start_datetime"].get<std::string>();
        std::string stacEndDate = properties["end_datetime"].get<std::string>();

        bool dateFlag = true;
        for (const auto& range: m_args->dates)
        {
            // If any of the date ranges overlap with the date range of the STAC
            // object, do not prune.
            //
            // TODO check if range.size() == 2 before accessing this array
            if (range.size() != 2)
            {
                throwError("Invalid range")
            }
            std::string userMinDate = range[0].get<std::string>();
            std::string userMaxDate = range[1].get<std::string>();
            //
            // TODO should we really be comparing dates as strings?
            if (userMinDate >= stacStartDate && userMinDate <= stacEndDate)
            {
                dateFlag = false;
            }
            else if (userMaxDate >= stacStartDate && userMaxDate <= stacEndDate)
            {
                dateFlag = false;
            }
            else if (userMinDate <= stacStartDate && userMaxDate >= stacEndDate)
            {
                dateFlag = false;
            }
        }
        if (dateFlag)
            return true;

    }

    // Properties
    // If STAC properties match *all* the supplied properties, it will not be pruned
    //
    // TODO this reads backwards
    // if match:
    //     doPrune = true
    if (!m_args->properties.empty())
    {
        for (auto &it: m_args->properties.items())
        {
            if (!properties.contains(it.key()))
            {
                log()->get(LogLevel::Warning) << "STAC Item does not contain "
                    "property " << it.key() << ". Continuing." << std::endl;
                continue;
            }

            NL::detail::value_t type = properties[it.key()].type();
            NL::detail::value_t argType = it.value().type();
            //Array of possibilities are Or'd together
            if (argType == NL::detail::value_t::array)
            {
                bool arrFlag (true);
                for (auto& val: it.value())
                    if (matchProperty(it.key(), val, properties, type))
                        arrFlag = false;
                if (arrFlag)
                    return true;
            }
            else
                if (!matchProperty(it.key(), it.value(), properties, type))
                    return true;
        }
    }

    // bbox
    // If STAC bbox matches *any* of the supplied bounds, it will not be pruned

    // TODO do we allow passing in of `polygon` options like readers.copc and
    // readers.ept as well?
    if (!m_args->bounds.empty())
    {
        if (stacJson.contains("geometry"))
        {
            NL::json geometry = stacJson["geometry"].get<NL::json>();
            Polygon f(geometry.dump());
            if (!f.valid())
            {
                std::stringstream msg;
                msg << "Polygon created from STAC 'geometry' key for '"
                    << itemId << "' is invalid";
                throw pdal_error(msg.str());
            }

            // TODO if the bounds is 3d already, why
            // do we convert it to 3d on the next line?
            if (m_args->bounds.is3d())
            {
                if (!m_args->bounds.to3d().overlaps(f.bounds()))
                    return true;
            }
            else
            {
                // TODO this is confusing, but I guess that
                // is a result of PDAL's bounds interface.
                // ie, we downcast the bounds to2d, then make
                // a BOX3D from that and compare it to f.bounds()
                // which is 2d or 3d?
                BOX2D bbox = m_args->bounds.to2d();
                if (!BOX3D(bbox).overlaps(f.bounds()))
                    return true;
            }
        }

        // TODO make a function that does bbox filtering or find one
        // or make one in PDALUtils
        else if (stacJson.contains("bbox"))
        {
            NL::json bboxJson = stacJson["bbox"].get<NL::json>();

            // TODO if we have a bad bbox?
            if (bboxJson.size() != 4 || bboxJson.size() != 6)
                log()->get(LogLevel::Error) << "bbox for '" << itemId << "' is not valid";

            if (bboxJson.size() == 4)
            {
                double minx = bboxJson[0];
                double miny = bboxJson[1];
                double maxx = bboxJson[2];
                double maxy = bboxJson[3];
                BOX2D bbox = BOX2D(minx, miny, maxx, maxy);
                if (!m_args->bounds.to2d().overlaps(bbox))
                    return true;
            }
            else if (bboxJson.size() == 6)
            {
                double minx = bboxJson[0];
                double miny = bboxJson[1];
                double minz = bboxJson[2];
                double maxx = bboxJson[3];
                double maxy = bboxJson[4];
                double maxz = bboxJson[5];
                BOX3D bbox = BOX3D(minx, miny, minz, maxx, maxy, maxz);
                if (!m_args->bounds.to3d().overlaps(bbox))
                    return true;
            }
        }
    }

    log()->get(LogLevel::Debug) << "Including: " << itemId << std::endl;

    std::lock_guard<std::mutex> lock(m_mutex);
    m_idList.push_back(itemId);
    return false;
}

QuickInfo StacReader::inspect()
{
    QuickInfo qi;

    initialize();

    for (auto& reader: m_readerList)
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
    for (auto& id: m_idList)
        metadata["ids"].push_back(id);


    if (metadata.contains("ids"))
    {
        std::string metadataStr = metadata["ids"].dump();
        qi.m_metadata.addWithType("stac_ids", metadataStr, "json", "STAC Reader ID List");
    }

    qi.m_valid = true;
    return qi;
}

void StacReader::prepared(PointTableRef table)
{
    m_readerList.back()->prepare(table);
}

void StacReader::ready(PointTableRef table)
{
    m_pvSet = m_readerList.back()->execute(table);
}

void StacReader::done(PointTableRef)
{
    m_stream.reset();
}

PointViewSet StacReader::run(PointViewPtr view)
{
    return m_pvSet;
}


} //namespace pdal