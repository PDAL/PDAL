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

#include "Item.hpp"

#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>

namespace pdal
{

namespace stac
{

using namespace StacUtils;

Item::Item(const nlohmann::json& json,
        const std::string& itemPath,
        const connector::Connector& connector,
        bool validate):
    m_json(json), m_path(itemPath), m_connector(connector),
    m_validate(validate)
{}


Item::~Item()
{}

Item::Item(const Item& item):
    m_json(item.m_json), m_path(item.m_path), m_connector(item.m_connector),
    m_driver(item.m_driver), m_schemaUrls(item.m_schemaUrls),
    m_readerOptions(item.m_readerOptions)
{}

bool Item::init(const Filters& filters, nlohmann::json rawReaderArgs,
        SchemaUrls schemaUrls)
{

    if (!filter(filters))
        return false;

    m_schemaUrls = schemaUrls;
    if (m_validate)
        validate();

    nlohmann::json readerArgs = handleReaderArgs(rawReaderArgs);
    m_readerOptions = setReaderOptions(readerArgs, m_driver);
    m_readerOptions.add("filename", m_assetPath);
    return true;
}

std::string Item::id()
{
    return stacId(m_json);
}

std::string Item::driver()
{
    return m_driver;
}

std::string Item::assetPath()
{
    return m_assetPath;
}

Options Item::options()
{
    return m_readerOptions;
}

nlohmann::json Item::handleReaderArgs(nlohmann::json rawReaderArgs)
{
    if (rawReaderArgs.is_object())
    {
        nlohmann::json array_args = nlohmann::json::array();
        array_args.push_back(rawReaderArgs);
        rawReaderArgs = array_args;
    }
    for (auto& opts: rawReaderArgs)
        if (!opts.is_object())
            throw pdal_error("Reader Args for reader '" + m_driver +
                "' must be a valid JSON object");

    nlohmann::json readerArgs;
    for (const nlohmann::json& readerPipeline: rawReaderArgs)
    {
        std::string driver =
            jsonValue<std::string>(readerPipeline, "type");
        if (rawReaderArgs.contains(driver))
            throw pdal_error("Multiple instances of the same driver in"
                " supplied reader arguments.");
        readerArgs[driver] = { };

        for (auto& arg: readerPipeline.items())
        {
            if (arg.key() == "type")
                continue;

            std::string key = arg.key();
            readerArgs[driver][key] = { };
            readerArgs[driver][key] = arg.value();
        }
    }
    return readerArgs;
}

Options Item::setReaderOptions(const nlohmann::json& readerArgs,
    const std::string& driver) const
{
    Options readerOptions;
    if (readerArgs.contains(driver)) {
        nlohmann::json args = jsonValue(readerArgs, driver);
        for (auto& arg : args.items())
        {
            std::string key = arg.key();
            nlohmann::json val = arg.value();
            nlohmann::detail::value_t type = val.type();

            // if value is of type string, dump() returns string with
            // escaped string inside and kills pdal program args
            std::string v;
            if (type == nlohmann::detail::value_t::string)
                v = jsonValue<std::string>(val);
            else
                v = arg.value().dump();
            readerOptions.add(key, v);
        }
    }

    return readerOptions;
}

std::string Item::extractDriverFromItem(const nlohmann::json& asset) const
{
    std::string output;

    std::map<std::string, std::string> contentTypes =
    {
        { "application/vnd.laszip+copc", "readers.copc"}
    };

    std::string assetPath = stacValue<std::string>(
        asset, "href", m_json);
    std::string dataUrl = handleRelativePath(m_path, assetPath);

    std::string contentType;

    if (asset.contains("type"))
    {
        contentType = stacValue<std::string>(asset, "type", m_json);
        for(const auto& ct: contentTypes)
            if (Utils::iequals(ct.first, contentType))
                return ct.second;
    }

    // Try to guess from the path
    std::string driver = m_factory.inferReaderDriver(dataUrl);
    if (driver.size())
        return driver;

    if (!FileUtils::fileExists(dataUrl))
    {
        // Use this to test if dataUrl is a valid endpoint
        // Try to make a HEAD request and get it from Content-Type
        try
        {
            StringMap headers = m_connector.headRequest(dataUrl);
            if (headers.find("Content-Type") != headers.end())
            {
                contentType = headers["Content-Type"];
                for(const auto& ct: contentTypes)
                    if (Utils::iequals(ct.first, contentType))
                        return ct.second;
            }
        }
        catch(std::exception& e)
        {
            throw stac_error(m_id, "item", "Failed to HEAD " + dataUrl +
                ". " + e.what());
        }
    }

    return output;
}

void Item::validate()
{

    nlohmann::json_schema::json_validator val(
        [this](const nlohmann::json_uri& json_uri, nlohmann::json& json) {
            json = m_connector.getJson(json_uri.url());
        },
        [](const std::string &, const std::string &) {}
    );

    // Validate against base Item schema first
    nlohmann::json schemaJson = m_connector.getJson(m_schemaUrls.item);
    val.set_root_schema(schemaJson);
    try {
        val.validate(m_json);
    }
    catch (std::exception &e)
    {
        throw stac_error(m_id, "item",
            "STAC schema validation Error in root schema: " +
            m_schemaUrls.item + ". \n\n" + e.what());
    }

    // Validate against stac extensions if present
    if (m_json.contains("stac_extensions"))
    {
        nlohmann::json extensions = stacValue(m_json, "stac_extensions");
        for (auto& extSchemaUrl: extensions)
        {
            std::string url = stacValue<std::string>(extSchemaUrl, "", m_json);

            try {
                nlohmann::json schemaJson = m_connector.getJson(url);
                val.set_root_schema(schemaJson);
                val.validate(m_json);
            }
            catch (std::exception& e) {
                std::string msg  =
                    "STAC Validation Error in extension: " + url +
                    ". Errors found: \n" + e.what();
                throw stac_error(m_id, "item", msg);

            }
        }
    }
}

void validateForFilter(nlohmann::json json)
{
    stacId(json);
    stacValue(json, "assets");
    stacValue(json, "properties");
    stacValue(json, "geometry");
}

bool matchProperty(std::string key, nlohmann::json val, nlohmann::json properties,
    nlohmann::detail::value_t type)
{
    switch (type)
    {
        case nlohmann::detail::value_t::string:
        {
            std::string desired = jsonValue<std::string>(val);
            std::string value = jsonValue<std::string>(properties, key);
            return value == desired;
            break;
        }
        case nlohmann::detail::value_t::number_unsigned:
        {
            uint64_t value = jsonValue<uint64_t>(properties, key);
            uint64_t desired = jsonValue<uint64_t>(val);
            return value == desired;
            break;
        }
        case nlohmann::detail::value_t::number_integer:
        {
            int value = jsonValue<int>(properties,key);
            int desired = jsonValue<int>(val);
            return value == desired;
            break;
        }
        case nlohmann::detail::value_t::number_float:
        {
            double value = jsonValue<double>(properties, key);
            double desired = jsonValue<double>(val);
            return value == desired;
            break;
        }
        case nlohmann::detail::value_t::boolean:
        {
            bool value = jsonValue<bool>(properties, key);
            bool desired = jsonValue<bool>(val);
            return value == desired;
            break;
        }
        default:
        {
            throw pdal_error("Data type of " + key +
                " is not supported for filtering.");
        }
    }
    return true;
}



bool Item::filter(const Filters& filters)
{
    validateForFilter(m_json);
    m_id = stacId(m_json);

    if (!filterAssets(filters.assetNames))
        return false;

    if (!filterIds(filters.ids))
        return false;

    if (!filterCol(filters.collections))
        return false;

    if (!filterDates(filters.datePairs))
        return false;

    if (!filterProperties(filters.properties))
        return false;

    if (!filterBounds(filters.bounds))
        return false;


    return true;
}


SpatialReference extractSRS(nlohmann::json& props)
{

    bool havePROJJSON = props.contains("proj:projjson");
    bool haveWKT2 = props.contains("proj:wkt2");
    bool haveWKT = props.contains("proj:wkt");
    bool haveEPSG = props.contains("proj:epsg");

    SpatialReference srs;
    if (havePROJJSON)
    {
        nlohmann::json projjson = jsonValue(props, "proj:projjson");
        srs.set(projjson.dump());
    } else if (haveWKT2)
    {
        std::string wkt = jsonValue(props, "proj:wkt2");
        srs.set(wkt);
    } else if (haveEPSG)
    {
        int projepsg = jsonValue(props, "proj:epsg");
        srs.set("EPSG:" + std::to_string(projepsg));
    } else if (haveWKT)
    {
        std::string wkt = jsonValue(props, "proj:wkt");
        srs.set(wkt);
    }

    return srs;
}

bool Item::filterBounds(Polygon bounds)
{
    // Polygons are always "valid" even if they're default constructed.
    // Same with bool(Polygon). maybe change this behavior
    if (!bounds.valid())
        throw pdal_error("User input polygon is invalid");
    if (!bounds.area())
        return true;

    //If stac item has null geometry and bounds have been included
    //for filtering, then the Item will be excluded.
    nlohmann::json geometry = stacValue(m_json, "geometry");
    if (geometry.type() == nlohmann::detail::value_t::null)
        return false;

    //STAC's base geometries will always be represented in 4326.
    const SpatialReference stacSrs("EPSG:4326");
    Polygon stacPolygon(geometry.dump(), stacSrs);
    if (!stacPolygon.valid())
        throw stac_error(m_id, "item",
            "Polygon created from STAC 'geometry' key is invalid");

    // If the user didn't provide an SRS via option, we can only filter
    // assuming it is 4326.

    if (!bounds.srsValid())
    {
        bounds.setSpatialReference(stacSrs);
    }

    // If the SRSs don't match, we'll project the STAC Item's boundary
    // to the same as the SRS given by the bounds and test accordingly
    if (bounds.getSpatialReference() != stacPolygon.getSpatialReference())
    {
        nlohmann::json props = stacValue(m_json, "properties");
        SpatialReference ref = extractSRS(props);
        stacPolygon.transform(ref);
    }

    // Could change the option to multiple polygons and loop thru them here
    if (!stacPolygon.disjoint(bounds))
        return true;

    return false;

}

bool Item::filterProperties(const nlohmann::json& filterProps)
{
    nlohmann::json itemProperties = stacValue(m_json, "properties");
    if (!filterProps.empty())
    {
        for (auto &it: filterProps.items())
        {
            std::string key = it.key();
            nlohmann::json stacVal = stacValue(itemProperties, key, m_json);
            nlohmann::detail::value_t stacType = stacVal.type();

            nlohmann::json filterVal = it.value();
            nlohmann::detail::value_t filterType = filterVal.type();

            //Array of possibilities are Or'd together
            if (filterType == nlohmann::detail::value_t::array)
            {
                bool arrFlag (true);
                for (auto& val: filterVal)
                    if (matchProperty(key, val, itemProperties, stacType))
                        return true;
            }
            else
                if (matchProperty(key, filterVal, itemProperties, stacType))
                    return true;
        }

        return false;
    }
    return true;
}

bool Item::filterDates(DatePairs dates)
{
    nlohmann::json properties = stacValue(m_json, "properties");

    // DateTime
    // If STAC datetime fits in *any* of the supplied ranges,
    // it will be accepted.
    if (!dates.empty())
    {
        if (properties.contains("datetime") &&
            properties.at("datetime").type() != nlohmann::detail::value_t::null)
        {
            std::string stacDateStr = stacValue(properties,
                "datetime", m_json);

            try
            {
                std::time_t stacTime = getStacTime(stacDateStr);
                for (const auto& range: dates)
                    if (stacTime >= range.first && stacTime <= range.second)
                        return true;
            }
            catch (pdal_error& e)
            {
                throw stac_error(m_id, "item", e.what());
            }

            return false;
        }
        else if (properties.contains("start_datetime") &&
            properties.contains("end_datetime"))
        {
                // Handle if STAC object has start and end datetimes instead of one
                std::string endDateStr = stacValue(properties,
                    "end_datetime", m_json);
                std::string startDateStr = stacValue(properties,
                    "end_datetime", m_json);

                std::time_t stacEndTime = getStacTime(endDateStr);
                std::time_t stacStartTime = getStacTime(startDateStr);

                for (const auto& range: dates)
                {
                    // If any of the date ranges overlap with the date range of the STAC
                    // object, do not prune.
                    std::time_t userMinTime = range.first;
                    std::time_t userMaxTime = range.second;

                    if (userMinTime >= stacStartTime && userMinTime <= stacEndTime)
                        return true;
                    else if (userMaxTime >= stacStartTime && userMaxTime <= stacEndTime)
                        return true;
                    else if (userMinTime <= stacStartTime && userMaxTime >= stacEndTime)
                        return true;
                }
                return false;
        }
        else
            throw stac_error(m_id, "item", "Unexpected layout of STAC dates");
    }
    return true;

}

bool Item::filterAssets(std::vector<std::string> assetNames)
{
    nlohmann::json asset;
    nlohmann::json assetList = stacValue(m_json, "assets");
    for (auto& name: assetNames)
    {
        if (assetList.contains(name))
        {
            asset = stacValue(assetList, name, m_json);
            m_driver = extractDriverFromItem(asset);
            std::string assetPath = stacValue(asset, "href", m_json);
            m_assetPath = handleRelativePath(m_path, assetPath);
        }
    }
    if (m_driver.empty())
        return false;
    return true;
}

// If STAC ID matches any ID in supplied list, it will be accepted
bool Item::filterIds(std::vector<RegEx> ids)
{
    if (!ids.empty())
    {
        for (auto& id: ids)
            if (std::regex_match(m_id, id.regex()))
                return true;
        return false;
    }
    return true;
}

bool Item::filterCol(std::vector<RegEx> ids)
{
    if (!ids.empty())
    {
        if (!m_json.contains("collection"))
            return false;

        std::string colId = stacValue<std::string>(
            m_json, "collection");
        for (auto& id: ids)
            if (std::regex_match(colId, id.regex()))
                return true;

        return false;
    }
    return true;

}

} //namespace stac

} //namespace pdal
