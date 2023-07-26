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
#include <pdal/Polygon.hpp>

#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>

namespace pdal
{

namespace stac
{

    Item::Item(const NL::json& json,
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

    bool Item::init(Filters filters, NL::json rawReaderArgs,
            SchemaUrls schemaUrls)
    {

        if (!filter(filters))
            return false;

        m_schemaUrls = schemaUrls;
        if (m_validate)
            validate();


        NL::json readerArgs = handleReaderArgs(rawReaderArgs);
        m_readerOptions = setReaderOptions(readerArgs, m_driver);
        m_readerOptions.add("filename", m_assetPath);
        return true;
    }

    std::string Item::id()
    {
        return m_json.at("id").get<std::string>();
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

    NL::json Item::handleReaderArgs(NL::json rawReaderArgs)
    {
        if (rawReaderArgs.is_object())
        {
            NL::json array_args = NL::json::array();
            array_args.push_back(rawReaderArgs);
            rawReaderArgs = array_args;
        }
        for (auto& opts: rawReaderArgs)
            if (!opts.is_object())
                throw pdal_error("Reader Args for reader '" + m_driver + "' must be a valid JSON object");

        NL::json readerArgs;
        for (NL::json& readerPipeline: rawReaderArgs)
        {
            if (!readerPipeline.contains("type"))
                throw pdal_error("No \"type\" key found in supplied reader arguments.");

            std::string driver = readerPipeline.at("type").get<std::string>();
            if (rawReaderArgs.contains(driver))
                throw pdal_error("Multiple instances of the same driver in supplied reader arguments.");
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

    Options Item::setReaderOptions(const NL::json& readerArgs, const std::string& driver) const
    {
        Options readerOptions;
        if (readerArgs.contains(driver)) {
            NL::json args = readerArgs.at(driver).get<NL::json>();
            for (auto& arg : args.items()) {
                NL::detail::value_t type = readerArgs.at(driver).at(arg.key()).type();
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

        return readerOptions;
    }

    std::string Item::extractDriverFromItem(const NL::json& asset) const
    {
        std::string output;

        std::map<std::string, std::string> contentTypes =
        {
            { "application/vnd.laszip+copc", "readers.copc"}
        };

        if (!asset.contains("href"))
            throw stac_error(m_id, "item", "asset does not contain an href!");

        std::string assetPath = asset.at("href").get<std::string>();
        std::string dataUrl = m_utils.handleRelativePath(m_path, assetPath);

        std::string contentType;

        if (asset.contains("type"))
        {
            contentType = asset.at("type").get<std::string>();
            for(const auto& ct: contentTypes)
                if (Utils::iequals(ct.first, contentType))
                    return ct.second;
        }

        // Try to guess from the URL
        std::string driver = m_factory.inferReaderDriver(dataUrl);
        if (driver.size())
            return driver;


        // Try to make a HEAD request and get it from Content-Type
        StringMap headers = m_connector.headRequest(dataUrl);
        if (headers.find("Content-Type") != headers.end())
        {
            contentType = headers["Content-Type"];
            for(const auto& ct: contentTypes)
                if (Utils::iequals(ct.first, contentType))
                    return ct.second;
        }

        return output;
    }

    void Item::validate()
    {

        nlohmann::json_schema::json_validator val(
            [this](const nlohmann::json_uri& json_uri, nlohmann::json& json) {
                NL::json tempJson = m_connector.getJson(json_uri.url());

                std::lock_guard<std::mutex> lock(m_mutex);
                json = tempJson;
            },
            [](const std::string &, const std::string &) {}
        );

        // Validate against base Item schema first
        NL::json schemaJson = m_connector.getJson(m_schemaUrls.item);
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
            for (auto& extSchemaUrl: m_json.at("stac_extensions"))
            {
                std::string url;
                try {
                    url = extSchemaUrl.get<std::string>();
                }
                catch (NL::detail::type_error e)
                {
                    throw stac_error(m_id, "item",
                        "Invalid stac extension: " + extSchemaUrl.dump());
                }

                try {
                    NL::json schemaJson = m_connector.getJson(url);
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

    void validateForFilter(NL::json json)
    {

        if (!json.contains("id"))
            throw pdal_error("JSON object does not contain required key 'id'");

        std::string id = json.at("id").get<std::string>();

        if (!json.contains("assets"))
            throw stac_error(id, "item", "Missing required key 'assets'");

        if (!json.contains("properties"))
            throw stac_error(id, "item", "Missing required key 'properties'");

        if (!json.contains("geometry") || !json.contains("bbox"))
            throw stac_error(id, "item",
                "STAC Item must have either 'geometry' or 'bbox' key.");

    }

    bool matchProperty(std::string key, NL::json val, NL::json properties, NL::detail::value_t type)
    {
        switch (type)
        {
            case NL::detail::value_t::string:
            {
                std::string desired = val.get<std::string>();
                std::string value = properties.at(key).get<std::string>();
                return value == desired;
                break;
            }
            case NL::detail::value_t::number_unsigned:
            {
                uint64_t value = properties.at(key).get<uint64_t>();
                uint64_t desired = val.get<uint64_t>();
                return value == desired;
                break;
            }
            case NL::detail::value_t::number_integer:
            {
                int value = properties.at(key).get<int>();
                int desired = val.get<int>();
                return value == desired;
                break;
            }
            case NL::detail::value_t::number_float:
            {
                int value = properties.at(key).get<double>();
                int desired = val.get<double>();
                return value == desired;
                break;
            }
            case NL::detail::value_t::boolean:
            {
                bool value = properties.at(key).get<bool>();
                bool desired = val.get<bool>();
                return value == desired;
                break;
            }
            default:
            {
                throw pdal_error("Data type of " + key +
                    " is not supported for pruning.");
            }
        }
        return true;
    }



    bool Item::filter(Filters filters)
    {
        validateForFilter(m_json);
        m_id = m_json.at("id").get<std::string>();

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

    bool Item::filterBounds(SrsBounds bounds)
    {
        if (!bounds.empty())
        {
            if (m_json.contains("geometry"))
            {
                NL::json geometry = m_json.at("geometry").get<NL::json>();
                Polygon f(geometry.dump());
                if (!f.valid())
                    throw stac_error(m_id, "item",
                        "Polygon created from STAC 'geometry' key is invalid");

                // Convert to BOX3D
                if (bounds.is3d())
                {
                    if (bounds.to3d().overlaps(f.bounds()))
                        return true;
                }
                else
                {
                    //Cast BOX2D to 3D to fit f.bounds which is 3d
                    BOX2D bbox = bounds.to2d();
                    if (BOX3D(bbox).overlaps(f.bounds()))
                        return true;
                }
            }

            // TODO make a function that does bbox filtering or find one
            // or make one in PDALUtils
            else if (m_json.contains("bbox"))
            {
                NL::json bboxJson = m_json.at("bbox").get<NL::json>();

                // TODO if we have a bad bbox?
                if (bboxJson.size() != 4 || bboxJson.size() != 6)
                    throw pdal_error("bbox for '" + m_id + "' is not valid");

                if (bboxJson.size() == 4)
                {
                    double minx = bboxJson[0];
                    double miny = bboxJson[1];
                    double maxx = bboxJson[2];
                    double maxy = bboxJson[3];
                    BOX2D bbox = BOX2D(minx, miny, maxx, maxy);
                    if (bounds.to2d().overlaps(bbox))
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
                    if (bounds.to3d().overlaps(bbox))
                        return true;
                }
            }
            return false;
        }
        return true;

    }

    bool Item::filterProperties(NL::json filterProps)
    {

        NL::json itemProperties = m_json.at("properties");
        if (!filterProps.empty())
        {
            for (auto &it: filterProps.items())
            {
                NL::detail::value_t type = itemProperties.at(it.key()).type();
                NL::detail::value_t argType = it.value().type();
                //Array of possibilities are Or'd together
                if (argType == NL::detail::value_t::array)
                {
                    bool arrFlag (true);
                    for (auto& val: it.value())
                        if (matchProperty(it.key(), val, itemProperties, type))
                            return true;
                }
                else
                    if (!matchProperty(it.key(), it.value(), itemProperties, type))
                        return true;
            }

            return false;
        }
        return true;
    }

    bool Item::filterDates(DatePairs dates)
    {
        NL::json properties = m_json.at("properties");

        // DateTime
        // If STAC datetime fits in *any* of the supplied ranges,
        // it will be accepted.
        if (!dates.empty())
        {
            if (properties.contains("datetime") &&
                properties.at("datetime").type() != NL::detail::value_t::null)
            {
                std::string stacDateStr;

                try
                {
                    stacDateStr = properties.at("datetime").get<std::string>();
                }
                catch (NL::detail::type_error e)
                {
                        throw pdal_error("Item (" + m_id + ") datetime must be of "
                            "type string and comply with RFC 3339 specs.");
                }

                try
                {
                    std::time_t stacTime = m_utils.getStacTime(stacDateStr);
                    for (const auto& range: dates)
                        if (stacTime >= range.first && stacTime <= range.second)
                            return true;
                }
                catch (stac_error e)
                {
                    throw stac_error(m_id, "item", e.what());
                }

                return false;
            }
            else if (properties.contains("start_datetime") &&
                properties.contains("end_datetime"))
            {
                    // Handle if STAC object has start and end datetimes instead of one
                    std::string endDateStr;
                    std::string startDateStr;
                    try {
                        endDateStr = properties.at("end_datetime").get<std::string>();
                        startDateStr = properties.at("start_datetime").get<std::string>();
                    }
                    catch(NL::detail::type_error e)
                    {
                        throw pdal_error("Item (" + m_id + ") start and end "
                            "datetimes must be of type string and comply with "
                            "RFC 3339 specs.");
                    }

                    std::time_t stacStartTime = m_utils.getStacTime(startDateStr);
                    std::time_t stacEndTime = m_utils.getStacTime(endDateStr);

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
                throw pdal_error("Unexpected layout of STAC dates for Item " +
                    m_id + ". Cannot filter.");
        }
        return true;

    }

    bool Item::filterAssets(std::vector<std::string> assetNames)
    {
        NL::json asset;
        for (auto& name: assetNames)
        {
            if (m_json.at("assets").contains(name))
            {
                asset = m_json.at("assets").at(name);
                m_driver = extractDriverFromItem(asset);
                std::string assetPath = asset.at("href").get<std::string>();
                m_assetPath = m_utils.handleRelativePath(m_path, assetPath);
            }
        }
        if (m_driver.empty())
            return false;
        return true;
    }

    // If STAC ID matches *any* ID in supplied list, it will be accepted
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

            std::string colId = m_json.at("collection").get<std::string>();
            for (auto& id: ids)
                if (std::regex_match(colId, id.regex()))
                    return true;

            return false;
        }
        return true;

    }

} //namespace stac

} //namespace pdal
