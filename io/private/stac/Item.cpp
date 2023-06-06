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
#include "Utils.hpp"
#include "../connector/Connector.hpp"

#include <pdal/StageWrapper.hpp>
#include <pdal/Polygon.hpp>

#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>

namespace pdal
{

namespace stac
{

    Item::Item(const NL::json& json,
            const std::string& itemPath,
            const connector::Connector& connector) :
        m_json(json), m_path(itemPath), m_connector(connector)
    {}

    Item::~Item()
    {}

    void Item::init(std::vector<std::string> assetNames, NL::json readerArgs)
    {
        NL::json asset;
        for (auto& name: assetNames)
        {
            if (m_json.at("assets").contains(name))
            {
                asset = m_json.at("assets").at(name);
                m_driver = extractDriverFromItem(asset);
                std::string assetPath = asset.at("href").get<std::string>();
                m_dataPath = handleRelativePath(m_path, assetPath);
            }
        }
        if (m_driver.empty())
            throw pdal_error("None of the asset names supplied exist in the STAC object.");

        Stage *stage = m_factory.createStage(m_driver);

        if (!stage)
        {
            std::stringstream msg;
            msg << "Unable to create driver '" << m_driver << "' for "
                << "for asset located at '" << m_dataPath <<"'";
            throw pdal_error(msg.str());
        }

        m_reader = dynamic_cast<Reader*>(stage);
        if (!m_reader)
        {
            std::stringstream msg;
            msg << "Unable to cast stage to reader for '" << m_driver << "' for "
                << "for asset located at '" << m_dataPath <<"'";
            throw pdal_error(msg.str());
        }

        Options readerOptions = setReaderOptions(readerArgs, m_driver);
        readerOptions.add("filename", m_dataPath);

        m_reader->setOptions(readerOptions);

        // std::lock_guard<std::mutex> lock(m_p->m_mutex);
        // m_merge.setInput(*reader);
        // reader->setLog(log());

    }

    std::string Item::driver()
    {
        return m_driver;
    }

    Reader* Item::reader()
    {
        return m_reader;
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
            throw pdal_error("asset does not contain an href!");

        std::string assetPath = asset.at("href").get<std::string>();
        std::string dataUrl = handleRelativePath(m_path, assetPath);

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

    void Item::read()
    {

    }

    void Item::validate()
    {
        std::function<void(const nlohmann::json_uri&, nlohmann::json&)> fetch = schemaFetch;
        nlohmann::json_schema::json_validator val(
            fetch,
            [](const std::string &, const std::string &) {}
        );
        const std::string schemaUrl = m_schemaUrl;
        for (auto& extSchemaUrl: m_json.at("stac_extensions"))
        {
            // log()->get(LogLevel::Debug) << "Processing extension " << extSchemaUrl << std::endl;
            NL::json schemaJson = m_connector.getJson(extSchemaUrl);
            val.set_root_schema(schemaJson);
            val.validate(m_json);
        }

        NL::json schemaJson = m_connector.getJson(schemaUrl);
        val.set_root_schema(schemaJson);
        val.validate(m_json);
    }

    void validateForPrune(NL::json json)
    {

        // TODO none of these error messages tell us
        // *which* itemId is invalid. Someone who is given
        // any of these errors is being told they're f'd right off and
        // given no information to do anything about it
        if (!json.contains("id"))
            throw pdal_error("JSON object does not contain required key 'id'");

        const std::string id = json.at("id").get<std::string>();

        if (!json.contains("properties"))
            throw pdal_error("JSON object " + id + " does not contain required key 'properties'");

        if (!json.contains("geometry") || !json.contains("bbox"))
            throw pdal_error("JSON object " + id + " does not contain one of 'geometry' or 'bbox'");

        NL::json prop = json.at("properties");
        if (
            !prop.contains("datetime") &&
            (!prop.contains("start_datetime") && !prop.contains("end_datetime"))
        )
            throw pdal_error("JSON object " + id + "  properties value not contain required key"
                "'datetime' or 'start_datetime' and 'end_datetime'");

        // TODO validate the date ranges and other validation-type stuff
        // that's going on in `prune`
        //

    }

    bool matchProperty(std::string key, NL::json val, NL::json properties, NL::detail::value_t type)
    {
        switch (type)
        {
            case NL::detail::value_t::string:
            {
                std::string desired = val.get<std::string>();
                // std::regex desired(d);
                std::string value = properties.at(key).get<std::string>();
                if (value != desired)
                    return false;
                // if (!std::regex_match(value, desired))
                //     return false;
                break;
            }
            case NL::detail::value_t::number_unsigned:
            {
                uint64_t value = properties.at(key).get<uint64_t>();
                uint64_t desired = val.get<uint64_t>();
                if (value != desired)
                    return false;
                break;
            }
            case NL::detail::value_t::number_integer:
            {
                int value = properties.at(key).get<int>();
                int desired = val.get<int>();
                if (value != desired)
                    return false;
                break;
            }
            case NL::detail::value_t::number_float:
            {
                int value = properties.at(key).get<double>();
                int desired = val.get<double>();
                if (value != desired)
                    return false;
                break;
            }
            case NL::detail::value_t::boolean:
            {
                bool value = properties.at(key).get<bool>();
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

    bool Item::prune(Filters filters)
    {
        validateForPrune(m_json);

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
        std::string itemId = m_json.at("id");
        bool idFlag = true;
        if (!filters.ids.empty())
        {
            for (auto& id: filters.ids)
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

        NL::json properties = m_json.at("properties");

        // DateTime
        // If STAC datetime fits in *any* of the supplied ranges, it will not be pruned
        if (!filters.dates.empty())
        {
            if (properties.contains("datetime") && properties.at("datetime").type() != NL::detail::value_t::null)
            {
                std::string stacDate = properties.at("datetime");

                // TODO This would remove three lines of stuff and
                // be much clearer
                //
                // bool haveDateFlag = !filters.dates.empty()
                bool dateFlag = true;
                if (filters.dates.empty())
                    dateFlag = false;
                for (auto& range: filters.dates)
                {
                    //If the extracted item date fits into any of the dates provided by
                    //the user, then do not prune this item based on dates.
                    if (range.size() != 2)
                        throw pdal_error("Invalid date range size!");

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
                std::string stacStartDate = properties.at("start_datetime").get<std::string>();
                std::string stacEndDate = properties.at("end_datetime").get<std::string>();

                bool dateFlag = true;
                for (const auto& range: filters.dates)
                {
                    // If any of the date ranges overlap with the date range of the STAC
                    // object, do not prune.
                    if (range.size() != 2)
                        throw pdal_error("Invalid date range size!");
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
        }

        // Properties
        // If STAC properties match *all* the supplied properties, it will not be pruned
        //
        // TODO this reads backwards
        // if match:
        //     doPrune = true
        if (!filters.properties.empty())
        {
            for (auto &it: filters.properties.items())
            {
                if (!properties.contains(it.key()))
                {
                    // log()->get(LogLevel::Warning) << "STAC Item does not contain "
                    //     "property " << it.key() << ". Continuing." << std::endl;
                    continue;
                }

                NL::detail::value_t type = properties.at(it.key()).type();
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
        if (!filters.bounds.empty())
        {
            if (m_json.contains("geometry"))
            {
                NL::json geometry = m_json.at("geometry").get<NL::json>();
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
                if (filters.bounds.is3d())
                {
                    if (!filters.bounds.to3d().overlaps(f.bounds()))
                        return true;
                }
                else
                {
                    // TODO this is confusing, but I guess that
                    // is a result of PDAL's bounds interface.
                    // ie, we downcast the bounds to2d, then make
                    // a BOX3D from that and compare it to f.bounds()
                    // which is 2d or 3d?
                    BOX2D bbox = filters.bounds.to2d();
                    if (!BOX3D(bbox).overlaps(f.bounds()))
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
                {
                    // log()->get(LogLevel::Error) << "bbox for '" << itemId << "' is not valid";
                }

                if (bboxJson.size() == 4)
                {
                    double minx = bboxJson[0];
                    double miny = bboxJson[1];
                    double maxx = bboxJson[2];
                    double maxy = bboxJson[3];
                    BOX2D bbox = BOX2D(minx, miny, maxx, maxy);
                    if (!filters.bounds.to2d().overlaps(bbox))
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
                    if (!filters.bounds.to3d().overlaps(bbox))
                        return true;
                }
            }
        }

        // log()->get(LogLevel::Debug) << "Including: " << itemId << std::endl;

        // {
        //     std::lock_guard<std::mutex> lock(m_p->m_mutex);
        //     m_p->m_idList.push_back(itemId);
        // }
        return false;
    }

} //namespace stac

} //namespace pdal