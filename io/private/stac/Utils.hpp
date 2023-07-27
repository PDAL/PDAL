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
#pragma once

#include <arbiter/arbiter.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_types.hpp>

namespace pdal
{

namespace stac
{
    pdal_error stac_error(std::string id, std::string stacType, std::string const& msg);
    pdal_error stac_error(std::string const& msg);


class StacUtils
{

public:
    StacUtils();
    ~StacUtils();

    static std::string handleRelativePath(std::string srcPath,
        std::string linkPath);
    static std::time_t getStacTime(std::string in);

    template <class T = NL::json>
    static T jsonValue(const NL::json& json, std::string key = "")
    {
        try
        {
            if (key.empty())
                return json.get<T>();
            return json.at(key).get<T>();
        }
        catch (NL::detail::exception e)
        {
            std::stringstream msg;
            msg << "Error: " << e.what() << ", for object " << json.dump();
            throw pdal_error(msg.str());
        }
    }

    template <class U = NL::json>
    static U stacValue(const NL::json& stac, std::string key = "",
        const NL::json& rootJson = {})
    {
        NL::json stacCheck = stac;
        if (!rootJson.empty())
            stacCheck = rootJson;
        std::string type = stacType(stacCheck);
        std::string id;
        if (type == "FeatureCollection")
            id = icSelfPath(stacCheck);
        else
            id = stacId(stacCheck);

        try
        {
            if (key.empty())
                return stac.get<U>();
            return stac.at(key).get<U>();
        }
        catch (NL::detail::exception e)
        {
            throw stac_error(id, type, e.what());
        }
    }

    static std::string stacId(const NL::json& stac)
    {
        std::stringstream msg;
        try
        {
            return stac.at("id").get<std::string>();
        }
        catch (NL::detail::out_of_range e)
        {
            msg << "Missing required key 'id'. " << e.what();
            throw pdal_error(msg.str());
        }
        catch (NL::detail::type_error e)
        {
            msg << "Required key 'id' is not of type 'string'. " << e.what();
            throw pdal_error(msg.str());
        }
    }

    static std::string stacType(const NL::json& stac)
    {
        std::string id = stacId(stac);
        std::stringstream msg;
        try
        {
            return stac.at("id").get<std::string>();
        }
        catch (NL::detail::out_of_range e)
        {
            msg << "Missing required key 'type' in id(" + id + ")." << e.what();
            throw pdal_error(msg.str());
        }
        catch (NL::detail::type_error e)
        {
            msg << "Invalid key 'type' in id(" + id + "). " << e.what();
            throw pdal_error(msg.str());
        }
    }

    static std::string icSelfPath(const NL::json& json)
    {
        try
        {
            NL::json links = jsonValue(json, "links");
            for (const NL::json& link: links)
            {
                std::string target = jsonValue<std::string>(link, "rel");
                if (target == "self")
                    return jsonValue<std::string>(link, "href");
            }
        }
        catch(NL::detail::exception)
        { }

        return "";
    }
};

}// stac
}// pdal
