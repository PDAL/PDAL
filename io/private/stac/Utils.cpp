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

#include "Utils.hpp"

namespace pdal
{

namespace stac
{

pdal_error stac_error(std::string id, std::string stacType,
    std::string const& msg)
{
    return pdal_error("STACError (" + stacType + ": " + id + "): " + msg);
}

pdal_error stac_error(std::string const& msg)
{
    return pdal_error("STACError: " + msg);
}

namespace StacUtils
{

std::string handleRelativePath(std::string srcPath, std::string linkPath)
{
    //Make absolute path of current item's directory, then create relative path from that

    //If the filepath is already absolute we don't need to bother.
    if (FileUtils::isAbsolutePath(linkPath))
        return linkPath;

    //If src item isn't an absolute path, we need to convert it to one for getDirectory to work
    if (!FileUtils::isAbsolutePath(srcPath))
        srcPath = FileUtils::toAbsolutePath(srcPath);
    //Get driectory of src item
    const std::string baseDir = FileUtils::getDirectory(srcPath);
    //Create absolute path from src item filepath, if it's not already
    // and join relative path to src item's dir path
    return FileUtils::toAbsolutePath(linkPath, baseDir);

}

std::time_t getStacTime(std::string in)
{
    std::istringstream dateStr(in);
    std::tm date {};
    dateStr >> std::get_time(&date, "%Y-%m-%dT%H:%M:%S");
    if (dateStr.fail())
        throw stac_error("Specified date (" + dateStr.str() +
            ") cannot be parsed. Dates must fit RFC 3339 specs.");
    return std::mktime(&date);
}

std::string stacId(const nlohmann::json& stac)
{
    std::stringstream msg;
    try
    {
        return stac.at("id").get<std::string>();
    }
    catch (nlohmann::detail::out_of_range& e)
    {
        msg << "Missing required key 'id'. " << e.what();
        throw pdal_error(msg.str());
    }
    catch (nlohmann::detail::type_error& e)
    {
        msg << "Required key 'id' is not of type 'string'. " << e.what();
        throw pdal_error(msg.str());
    }
}

std::string stacType(const nlohmann::json& stac)
{
    try
    {
        return stac.at("type").get<std::string>();
    }
    catch (nlohmann::detail::out_of_range& e)
    {
        std::stringstream msg;
        msg << "Missing required key 'type'. " << e.what();
        throw pdal_error(msg.str());
    }
    catch (nlohmann::detail::type_error& e)
    {
        std::stringstream msg;
        msg << "Invalid key value 'type'. " << e.what();
        throw pdal_error(msg.str());
    }
}

std::string icSelfPath(const nlohmann::json& json)
{
    try
    {
        nlohmann::json links = jsonValue(json, "links");
        for (const nlohmann::json& link: links)
        {
            std::string target = jsonValue<std::string>(link, "rel");
            if (target == "self")
                return jsonValue<std::string>(link, "href");
        }
    }
    catch(std::runtime_error& )
    {
        return "";
    }

    return "";
}


}//StacUtils
}//stac
}//pdal
