/******************************************************************************
* Copyright (c) 2024, Hobu Inc.
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

#pragma once

#include <string>

#include <nlohmann/json.hpp>
#include <pdal/Options.hpp>

namespace pdal
{
namespace Utils
{

inline StatusWithReason parseJson(const std::string& s, NL::json& json)
{
    try
    {
        json = NL::json::parse(s);
    }
    catch (NL::json::parse_error& err)
    {
        // Look for a right bracket -- this indicates the start of the
        // actual message from the parse error.
        std::string s(err.what());
        auto pos = s.find(']');
        if (pos != std::string::npos)
            s = s.substr(pos + 1);
        return { -1, s };
    }
    return true;
}

inline std::string inferJsonType(const std::string& s)
{
    NL::json val;
    StatusWithReason status = parseJson(s, val);
    // Not a big deal if this fails.
    if (!status)
        return "string";

    if (val.is_object() || val.is_array())
        return "json";
    else if (val.is_number_float())
        return "double";
    else if (val.is_number_unsigned())
        return "nonNegativeInteger";
    else if (val.is_number_integer())
        return "integer";
    else if (val.is_boolean())
        return "boolean";
    else
        return "string";
}

template <class T = NL::json>
inline T jsonValue(const NL::json& json, std::string key = "")
{
    try
    {
        if (key.empty())
            return json.get<T>();
        return json.at(key).get<T>();
    }
    catch (NL::detail::exception& e)
    {
        std::stringstream msg;
        msg << "Error: " << e.what() << ", for object " << json.dump();
        throw pdal_error(msg.str());
    }
}

// Functions for parsing StacReader & TindexReader reader args

inline NL::json handleReaderArgs(NL::json rawReaderArgs)
{
    if (rawReaderArgs.is_object())
    {
        NL::json array_args = NL::json::array();
        array_args.push_back(rawReaderArgs);
        rawReaderArgs = array_args;
    }
    for (auto& opts: rawReaderArgs)
        if (!opts.is_object())
            throw pdal_error("Reader Args for each reader must be a valid JSON object");

    NL::json readerArgs;
    for (const NL::json& readerPipeline: rawReaderArgs)
    {
        if (!readerPipeline.contains("type"))
            throw pdal_error("No \"type\" key found in supplied reader arguments.");

        std::string driver = jsonValue<std::string>(readerPipeline, "type");
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

inline Options setReaderOptions(const NL::json& readerArgs,
    const std::string& driver, const std::string& filename)
{
    Options readerOptions;
    bool filenameSet = false;
    if (readerArgs.contains(driver)) {
        NL::json args = jsonValue(readerArgs, driver);
        for (auto& arg : args.items())
        {
            std::string key = arg.key();
            NL::json val = arg.value();
            NL::detail::value_t type = val.type();
            // We treat a partial FileSpec as a special case
            if (key == "filename")
            {
                if (!val.is_object())
                    throw pdal_error("value for " + driver + " 'filename' argument " +
                        "must be a valid JSON object.");
                if (val.contains("path"))
                    val.erase("path");
                val += {"path", filename};

                // This doesn't check if the driver supports headers/queries: if not,
                // the reader will only use the filename
                readerOptions.replace("filename", val.dump());
                filenameSet = true;
                continue;
            }

            // if value is of type string, dump() returns string with
            // escaped string inside and kills pdal program args
            std::string v;
            if (type == NL::detail::value_t::string)
                v = jsonValue<std::string>(val);
            else
                v = arg.value().dump();
            readerOptions.add(key, v);
        }
    }
    if (!filenameSet)
        readerOptions.add("filename", filename);

    return readerOptions;
}

} // namespace Utils
} // namespace pdal

