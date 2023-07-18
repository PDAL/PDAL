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
#include <pdal/JsonFwd.hpp>
#include <schema-validator/json-schema.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

namespace stac
{

namespace
{
    std::mutex mutex;
    std::unique_ptr<arbiter::Arbiter> arbiter;
}
    static void schemaFetch(const nlohmann::json_uri& json_uri, nlohmann::json& json)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!arbiter) arbiter.reset(new arbiter::Arbiter());
        }

        std::string jsonStr = arbiter->get(json_uri.url());
        json = nlohmann::json::parse(jsonStr);
    }

    static const std::string handleRelativePath(std::string srcPath, std::string linkPath)
    {
        //Make absolute path of current item's directory, then create relative path from that

        //Get driectory of src item
        const std::string baseDir = FileUtils::getDirectory(srcPath);
        if (FileUtils::isAbsolutePath(linkPath))
            return linkPath;
        //Create absolute path from src item filepath, if it's not already
        // and join relative path to src item's dir path
        return FileUtils::toAbsolutePath(linkPath, baseDir);

    }

    static std::time_t getStacTime(std::string in)
    {
        std::istringstream dateStr(in);
        std::tm date = {};
        dateStr >> std::get_time(&date, "%Y-%m-%dT%H:%M:%S");
        if (dateStr.fail())
            throw pdal_error("Date(" + dateStr.str() + ") cannot be parsed.");
        return std::mktime(&date);
    }

}// stac
}// pdal
