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
#pragma once

#include "../lepcc/src/include/lepcc_c_api.h"
#include "../lepcc/src/include/lepcc_types.h"
#include <cstddef>
#include <string>
#include <cstdint>
#include <json/json.h>
#include <vector>
#include <pdal/pdal_types.hpp>
#include <pdal/PointLayout.hpp>


namespace pdal
{
    /*Return value of data in json format*/
    static inline Json::Value parse(const std::string& data)
    {
        Json::Value json;
        Json::Reader reader;
        if (data.size())
        {
            if (!reader.parse(data, json, false))
            {
                const std::string jsonError(reader.getFormattedErrorMessages());
                if (!jsonError.empty())
                {
                    throw pdal_error("Error during parsing: " + jsonError);
                }
            }
        }
        return json;
    }

    struct compare3d
    {
        bool operator()(const lepcc::Point3D & first,
                const lepcc::Point3D & second) const
        {
            if(first.x < second.x)
                return true;
            if(first.x > second.x)
                return false;
            if(first.y < second.y)
                return true;
            if(first.y > second.y)
                return false;
            if(first.z < second.z)
                return true;
            return false;
        }
    };

    std::vector<lepcc::Point3D> decompressXYZ(
            std::vector<char>* compData);

    std::vector<lepcc::RGB_t> decompressRGB(
            std::vector<char>* compData);

    std::vector<uint16_t> decompressIntensity(
            std::vector<char>* compData);
}
