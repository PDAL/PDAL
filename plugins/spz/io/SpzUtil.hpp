/******************************************************************************
* Copyright (c) 2025, Isaac Bell (isaac@hobu.co)
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

#include <map>

#include <pdal/util/Utils.hpp>
#include <load-spz.h>

namespace spz {

CoordinateSystem getCoordinateSystem(const std::string &name)
{
    std::map<std::string, spz::CoordinateSystem> names = {
        {"LDB", CoordinateSystem::LDB},
        {"RDB", CoordinateSystem::RDB},
        {"LUB", CoordinateSystem::LUB},
        {"RUB", CoordinateSystem::RUB},
        {"LDF", CoordinateSystem::LDF},
        {"RDF", CoordinateSystem::RDF},
        {"LUF", CoordinateSystem::LUF},
        {"RUF", CoordinateSystem::RUF},
    };
    auto it = names.find(pdal::Utils::toupper(name));
    if (it != names.end()) {
        return it->second;
    }
    return CoordinateSystem::UNSPECIFIED;
}

}  // namespace spz