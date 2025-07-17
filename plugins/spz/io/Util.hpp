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