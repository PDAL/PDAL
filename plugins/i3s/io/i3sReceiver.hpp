//i3sReceiver.hpp
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

    struct I3SArgs
    {
      Bounds bounds;
      int threads = 8;
    };

    struct compare3d
    {
        bool operator()(const lepcc::Point3D & first, const lepcc::Point3D & second) const 
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
