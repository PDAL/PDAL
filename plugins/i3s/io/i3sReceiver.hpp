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
    std::string url;
    Json::Value body;
    std::string name;
    std::string itemId;
    std::string intensity;
  };


  std::vector<lepcc::Point3D> decompress(
          bool elevation, bool rgb, std::vector<char>* compData, int nodeNum);
}
