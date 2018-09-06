// I3SReader.cpp

#include "i3sReader.hpp"
#include "../lepcc/src/include/lepcc_c_api.h"                                   
#include "../lepcc/src/include/lepcc_types.h" 

#include <istream>
#include <cstdint>
#include <cstring>                                               
#include <cmath>                                                    
#include <stdio.h>                                               
#include <iostream>                                                
#include <vector>                                                    
#include <algorithm>                                                
#include <chrono>                                                               
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/compression/LazPerfCompression.hpp>


namespace pdal
{
  static PluginInfo const s_info
  {
    "readers.i3s",
    "Read file from i3s server",
    "http://why.com/doyouthink/ihave/documentation"
  };

  CREATE_SHARED_STAGE(I3SReader, s_info)



  std::string I3SReader::getName() const { return s_info.name; }

  void I3SReader::initialize(PointTableRef table)
  {

      Json::Value config;
      if (log()->getLevel() > LogLevel::Debug4)
          config["arbiter"]["verbose"] = true;
      m_arbiter.reset(new arbiter::Arbiter(config));


      if (m_filename.size() && m_args.url.empty())
      {
          m_args.url = m_filename;
          const std::string pre("i3s://");
          if (m_args.url.find(pre) == 0)
              m_args.url = m_args.url.substr(pre.size());
          if (m_args.url.find("https://") == std::string::npos)
              m_args.url = "https://" + m_args.url;
      }

      log()->get(LogLevel::Debug) << "Fetching info from " << m_args.url <<
          std::endl;


    /*request 3scenelayerinfo: URL Pattern <scene-server-url/layers/<layer-id>*/
      try
      {
          m_args.body = parse(m_arbiter->get(m_args.url));
      }
      catch (std::exception& e)
      {
          throw pdal_error(std::string("Failed to fetch info: ") + e.what());
      }
      Json::StreamWriterBuilder writer;
      writer.settings_["indentation"] = "";
      m_args.name = Json::writeString(writer, m_args.body["name"]);
      m_args.itemId = Json::writeString(writer, m_args.body["serviceItemId"]);
  }

  void I3SReader::addArgs(ProgramArgs& args)
  {
      args.add("url", "URL", m_args.url);
      args.add("body", "JSON formatted body", m_args.body);
      args.add("itemId", "ID of the current item", m_args.itemId);
      args.add("name", "Name of the point cloud data", m_args.name);
  }

  void I3SReader::addDimensions(PointLayoutPtr layout)
  {

      Json::StreamWriterBuilder writer;
      writer.settings_["indentation"] = "";

      for (int j = 0; j < m_args.body["layers"].size(); j++) 
      {
        
          Json::Value attributes = 
              m_args.body["layers"][j]["attributeStorageInfo"];

          for(int i = 0; i < attributes.size(); i ++)
          {

              std::string readName = 
                  Json::writeString(writer, attributes[i]["name"]);
              //remove quotes from json object
              readName.erase(
                  remove( readName.begin(), readName.end(), '\"' ),
                  readName.end()
              );
              //remove random underscores
              readName.erase(
                  remove( readName.begin(), readName.end(), '_' ),
                  readName.end()
              );

              if(Dimension::id(readName) != Dimension::Id::Unknown)
              {
                  Dimension::Id name = Dimension::id(readName);
                  layout->registerDim(name);
                  m_layout.registerDim(name);
              }else if(readName.compare("RGB") == 0)
              {
                  m_layout.registerDim(Dimension::Id::Red);
                  m_layout.registerDim(Dimension::Id::Green);
                  m_layout.registerDim(Dimension::Id::Blue);
                  
                  layout->registerDim(Dimension::Id::Red);
                  layout->registerDim(Dimension::Id::Green);
                  layout->registerDim(Dimension::Id::Blue);
              }
              else if(readName.compare("FLAGS") == 0)
              {
                  m_layout.registerDim(Dimension::Id::Flag);
                  layout->registerDim(Dimension::Id::Flag);
              }
              else if(readName.compare("RETURNS") == 0)
              {
                  m_layout.registerDim(Dimension::Id::NumberOfReturns);
                  layout->registerDim(Dimension::Id::NumberOfReturns);
              }
              else if(readName.compare("ELEVATION") == 0)
              {
                  layout->registerDim(Dimension::Id::X);
                  layout->registerDim(Dimension::Id::Y);
                  layout->registerDim(Dimension::Id::Z);

                  m_layout.registerDim(Dimension::Id::X);
                  m_layout.registerDim(Dimension::Id::Y);
                  m_layout.registerDim(Dimension::Id::Z);
              }
              else if(readName.compare("CLASSCODE") == 0)
              {
                  m_layout.registerDim(Dimension::Id::ClassFlags);
                  layout->registerDim(Dimension::Id::ClassFlags);
              }
              else if(readName.compare("POINTSRCID") == 0)
              {
                  m_layout.registerDim(Dimension::Id::PointSourceId);
                  layout->registerDim(Dimension::Id::PointSourceId);
              }
              else//if it isn't in pdal dimensions and isn't a special case
              {
                  std::string type;
                  if (attributes[i].isMember("attributeValues")) 
                  {
                      std::string type;
                      type = Json::writeString(writer,
                          attributes[i]["attributeValues"]["valueType"]);
                      layout->registerOrAssignDim(readName, 
                              pdal::Dimension::type(type));
                      m_layout.registerOrAssignDim(readName, 
                              pdal::Dimension::type(type));
                  }
                  else
                  {
                      layout->registerOrAssignDim(readName, 
                              Dimension::Type::Double);
                      m_layout.registerOrAssignDim(readName, 
                              Dimension::Type::Double);
                  }
              }
          }
      }
  }

  void I3SReader::ready(PointTableRef)
  {
      Json::StreamWriterBuilder writer;
      writer.settings_["indentation"] = "";
      Json::Value spatialJson = 
          m_args.body["layers"][0]["spatialReference"];
      std::string spatialStr =
          "EPSG:" +
          Json::writeString(writer,
          spatialJson["wkid"]);
      //if(spatialJson.isMember("vcsWkid"))
      //    spatialStr += ("+" +
      //        Json::writeString(writer,
      //        spatialJson["vcsWkid"])
      //    );
      SpatialReference ref(spatialStr);
      setSpatialReference(ref);
  }


  point_count_t I3SReader::read(PointViewPtr view, point_count_t count)
  {
      /*
      -3scenelayerinfo: URL Pattern <scene-server-url/layers/<layer-id>
      -node index document: <layer-url >/nodes/<node-id>
      -shared resources: <node-url>/shared/
      -feature data: <node-url>/features/<feature-data-bundle-id>
      -geometry data: <node-url>/geometries/<geometry-data-bundle-id>
      -texture data: <node-url>/textures/<texture-data-bundle-id>
      */
        
      /*Determine number of nodes*/
      Json::Value config;

      std::string nodeUrl = m_args.url + "/layers/0/nodepages/0";
      //Json::Value nodeIndex = parse(m_arbiter->get(nodeUrl));
      //int nodeSize = nodeIndex.size();
      std::cout << nodeUrl << std::endl ; 
      //std::cout <<"Hello worldi" << nodeIndex << std::endl;
      int pointCloudCount = 0;
      for(int i = 0; i < 64; i++)
      {
          /*Retrieve binary*/
          std::string urlGeo = m_args.url + 
              "/layers/0/nodes/" + std::to_string(i) + "/geometries/0";
          std::vector<char> response = m_arbiter->getBinary(urlGeo);
          std::vector<lepcc::Point3D> pointcloud = 
              decompress(true, true, &response, i);
          for(std::size_t j = 0; j < pointcloud.size(); j ++)
          {    
              view->setField(pdal::Dimension::Id::X, pointCloudCount, pointcloud[j].x);
              view->setField(pdal::Dimension::Id::Y, pointCloudCount, pointcloud[j].y);
              view->setField(pdal::Dimension::Id::Z, pointCloudCount, pointcloud[j].z);
              pointCloudCount++;
          }
      }
      const std::size_t pointSize(view->layout()->pointSize());


      return 0;
  }

  void I3SReader::done(PointTableRef)
  {
    m_stream.reset();
  }
} //namespace pdal
