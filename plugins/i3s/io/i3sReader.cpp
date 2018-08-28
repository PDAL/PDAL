// I3SReader.cpp

#include "i3sReader.hpp"

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

    //register default values X, Y and Z
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    for (int j = 0; j < m_args.body["layers"].size(); j++) {

      Json::Value attributes = m_args.body["layers"][j]["attributeStorageInfo"];

      for(int i = 0; i < attributes.size(); i ++)
      {
        
        std::string name = Json::writeString(writer, attributes[i]["name"]);
        
        name.erase(
          remove( name.begin(), name.end(), '\"' ),
          name.end()
        );

        std::string type;
        /*Check the entry has an attributevalues object
         TODO check if name is a part of pdal's wanted parameters
         TODO manage struct from header file
         */ 
        if (attributes[i].isMember("attributeValues")) {
          type = Json::writeString(writer,
              attributes[i]["attributeValues"]["valueType"]);
          layout->registerOrAssignDim(name, pdal::Dimension::type(type));
          m_layout.registerOrAssignDim(name, pdal::Dimension::type(type));
        }
        else{
          layout->registerOrAssignDim(name, Dimension::Type::Double);
          m_layout.registerOrAssignDim(name, Dimension::Type::Double);
        }

      }
    }
  }

  void I3SReader::ready(PointTableRef)
  {

    SpatialReference ref("EPSG:4385");
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
    
    log()->get(LogLevel::Debug) << "fetching info from " << 
        m_args.url + "/layers/0/nodes/0/geometries/0" << "\n"; 

    std::string urlGeo = m_args.url + "/layers/0/nodes/0/geometries/0";
     
    auto response(m_arbiter->getBinary(urlGeo));
        
    const std::size_t pointSize(view->layout()->pointSize());
    uint32_t numPoints(0);
    std::copy(
            response.data() + response.size() - sizeof(uint32_t),
            response.data() + response.size(),
            reinterpret_cast<char*>(&numPoints));
    log()->get(LogLevel::Debug) <<
        "Fetched " << numPoints << " points" << std::endl;
    log()->get(LogLevel::Debug) <<
        "Fetched " << response.size() << " bytes" << std::endl;
 

    view->setField(pdal::Dimension::Id::X, 0, 12);
    view->setField(pdal::Dimension::Id::Y, 0, 13);
    view->setField(pdal::Dimension::Id::Z, 0, 14);
    view->setField(pdal::Dimension::Id::Intensity, 0, 55);
    
    

    return 0;
  }

  void I3SReader::done(PointTableRef)
  {
    m_stream.reset();
  }

} //namespace pdal
