// I3SReader.cpp

#include "i3sReader.hpp"
#include "../lepcc/src/include/lepcc_c_api.h"                                   
#include "../lepcc/src/include/lepcc_types.h" 
#include "pool.hpp"

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
#include <pdal/util/Bounds.hpp>
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
        
        //set up arbiter
        Json::Value config;
        if (log()->getLevel() > LogLevel::Debug4)
            config["arbiter"]["verbose"] = true;
        m_arbiter.reset(new arbiter::Arbiter(config));

        /*Check for */
        if (m_filename.size() && m_args.url.empty())
        {
            m_args.url = m_filename;
            const std::string pre("i3s://");
            if (m_args.url.find(pre) == 0)
                m_args.url = m_args.url.substr(pre.size());
            if (m_args.url.find("https://") == std::string::npos)
                m_args.url = "https://" + m_args.url;
        }
        //TODO remove tailing '/'
        if(m_args.url.back() == '/')
            m_args.url.pop_back();

        log()->get(LogLevel::Debug) << "Fetching info from " << m_args.url <<
            std::endl;

        try
        {
            m_info = parse(m_arbiter->get(m_args.url));
        }
        catch (std::exception& e)
        {
            throw pdal_error(std::string("Failed to fetch info: ") + e.what());
        }
        //create pdal Bounds

        if(m_args.bounds.size())
        {
            if(m_args.bounds.find('(') != std::string::npos)
            {
                std::istringstream iss(m_args.bounds);
                iss >> m_bounds;
            }    
        }
        log()->get(LogLevel::Debug) << "\n\nFileName: " 
            << m_args.url << std::endl;
        log()->get(LogLevel::Debug) << "\nThread Count: " 
            << m_args.threads << std::endl;
        log()->get(LogLevel::Debug) << "Bounds: " 
            << m_args.bounds << std::endl;
    }

    void I3SReader::addArgs(ProgramArgs& args)
    {
        args.add("url", "URL", m_args.url);
        args.add("bounds", "Bounds of the point cloud", m_args.bounds);
        args.add("depth", "Resolution of the point cloud", m_args.depth);
        args.add("threads", "Number of threads to be used."
                "This number will be squared", m_args.threads);
    }

    void I3SReader::addDimensions(PointLayoutPtr layout)
    {

        Json::StreamWriterBuilder writer;
        writer.settings_["indentation"] = "";

        for (int j = 0; j < m_info["layers"].size(); j++) 
        {
          
            Json::Value attributes = 
                m_info["layers"][j]["attributeStorageInfo"];

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
            m_info["layers"][0]["spatialReference"];
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
        -3Dscenelayerinfo: URL Pattern <scene-server-url/layers/<layer-id>
        -node index document: <layer-url >/nodepages/<root-node>
        -shared resources: <node-url>/shared/
        -feature data: <node-url>/features/<feature-data-bundle-id>
        -geometry data: <node-url>/geometries/<geometry-data-bundle-id>
        -texture data: <node-url>/textures/<texture-data-bundle-id>
        */
        
        std::string nodeUrl = m_args.url + "/layers/0/nodepages/";
        int nodePageIndex = 0;
        /*Retrive initial node page*/
        Json::Value nodeIndexJson =
            parse(m_arbiter->get(nodeUrl + std::to_string(nodePageIndex)));
        int index = 0;
        std::vector<int> nodePullArr;
        int totalCount = 0;
        int pageIndex = 0;
        int vertexCount = 0;
        while(!nodeIndexJson.isMember("error"))
        {
            int pageSize = nodeIndexJson["nodes"].size();
            int initialNode = nodeIndexJson["nodes"][0]["resourceId"].asInt();
            nodePullArr.resize(initialNode + pageSize);
            for(int i = 0; i < pageSize; i++)
            {
                
                int localCount = 
                    nodeIndexJson["nodes"][i]["vertexCount"].asInt();
                if(nodeIndexJson["nodes"][i]["childCount"].asInt() == 0)
                {
                    nodePullArr[index++] = initialNode + i;
                    vertexCount = vertexCount + localCount; 
                }

            }
            nodeIndexJson = 
                parse(m_arbiter->get(nodeUrl+std::to_string(++pageIndex)));
        }
        std::cout << "Fetching binaries" << std::endl;
        Pool p(m_args.threads);
        for(int i = 0; i < index; i++) 
        {
            std::cout << "\r" << i << "/" << index;
            std::cout.flush();
            std::string localUrl = m_args.url + 
                "/layers/0/nodes/" + std::to_string(nodePullArr[i]); 


            p.add([localUrl, this, &view]()
            {
                createView(localUrl, view);
            });
        }
        std::cout << "\nFinal point count: " << view->size() << std::endl;
        const std::size_t pointSize(view->layout()->pointSize());
        return 0;
    }

    void I3SReader::createView(std::string localUrl, PointViewPtr view)
    {
            Pool p(m_args.threads); 
            std::vector<char> response;
            p.add([this, &response, localUrl](){    
                /*Retrieve geometry binary*/
                std::string urlGeo = localUrl + "/geometries/0";
                //TODO add try catch block on arbiter(what if inet goes down)
                response = m_arbiter->getBinary(urlGeo);
            });

            std::vector<char> intensityResponse;
            p.add([this, &intensityResponse, localUrl](){
                /*Retrieve intensity data*/
                std::string urlIntensity = localUrl + "/attributes/2";
                intensityResponse = 
                    m_arbiter->getBinary(urlIntensity);
            });
            /*Retrieve rgb data*/


            std::vector<char> rgbResponse;
            p.add([this, &rgbResponse, localUrl]()
            {    
                std::string urlRgb = localUrl + "/attributes/4"; 
                rgbResponse = m_arbiter->getBinary(urlRgb);
            });
            /*Retrieve Class Code*/
            std::vector<char> classFlags; 
            
            p.add([this, &classFlags, localUrl]()
            {    
                std::string urlClass = localUrl + "/attributes/8";
                classFlags = m_arbiter->getBinary(urlClass);
            });
            /*Flags*/
            std::vector<char> flags;
            p.add([this, &flags, localUrl]()
            {
                std::string urlFlags = localUrl + "/attributes/16";
                flags = m_arbiter->getBinary(urlFlags);
            });
            /*Returns*/
            std::vector<char> returns;
            p.add([this, &returns, localUrl]()
            {
                std::string urlReturns = localUrl + "/attributes/32";
                returns = m_arbiter->getBinary(urlReturns);
            });

            p.await();
            /*Decompression methods*/
            std::vector<lepcc::Point3D> pointcloud = 
                decompressXYZ(&response);

            std::vector<uint16_t> intensity = 
                decompressIntensity(&intensityResponse);

            std::vector<lepcc::RGB_t> rgbPoints = 
                decompressRGB(&rgbResponse);
            

            std::lock_guard<std::mutex> lock(m_mutex);
            
            /*Iterate through vector item and add to view*/
            for(std::size_t j = 0; j < pointcloud.size(); j ++)
            {  
                double x = pointcloud[j].x;
                double y = pointcloud[j].y;
                double z = pointcloud[j].z;
                if((m_bounds.is3d() && m_bounds.to3d().contains(x,y,z))
                || (!m_bounds.is3d() && m_bounds.to2d().contains(x,y))
                || !m_args.bounds.size())
                {

                    PointId id = view->size();

                    //XYZ
                    view->setField(pdal::Dimension::Id::X,
                            id, pointcloud[j].x);
                    view->setField(pdal::Dimension::Id::Y,
                            id, pointcloud[j].y);
                    view->setField(pdal::Dimension::Id::Z,
                            id, pointcloud[j].z);

                    //RGB
                    view->setField(pdal::Dimension::Id::Red,
                            id, rgbPoints[j].r);
                    view->setField(pdal::Dimension::Id::Green,
                            id, rgbPoints[j].g);
                    view->setField(pdal::Dimension::Id::Blue,
                            id, rgbPoints[j].b);

                    //INTENSITY
                    view->setField(pdal::Dimension::Id::Intensity,
                            id, intensity[j]);

                    //CLASSCODES
                    view->setField(pdal::Dimension::Id::ClassFlags,
                            id, classFlags[j]);

                    //FLAGS
                    view->setField(pdal::Dimension::Id::Flag,
                            id, flags[j]);

                    //RETURNS
                    view->setField(pdal::Dimension::Id::NumberOfReturns,
                            id, returns[j]);
                }
            }

    }


    void I3SReader::done(PointTableRef)
    {
      m_stream.reset();
    }
} //namespace pdal
