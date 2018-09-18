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
#include <gdal.h>



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

        /*Check for i3s protocol and remove */
        const std::string pre("i3s://");
        if (m_filename.find(pre) == 0)
            m_filename = m_filename.substr(pre.size());

        // remove tailing '/'
        if (m_filename.back() == '/')
            m_filename.pop_back();

        log()->get(LogLevel::Debug) << "Fetching info from " << m_filename <<
            std::endl;

        try
        {
            m_info = parse(m_arbiter->get(m_filename));
        }
        catch (std::exception& e)
        {
            throw pdal_error(std::string("Failed to fetch info: ") + e.what());
        }
        //create pdal Bounds
        m_bounds = createBounds();
        m_nodeCap = 
            m_info["layers"][0]["store"]["index"]["nodesPerPage"].asInt();

        log()->get(LogLevel::Debug) << "\n\nFileName: " 
            << m_filename << std::endl;
        log()->get(LogLevel::Debug) << "\nThread Count: " 
            << m_args.threads << std::endl;
        log()->get(LogLevel::Debug) << "Bounds: " 
            << m_args.bounds << std::endl;
    }


    void I3SReader::addArgs(ProgramArgs& args)
    {
        args.add("bounds", "Bounds of the point cloud", m_args.bounds);
        args.add("threads", "Number of threads to be used."
                "This number will be squared", m_args.threads);
    }

    void I3SReader::addDimensions(PointLayoutPtr layout)
    {

        Json::StreamWriterBuilder writer;
        writer.settings_["indentation"] = "";

        for (uint64_t j = 0; j < m_info["layers"].size(); j++) 
        {
          
            Json::Value attributes = 
                m_info["layers"][static_cast<Json::ArrayIndex>(j)]["attributeStorageInfo"];
            for (uint64_t i = 0; i < attributes.size(); i++)
            {

                std::string readName = 
                    Json::writeString(writer, attributes
                            [static_cast<Json::ArrayIndex>(i)]["name"]);
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

                if(readName == "INTENSITY")
                {
                    isIntensity = true;
                    layout->registerDim(Dimension::Id::Intensity);
                }else if (readName.compare("RGB") == 0)
                {
                    isRGB = true;
                    layout->registerDim(Dimension::Id::Red);
                    layout->registerDim(Dimension::Id::Green);
                    layout->registerDim(Dimension::Id::Blue);
                }
                else if (readName.compare("FLAGS") == 0)
                {
                    isFlags = true;
                    layout->registerDim(Dimension::Id::Flag);
                }
                else if (readName.compare("RETURNS") == 0)
                {
                    isReturns = true;
                    layout->registerDim(Dimension::Id::NumberOfReturns);
                }
                else if (readName.compare("ELEVATION") == 0)
                {
                    isElevation = true;
                    layout->registerDim(Dimension::Id::X);
                    layout->registerDim(Dimension::Id::Y);
                    layout->registerDim(Dimension::Id::Z);
                }
                else if (readName.compare("CLASSCODE") == 0)
                {
                    isClass = true;
                    layout->registerDim(Dimension::Id::ClassFlags);
                }
                else if (readName.compare("POINTSRCID") == 0)
                {
                    isSourceId = true;
                    layout->registerDim(Dimension::Id::PointSourceId);
                }
                else//if it isn't in pdal dimensions and isn't a special case
                {
                    std::string type;
                    if (attributes[static_cast<Json::ArrayIndex>(i)].isMember(
                                "attributeValues")) 
                    {
                        std::string type;
                        type = Json::writeString(writer,
                            attributes[static_cast<Json::ArrayIndex>(i)]
                            ["attributeValues"]["valueType"]);
                        layout->registerOrAssignDim(readName, 
                                pdal::Dimension::type(type));
                    }
                    else
                    {
                        layout->registerOrAssignDim(readName, 
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
        
        std::string nodeUrl = m_filename + "/layers/0/nodepages/";
        int nodePageIndex = 0;
        /*Retrive initial node page*/
        Json::Value nodeIndexJson =
            parse(m_arbiter->get(nodeUrl + std::to_string(nodePageIndex)));
        int index = 0;
        std::vector<int> nodeArr;
        int totalCount = 0;
        int pageIndex = 0;
        int vertexCount = 0;
        //tree traversal
        while (!nodeIndexJson.isMember("error"))
        {
            int pageSize = nodeIndexJson["nodes"].size();
            int initialNode = nodeIndexJson["nodes"][0]["resourceId"].asInt();
            nodeArr.resize(initialNode + pageSize);
            for (int i = 0; i < pageSize; i++)
            {
                int localCount = 
                    nodeIndexJson["nodes"][i]["vertexCount"].asInt();
                BOX3D nodeBox = parseBox(nodeIndexJson["nodes"][i]);
                int cCount = nodeIndexJson["nodes"][i]["childCount"].asInt();

                bool overlap = m_bounds.overlaps(nodeBox);
                if (cCount == 0 && overlap)
                {
                    nodeArr[index++] = (initialNode + i);
                    vertexCount = vertexCount + localCount; 
                }

            }
            nodeIndexJson = 
                parse(m_arbiter->get(nodeUrl+std::to_string(++pageIndex)));
        }

        //Create view with overlapping leaf nodes
        std::cout << "Fetching binaries" << std::endl;
        Pool p(m_args.threads);
        //index = nodeArr.size();
        for (int i = 0; i < index; i++) 
        {
            std::cout << "\r" << i << "/" << index;
            std::cout.flush();
            std::string localUrl = m_filename + 
                "/layers/0/nodes/" + std::to_string(nodeArr[i]); 

            p.add([localUrl, this, &view]()
            {
                createView(localUrl, view);
            });
        }
        std::cout << "\nFinal point count: " << view->size() << std::endl;
        const std::size_t pointSize(view->layout()->pointSize());
        return view->size();
    }


    BOX3D I3SReader::parseBox(Json::Value base)
    {
        Json::Value center = base["obb"]["center"];
        Json::Value hsize = base["obb"]["halfSize"];
        Json::Value quat = base["obb"]["quaternion"];

        //pull the data from the json object
        double x, y, z;
        x = center[0].asDouble();
        y = center[1].asDouble();
        z = center[2].asDouble();

        double hx, hy, hz;
        hx = hsize[0].asDouble();
        hy = hsize[1].asDouble();
        hz = hsize[2].asDouble();

        double w, i, j, k;
        w = quat[0].asDouble();
        i = quat[1].asDouble();
        j = quat[2].asDouble();
        k = quat[3].asDouble();

        //Calculations
        Eigen::Quaterniond q(w, i, j, k);
        q.normalize();

        //Here we'll convert our halfsizes to x, y, and z vectors in respective
        //directions, which will give us new bounding planes
        Eigen::Vector3d vxmax(hx,   0,   0);
        Eigen::Vector3d vymax(0,   hy,   0);
        Eigen::Vector3d vzmax(0,    0,  hz);

        //Create quaternion-like vectors
        Eigen::Quaterniond pxmax, pxmin, pymax, pymin, pzmax, pzmin;
        pxmax.w() = 0;
        pymax.w() = 0;
        pzmax.w() = 0;
        pxmax.vec() = vxmax;
        pymax.vec() = vymax;
        pzmax.vec() = vzmax;

        //Rotate all the individual vectors
        //gives us offset for the of the new x/y/zmax/min planes
        Eigen::Quaterniond rxmax = q * pxmax * q.inverse(); 
        Eigen::Quaterniond rymax = q * pymax * q.inverse(); 
        Eigen::Quaterniond rzmax = q * pzmax * q.inverse(); 


        m_srsIn.set("EPSG:4326");
        m_srsOut.set("EPSG:4978");
        m_out_ref_ptr = OSRNewSpatialReference(m_srsOut.getWKT().c_str());
        setSpatialReference(m_srsOut);
        m_in_ref_ptr = OSRNewSpatialReference(m_srsIn.getWKT().c_str());
        m_transform_ptr = OCTNewCoordinateTransformation(m_in_ref_ptr,
            m_out_ref_ptr);
        double newz = 0;
        OCTTransform(m_transform_ptr, 1, &x, &y, &newz);

        //Create new bounding planes
        double maxx = (rxmax.vec()[0] < 0) 
            ? (x-rxmax.vec()[0]):(x+rxmax.vec()[0]);
        double minx = (rxmax.vec()[0] > 0) 
            ? (x-rxmax.vec()[0]):(x+rxmax.vec()[0]);
        double maxy = (rymax.vec()[1] < 0) 
            ? (y-rymax.vec()[1]):(y+rymax.vec()[1]);
        double miny = (rymax.vec()[1] > 0) 
            ? (y-rymax.vec()[1]):(y+rymax.vec()[1]);
        double maxz = (rzmax.vec()[2] < 0) 
            ? (z-rzmax.vec()[2]):(z+rzmax.vec()[2]);
        double minz = (rzmax.vec()[2] > 0) 
            ? (z-rzmax.vec()[2]):(z+rzmax.vec()[2]);

        m_srsIn.set("EPSG:4978");
        m_srsOut.set("EPSG:4326");

        if (m_transform_ptr)
            OCTDestroyCoordinateTransformation(m_transform_ptr);
        if (m_in_ref_ptr)
            OSRDestroySpatialReference(m_in_ref_ptr);
        if (m_out_ref_ptr)
            OSRDestroySpatialReference(m_out_ref_ptr);
        m_out_ref_ptr = OSRNewSpatialReference(m_srsOut.getWKT().c_str());
        m_in_ref_ptr = OSRNewSpatialReference(m_srsIn.getWKT().c_str());
        setSpatialReference(m_srsOut);
        m_transform_ptr = OCTNewCoordinateTransformation(m_in_ref_ptr,
                m_out_ref_ptr);

        double tempz = newz;

        OCTTransform(m_transform_ptr, 1, &minx, &miny, &tempz);

        OCTTransform(m_transform_ptr, 1, &maxx, &maxy, &newz);

        BOX3D returnBox(minx, miny, minz, maxx, maxy, maxz);
                        
        //populate BOX3D

        return returnBox;
    }

    void I3SReader::createView(std::string localUrl, PointViewPtr view)
    {
            Pool p(m_args.threads); 
            std::vector<char> response;
            p.add([this, &response, localUrl]( )
            {    
                std::string fetchUrl = localUrl + "/geometries/0";
                response = m_arbiter->getBinary(fetchUrl);
            });


            std::vector<char> intensityResponse;
            std::vector<char> rgbResponse;
            std::vector<char> classFlags; 
            std::vector<char> flags;
            std::vector<char> returns;
            std::vector<char> pointSrcId;
            if(isIntensity)
            {
               fetchBinary(intensityResponse, localUrl, 2, p); 
            }
            if(isRGB)
            {
               fetchBinary(rgbResponse, localUrl, 4, p); 
            }
            if(isClass)
            {
               fetchBinary(classFlags, localUrl, 8, p); 
            }
            if(isFlags)
            {
               fetchBinary(flags, localUrl, 16, p); 
            }
            if(isReturns)
            {
                fetchBinary(returns, localUrl, 32, p);
            }
            if(isSourceId)
            {
                fetchBinary(pointSrcId, localUrl, 64, p);
            }

            p.await();
            //Decompression methods
            std::vector<lepcc::Point3D> pointcloud = 
                decompressXYZ(&response);

            std::vector<uint16_t> intensity = 
                decompressIntensity(&intensityResponse);

            std::vector<lepcc::RGB_t> rgbPoints = 
                decompressRGB(&rgbResponse);
            

            std::lock_guard<std::mutex> lock(m_mutex);
            //Iterate through vector item and add to view

            m_count = 0;
            for (std::size_t j = 0; j < pointcloud.size(); j ++)
            {  
                double x = pointcloud[j].x;
                double y = pointcloud[j].y;
                double z = pointcloud[j].z;
                if (m_bounds.contains(x,y,z))
                {
                    m_count ++;
                    PointId id = view->size();
                    
                    //XYZ
                    view->setField(pdal::Dimension::Id::X,
                            id, pointcloud[j].x);
                    view->setField(pdal::Dimension::Id::Y,
                            id, pointcloud[j].y);
                    view->setField(pdal::Dimension::Id::Z,
                            id, pointcloud[j].z);

                    if(isRGB){
                        //RGB
                        view->setField(pdal::Dimension::Id::Red,
                                id, rgbPoints[j].r);
                        view->setField(pdal::Dimension::Id::Green,
                                id, rgbPoints[j].g);
                        view->setField(pdal::Dimension::Id::Blue,
                                id, rgbPoints[j].b);
                    }
                    if (isIntensity)
                    {
                        //INTENSITY
                        view->setField(pdal::Dimension::Id::Intensity,
                                id, intensity[j]);
                    }
                    if (isClass)
                    {
                        //CLASSCODES
                        view->setField(pdal::Dimension::Id::ClassFlags,
                                id, classFlags[j]);
                    }
                    if (isFlags)
                    {
                        //FLAGS
                        view->setField(pdal::Dimension::Id::Flag,
                                id, flags[j]);
                    }
                    if (isReturns)
                    {
                        //RETURNS
                        view->setField(pdal::Dimension::Id::NumberOfReturns,
                                id, returns[j]);
                    }
                    if(isSourceId)
                    {
                        view->setField(pdal::Dimension::Id::PointSourceId,
                                id, pointSrcId[j]);
                    }
                }
            }
    }


    BOX3D I3SReader::createBounds()
    {
        if (m_args.bounds.is3d())
            return m_args.bounds.to3d();

        // If empty, return maximal extents to select everything.
        const BOX2D b(m_args.bounds.to2d());
        if (b.empty())
        {
            double mn((std::numeric_limits<double>::lowest)());
            double mx((std::numeric_limits<double>::max)());
            return BOX3D(mn, mn, mn, mx, mx, mx);
        }

        // Finally if 2D and non-empty, convert to 3D with all-encapsulating
        // Z-values.
        return BOX3D(
                b.minx, b.miny, (std::numeric_limits<double>::lowest)(),
                b.maxx, b.maxy, (std::numeric_limits<double>::max)());
    }

    void I3SReader::fetchBinary(std::vector<char>& response, std::string url, int attNum, Pool& p)
    {
        //Add to thread pool and fetch the binary data form arbiter
        p.add([this, &response, url, attNum]( )
        {    
            std::string fetchUrl = url + "/attributes/" + std::to_string(attNum); 
            response = m_arbiter->getBinary(fetchUrl);
        });
    }

    void I3SReader::done(PointTableRef)
    {
      m_stream.reset();
    }
} //namespace pdal
