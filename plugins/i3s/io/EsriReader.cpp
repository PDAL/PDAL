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

#include "EsriReader.hpp"
#include "../lepcc/src/include/lepcc_c_api.h"
#include "../lepcc/src/include/lepcc_types.h"
#include "pool.hpp"
#include "SlpkExtractor.hpp"

#include <istream>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/compression/LazPerfCompression.hpp>
#include <gdal.h>

namespace pdal
{
    void EsriReader::addArgs(ProgramArgs& args)
    {
        args.add("bounds", "Bounds of the point cloud", m_args.bounds);
        args.add("threads", "Number of threads to be used."
                "This number will be squared", m_args.threads);
    }

    void EsriReader::initialize(PointTableRef table)
    {
        Json::Value config;
        if (log()->getLevel() > LogLevel::Debug4)
            config["arbiter"]["verbose"] = true;
        m_arbiter.reset(new arbiter::Arbiter(config));

        const std::string pre("i3s://");
        if (Utils::startsWith(m_filename, pre))
            m_filename = m_filename.substr(pre.size());

        if (m_filename.back() == '/')
            m_filename.pop_back();

        log()->get(LogLevel::Debug) << "Fetching info from " << m_filename <<
            std::endl;

        try
        {
            initInfo();
        }
        catch (std::exception& e)
        {
            throw pdal_error(std::string("Failed to fetch info: ") + e.what());
        }

        //create pdal Bounds
        m_bounds = createBounds();
        //find number of nodes per nodepage
        m_nodeCap = m_info["store"]["index"]["nodesPerPage"].asInt();

        log()->get(LogLevel::Debug) << "\n\nFileName: "
            << m_filename << std::endl;
        log()->get(LogLevel::Debug) << "\nThread Count: "
            << m_args.threads << std::endl;
        log()->get(LogLevel::Debug) << "Bounds: "
            << m_args.bounds << std::endl;
    }

    void EsriReader::addDimensions(PointLayoutPtr layout)
    {
        Json::Value attributes = m_info["attributeStorageInfo"];

        for (Json::ArrayIndex i = 0; i < attributes.size(); i++)
        {
            std::string readName = attributes[i]["name"].asString();
            std::string id = attributes[i]["key"].asString();

            //remove quotes from json object
            readName.erase(
                std::remove(readName.begin(), readName.end(), '\"'),
                readName.end());

            //remove random underscores
            readName.erase(
                std::remove(readName.begin(), readName.end(), '_'),
                readName.end());

            if (readName == "INTENSITY")
            {
                isIntensity = true;
                idIntensity = id;
                layout->registerDim(Dimension::Id::Intensity);
            }
            else if (readName.compare("RGB") == 0)
            {
                isRGB = true;
                idRGB = id;
                layout->registerDim(Dimension::Id::Red);
                layout->registerDim(Dimension::Id::Green);
                layout->registerDim(Dimension::Id::Blue);
            }
            else if (readName.compare("FLAGS") == 0)
            {
                isFlags = true;
                idFlags = id;
                layout->registerDim(Dimension::Id::Flag);
            }
            else if (readName.compare("RETURNS") == 0)
            {
                isReturns = true;
                idReturns = id;
                layout->registerDim(Dimension::Id::NumberOfReturns);
            }
            else if (readName.compare("ELEVATION") == 0)
            {
                isElevation = true;
                idElevation = id;
                layout->registerDim(Dimension::Id::X);
                layout->registerDim(Dimension::Id::Y);
                layout->registerDim(Dimension::Id::Z);
            }
            else if (readName.compare("CLASSCODE") == 0)
            {
                idClass = id;
                isClass = true;
                layout->registerDim(Dimension::Id::ClassFlags);
            }
            else if (readName.compare("POINTSRCID") == 0)
            {
                idSourceId = id;
                isSourceId = true;
                layout->registerDim(Dimension::Id::PointSourceId);
            }
            else if (attributes[i].isMember("attributeValues"))
            {
                std::string s =
                    attributes[i]["attributeValues"]["valueType"].asString();
                layout->registerOrAssignDim(readName, pdal::Dimension::type(s));
            }
            else
            {
                layout->registerOrAssignDim(readName, Dimension::Type::Double);
            }
        }
    }

    void EsriReader::ready(PointTableRef)
    {
        Json::Value spatialJson =
            m_info["spatialReference"];
        std::string spatialStr =
            "EPSG:" + spatialJson["wkid"].asString();
        SpatialReference ref(spatialStr);
        m_i3sRef = ref;
        setSpatialReference(m_i3sRef);
    }


    point_count_t EsriReader::read(PointViewPtr view, point_count_t count)
    {
        /*
        -3Dscenelayerinfo: URL Pattern <scene-server-url/layers/<layer-id>
        -node index document: <layer-url >/nodepages/<iterative node page id>
        -shared resources: <node-url>/shared/
        -feature data: <node-url>/features/<feature-data-bundle-id>
        -geometry data: <node-url>/geometries/<geometry-data-bundle-id>
        -texture data: <node-url>/textures/<texture-data-bundle-id>
        */

        //Build the node list that will tell us which nodes overlap with bounds
        std::vector<int> nodes;
        buildNodeList(nodes, 0);

        //Create view with overlapping leaf nodes
        //Will create a thread pool on the createview class and iterate
        //through the node list for the nodes to be pulled.
        std::cout << "Fetching binaries" << std::endl;
        Pool p(m_args.threads);
        for (std::size_t i = 0; i < nodes.size(); i++)
        {
            std::cout << "\r" << i << "/" << nodes.size();
            std::cout.flush();
            std::string localUrl;

            localUrl = m_filename + "/nodes/" + std::to_string(nodes[i]);

            p.add([localUrl, this, &view]()
            {
                createView(localUrl, view);
            });
        }
        const std::size_t pointSize(view->layout()->pointSize());
        return view->size();
    }

    // Create the BOX3D for the node. This will be used for testing overlap.
    BOX3D EsriReader::parseBox(Json::Value base)
    {
        Json::Value center = base["obb"]["center"];
        Json::Value hsize = base["obb"]["halfSize"];
        Json::Value quat = base["obb"]["quaternion"];

        //pull the data from the json object
        double x = center[0].asDouble();
        double y = center[1].asDouble();
        const double z = center[2].asDouble();

        const double hx = hsize[0].asDouble();
        const double hy = hsize[1].asDouble();
        const double hz = hsize[2].asDouble();

        const double w = quat[0].asDouble();
        const double i = quat[1].asDouble();
        const double j = quat[2].asDouble();
        const double k = quat[3].asDouble();

        // Create quat object and normalize it for use
        Eigen::Quaterniond q(w, i, j, k);
        q.normalize();

        // Here we'll convert our halfsizes to x, y, and z vectors in respective
        // directions, which will give us new bounding planes
        Eigen::Vector3d vxmax(hx,   0,   0);
        Eigen::Vector3d vymax(0,   hy,   0);
        Eigen::Vector3d vzmax(0,    0,  hz);

        // Create quaternion-like vectors
        Eigen::Quaterniond pxmax, pxmin, pymax, pymin, pzmax, pzmin;
        pxmax.w() = 0;
        pymax.w() = 0;
        pzmax.w() = 0;
        pxmax.vec() = vxmax;
        pymax.vec() = vymax;
        pzmax.vec() = vzmax;

        // Rotate all the individual vectors
        // gives us offset for the of the new x/y/zmax/min planes
        Eigen::Quaterniond rxmax = q * pxmax * q.inverse();
        Eigen::Quaterniond rymax = q * pymax * q.inverse();
        Eigen::Quaterniond rzmax = q * pzmax * q.inverse();

        double mx = std::max(
                rxmax.vec()[0],
                std::max(
                    std::fabs(rymax.vec()[0])),
                    std::fabs(rzmax.vec()[0]));

        double my = std::max(
                rymax.vec()[1],
                std::max(
                    std::fabs(rymax.vec()[1])),
                    std::fabs(rzmax.vec()[1]));

        double mz = std::max(
                rymax.vec()[2],
                std::max(
                    std::fabs(rymax.vec()[2])),
                    std::fabs(rzmax.vec()[2]));



        // Convert to EPSG:4978 so the offsets(meters) will match up
        m_srsIn.set(m_i3sRef.getWKT());
        m_srsOut.set("EPSG:4978");
        m_out_ref_ptr = OSRNewSpatialReference(m_srsOut.getWKT().c_str());
        setSpatialReference(m_srsOut);
        m_in_ref_ptr = OSRNewSpatialReference(m_srsIn.getWKT().c_str());
        m_transform_ptr = OCTNewCoordinateTransformation(m_in_ref_ptr,
            m_out_ref_ptr);

        // create fake z for the transformation so we get values at the surface
        double newz = 0;
        OCTTransform(m_transform_ptr, 1, &x, &y, &newz);

        // Create new bounding planes in 4978
        double maxx = x + mx;
        double minx = x - mx;
        double maxy = y + my;
        double miny = y - my;
        double maxz = z + mz;
        double minz = z - mz;

        // Transform back to original spatial reference
        m_srsIn.set("EPSG:4978");
        m_srsOut.set(m_i3sRef.getWKT());

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



        //Create another tempz so we can apply this transformation twice
        double tempz = newz;
        OCTTransform(m_transform_ptr, 1, &minx, &miny, &tempz);
        OCTTransform(m_transform_ptr, 1, &maxx, &maxy, &newz);

        return BOX3D(minx, miny, minz, maxx, maxy, maxz);
    }

    void EsriReader::createView(std::string localUrl, PointViewPtr view)
    {
        const std::string geomUrl = localUrl + "/geometries/";
        auto xyz = fetchBinary(geomUrl, "0", ".bin.pccxyz");

        std::vector<char> intensityResponse;
        std::vector<char> rgbResponse;
        std::vector<char> classFlags;
        std::vector<char> flags;
        std::vector<char> returns;
        std::vector<uint16_t> pointSrcId;

        const std::string attrUrl = localUrl + "/attributes/";

        if (isIntensity)
        {
           intensityResponse = fetchBinary(attrUrl, idIntensity,
                   ".bin.pccint");
        }
        if (isRGB)
        {
           rgbResponse = fetchBinary(attrUrl, idRGB, ".bin.pccrgb");
        }
        if (isClass)
        {
           classFlags = fetchBinary(attrUrl, idClass, ".bin.gz");
        }
        if (isFlags)
        {
           flags = fetchBinary(attrUrl, idFlags, ".bin.gz");
        }
        if (isReturns)
        {
            returns = fetchBinary(attrUrl, idReturns, ".bin.gz");
        }
        /*if (isSourceId)
        {
            //fetchBinary(pointSrcId, localUrl, idSourceId, ".bin.gz");
            std::string fetchUrl = localUrl + "/attributes/" + idSourceId;
            if (m_file)
            {
                auto compressed =
                    m_arbiter->getBinary(fetchUrl + "bin.gz");
                m_decomp.decompress<std::vector<uint16_t>>(
                        pointSrcId, compressed.data(),
                    compressed.size());
            }else
            {
                pointSrcId = m_arbiter->getBinary(fetchUrl);
            }
        }*/

        // Decompression methods
        std::vector<lepcc::Point3D> pointcloud = decompressXYZ(&xyz);
        std::vector<uint16_t> intensity =
            decompressIntensity(&intensityResponse);
        std::vector<lepcc::RGB_t> rgbPoints = decompressRGB(&rgbResponse);

        std::lock_guard<std::mutex> lock(m_mutex);

        //Iterate through vector item and add to view
        for (std::size_t j = 0; j < pointcloud.size(); j++)
        {
            double x = pointcloud[j].x;
            double y = pointcloud[j].y;
            double z = pointcloud[j].z;
            if (m_bounds.contains(x, y, z))
            {
                PointId id = view->size();

                view->setField(pdal::Dimension::Id::X,
                        id, pointcloud[j].x);
                view->setField(pdal::Dimension::Id::Y,
                        id, pointcloud[j].y);
                view->setField(pdal::Dimension::Id::Z,
                        id, pointcloud[j].z);

                if (isRGB)
                {
                    view->setField(pdal::Dimension::Id::Red,
                            id, rgbPoints[j].r);
                    view->setField(pdal::Dimension::Id::Green,
                            id, rgbPoints[j].g);
                    view->setField(pdal::Dimension::Id::Blue,
                            id, rgbPoints[j].b);
                }
                if (isIntensity)
                {
                    view->setField(pdal::Dimension::Id::Intensity,
                            id, intensity[j]);
                }
                if (isClass)
                {
                    view->setField(pdal::Dimension::Id::ClassFlags,
                            id, classFlags[j]);
                }
                if (isFlags)
                {
                    view->setField(pdal::Dimension::Id::Flag,
                            id, flags[j]);
                }
                if (isReturns)
                {
                    view->setField(pdal::Dimension::Id::NumberOfReturns,
                            id, returns[j]);
                }
                /*if (isSourceId)
                {
                    view->setField(pdal::Dimension::Id::PointSourceId,
                            id, pointSrcId[j]);
                }
                */
            }
        }
    }

    //Create bounding box that the user specified
    BOX3D EsriReader::createBounds()
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

    void EsriReader::done(PointTableRef)
    {
      m_stream.reset();
    }
} //namespace pdal
