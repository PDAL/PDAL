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
        layout->registerDim(Dimension::Id::X);
        layout->registerDim(Dimension::Id::Y);
        layout->registerDim(Dimension::Id::Z);
        for (Json::ArrayIndex i = 0; i < attributes.size(); i++)
        {
            dimData data;
            std::string readName = attributes[i]["name"].asString();
            int key = std::stoi(attributes[i]["key"].asString());
            data.key = key;

            if(attributes[i].isMember("valueType"))
            {
                std::string type = attributes[i]["valueType"].asString();
                data.dataType = type;
            }

            else if (readName == "RGB")
            {
                layout->registerDim(Dimension::Id::Red);
                layout->registerDim(Dimension::Id::Green);
                layout->registerDim(Dimension::Id::Blue);
                // Since RGB are always packed together and handled specially,
                // we'll use Red as our indicator that RGB exists.
                m_dimMap[Dimension::Id::Red] = data;
            }
            else if (m_dimensions.find(readName) != m_dimensions.end())
            {
                layout->registerDim(m_dimensions.at(readName));
                m_dimMap[m_dimensions.at(readName)] = data;

            }
            else if (attributes[i].isMember("attributeValues"))
            {
                std::string s =
                    attributes[i]["attributeValues"]["valueType"].asString();

                const pdal::Dimension::Id id = layout->registerOrAssignDim(
                        readName, pdal::Dimension::type(s));
                m_dimMap[id] = data;
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
        // Pull XYZ in lat/lon.
        Json::Value center = base["obb"]["center"];
        double x = center[0].asDouble();
        double y = center[1].asDouble();
        double z = center[2].asDouble();

        // Convert to EPSG:4978 so the offsets(meters) will match up
        m_srsIn.set(m_i3sRef.getWKT());
        m_srsOut.set("EPSG:4978");
        m_out_ref_ptr = OSRNewSpatialReference(m_srsOut.getWKT().c_str());
        setSpatialReference(m_srsOut);
        m_in_ref_ptr = OSRNewSpatialReference(m_srsIn.getWKT().c_str());
        m_transform_ptr = OCTNewCoordinateTransformation(m_in_ref_ptr,
            m_out_ref_ptr);

        // create fake z for the transformation so we get values at the surface
        OCTTransform(m_transform_ptr, 1, &x, &y, &z);

        // std::cout << x << " " << y << " " << z << std::endl;

        Json::Value hsize = base["obb"]["halfSize"];
        const double hx = hsize[0].asDouble();
        const double hy = hsize[1].asDouble();
        const double hz = hsize[2].asDouble();

        Json::Value quat = base["obb"]["quaternion"];
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

        double minx, miny, minz, maxx, maxy, maxz;
        minx = miny = minz = std::numeric_limits<double>::max();
        maxx = maxy = maxz = std::numeric_limits<double>::lowest();


        for (std::size_t i(0); i < 8; ++i)
        {
            double a(x), b(y), c(z);

            a += rxmax.vec()[0] * (i & 1 ? 1.0 : -1.0);
            b += rxmax.vec()[1] * (i & 1 ? 1.0 : -1.0);
            c += rxmax.vec()[2] * (i & 1 ? 1.0 : -1.0);

            a += rymax.vec()[0] * (i & 2 ? 1.0 : -1.0);
            b += rymax.vec()[1] * (i & 2 ? 1.0 : -1.0);
            c += rymax.vec()[2] * (i & 2 ? 1.0 : -1.0);

            a += rzmax.vec()[0] * (i & 4 ? 1.0 : -1.0);
            b += rzmax.vec()[1] * (i & 4 ? 1.0 : -1.0);
            c += rzmax.vec()[2] * (i & 4 ? 1.0 : -1.0);

            m_srsIn.set("EPSG:4978");
            m_srsOut.set(m_i3sRef.getWKT());

            if (m_transform_ptr)
                OCTDestroyCoordinateTransformation(m_transform_ptr);
            if (m_in_ref_ptr)
                OSRDestroySpatialReference(m_in_ref_ptr);
            if (m_out_ref_ptr)
                OSRDestroySpatialReference(m_out_ref_ptr);
            m_out_ref_ptr = OSRNewSpatialReference(
                    m_srsOut.getWKT().c_str());
            m_in_ref_ptr = OSRNewSpatialReference(
                    m_srsIn.getWKT().c_str());
            setSpatialReference(m_srsOut);
            m_transform_ptr = OCTNewCoordinateTransformation(
                    m_in_ref_ptr, m_out_ref_ptr);

            OCTTransform(m_transform_ptr, 1, &a, &b, &c);

            minx = std::min(minx, a);
            miny = std::min(miny, b);
            minz = std::min(minz, c);
            maxx = std::max(maxx, a);
            maxy = std::max(maxy, b);
            maxz = std::max(maxz, c);
        }
        return BOX3D(minx, miny, minz, maxx, maxy, maxz);
    }

    void EsriReader::createView(std::string localUrl, PointViewPtr view)
    {
        //pull the geometries to start
        const std::string geomUrl = localUrl + "/geometries/";
        auto xyz = fetchBinary(geomUrl, "0", ".bin.pccxyz");
        std::vector<lepcc::Point3D> pointcloud = decompressXYZ(&xyz);

        std::vector<int> selected;
        uint64_t startId;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            startId = view->size();

            for (uint64_t j = 0; j < pointcloud.size(); ++j)
            {
                double x = pointcloud[j].x;
                double y = pointcloud[j].y;
                double z = pointcloud[j].z;

                if (m_bounds.contains(x, y, z))
                {
                    PointId id = view->size();
                    selected.push_back(j);
                    view->setField(pdal::Dimension::Id::X, id, pointcloud[j].x);
                    view->setField(pdal::Dimension::Id::Y, id, pointcloud[j].y);
                    view->setField(pdal::Dimension::Id::Z, id, pointcloud[j].z);
                }
            }
        }

        const std::string attrUrl = localUrl + "/attributes/";

        for (const auto& dimEntry : m_dimMap)
        {
            const Dimension::Id dimId(dimEntry.first);
            const uint64_t key(dimEntry.second.key);
            std::string dataType(dimEntry.second.dataType);

            if (dimId == Dimension::Id::Red)
            {
                auto data = fetchBinary(
                        attrUrl, std::to_string(key), ".bin.pccrgb");
                std::vector<lepcc::RGB_t> rgbPoints = decompressRGB(&data);

                std::lock_guard<std::mutex> lock(m_mutex);

                for (std::size_t i(0); i < selected.size(); ++i)
                {
                    view->setField(pdal::Dimension::Id::Red,
                            startId + i, rgbPoints[selected[i]].r);
                    view->setField(pdal::Dimension::Id::Green,
                            startId + i, rgbPoints[selected[i]].g);
                    view->setField(pdal::Dimension::Id::Blue,
                            startId + i, rgbPoints[selected[i]].b);
                }
            }
            else if (dimId == Dimension::Id::Intensity)
            {
                auto data = fetchBinary(attrUrl, std::to_string(key),
                        ".bin.pccint");

                std::vector<uint16_t> intensity = decompressIntensity(&data);

                std::lock_guard<std::mutex> lock(m_mutex);
                for (std::size_t i(0); i < selected.size(); ++i)
                {
                    view->setField(pdal::Dimension::Id::Intensity,
                            startId + i, intensity[selected[i]]);
                }
            }
            else
            {
                const auto data = fetchBinary(
                        attrUrl, std::to_string(key), ".bin.gz");

                std::lock_guard<std::mutex> lock(m_mutex);

                // TODO Correct types.
                if(dataType == "Uint8")
                    setAs<uint8_t>(dimId, data, selected, view, startId);
                else if(dataType == "Uint16")
                    setAs<uint16_t>(dimId, data, selected, view, startId);
                else if(dataType == "Uint32")
                    setAs<uint32_t>(dimId, data, selected, view, startId);
                else if(dataType == "Uint64")
                    setAs<uint64_t>(dimId, data, selected, view, startId);
                else if(dataType == "Int8")
                    setAs<int8_t>(dimId, data, selected, view, startId);
                else if(dataType == "Int16")
                    setAs<int16_t>(dimId, data, selected, view, startId);
                else if(dataType == "Int32")
                    setAs<int32_t>(dimId, data, selected, view, startId);
                else if(dataType == "Int64")
                    setAs<int64_t>(dimId, data, selected, view, startId);
                else if(dataType == "Double")
                    setAs<double>(dimId, data, selected, view, startId);
                else if(dataType == "Float64")
                    setAs<double>(dimId, data, selected, view, startId);
                else if(dataType == "Float32")
                    setAs<float>(dimId, data, selected, view, startId);
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
