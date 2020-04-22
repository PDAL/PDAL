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

#include <Eigen/Geometry>
#include <pdal/private/SrsTransform.hpp>

#include "../lepcc/src/include/lepcc_types.h"

#include "EsriUtil.hpp"
#include "pool.hpp"
#include "SlpkExtractor.hpp"


namespace pdal
{
EsriReader::EsriReader()
{}


EsriReader::~EsriReader()
{}


void EsriReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "Bounds of the point cloud", m_args.bounds);
    args.add("threads", "Number of threads to be used." , m_args.threads);
    args.add("dimensions", "Dimensions to be used in pulls", m_args.dimensions);
    args.add("min_density", "Minimum point density",
            m_args.min_density, -1.0);
    args.add("max_density", "Maximum point density",
            m_args.max_density, -1.0);
}


void EsriReader::initialize(PointTableRef table)
{
    //create proper density if min was set but max wasn't
    if (m_args.min_density >= 0 && m_args.max_density < 0)
        m_args.max_density = (std::numeric_limits<double>::max)();

    //create dimensions map for future lookup
    if (!m_args.dimensions.empty())
    {
        for (std::string& dim : m_args.dimensions)
        {
            dim = Utils::toupper(dim);
            if (esriDims.find(dim) != esriDims.end())
                m_dimensions[dim] = esriDims.at(dim);
            else
                m_dimensions[dim];
        }
    }

    m_arbiter.reset(new arbiter::Arbiter());

    //adjust filename string
    const std::string pre("i3s://");
    if (Utils::startsWith(m_filename, pre))
        m_filename = m_filename.substr(pre.size());

    if (m_filename.back() == '/')
        m_filename.pop_back();

    log()->get(LogLevel::Debug) << "Fetching info from " << m_filename <<
        std::endl;

    //personalize for slpk or i3s
    try
    {
        initInfo();
    }
    catch (std::exception& e)
    {
        throwError(std::string("Failed to fetch info: ") + e.what());
    }

    //create const for looking into
    const NL::json jsonBody = m_info;

    //find version
    if (jsonBody["store"].contains("version"))
        m_version = Version(jsonBody["store"]["version"].get<std::string>());

    if (Version("2.0") < m_version || m_version < Version("1.6"))
        log()->get(LogLevel::Warning) << "This version may not work with "
            "the current implementation of i3s/slpk reader" << std::endl;

    //find number of nodes per nodepage
    if (jsonBody["store"]["index"].contains("nodesPerPage"))
        m_nodeCap = jsonBody["store"]["index"]["nodesPerPage"].get<int>();
    else if (jsonBody["store"]["index"].contains("nodePerIndexBlock"))
        m_nodeCap = jsonBody["store"]["index"]["nodePerIndexBlock"].get<int>();
    else
    {
        log()->get(LogLevel::Warning) <<
            "Number of nodes per page not specified. Default is 64." <<
                std::endl;
        m_nodeCap = 64;
    }

    //find the type of encoding
    if (jsonBody["store"]["defaultGeometrySchema"].contains("encoding"))
    {
        std::string encoding = jsonBody["store"]
            ["defaultGeometrySchema"]["encoding"].get<std::string>();
        if (encoding != "lepcc-xyz")
            throwError(std::string("Only lepcc encoding is supported "
                "by this driver"));
    }

    //create spatial reference objects
    NL::json wkid = m_info["spatialReference"]["wkid"];
    int system(0);
    if (wkid.is_string())
    {
        std::string sval = wkid.get<std::string>();
        try
        {
            system = std::stoi(sval);
        }
        catch (...)
        {}
        if (system < 2000)
            throwError("Invalid wkid string '" + sval + "' for spatial "
                "reference.");
    }
    else if (wkid.is_number())
    {
        system = wkid.get<int64_t>();
        if (system < 2000)
            throwError("Invalid wkid value '" + std::to_string(system) +
                "' for spatial reference.");
    }
    std::string systemString("EPSG:" + std::to_string(system));
    m_nativeSrs.set(systemString);
    if (m_nativeSrs.empty())
        throwError("Unable to create spatial reference for i3s for '" +
            systemString + "'.");
    setSpatialReference(m_nativeSrs);

    m_ecefTransform.reset(new SrsTransform(m_nativeSrs, "EPSG:4978"));
    createBounds();
}


// We check to see if tiles (nodepages) intersect our bounding box by
// converting them both to ECEF.
void EsriReader::createBounds()
{
    const double mn((std::numeric_limits<double>::lowest)());
    const double mx((std::numeric_limits<double>::max)());
    if (m_args.bounds.is3d())
    {
        m_bounds = m_args.bounds.to3d();
        for (size_t i = 0; i < 8; ++i)
        {
            double a = (i & 1 ? m_bounds.minx: m_bounds.maxx);
            double b = (i & 2 ? m_bounds.miny: m_bounds.maxy);
            double c = (i & 4 ? m_bounds.minz: m_bounds.maxz);
            m_ecefTransform->transform(a, b, c);
            m_ecefBounds.grow(a, b, c);
        }
    }
    else
    {
        // No bounds specified.

        // Distance to the center of the earth is 6.3 million meters, so
        // 10 million should be a reasonable max for ECEF.
        BOX2D b = m_args.bounds.to2d();
        if (b.empty())
        {
            m_bounds = BOX3D(mn, mn, mn, mx, mx, mx);
            m_ecefBounds = BOX3D(-10e6, -10e6, -10e6, 10e6, 10e6, 10e6);
        }
        else
        {
            m_bounds = BOX3D(b); // Will set z values to 0.
            m_ecefBounds = m_bounds;
            m_ecefTransform->transform(m_ecefBounds.minx, m_ecefBounds.miny,
                m_ecefBounds.minz);
            m_ecefTransform->transform(m_ecefBounds.maxx, m_ecefBounds.maxy,
                m_ecefBounds.maxz);
            m_bounds.minz = mn;
            m_bounds.maxz = mx;
            m_ecefBounds.minz = -10e6;
            m_ecefBounds.maxz = 10e6;
        }
    }
    // Transformation can invert coordinates
    if (m_ecefBounds.minx > m_ecefBounds.maxx)
        std::swap(m_ecefBounds.minx, m_ecefBounds.maxx);
    if (m_ecefBounds.miny > m_ecefBounds.maxy)
        std::swap(m_ecefBounds.miny, m_ecefBounds.maxy);
    if (m_ecefBounds.minz > m_ecefBounds.maxz)
        std::swap(m_ecefBounds.minz, m_ecefBounds.maxz);
}


void EsriReader::addDimensions(PointLayoutPtr layout)
{
    if (!m_info.contains("attributeStorageInfo"))
        throwError("Attributes do not exist for this object");
    const NL::json& attributes = m_info["attributeStorageInfo"];
    //automatically add xyz point dimensions
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    for (auto el : attributes)
    {
        dimData data;
        std::string readName = el["name"].get<std::string>();
        data.name = readName;

        //test if this dimensions was requested by user
        if (!m_args.dimensions.empty() &&
            (m_dimensions.find(readName) == m_dimensions.end()))
            continue;

        data.key = std::stoi(el["key"].get<std::string>());

        if (!el.contains("attributeValues"))
        {
            //Expect that Elevation will be bundled with xyz
            if (readName != "ELEVATION")
                log()->get(LogLevel::Warning) <<
                    "Attribute does not have a type." <<
                    std::endl;
            continue;
        }

        data.dataType = el["attributeValues"]["valueType"].get<std::string>();

        if (readName == "RGB")
        {
            layout->registerDim(Dimension::Id::Red);
            layout->registerDim(Dimension::Id::Green);
            layout->registerDim(Dimension::Id::Blue);
            // Since RGB are always packed together and handled specially,
            // we'll use Red as our indicator that RGB exists.

            m_dimMap[Dimension::Id::Red] = data;
        }
        else if (readName == "RETURNS")
        {
            layout->registerDim(Dimension::Id::NumberOfReturns);
            layout->registerDim(Dimension::Id::ReturnNumber);
            //These are packed together. We'll refer to it as NumberOfReturns
            m_dimMap[Dimension::Id::NumberOfReturns] = data;
        }
        //Available dimension types can be found at https://git.io/fAbxS
        else if (esriDims.find(readName) != esriDims.end())
        {
            data.dimType = Dimension::Type::None;

            auto it = dimTypes.find(data.dataType);
            if (it != dimTypes.end())
                data.dimType = it->second;

            if (data.dimType != Dimension::Type::None)
            {
                layout->registerDim(esriDims.at(readName));
                m_dimMap[esriDims.at(readName)] = data;
            }
            else
            {
                log()->get(LogLevel::Warning) <<
                    "Could not find Dimension type for " <<
                    readName << std::endl;
            }
        }
        else
        {
            std::string s =
                el["attributeValues"]["valueType"].get<std::string>();
            const pdal::Dimension::Id id = layout->registerOrAssignDim(
                    readName, pdal::Dimension::type(s));
            if(data.dimType != Dimension::Type::None)
                m_dimMap[id] = data;
        }
    }
}


void EsriReader::ready(PointTableRef)
{
    //output arguments for debugging
    log()->get(LogLevel::Debug) << "filename: " <<
        m_filename << std::endl;
    log()->get(LogLevel::Debug) << "threads: " <<
        m_args.threads << std::endl;
    log()->get(LogLevel::Debug) << "bounds: " <<
        m_args.bounds << std::endl;
    log()->get(LogLevel::Debug) << "min_density: " <<
        m_args.min_density << std::endl;
    log()->get(LogLevel::Debug) << "max_density: " <<
        m_args.max_density << std::endl;
    log()->get(LogLevel::Debug) << "dimensions: " << std::endl;

    for (std::string& dim : m_args.dimensions)
        log()->get(LogLevel::Debug) << "    -" << dim <<std::endl;
}


point_count_t EsriReader::read(PointViewPtr view, point_count_t count)
{
    /*
    -3Dscenelayerinfo: <scene-server-url/layers/<layer-id>
    -node index document: <layer-url >/nodepages/<iterative node page id>
    -shared resources: <node-url>/shared/
    -feature data: <node-url>/features/<feature-data-bundle-id>
    -geometry data: <node-url>/geometries/<geometry-data-bundle-id>
    -texture data: <node-url>/textures/<texture-data-bundle-id>
    */

    // Build the node list that will tell us which nodes overlap with bounds
    std::vector<int> nodes;
    NL::json initJson = fetchJson(m_filename + "/nodepages/0");
    log()->get(LogLevel::Debug) << "Traversing metadata" << std::endl;
    traverseTree(initJson, 0, nodes, 0, 0);

    // Create view with overlapping nodes at desired depth
    // Will create a thread pool on the createview class and iterate
    // through the node list for the nodes to be pulled.
    log()->get(LogLevel::Debug) << "Fetching binaries" << std::endl;
    Pool p(m_args.threads);
    for (std::size_t i = 0; i < nodes.size(); i++)
    {
        log()->get(LogLevel::Debug) << "\r" << i << "/" << nodes.size();
        std::string localUrl;

        localUrl = m_filename + "/nodes/" + std::to_string(nodes[i]);
        int nodeIndex(nodes[i]);

        p.add([localUrl, nodeIndex, this, &view]()
        {
            createView(localUrl, nodeIndex, *view);
        });
    }
    p.await();
    return view->size();
}


// Traverse tree through nodepages. Create a nodebox for each node in
// the tree and test if it overlaps with the bounds created by user.
// If it's a leaf node(the highest resolution) and it overlaps, add
// it to the list of nodes to be pulled later.
void EsriReader::traverseTree(NL::json page, int index,
    std::vector<int>& nodes, int depth, int pageIndex)
{
    // find node information
    int firstNode = page["nodes"][0]["resourceId"].get<int>();
    int name = page["nodes"][index]["resourceId"].get<int>();
    int firstChild = page["nodes"][index]["firstChild"].get<int>();
    int cCount = page["nodes"][index]["childCount"].get<int>();

    // find density information
    double area = page["nodes"][index][
        m_version >= Version("2.0") ?
            "lodThreshold" :
            "effectiveArea" ].get<double>();
    int pCount = page["nodes"][index][
        m_version >= Version("2.0") ?
            "vertexCount" :
            "pointCount" ].get<int>();

    double density = pCount / area;

    // update maximum node to stop reading files at the right time
    if ((firstChild + cCount - 1) > m_maxNode)
    {
        m_maxNode = firstChild + cCount - 1;
    }


    BOX3D nodeBox = createCube(page["nodes"][index]);

    // We're always comparing ECEF rectangular solids.
    bool overlap = m_ecefBounds.overlaps(nodeBox);

    // if it doesn't overlap, then none of the nodes in this subtree will
    if (!overlap)
        return;


    // if it's a child node and we're fetching full density, add leaf nodes
    if (m_args.max_density == -1 && m_args.min_density == -1 && cCount == 0)
    {
        nodes.push_back(name);
        return;
    }
    else
    {
        if (density < m_args.max_density && density > m_args.min_density)
        {
            nodes.push_back(name);
        }
        // if we've already reached the last node, stop the process, otherwise
        // increment depth and begin looking at child nodes
        if (name == m_maxNode)
            return;

        ++depth;
        for (int i = 0; i < cCount; ++i)
        {
            if ((firstChild + i) > (firstNode + m_nodeCap - 1))
            {
                pageIndex =
                    (m_version >= Version("2.0") ?
                        (firstChild + i) / m_nodeCap :
                        ((firstChild + i) / m_nodeCap) * m_nodeCap);

                if (m_nodepages.find(pageIndex) != m_nodepages.end())
                    page = m_nodepages[pageIndex];
                else
                {
                    page = fetchJson(m_filename + "/nodepages/" +
                        std::to_string(pageIndex));
                    m_nodepages[pageIndex] = page;
                }
            }
            if (pageIndex != 0)
                index =
                    (m_version >= Version("2.0") ?
                        (firstChild + i) % (m_nodeCap * pageIndex):
                        (firstChild + i) % (pageIndex));
            else
                index = firstChild + i;
            traverseTree(page, index, nodes, depth, pageIndex);
        }
    }
}


//Finds a sphere from the given center and halfsize vector of the OBB
//and makes a cube around it. This should help with collision detection
//and pruning of nodes before fetching binaries.
BOX3D EsriReader::createCube(const NL::json& base)
{
    // Pull XYZ in lat/lon.
    const NL::json& center = base["obb"]["center"];
    double x = center[0].get<double>();
    double y = center[1].get<double>();
    double z = center[2].get<double>();

    // We believe that half-sizes are in meters.
    const NL::json& hsize = base["obb"]["halfSize"];
    double hx = hsize[0].get<double>();
    double hy = hsize[1].get<double>();
    double hz = hsize[2].get<double>();

    // transform (x,y,z) to ECEF to match the half sizes in meters.
    m_ecefTransform->transform(x, y, z);

    // ABELL - Not sure why we're multiplying by sqrt(2).  The rest of the
    // calculation is to get a radius of a sphere that encloses the OBB.
    double r = std::sqrt(2) *
        std::sqrt(std::pow(hx, 2) + std::pow(hy, 2) + std::pow(hz, 2));

    // Create cube around the sphere oriented as ECEF.
    return BOX3D(x - r, y - r, z - r, x + r, y + r, z + r);
}


void EsriReader::createView(std::string localUrl, int nodeIndex,
    PointView& view)
{
    // pull the geometries to start
    const std::string geomUrl = localUrl + "/geometries/";
    auto xyzFetch = fetchBinary(geomUrl, "0", ".bin.pccxyz");
    std::vector<lepcc::Point3D> xyz;
    try
    {
        xyz = EsriUtil::decompressXYZ(&xyzFetch);
    }
    catch (const EsriUtil::decompression_error& e)
    {
        throwError(e.what());
    }

    std::vector<point_count_t> selected;
    uint64_t startId;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        startId = view.size();


        for (uint64_t j = 0; j < xyz.size(); ++j)
        {
            double x = xyz[j].x;
            double y = xyz[j].y;
            double z = xyz[j].z;

            if (m_bounds.contains(x, y, z))
            {
                PointId id = view.size();
                selected.push_back(j);
                view.setField(pdal::Dimension::Id::X, id, xyz[j].x);
                view.setField(pdal::Dimension::Id::Y, id, xyz[j].y);
                view.setField(pdal::Dimension::Id::Z, id, xyz[j].z);
            }
        }
    }

    const std::string attrUrl = localUrl + "/attributes/";

    //the extensions seen in this part correspond with slpk
    for (const auto& dimEntry : m_dimMap)
    {
        const Dimension::Id dimId(dimEntry.first);
        const Dimension::Type dimType(dimEntry.second.dimType);
        const uint64_t key(dimEntry.second.key);
        const std::string name(dimEntry.second.name);

        if (dimId == Dimension::Id::Red)
        {
            auto data = fetchBinary(
                    attrUrl, std::to_string(key), ".bin.pccrgb");
            std::vector<lepcc::RGB_t> rgbPoints;
            try
            {
                rgbPoints = EsriUtil::decompressRGB(&data);
            }
            catch (const EsriUtil::decompression_error& e)
            {
                throwError(e.what());
            }

            std::size_t dimSize = Dimension::size(dimType);
            if (rgbPoints.size() != xyz.size())
            {
                throwError(std::string("Bad data fetch. Data id: " +
                            dimEntry.second.name));
            }

            std::lock_guard<std::mutex> lock(m_mutex);
            for (std::size_t i(0); i < selected.size(); ++i)
            {
                view.setField(pdal::Dimension::Id::Red,
                        startId + i, rgbPoints[selected[i]].r);
                view.setField(pdal::Dimension::Id::Green,
                        startId + i, rgbPoints[selected[i]].g);
                view.setField(pdal::Dimension::Id::Blue,
                        startId + i, rgbPoints[selected[i]].b);
            }
        }
        else if (dimId == Dimension::Id::Intensity)
        {
            auto data = fetchBinary(attrUrl, std::to_string(key),
                    ".bin.pccint");

            std::vector<uint16_t> intensity;
            try
            {
                intensity = EsriUtil::decompressIntensity(&data);
            }
            catch (const EsriUtil::decompression_error& e)
            {
                throwError(e.what());
            }

            if (intensity.size() != xyz.size())
            {
                throwError(std::string("Bad data fetch. Data id: " +
                            dimEntry.second.name));
            }

            std::lock_guard<std::mutex> lock(m_mutex);
            for (std::size_t i(0); i < selected.size(); ++i)
            {
                view.setField(pdal::Dimension::Id::Intensity,
                        startId + i, intensity[selected[i]]);
            }
        }
        else if (dimId == Dimension::Id::NumberOfReturns)
        {

            const std::vector<char> data = fetchBinary(
                    attrUrl, std::to_string(key), ".bin.gz");

            const uint8_t* returnData =
                reinterpret_cast<const uint8_t*> (data.data());

            if (data.size() != xyz.size())
                throwError(std::string("Bad data fetch. Data id: " +
                            dimEntry.second.name));


            std::lock_guard<std::mutex> lock(m_mutex);
            for (std::size_t i(0); i < selected.size(); ++i)
            {
                //unpack returns to return number and number of returns
                uint8_t offset = returnData[selected[i]];
                uint8_t returnNum = offset & 0x0F;//4 lsb
                uint8_t numReturns = (offset >> 4) & 0x0F;//4 msb

                view.setField(Dimension::Id::ReturnNumber,
                        startId + i, returnNum);
                view.setField(Dimension::Id::NumberOfReturns,
                        startId + i, numReturns);
            }
        }
        else
        {
            const auto data = fetchBinary(
                    attrUrl, std::to_string(key), ".bin.gz");

            std::size_t dimSize = Dimension::size(dimType);

            if (data.size() != xyz.size() * dimSize)
                throwError(std::string("Bad data fetch. Data id: " +
                            dimEntry.second.name));

            std::lock_guard<std::mutex> lock(m_mutex);
            for (std::size_t i(0); i < selected.size(); ++i)
            {
                view.setField(dimId, dimType, startId + i,
                        data.data() + selected[i] * dimSize);
            }
        }
    }
}


void EsriReader::done(PointTableRef)
{
    m_stream.reset();
}

} //namespace pdal

