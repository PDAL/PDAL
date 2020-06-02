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
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/SrsTransform.hpp>

#include "SlpkExtractor.hpp"
#include "../lepcc/src/include/lepcc_types.h"

namespace pdal
{

using namespace i3s;

namespace
{
    const std::map<std::string, pdal::Dimension::Id> dimMapping
    {
        {"CLASS_CODE",  Dimension::Id::Classification},
        {"FLAGS",       Dimension::Id::Flag},
        {"USER_DATA",   Dimension::Id::UserData},
        {"POINT_SRC_ID",Dimension::Id::PointSourceId},
        {"GPS_TIME",    Dimension::Id::GpsTime},
        {"SCAN_ANGLE",  Dimension::Id::ScanAngleRank}
    };

    const std::map<std::string, pdal::Dimension::Type> typeMapping
    {
        {"UInt8", Dimension::Type::Unsigned8},
        {"UInt16", Dimension::Type::Unsigned16},
        {"UInt32", Dimension::Type::Unsigned32},
        {"UInt64", Dimension::Type::Unsigned64},
        {"Int8", Dimension::Type::Signed8},
        {"Int16", Dimension::Type::Signed16},
        {"Int32", Dimension::Type::Signed32},
        {"Int64", Dimension::Type::Signed64},
        {"Float64", Dimension::Type::Double},
        {"Float32", Dimension::Type::Float}
    };
}

struct EsriReader::Args
{
    Bounds bounds;
    uint16_t threads = 8;
    std::vector<std::string> dimensions;
    double min_density;
    double max_density;
};

struct EsriReader::DimData
{
    DimData() : key(0), type(Dimension::Type::None),
        dstId(Dimension::Id::Unknown), pos(-1)
    {}

    int key;
    std::string dataType;
    Dimension::Type type;
    Dimension::Id dstId;
    std::string name;
    int pos;
};

class EsriReader::TileContents
{
public:
    TileContents()
    {}
    TileContents(const std::string& url, size_t extraDimCount) :
        m_url(url), m_data(extraDimCount)
    {}

    size_t size() const
    { return m_xyz.size(); }

    std::string m_url;
    std::vector<lepcc::Point3D> m_xyz;
    std::vector<lepcc::RGB_t> m_rgb;
    std::vector<uint16_t> m_intensity;
    std::vector<std::vector<char>> m_data;
    std::string m_error;
};

EsriReader::EsriReader() : m_args(new Args)
{}


EsriReader::~EsriReader()
{}


void EsriReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "Bounds of the point cloud", m_args->bounds);
    args.add("threads", "Number of threads to be used." , m_args->threads);
    args.add("dimensions", "Dimensions to be used in pulls",
        m_args->dimensions);
    args.add("min_density", "Minimum point density", m_args->min_density, -1.0);
    args.add("max_density", "Maximum point density", m_args->max_density, -1.0);
}


void EsriReader::initialize(PointTableRef table)
{
    //create proper density if min was set but max wasn't
    if (m_args->min_density >= 0 && m_args->max_density < 0)
        m_args->max_density = (std::numeric_limits<double>::max)();

    m_pool.reset(new ThreadPool(m_args->threads));

    for (std::string& s : m_args->dimensions)
        s = Utils::toupper(s);

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
        m_info = initInfo();
    }
    catch (std::exception& e)
    {
        throwError(std::string("Failed to fetch info: ") + e.what());
    }

    //create const for looking into
    const NL::json jsonBody = m_info;

    //find version
    if (jsonBody["store"].contains("version"))
    {
        std::string verName = jsonBody["store"]["version"].get<std::string>();
        m_version = Version(verName);
    }

    if (m_version > Version("2.0") || m_version < Version("1.6"))
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
    if (m_args->bounds.is3d())
    {
        m_bounds = m_args->bounds.to3d();
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
        BOX2D b = m_args->bounds.to2d();
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
    using namespace Dimension;

    if (!m_info.contains("attributeStorageInfo"))
        throwError("Attributes do not exist for this object");
    const NL::json& attributes = m_info["attributeStorageInfo"];

    layout->registerDims({Id::X, Id::Y, Id::Z});

    m_extraDimCount = 0;
    for (auto el : attributes)
    {
        DimData dim;

        dim.name = Utils::toupper(el["name"].get<std::string>());
        if (!Utils::contains(m_args->dimensions, dim.name))
            continue;

        dim.key = std::stoi(el["key"].get<std::string>());

        if (!el.contains("attributeValues"))
        {
            //Expect that Elevation will be bundled with xyz
            if (dim.name != "ELEVATION")
                log()->get(LogLevel::Warning) <<
                    "Attribute does not have a type." <<
                    std::endl;
            continue;
        }

        if (dim.name == "RGB")
        {
            layout->registerDim(Id::Red);
            layout->registerDim(Id::Green);
            layout->registerDim(Id::Blue);
        }
        else if (dim.name == "RETURNS")
        {
            layout->registerDim(Id::NumberOfReturns);
            layout->registerDim(Id::ReturnNumber);
            dim.type = Type::Unsigned8;
            dim.pos = m_extraDimCount++;
        }
        else if (dim.name == "INTENSITY")
        {
            layout->registerDim(Id::Intensity);
        }
        else
        {
            std::string dimTypeName = 
                el["attributeValues"]["valueType"].get<std::string>();
            auto typeIt = typeMapping.find(dimTypeName);
            if (typeIt == typeMapping.end())
                throwError("Invalid dimension type '" + dimTypeName + "'.");
            dim.type = typeIt->second;

            auto dimIt = dimMapping.find(dim.name);
            if (dimIt != dimMapping.end())
            {
                layout->registerDim(dimIt->second);
                dim.dstId = dimIt->second;
            }
            else
                dim.dstId = layout->registerOrAssignDim(dim.name, dim.type);
            dim.pos = m_extraDimCount++;
        }
        m_esriDims.push_back(dim);
    }
}


void EsriReader::ready(PointTableRef table)
{
    //output arguments for debugging
    log()->get(LogLevel::Debug) << "filename: " <<
        m_filename << std::endl;
    log()->get(LogLevel::Debug) << "threads: " <<
        m_args->threads << std::endl;
    log()->get(LogLevel::Debug) << "bounds: " <<
        m_args->bounds << std::endl;
    log()->get(LogLevel::Debug) << "min_density: " <<
        m_args->min_density << std::endl;
    log()->get(LogLevel::Debug) << "max_density: " <<
        m_args->max_density << std::endl;
    log()->get(LogLevel::Debug) << "dimensions: " << std::endl;

    for (std::string& dim : m_args->dimensions)
        log()->get(LogLevel::Debug) << "    -" << dim <<std::endl;

    /*
    -3Dscenelayerinfo: <scene-server-url/layers/<layer-id>
    -node index document: <layer-url >/nodepages/<iterative node page id>
    -shared resources: <node-url>/shared/
    -feature data: <node-url>/features/<feature-data-bundle-id>
    -geometry data: <node-url>/geometries/<geometry-data-bundle-id>
    -texture data: <node-url>/textures/<texture-data-bundle-id>
    */

    // Build the node list that will tell us which nodes overlap with bounds
    std::string filename = m_filename + "/nodepages/0";
    std::string s = fetchJson(filename);
    NL::json initJson = i3s::parse(s,
        "Invalid json in file '" + filename + "'.");
    log()->get(LogLevel::Debug) << "Traversing metadata" << std::endl;
    traverseTree(initJson, 0, 0, 0);

    // If we're running in standard mode, queue up all the requests for data.
    // In streaming mode, queue up at most 4 to avoid having a ton of data
    // show up at once. Others requests will be queued as the results
    // are handled.
    m_tilesToProcess = m_nodes.size();
    m_pointId = 0;
    if (table.supportsView())
    {
        for (int i : m_nodes)
            load(i);
    }
    else
    {
        int count = 4;
        m_curNodeIdx = 0;
        while (m_curNodeIdx < m_nodes.size() && count)
        {
            load(m_nodes[m_curNodeIdx++]);
            count--;
        }
    }
}


void EsriReader::checkTile(const TileContents& tile)
{
    if (tile.m_error.size())
    {
        m_pool->stop();
        throwError("Error reading tile '" + tile.m_url + "': " +
            tile.m_error + ".");
    }
}


point_count_t EsriReader::read(PointViewPtr view, point_count_t count)
{
    point_count_t numRead = 0;

    do
    {
        std::unique_lock<std::mutex> l(m_mutex);
        if (m_contents.size())
        {
            TileContents tile = std::move(m_contents.front());
            m_contents.pop();
            l.unlock();
            checkTile(tile);
            process(view, tile, count - numRead);
            numRead += tile.size();
            m_tilesToProcess--;
        }
        else
            m_contentsCv.wait(l);
    } while (m_tilesToProcess && numRead < count);

    // Wait for any running threads to finish and don't start any others.
    // Only relevant if we hit the count limit before reading all the tiles.
    m_pool->stop();

    return numRead;
}

bool EsriReader::processOne(PointRef& point)
{
top:
    if (m_tilesToProcess == 0)
        return false;

    // If there is no active tile, grab one off the queue and ask for
    // another if there are more.  If none are available, wait.
    if (!m_currentTile)
    {
        do
        {
            std::unique_lock<std::mutex> l(m_mutex);
            if (m_contents.size())
            {
                m_currentTile.reset(
                    new TileContents(std::move(m_contents.front())));
                m_contents.pop();
                l.unlock();
                if (m_curNodeIdx < m_nodes.size())
                    load(m_nodes[m_curNodeIdx++]);
                break;
            }
            else
                m_contentsCv.wait(l);
        } while (true);
        checkTile(*m_currentTile);
    }

    bool ok = processPoint(point, *m_currentTile);

    // If we've processed all the points in the current tile, pop it.
    // If we've processed all the tiles, return false to indicate that
    // we're done.
    if (m_pointId == m_currentTile->size())
    {
        m_pointId = 0;
        m_currentTile.reset();
        --m_tilesToProcess;
    }

    // If we didn't pass a point, try again.
    if (!ok)
        goto top;

    return true;
}


void EsriReader::process(PointViewPtr dstView, const TileContents& tile,
    point_count_t count)
{
    m_pointId = 0;
    PointRef dst(*dstView);
    for (PointId idx = 0; idx < tile.size(); ++idx)
    {
        if (count-- == 0)
            return;
        dst.setPointId(dstView->size());
        processPoint(dst, tile);
    }
}


bool EsriReader::processPoint(PointRef& dst, const TileContents& tile)
{
    using namespace Dimension;

    PointId pointId = m_pointId++;

    double x = tile.m_xyz[m_pointId].x;
    double y = tile.m_xyz[m_pointId].y;
    double z = tile.m_xyz[m_pointId].z;

    if (!m_bounds.contains(x, y, z))
        return false;

    dst.setField(Id::X, x);
    dst.setField(Id::Y, y);
    dst.setField(Id::Z, z);

    for (const DimData& dim : m_esriDims)
    {
        if (dim.name == "RGB")
        {
            dst.setField(Id::Red, tile.m_rgb[m_pointId].r);
            dst.setField(Id::Green, tile.m_rgb[m_pointId].g);
            dst.setField(Id::Blue, tile.m_rgb[m_pointId].b);
        }
        else if (dim.name == "INTENSITY")
            dst.setField(Id::Intensity, tile.m_intensity[m_pointId]);
        else if (dim.name == "RETURNS")
        {
            const std::vector<char>& d = tile.m_data[dim.pos];
            dst.setField(Id::ReturnNumber, d[m_pointId] & 0x0F);
            dst.setField(Id::NumberOfReturns, d[m_pointId] >> 4);
        }
        else
        {
            const std::vector<char>& d = tile.m_data[dim.pos];
            dst.setField(dim.dstId, dim.type,
                d.data() + m_pointId * Dimension::size(dim.type));
        }
    }
    return true;
}

// Traverse tree through nodepages. Create a nodebox for each node in
// the tree and test if it overlaps with the bounds created by user.
// If it's a leaf node(the highest resolution) and it overlaps, add
// it to the list of nodes to be pulled later.
void EsriReader::traverseTree(NL::json page, int index, int depth,
    int pageIndex)
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
    if (m_args->max_density == -1 && m_args->min_density == -1 && cCount == 0)
    {
        m_nodes.push_back(name);
        return;
    }
    else
    {
        if (density < m_args->max_density && density > m_args->min_density)
        {
            m_nodes.push_back(name);
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
                    std::string filename = m_filename + "/nodepages/" +
                        std::to_string(pageIndex);
                    std::string s = fetchJson(filename);
                    page = i3s::parse(s,
                        "Invalid JSON in file '" + filename + "'.");
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
            traverseTree(page, index, depth, pageIndex);
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


void EsriReader::load(int nodeId)
{
    std::string localUrl = m_filename + "/nodes/" + std::to_string(nodeId);
    m_pool->add([this, localUrl]()
        {
            TileContents tile;
            try
            {
                tile = loadUrl(localUrl);
            }
            //ABELL - Need to make sure we trap all the errors
            // from size check, fetchBinary, decompress...
            catch (const i3s::EsriError& e)
            {
                tile.m_error = e.what();
            }

            // Stick the loaded tile on the output queue.
            {
                std::unique_lock<std::mutex> l(m_mutex);
                m_contents.push(std::move(tile));
            }
            m_contentsCv.notify_one();
        }
    );
}


EsriReader::TileContents EsriReader::loadUrl(const std::string& localUrl)
{
    auto checkSize = [](const DimData& dim, size_t exp, size_t actual)
    {
        std::string err;
        if (exp != actual)
            err = "Bad fetch of data for field '" + dim.name + "'.";
        return err;
    };

    TileContents tile(localUrl, m_extraDimCount);

    const std::string geomUrl = localUrl + "/geometries/";
    auto xyzFetch = fetchBinary(geomUrl, "0", ".bin.pccxyz");
    tile.m_xyz = i3s::decompressXYZ(&xyzFetch);

    size_t size = tile.m_xyz.size();
    const std::string attrUrl = localUrl + "/attributes/";
    for (const DimData& dim : m_esriDims)
    {
        if (dim.name == "RGB")
        {
            auto data = fetchBinary(attrUrl, std::to_string(dim.key),
                ".bin.pccrgb");
            tile.m_rgb = i3s::decompressRGB(&data);
            tile.m_error = checkSize(dim, size, tile.m_rgb.size());
        }
        else if (dim.name == "INTENSITY")
        {
            auto data = fetchBinary(attrUrl, std::to_string(dim.key),
                ".bin.pccint");
            tile.m_intensity = i3s::decompressIntensity(&data);
            tile.m_error = checkSize(dim, size, tile.m_intensity.size());
        }
        else
        {
            std::vector<char>& data = tile.m_data[dim.pos];
            data = fetchBinary(attrUrl, std::to_string(dim.key), ".bin.gz");
            tile.m_error = checkSize(dim, size * Dimension::size(dim.type),
                data.size());
        }
        if (tile.m_error.size())
            break;
    }
    return tile;
}

} //namespace pdal

