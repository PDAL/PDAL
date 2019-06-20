/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include "CesiumWriter.hpp"

#include <iostream>
#include <nlohmann/json.hpp>

#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/OStream.hpp>

namespace pdal
{

class CesiumWriter::Tile
{
public:
    Tile(PointViewPtr v, const std::string& b3dmFilename,
            const std::string& binFilename) :
        m_view(v), m_b3dmFilename(b3dmFilename), m_binFilename(binFilename)
    {}

    std::string b3dmFilename() const
        { return m_b3dmFilename; }
    std::string binFilename() const
        { return m_binFilename; }
    uint32_t byteLength() const
        { return indexByteLength() + vertexByteLength(); }
    uint32_t indexOffset() const
        { return 0; }
    uint32_t indexByteLength() const
        { return m_view->mesh()->size() * sizeof(uint32_t) * 3; }  // 3 vertices
    uint32_t indexCount() const
        { return m_view->mesh()->size() * 3; }
    uint32_t vertexOffset() const
        { return indexByteLength(); }
    uint32_t vertexByteLength() const
        { return m_view->size() * sizeof(float) * 3; }  // 3 for X/Y/Z
    uint32_t vertexCount() const
        { return m_view->size(); }
    BOX3D& bounds()
        { return m_bounds; }
    const BOX3D& bounds() const
        { return m_bounds; }

private:
    PointViewPtr m_view;
    std::string m_b3dmFilename;
    std::string m_binFilename;
    BOX3D m_bounds;
};

namespace
{

// 3D Tiles JSON strings need to be padded with spaces to 8 byte boundaries.
void pad(std::string& j)
{
    size_t s = j.size() % 8;
    if (s)
        j.append(8 - s, ' ');
}

std::array<double, 12> bounds(BOX3D b)
{
    double width = b.maxx - b.minx;
    double length = b.maxy - b.miny;
    double depth = b.maxz - b.minz;
    double x = b.minx + (width / 2);
    double y = b.miny + (length / 2);
    double z = b.minz + (depth / 2);

    // Center and vectors from the center to the sides in X/Y/Z dimensions.
    std::array<double, 12> ob =
        {
          x,          y,           z,
          width / 2,  0 ,          0,
          0,          length / 2,  0,
          0,          0,           depth / 2
        };
    return ob;
}

}  // unnamed namespace

static StaticPluginInfo const s_info
{
    "writers.cesium",
    "Cesium Writer",
    "http://pdal.io/stages/writers.cesium.html",
    { "gltf" }
};

CREATE_STATIC_STAGE(CesiumWriter, s_info)

std::string CesiumWriter::getName() const { return s_info.name; }

void CesiumWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output file directory", m_filename);
}


std::string CesiumWriter::binFilename(int tileNum) const
{
    return (m_filename + "/tile_" + std::to_string(tileNum) + ".bin");
}


std::string CesiumWriter::b3dmFilename(int tileNum) const
{
    return (m_filename + "/tile_" + std::to_string(tileNum) + ".b3dm");
}


void CesiumWriter::initialize(PointTableRef table)
{
    if (!FileUtils::directoryExists(m_filename) &&
        !FileUtils::createDirectory(m_filename))
        throwError("Can't create directory '" + m_filename + "'.");
}


void CesiumWriter::write(const PointViewPtr v)
{
    TriangularMesh *mesh = v->mesh();
    if (!mesh)
    {
        log()->get(LogLevel::Warning) << "Attempt to write point view with "
            "no mesh. Skipping.\n";
        return;
    }

    ++m_tileNum;
    Tile tile(v, b3dmFilename(m_tileNum), binFilename(m_tileNum));
    BOX3D& bounds = tile.bounds();

    uint64_t len = tile.byteLength() + 1000;   // 1000 for JSON ?
    if (len > (std::numeric_limits<uint32_t>::max)())
    {
        log()->get(LogLevel::Warning) << "Attempt to write point view too "
            "large for current handling.  Skipping.\n";
        return;
    }

    OLeStream out(Utils::createFile(tile.binFilename()));

    for (const Triangle& t : *mesh)
        out << (uint32_t)t.m_a << (uint32_t)t.m_b << (uint32_t)t.m_c;
    for (PointId i = 0; i < v->size(); ++i)
    {
        float x = v->getFieldAs<float>(Dimension::Id::X, i);
        float y = v->getFieldAs<float>(Dimension::Id::Y, i);
        float z = v->getFieldAs<float>(Dimension::Id::Z, i);

        bounds.grow(x, y, z);
        out << x << y << z;
    }
    out.close();
    m_tiles.push_back(tile);
}


void CesiumWriter::done(PointTableRef table)
{
    writeTilesetFile();
    for (Tile& t : m_tiles)
        writeB3dmFile(t);
}


void CesiumWriter::writeTilesetFile()
{
    NL::json tileset, root;

    BOX3D fullBounds;
    size_t cnt = 1;
    for (Tile& t : m_tiles)
    {
        NL::json c;

        fullBounds.grow(t.bounds());
        c["boundingVolume"]["box"] = bounds(t.bounds());
        c["geometricError"] = 0;
        c["content"]["uri"] = FileUtils::getFilename(t.b3dmFilename());
        root["children"].push_back(c);
    }
    root["geometricError"] = 10;
    root["refine"] = "ADD";
    root["boundingVolume"]["box"] = bounds(fullBounds);

    tileset["asset"]["version"] = "1.0";
    tileset["geometricError"] = .1;
    tileset["root"] = root;

    std::string json = tileset.dump();
    pad(json);

    std::string filename = m_filename + "/tileset.json";
    std::ofstream out(filename, std::ios::out | std::ios::trunc);
    out << json;
}


void CesiumWriter::writeB3dmFile(const Tile& tile)
{
    OLeStream out(Utils::createFile(b3dmFilename(m_tileNum)));
    std::streampos b3dmStart = out.position();
    writeB3dmHeader(out, tile);

    std::streampos gltfStart = out.position();
    writeGltfHeader(out);
    writeGltfJsonHeader(out);
    std::streampos gltfEnd = out.position();

    // Location of the length is 8 bytes from the start.
    out.seek(b3dmStart + std::streampos(8));
    out << (uint32_t)(gltfEnd - b3dmStart);

    // Location of the gltf size is eight bytes from the beginning of
    // the gltf data.
    out.seek(gltfStart + std::streampos(8));
    out << (uint32_t)(gltfEnd - gltfStart);

    // Location of the b3dm size is eight bytes from the beginning.
    out.seek(8);
    out << (uint32_t)(gltfEnd);
}


void CesiumWriter::writeB3dmHeader(OLeStream& out, const Tile& tile)
{
    // Create JSON string
    NL::json featureJson;

    featureJson["BATCH_LENGTH"] = 0;
    std::string f = featureJson.dump();
    pad(f);

    out.put("b3dm");               // Magic
    out << (uint32_t)1;            // Version
    out << (uint32_t)0;            // Full size
    out << (uint32_t)f.size();     // Feature table JSON size
    out << (uint32_t)0;            // Feature table binary size
    out << (uint32_t)0;            // Batch table JSON size
    out << (uint32_t)0;            // Batch table binary size

    // Write the feature table.
    out.put(f);

    // There is no batch table.
}


void CesiumWriter::writeGltfHeader(OLeStream& out)
{
    out.put("glTF");               // Magic
    out << (uint32_t)2;            // Version
    out << (uint32_t)0;            // Full size
}


void CesiumWriter::writeGltfJsonHeader(OLeStream& out)
{
    NL::json j;
    int buf = 0;

    j["asset"]["version"] = "2.0";

    for (const Tile& t : m_tiles)
    {
        j["buffers"].push_back(
            {
                { "byteLength", t.byteLength() },
                { "uri", FileUtils::getFilename(t.binFilename()) }
            }
        );

        j["bufferViews"].push_back(
            {
                { "buffer", buf },
                { "byteOffset", t.indexOffset() },
                { "byteLength", t.indexByteLength() },
                { "target", 34963 }      // Vertex indices code
            }
        );
        j["bufferViews"].push_back(
            {
                { "buffer", buf },
                { "byteOffset", t.vertexOffset() },
                { "byteLength", t.vertexByteLength() },
                { "target", 34962 }      // Vertices code
            }
        );
        j["accessors"].push_back(
            {
                { "bufferView", buf },
                { "componentType", 5125 },      // unsigned int code
                { "type", "SCALAR" },
                { "count", t.indexCount() }
            }
        );
        const BOX3D& b = t.bounds();
        j["accessors"].push_back(
            {
                { "bufferView", buf + 1 },
                { "componentType", 5126 },      // float code
                { "type", "VEC3" },
                { "count", t.vertexCount() },
                { "min", { b.minx, b.miny, b.minz } },
                { "max", { b.maxx, b.maxy, b.maxz } }
            }
        );
        buf++;
    }

    NL::json mesh;
    mesh["primitives"].push_back(
        {
            { "attributes", {{"POSITION", 1}} },
            { "indices", 0 },
            { "material", 0 }
        }
    );

    j["meshes"].push_back(mesh);
    j["scene"] = 0;

    j["nodes"].push_back(
        {
            { "mesh", 0 },
            { "matrix", { 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1 } }
        }
    );

    j["scenes"].push_back(
        {
            { "nodes", { 0 } }
        }
    );

    j["materials"].push_back(
        {
            {
                "pbrMetallicRoughness",
                {
                    { "metallicFactor", 0 },
                    { "baseColorFactor", { .8, 0, 0, 1 } }
                }
            },
            { "name", "Red" },
            { "doubleSided", true }
        }
    );

    std::string js(j.dump());
    pad(js);

    out << (uint32_t)js.size();    // JSON chunk size
    out.put("JSON");               // Chunk type
    out.put(js);
}

} // namespace pdal
