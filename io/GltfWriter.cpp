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

#include "GltfWriter.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/OStream.hpp>

namespace pdal
{

namespace
{
const size_t HeaderSize = 12;
const size_t JsonChunkDataSize = 5000;
const size_t ChunkHeaderSize = 8;
}

static StaticPluginInfo const s_info
{
    "writers.gltf",
    "Gltf Writer",
    "https://pdal.org/stages/writers.gltf.html",
    { "gltf", "glb" }
};

CREATE_STATIC_STAGE(GltfWriter, s_info)

GltfWriter::GltfWriter()
{}

GltfWriter::~GltfWriter()
{}

std::string GltfWriter::getName() const
{
    return s_info.name;
}

void GltfWriter::addArgs(ProgramArgs& args)
{
    args.add("metallic", "Metallic factor [0-1]", m_metallic);
    args.add("roughness", "Roughness factor [0-1]", m_roughness);
    args.add("red", "Base red factor [0-1]", m_red);
    args.add("green", "Base green factor [0-1]", m_green);
    args.add("blue", "Base blue factor [0-1]", m_blue);
    args.add("alpha", "Alpha factor [0-1]", m_alpha, 1.0);
    args.add("double_sided", "Whether the material should be applied to "
             "both sides of the faces.", m_doubleSided);
    args.add("colors", "Write color data for each vertex.  Note that most renderers will "
        "interpolate the color of each vertex across a face, so this may look odd.",
        m_colorVertices);
    args.add("normals", "Write vertex normals", m_writeNormals);
}


void GltfWriter::prepared(PointTableRef table)
{
    const bool hasNormals = table.layout()->hasDim(Dimension::Id::NormalX) &&
        table.layout()->hasDim(Dimension::Id::NormalY) &&
        table.layout()->hasDim(Dimension::Id::NormalZ);

    if (!hasNormals && m_writeNormals)
    {
        log()->get(LogLevel::Warning) << getName() << ": Option 'normals' is set to "
            "true, but one or more of the normal dimensions are missing. Not writing vertex "
            "normals." << std::endl;
        m_writeNormals = false;
    }

    // Only color vertices if color dimensions are present and the option is
    // enabled
    const bool hasColors = table.layout()->hasDim(Dimension::Id::Red) &&
        table.layout()->hasDim(Dimension::Id::Green) &&
        table.layout()->hasDim(Dimension::Id::Blue);

    if (!hasColors && m_colorVertices)
    {
        log()->get(LogLevel::Warning) << getName() << ": Option 'colors' is set to "
            "true, but one or more color dimensions are missing. Not writing vertex "
            "colors." << std::endl;
        m_colorVertices = false;
    }
}


void GltfWriter::ready(PointTableRef table)
{
    m_stream.reset(new OLeStream(filename()));

    // We write the data before we write the header.  To facilitate, we seek
    // to a point where we're pretty darn sure that we can write the header
    // with no problem later.  We'll verify and throw an error if the
    // assumption is bad.
    m_stream->seek(HeaderSize + JsonChunkDataSize + (2 * ChunkHeaderSize));
    m_binSize = 0;
}


void GltfWriter::write(const PointViewPtr v)
{
    TriangularMesh *mesh = v->mesh();
    if (!mesh)
    {
        log()->get(LogLevel::Warning) << "Attempt to write point view with no mesh. Skipping.\n";
        return;
    }

    OLeStream& out = *m_stream;

    ViewData vd;
    vd.m_indexCount = mesh->size() * 3;
    vd.m_vertexCount = v->size();
    vd.m_indexOffset = m_binSize;
    vd.m_indexByteLength = vd.m_indexCount * sizeof(uint32_t);
    vd.m_vertexOffset = vd.m_indexOffset + vd.m_indexByteLength;
    vd.m_vertexByteLength = v->size() * sizeof(float) * 3;  // 3 for X,Y,Z

    if (m_writeNormals)
        // Add the length of 3 normals to the vertex byte length
        vd.m_vertexByteLength += v->size() * sizeof(float) * 3; // 3 for X,Y,Z

    if (m_colorVertices)
        // Add the length of 3 colors to the vertex byte length
        vd.m_vertexByteLength += v->size() * sizeof(float) * 3; // 3 for R,G,B

    m_binSize += vd.m_indexByteLength + vd.m_vertexByteLength;
    m_totalSize = static_cast<size_t>(out.position()) + m_binSize;
    if (m_totalSize > (std::numeric_limits<uint32_t>::max)())
        throwError("Data too large for file.");

    for (const Triangle& t : *mesh)
        out << (uint32_t)t.m_a << (uint32_t)t.m_b << (uint32_t)t.m_c;

    for (PointId i = 0; i < v->size(); ++i)
    {
        float x = v->getFieldAs<float>(Dimension::Id::X, i);
        float y = v->getFieldAs<float>(Dimension::Id::Y, i);
        float z = v->getFieldAs<float>(Dimension::Id::Z, i);

        vd.m_bounds.grow(x, y, z);
        out << x << y << z;

        // This assumes that the normals are unit vectors. Doesn't seem worth verifying.
        if (m_writeNormals)
        {
            out << v->getFieldAs<float>(Dimension::Id::NormalX, i) <<
                v->getFieldAs<float>(Dimension::Id::NormalY, i) <<
                v->getFieldAs<float>(Dimension::Id::NormalZ, i);
        }

        // PDAL colors should be 16 bit unsigned and we need floating values [0, 1].
        if (m_colorVertices)
        {
            const double scale = (std::numeric_limits<uint16_t>::max)();
            double r = v->getFieldAs<double>(Dimension::Id::Red, i) / scale;
            double g = v->getFieldAs<double>(Dimension::Id::Green, i) / scale;
            double b = v->getFieldAs<double>(Dimension::Id::Blue, i) / scale;

            out << (float)r << (float)g << (float)b;
        }

    }

    m_viewData.push_back(vd);
}


void GltfWriter::done(PointTableRef table)
{
    // Go back to the beginning to write the headers.
    m_stream->seek(0);
    writeGltfHeader();
    writeJsonChunk();
    writeBinHeader();
}


void GltfWriter::writeGltfHeader()
{
    OLeStream& out = *m_stream;

    out.put("glTF");               // Magic
    out << (uint32_t)2;            // Version
    out << (uint32_t)m_totalSize;  // Full size
}


void GltfWriter::writeJsonChunk()
{
    OLeStream& out = *m_stream;

    NL::json j;

    j["asset"]["version"] = "2.0";

    j["buffers"].push_back(
    {
        { "byteLength", m_binSize }   // Total size of bin data.
    }
    );

    uint16_t elementSize =  sizeof(float) * 3; // X, Y, Z
    if (m_writeNormals)
        elementSize += sizeof(float) * 3; // NormalX, NormalY, NormalZ
    if (m_colorVertices)
        elementSize += sizeof(float) * 3; // R, G, B

    NL::json mesh;
    uint16_t nextAccessorIdx = 0;
    uint16_t nextBufferViewIdx = 0;
    for (const ViewData& vd : m_viewData)
    {
        NL::json meshAttributes({});

        // Buffer views
        // Vertex indices (faces)
        const uint16_t faceBufferViewIdx = nextBufferViewIdx++;
        j["bufferViews"].push_back(
            {
                { "buffer", 0 },
                { "byteOffset", vd.m_indexOffset },
                { "byteLength", vd.m_indexByteLength },
                { "target", 34963 }      // Vertex indices code
            }
        );

        // Vertex attributes (positions, normals, and colors)
        const uint16_t attrBufferViewIdx = nextBufferViewIdx++;
        j["bufferViews"].push_back(
            {
                { "buffer", 0 },
                { "byteOffset", vd.m_vertexOffset },
                { "byteLength", vd.m_vertexByteLength },
                { "target", 34962 },      // Vertices code
                { "byteStride", elementSize },
            }
        );

        // Accessors
        uint16_t faceAccessorIdx = nextAccessorIdx++;
        j["accessors"].push_back(
            {
                { "bufferView", faceBufferViewIdx },
                { "componentType", 5125 },      // unsigned int code
                { "type", "SCALAR" },
                { "count", vd.m_indexCount }
            }
        );
        const BOX3D& b = vd.m_bounds;
        uint16_t byteOffset = 0;

        // Vertex position accessor
        meshAttributes["POSITION"] = nextAccessorIdx++;
        j["accessors"].push_back(
            {
                { "bufferView", attrBufferViewIdx },
                { "componentType", 5126 },      // float code
                { "type", "VEC3" },
                { "count", vd.m_vertexCount },
                { "byteOffset", byteOffset },
                { "min", { b.minx, b.miny, b.minz } },
                { "max", { b.maxx, b.maxy, b.maxz } }
            }
        );

        if (m_writeNormals)
        {
            // Vertex normal accessor
            byteOffset += sizeof(float) * 3;
            meshAttributes["NORMAL"] = nextAccessorIdx++;
            j["accessors"].push_back(
                {
                    { "bufferView", attrBufferViewIdx },
                    { "componentType", 5126 },      // float code
                    { "type", "VEC3" },
                    { "byteOffset", byteOffset },
                    { "count", vd.m_vertexCount },
                }
            );
        }

        if (m_colorVertices)
        {
            // Vertex color accessor
            byteOffset += sizeof(float) * 3;
            meshAttributes["COLOR_0"] = nextAccessorIdx++;
            j["accessors"].push_back(
                {
                    { "bufferView", attrBufferViewIdx },
                    { "componentType", 5126 },      // float code
                    { "type", "VEC3" },
                    { "byteOffset", byteOffset },
                    { "count", vd.m_vertexCount },
                }
            );
        }

        mesh["primitives"].push_back(
            {
                { "attributes", meshAttributes },
                { "indices", faceAccessorIdx },
                { "material", 0 }
            }
        );
    }

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

    // This seems very crude.  But I'm not sure we can do much else at this
    // point.
    j["materials"].push_back(
        {
            {
                "pbrMetallicRoughness",
                {
                    { "metallicFactor", m_metallic },
                    { "roughnessFactor", m_roughness },
                    { "baseColorFactor", { m_red, m_blue, m_green, m_alpha } }
                }
            },
            { "name", "Color" },
            { "doubleSided", m_doubleSided }
        }
    );

    std::string js(j.dump());
    if (js.size() > JsonChunkDataSize)
        throwError("JSON header too large. "
            "Please open a issue: 'https://github.com/PDAL/PDAL/issues'.");

    // Pad JSON to chunk size.
    js += std::string(JsonChunkDataSize - js.size(), ' ');

    out << (uint32_t)JsonChunkDataSize;  // JSON chunk size
    out.put("JSON");                     // Chunk type
    out.put(js);
}


void GltfWriter::writeBinHeader()
{
    OLeStream& out = *m_stream;

    out << (uint32_t)m_binSize;
    out.put("BIN", 4);
}

} // namespace pdal
