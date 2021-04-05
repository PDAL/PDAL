/******************************************************************************
* Copyright (c) 2020, Hobu Inc., info@hobu.co
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "ObjReader.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.obj",
    "Obj Reader",
    "http://pdal.io/stages/readers.obj.html",
    { "obj" }
};

CREATE_STATIC_STAGE(ObjReader, s_info)

std::string ObjReader::getName() const { return s_info.name; }

void ObjReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims( {
        Dimension::Id::X,
        Dimension::Id::Y,
        Dimension::Id::Z,
        Dimension::Id::W,
        Dimension::Id::TextureU,
        Dimension::Id::TextureV,
        Dimension::Id::TextureW,
        Dimension::Id::NormalX,
        Dimension::Id::NormalY,
        Dimension::Id::NormalZ,
    });
}

void ObjReader::ready(PointTableRef)
{
    m_istream = Utils::openFile(m_filename, false);
    if (!m_istream)
        throwError("Couldn't open '" + m_filename + "'.");
    m_index = 0;
}

void ObjReader::done(PointTableRef)
{
    Utils::closeFile(m_istream);
}

point_count_t ObjReader::read(PointViewPtr view, point_count_t cnt)
{
    m_mesh = view->createMesh("obj");
    if (!m_mesh)
    {
        throwError("Failed to create mesh");
    }

    while (true)
    {
        FACE face;
        if (!readFace(face, view))
            break;

        auto triangles = triangulate(face);
        for(const auto& tri : triangles) {
            newTriangle(view, tri);
        }
    }
    return m_index;
}

void ObjReader::newTriangle(PointViewPtr view, TRI tri)
{
    // checks if a point exists yet, if not, adds it to the point table via addPoint()
    auto insertPoint = [view, this](VTN vertex)
    {
        PointId id;
        auto it = m_points.find(vertex);
        if (it == m_points.end())
        {
            id = addPoint(view, vertex);
            m_points.insert({vertex, id});
        }
        else
            id = it->second;
        return id;
    };
    PointId p1 = insertPoint(tri[0]);
    PointId p2 = insertPoint(tri[1]);
    PointId p3 = insertPoint(tri[2]);
    m_mesh->add(p1, p2, p3);
}

// adds a point to the point table
PointId ObjReader::addPoint(PointViewPtr view, VTN vertex)
{
    XYZW v, t, n;
    PointRef pt = view->point(m_index);
    m_index++;

    int64_t vertexIndex = std::get<0>(vertex) - 1;
    if (vertexIndex < 0 || (size_t)vertexIndex >= m_vertices.size())
        throwError("Vertex index '" + std::to_string(vertexIndex + 1) + "` specified "
            "for face doesn't exist.");
    v = m_vertices.at(vertexIndex);
    pt.setField(Dimension::Id::X, v.x);
    pt.setField(Dimension::Id::Y, v.y);
    pt.setField(Dimension::Id::Z, v.z);
    pt.setField(Dimension::Id::W, v.w);

    int64_t textureIndex = std::get<1>(vertex) - 1;
    if (textureIndex >=  0) {
        if ((size_t)textureIndex >= m_textureVertices.size())
            throwError("Texture vertex index '" + std::to_string(textureIndex + 1) + "' specified "
                "for face doesn't exist.");
        t = m_textureVertices.at(textureIndex);
        pt.setField(Dimension::Id::TextureU, t.x);
        pt.setField(Dimension::Id::TextureV, t.y);
        pt.setField(Dimension::Id::TextureW, t.z);
    }

    int64_t normalIndex = std::get<2>(vertex) - 1;
    if (normalIndex >= 0) {
        if ((size_t)normalIndex >= m_normalVertices.size())
            throwError("Normal vertex index '" + std::to_string(normalIndex + 1) + "' specified "
                "for face doesn't exist.");
        n = m_normalVertices.at(normalIndex);
        pt.setField(Dimension::Id::NormalX, n.x);
        pt.setField(Dimension::Id::NormalY, n.y);
        pt.setField(Dimension::Id::NormalZ, n.z);
    }
    return pt.pointId();
}

void ObjReader::newVertex(double x, double y, double z)
{
    // w defaults to 1 according to https://en.wikipedia.org/wiki/Wavefront_.obj_file
    m_vertices.push_back({x, y, z, 1});
}

void ObjReader::newVertex(double x, double y, double z, double w)
{
    m_vertices.push_back({x, y, z, w});
}

// undefined texture values default to 0 according to https://en.wikipedia.org/wiki/Wavefront_.obj_file
void ObjReader::newTextureVertex(double x)
{
    m_textureVertices.push_back({x, 0, 0, 0});
}

void ObjReader::newTextureVertex(double x, double y)
{
    m_textureVertices.push_back({x, y, 0, 0});
}

void ObjReader::newTextureVertex(double x, double y, double z)
{
    m_textureVertices.push_back({x, y, z, 0});
}

void ObjReader::newNormalVertex(double x, double y, double z)
{
    m_normalVertices.push_back({x, y, z, 0});
}

bool ObjReader::readFace(FACE& face, PointViewPtr view)
{
    long long lineOfFile = 0;
    while(true) {
        if(m_istream->peek() == EOF)
            return false;

        std::string line;
        std::getline(*m_istream, line);
        lineOfFile++;
        Utils::trim(line);
        if (line.length() == 0)
            continue;

        StringList fields = Utils::split2(line, ' ');
        std::string key = fields[0];
        if (key == "#")
        {
            // Comment: Do nothing
        }
        else if (key == "v")
        {
            // Vertex
            size_t numDims = fields.size() - 1;

            auto throwVertexError = [this, line, lineOfFile]()
            {
                std::stringstream errorMessage;
                errorMessage << "Could not convert vertex specification to double on line #"
                    << lineOfFile << ": '" << line << "'" << std::endl;
                throwError(errorMessage.str()); 
            };

            if(numDims < 3) {
                throwVertexError();
            }
            else if(numDims == 3) {
                double x, y, z;
                if (Utils::fromString(fields[1], x) && Utils::fromString(fields[2], y) &&
                    Utils::fromString(fields[3], z))
                    newVertex(x, y, z);
                else
                    throwVertexError();
            }
            else if(numDims > 3) {
                double x, y, z, w;
                if (Utils::fromString(fields[1], x) && Utils::fromString(fields[2], y) &&
                    Utils::fromString(fields[3], z) && Utils::fromString(fields[4], w))
                    newVertex(x, y, z, w);
                else
                    throwVertexError();
            }
        }
        else if (key == "vt")
        {
            auto throwTextureError = [this, line, lineOfFile]()
            {
                std::stringstream ss;
                ss << "Could not convert texture vertex specification to double on line #"
                    << lineOfFile << ": '" << line << "'" << std::endl;
                throwError(ss.str());
            };
            // Vertex texture
            if(fields.size() == 4) {
                double x, y, z;
                if (Utils::fromString(fields[1], x) && Utils::fromString(fields[2], y) &&
                    Utils::fromString(fields[3], z))
                    newTextureVertex( x, y, z );
                else
                    throwTextureError();
            }
            else if(fields.size() == 3) {
                double x, y;
                if (Utils::fromString(fields[1], x) && Utils::fromString(fields[2], y))
                    newTextureVertex( x, y );
                else
                    throwTextureError();
            }
            else if(fields.size() == 2) {
                double x;
                if (Utils::fromString(fields[1], x))
                    newTextureVertex( x );
                else
                    throwTextureError();
            }

        }
        else if (key == "vn")
        {
            // Vertex normal
            double x, y, z;
            if (fields.size() >= 4 && Utils::fromString(fields[1], x) &&
                Utils::fromString(fields[2], y) && Utils::fromString(fields[3], z))
                newNormalVertex( x, y, z );
            else
            {
                std::stringstream ss;
                ss << "Could not convert normal vertex specification to double on line #"
                    << lineOfFile << ": '" << line << "'" << std::endl;
                throwError(ss.str());
            }
        }
        else if (key == "f")
        {
            // Face
            if (fields.size() < 4)
                throwError("Not enough vertices in face specification.");
            StringList vertices(fields.begin() + 1, fields.end());
            extractFace(vertices, face);
            return true;
        }
    }
}

void ObjReader::extractFace(StringList fields, ObjReader::FACE& face)
{
    for (const std::string& field : fields)
        face.push_back(extractVertex(field));
}

std::vector<ObjReader::TRI> ObjReader::triangulate(FACE face)
{
    std::vector<TRI> triangles;

    unsigned int totalTriangles = face.size() - 2;
    while(triangles.size() < totalTriangles)
    {
        TRI tri;
        tri[0] = face[0];
        tri[1] = face[triangles.size()+1];
        tri[2] = face[triangles.size()+2];
        triangles.push_back(tri);
    }
    return triangles;
}

ObjReader::VTN ObjReader::extractVertex(const std::string& vstring)
{
    VTN vtn{ -1, -1, -1 };
    std::string s(vstring);
    Utils::trim(s);
    StringList parts = Utils::split(s, '/');
    for(auto& part : parts)
        Utils::trim(part);

    if (parts.size() > 3)
        throwError("Too many items in vertex specification.");

    char* p;
    long index = std::strtol(parts[0].c_str(), &p, 10);
    if (index == 0 || p != parts[0].c_str() + parts[0].size())
        throwError("Invalid index in face specification.");
    else if (index < 0)
        std::get<0>(vtn) = m_vertices.size() - index;
    else
        std::get<0>(vtn) = index;

    if (parts.size() > 1)
    {
        if (parts[1].length() > 0)
        {
            index = std::strtol(parts[1].c_str(), &p, 10);
            if (index == 0 || p != parts[1].c_str() + parts[1].size())
                throwError("Invalid index in face specification.");
            else if(index < 0)
                std::get<1>(vtn) = m_vertices.size() - index;
            else
                std::get<1>(vtn) = index;
        }
    }

    if (parts.size() > 2)
    {
        if (parts[2].length() > 0)
        {
            index = std::strtol(parts[2].c_str(), &p, 10);
            if (index == 0 || p != parts[2].c_str() + parts[2].size())
                throwError("Invalid index in face specification.");
            else if(index < 0)
                std::get<2>(vtn) = m_vertices.size() - index;
            else
                std::get<2>(vtn) = index;
        }
    }
    return vtn;
}

} // namespace pdal

