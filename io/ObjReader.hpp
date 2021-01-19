/******************************************************************************
* Copyright (c) 2020, Hobu Inc. (info@hobu.co)
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

#pragma once

#include <istream>
#include <map>
#include <tuple>
#include <vector>

#include <pdal/Reader.hpp>

namespace pdal
{

class PDAL_DLL ObjReader : public Reader
{
public:
    std::string getName() const;

private:
    /**
      Retrieve summary information for the file. NOTE - entire file must
      be read to retrieve summary for obj files.

      \param table  Point table being initialized.
    */
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual void done(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t numPts);

private:
    struct XYZW
    {
        double x;
        double y;
        double z;
        double w;
    };
    std::vector<XYZW> m_vertices;
    std::vector<XYZW> m_textureVertices;
    std::vector<XYZW> m_normalVertices;
    TriangularMesh *m_mesh;
    using VTN = std::tuple<int64_t, int64_t, int64_t>;
    std::map<VTN, PointId> m_points;
    std::istream *m_istream;
    point_count_t m_index;

    using TRI = std::array<VTN, 3>;
    using FACE = std::vector<VTN>;

    void newVertex(double x, double y, double z);
    void newVertex(double x, double y, double z, double w);
    void newTextureVertex(double x);
    void newTextureVertex(double x, double y);
    void newTextureVertex(double x, double y, double z);
    void newNormalVertex(double x, double y, double z);
    void newTriangle(PointViewPtr view, TRI tri);
    bool readFace(FACE& vertices, PointViewPtr view);
    void extractFace(StringList fields, FACE& face);
    VTN extractVertex(const std::string& vstring);
    std::vector<TRI> triangulate(FACE face);
    PointId addPoint(PointViewPtr view, VTN vertex);
};

} // namespace pdal
