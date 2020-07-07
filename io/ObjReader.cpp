/******************************************************************************
* Copyright (c) 2020, Hobu, Inc.
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

#include "ObjReader.hpp"

#include <pdal/util/FileUtils.hpp>





namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.obj",
    "OBJ Reader",
    "http://pdal.io/stages/readers.obj.html",
    { "obj" }
};

CREATE_STATIC_STAGE(ObjReader, s_info)

std::string ObjReader::getName() const { return s_info.name; }

void ObjReader::addArgs(ProgramArgs& args)
{
    args.add("material", "Material filename", m_materialFileName);
    args.add("triangulate", "Triangulate", m_triangulate, false);
    args.add("color_scale_factor", "Color scale factor", m_color_scale_factor, 255.0);
}

void ObjReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    layout->registerDim(Dimension::Id::NormalX);
    layout->registerDim(Dimension::Id::NormalY);
    layout->registerDim(Dimension::Id::NormalZ);
    layout->registerDim(Dimension::Id::TextureX);
    layout->registerDim(Dimension::Id::TextureY);
    layout->registerDim(Dimension::Id::Red);
    layout->registerDim(Dimension::Id::Green);
    layout->registerDim(Dimension::Id::Blue);
}


void ObjReader::initialize()
{
    bool exists = FileUtils::fileExists(m_filename);
    if (!exists)
        throwError("Given filename '"
                    + m_filename +
                    "' does not exist");

    if (m_materialFileName.size())
    {
        exists = FileUtils::fileExists(m_materialFileName);
        if (!exists)
            throwError("Given material filename '"
                        + m_materialFileName +
                        "' does not exist");
    }
}
void ObjReader::ready(PointTableRef)
{

    if (! m_materialFileName.size())
        m_materialFileName = FileUtils::stem(m_filename) + ".obj.mtl";
    bool haveMat = FileUtils::fileExists(m_materialFileName);

    log()->get(LogLevel::Debug) << "Mat file exists?: "
                                << haveMat << std::endl;

    log()->get(LogLevel::Debug) << "Mat file name: "
                                << m_materialFileName << std::endl;


    std::string basepath;
    basepath = FileUtils::getDirectory(m_filename);

    bool ret = tinyobj::LoadObj(&m_attributes,
                                &m_shapes,
                                &m_materials,
                                &m_warnings,
                                &m_errors,
                                m_filename.c_str(),
                                basepath.c_str(),
                                m_triangulate);
    if (m_warnings.size())
    {
        log()->get(LogLevel::Warning) << m_warnings << std::endl;
    }
    if (m_errors.size())
        throwError(m_errors);

    // reset point count
    m_numPts = 0;

    if (m_shapes.size())
    {
        for (size_t s = 0; s < m_shapes.size(); s++)
            m_numPts += m_shapes[s].mesh.num_face_vertices.size();
    }
    else
    {
        m_numPts = m_attributes.vertices.size() / 3.0;
    }

    m_index = 0;

    log()->get(LogLevel::Debug) << "Number of shapes "
                                << m_shapes.size() << std::endl;
    log()->get(LogLevel::Debug) << "Number of attributes "
                                << m_attributes.vertices.size() << std::endl;
    log()->get(LogLevel::Debug) << "Number of vertices "
                                << m_numPts << std::endl;

}

bool ObjReader::processOne(PointRef& point)
{

    if (m_index < m_numPts )
    {
        fillPoint(point);
        m_index++;
        updateIndexes();
        return true;
    }
    return false;

}


void ObjReader::fillPoint(PointRef& point)
{
    tinyobj::index_t idx;
    if (m_shapes.size() )
        idx = m_shapes[m_shape_idx].mesh.indices[m_face_offset + m_vert_idx];
    else
    {
        idx.vertex_index = m_vert_idx;
        idx.texcoord_index = m_vert_idx;
        idx.normal_index = m_vert_idx;
    }

    tinyobj::real_t vx = m_attributes.vertices[3*idx.vertex_index+0];
    tinyobj::real_t vy = m_attributes.vertices[3*idx.vertex_index+1];
    tinyobj::real_t vz = m_attributes.vertices[3*idx.vertex_index+2];

    point.setField(Dimension::Id::X, vx);

    tinyobj::real_t nx(0);
    tinyobj::real_t ny(0);
    tinyobj::real_t nz(0);
    if (m_attributes.normals.size())
    {
        nx = m_attributes.normals[3 * idx.normal_index + 0];
        ny = m_attributes.normals[3 * idx.normal_index + 1];
        nz = m_attributes.normals[3 * idx.normal_index + 2];
    }

    tinyobj::real_t tx(0);
    tinyobj::real_t ty(0);
    if (m_attributes.texcoords.size())
    {
        tx = m_attributes.texcoords[2 * idx.texcoord_index + 0];
        ty = m_attributes.texcoords[2 * idx.texcoord_index + 1];
    }

    tinyobj::real_t red(0);
    tinyobj::real_t green(0);
    tinyobj::real_t blue(0);
    if (m_attributes.colors.size())
    {
        red =   m_attributes.colors[ 3 * idx.vertex_index + 0];
        green = m_attributes.colors[ 3 * idx.vertex_index + 1];
        blue =  m_attributes.colors[ 3 * idx.vertex_index + 2];
    }

    point.setField(Dimension::Id::X, vx);
    point.setField(Dimension::Id::Y, vy);
    point.setField(Dimension::Id::Z, vz);

    point.setField(Dimension::Id::NormalX, nx);
    point.setField(Dimension::Id::NormalY, ny);
    point.setField(Dimension::Id::NormalZ, nz);

    point.setField(Dimension::Id::TextureX, tx);
    point.setField(Dimension::Id::TextureY, ty);

    point.setField(Dimension::Id::Red, red * m_color_scale_factor);
    point.setField(Dimension::Id::Green, green * m_color_scale_factor);
    point.setField(Dimension::Id::Blue, blue * m_color_scale_factor);

}

void ObjReader::updateIndexes()
{

    // If there's no shapes, we just increment our vertex id.
    // If there are shapes, we have to walk the faces
    if (m_shapes.size() < m_shape_idx)
    {
        if (m_face_idx == m_shapes[m_shape_idx].mesh.num_face_vertices.size())
        {
            m_face_idx = 0;
            m_face_offset = 0;
            m_shape_idx++;
        }
    }

    if (m_vert_idx == m_face_vertex_id && m_shapes.size())
    {
        m_face_offset += m_face_vertex_id;
        m_vert_idx = 0;
        m_face_idx++;
        m_face_vertex_id = m_shapes[m_shape_idx].mesh.num_face_vertices[m_face_idx];
    } else
    {
        m_vert_idx++;
    }

}

point_count_t ObjReader::read(PointViewPtr view, point_count_t count)
{

    PointId nextId = view->size();
    PointId idx = m_index;
    point_count_t numRead = 0;
    while (numRead < count && idx < m_numPts)
    {
        PointRef point = view->point(nextId);
        processOne(point);
        if (m_cb)
            m_cb(*view, nextId);

        idx++;
        nextId++;
        numRead++;
    }
    m_index = idx;
    return numRead;
}

} // namespace pdal
