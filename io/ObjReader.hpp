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

#pragma once

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/util/ProgramArgs.hpp>


#ifndef TINYOBJLOADER_USE_DOUBLE
#define TINYOBJLOADER_USE_DOUBLE
#endif


#define TINYOBJLOADER_IMPLEMENTATION
#include <tinyobjloader/tiny_obj_loader.h>


namespace pdal
{

class PDAL_DLL ObjReader : public Reader
{
public:
    ObjReader()
        : Reader()
        , m_numPts(0)
        , m_index(0)
        , m_shape_idx(0)
        , m_face_idx(0)
        , m_vert_idx(0)
        {}

    std::string getName() const override;

private:
    // Number of points in the file.
    point_count_t m_numPts;
    point_count_t m_index;
    Dimension::IdList m_dims;

    bool m_triangulate;

    tinyobj::attrib_t m_attributes;
    std::vector<tinyobj::shape_t> m_shapes;
    std::vector<tinyobj::material_t> m_materials;

    std::string m_warnings;
    std::string m_errors;
    std::string m_materialFileName;

    size_t m_shape_idx;
    size_t m_face_idx;
    size_t m_vert_idx;


    virtual void initialize() override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual void addArgs(ProgramArgs& args) override;
    virtual bool eof()
        { return m_index >= m_numPts; }

};

} // namespace pdal
