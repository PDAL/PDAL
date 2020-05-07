/******************************************************************************
* Copyright (c) 2018, Hobu Inc., info@hobu.co
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

#include <pdal/Writer.hpp>

namespace pdal
{
class OLeStream;

typedef std::shared_ptr<std::ostream> FileStreamPtr;

class PDAL_DLL GltfWriter : public Writer
{
    struct ViewData;

public:
    GltfWriter();
    ~GltfWriter();
    GltfWriter(const GltfWriter&) = delete;
    GltfWriter& operator=(const GltfWriter&) = delete;

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr v);
    virtual void done(PointTableRef table);
    virtual void prepared(PointTableRef table);

    void writeGltfHeader();
    void writeJsonChunk();
    void writeBinHeader();

    std::string m_filename;
    std::unique_ptr<OLeStream> m_stream;
    std::vector<ViewData> m_viewData;
    size_t m_totalSize;
    size_t m_binSize;
    bool m_writeNormals;

    double m_metallic;
    double m_roughness;
    double m_red;
    double m_green;
    double m_blue;
    double m_alpha;
    bool m_doubleSided;
    bool m_colorVertices;
};


struct GltfWriter::ViewData
{
    BOX3D m_bounds;
    size_t m_indexOffset;
    size_t m_indexByteLength;
    size_t m_indexCount;
    size_t m_vertexOffset;
    size_t m_vertexByteLength;
    size_t m_vertexCount;
};


} // namespace pdal
