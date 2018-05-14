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

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.cesium",
    "Cesium Writer",
    "http://pdal.io/stages/writers.cesium.html",
    { "gltf" }
};

CREATE_STATIC_STAGE(CesiumWriter, s_info)

std::string CesiumWriter::getName() const { return s_info.name; }

struct FileStreamDeleter
{

    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
        {
            ptr->flush();
            Utils::closeFile(ptr);
        }
    }
};


void CesiumWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename);
}


void CesiumWriter::initialize(PointTableRef table)
{
    m_stream = FileStreamPtr(Utils::createFile(m_filename, true),
        FileStreamDeleter());
    if (!m_stream)
        throwError("Couldn't open '" + m_filename + "' for output.");
}


void CesiumWriter::ready(PointTableRef table)
{
/**
    m_stream.reset(new OLEStream(m_filename));
    m_stream << "glTF";         // Magic
    m_stream << (uint32_t)2;    // Version
    m_stream <<
**/
}


} // namespace pdal
