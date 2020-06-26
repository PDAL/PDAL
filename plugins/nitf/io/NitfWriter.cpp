/****************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <memory>
#include <vector>

#include "NitfWriter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#ifndef IMPORT_NITRO_API
#define IMPORT_NITRO_API
#endif
#include <nitro/c++/import/nitf.hpp>
#include "tre_plugins.hpp"

// NOTES
//
// is it legal to write a LAZ file?
// syntactically, how do we name all the LAS writer options that we will
// pass to the las writer?

namespace pdal
{

static PluginInfo const s_info
{
    "writers.nitf",
    "NITF Writer",
    "http://pdal.io/stages/writers.nitf.html"
};

CREATE_SHARED_STAGE(NitfWriter, s_info)

std::string NitfWriter::getName() const { return s_info.name; }

BOX3D NitfWriter::reprojectBoxToDD(const SpatialReference& reference,
    const BOX3D& box)
{
    if (reference.empty())
        return BOX3D();

    BOX3D output(box);
    if (!gdal::reprojectBounds(output, reference.getWKT(), "EPSG:4326"))
        throwError("Couldn't reproject corner points to geographic: " +
            gdal::lastError());
    return output;
}


NitfWriter::NitfWriter()
{
    try
    {
        m_nitf.initialize();
    }
    catch (const NitfFileWriter::error& err)
    {
        throwError(err.what());
    }
}


void NitfWriter::addArgs(ProgramArgs& args)
{
    LasWriter::addArgs(args);
    m_nitf.addArgs(args);
}


void NitfWriter::writeView(const PointViewPtr view)
{
    LasWriter::writeView(view);
}


void NitfWriter::readyFile(const std::string& filename,
    const SpatialReference& srs)
{
    m_nitf.setFilename(filename);

    Utils::writeProgress(m_progressFd, "READYFILE", filename);
    prepOutput(&m_oss, srs);
}


void NitfWriter::doneFile()
{
    finishOutput();

    std::streambuf *buf = m_oss.rdbuf();
    std::streamoff size = buf->pubseekoff(0, m_oss.end);
    buf->pubseekoff(0, m_oss.beg);

    std::vector<char> bytes(size);
    buf->sgetn(bytes.data(), size);
    m_oss.clear();
    m_nitf.wrapData(bytes.data(), size);
    m_nitf.setBounds(reprojectBoxToDD(m_srs, m_lasHeader.getBounds()));

    try
    {
        m_nitf.write();
    }
    catch (const NitfFileWriter::error& err)
    {
        throwError(err.what());
    }
}

} // namespaces
