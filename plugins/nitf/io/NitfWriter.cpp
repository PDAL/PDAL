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

#include <pdal/GDALUtils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PointView.hpp>

#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wunused-private-field"
#endif

#ifndef IMPORT_NITRO_API
#define IMPORT_NITRO_API
#endif
#include <nitro/c++/import/nitf.hpp>
#include "tre_plugins.hpp"

#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic pop
#endif
#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic pop
#endif

// NOTES
//
// is it legal to write a LAZ file?
// syntactically, how do we name all the LAS writer options that we will
// pass to the las writer?

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.nitf",
    "NITF Writer",
    "http://pdal.io/stages/writers.nitf.html" );

CREATE_SHARED_PLUGIN(1, 0, NitfWriter, Writer, s_info)

std::string NitfWriter::getName() const { return s_info.name; }

BOX3D NitfWriter::reprojectBoxToDD(const SpatialReference& reference,
    const BOX3D& box)
{
    if (reference.empty())
        return BOX3D();

    BOX3D output(box);
    if (!gdal::reprojectBounds(output, reference.getWKT(), "EPSG:4326"))
    {
        std::ostringstream msg;

        msg << getName() << ": Couldn't reproject corner points to "
            "geographic: " << gdal::lastError();
        throw pdal_error(msg.str());
    }
    return output;
}


NitfWriter::NitfWriter()
{
    register_tre_plugins();
}


void NitfWriter::addArgs(ProgramArgs& args)
{
    LasWriter::addArgs(args);
    m_nitf.addArgs(args);
}


void NitfWriter::writeView(const PointViewPtr view)
{
    //ABELL - Think we can just get this from the LAS file header
    //  when we're done.
    view->calculateBounds(m_bounds);
    LasWriter::writeView(view);
}


void NitfWriter::readyFile(const std::string& filename,
    const SpatialReference& srs)
{
    m_nitf.setFilename(filename);
    m_error.setFilename(filename);

    Utils::writeProgress(m_progressFd, "READYFILE", filename);
    prepOutput(&m_oss, srs);
}


void NitfWriter::doneFile()
{
    finishOutput();

    std::streambuf *buf = m_oss.rdbuf();
    long size = buf->pubseekoff(0, m_oss.end);
    buf->pubseekoff(0, m_oss.beg);

    std::vector<char> bytes(size);
    buf->sgetn(bytes.data(), size);
    m_oss.clear();
    m_nitf.wrapData(bytes.data(), size);
    m_nitf.setBounds(reprojectBoxToDD(m_srs, m_bounds));

    try
    {
        m_nitf.write();
    }
    catch (except::Throwable & t)
    {
        std::ostringstream oss;
        // std::cout << t.getTrace();
        throw pdal_error(t.getMessage());
    }
}

} // namespaces
