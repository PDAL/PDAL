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

#include <pdal/pdal_macros.hpp>

#include <pdal/PointView.hpp>
#include <pdal/GlobalEnvironment.hpp>

#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wredundant-decls"
#  pragma GCC diagnostic ignored "-Wextra"
#  pragma GCC diagnostic ignored "-Wcast-qual"
   // The following pragma doesn't actually work:
   //   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61653
   //#  pragma GCC diagnostic ignored "-Wliteral-suffix"
#endif
#include <ogr_spatialref.h>
#include <cpl_conv.h>
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wunused-private-field"
#endif

#define IMPORT_NITRO_API
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

BOX3D reprojectBoxToDD(const SpatialReference& reference, const BOX3D& box)
{
    if (reference.empty())
        return BOX3D();

    BOX3D output(box);

    OGRSpatialReferenceH current =
        OSRNewSpatialReference(reference.getWKT(SpatialReference::eCompoundOK,
            false).c_str());
    OGRSpatialReferenceH dd = OSRNewSpatialReference(0);

    OGRErr err = OSRSetFromUserInput(dd, const_cast<char *>("EPSG:4326"));
    if (err != OGRERR_NONE)
        throw std::invalid_argument("could not import coordinate system "
            "into OGRSpatialReference SetFromUserInput");

    OGRCoordinateTransformationH transform =
        OCTNewCoordinateTransformation(current, dd);

    int ret = OCTTransform(transform, 1,
        &output.minx, &output.miny, &output.minz);
    if (ret == 0)
    {
        std::ostringstream msg;
        msg << "Could not project point for reprojectBoxToDD::min" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }

    OCTTransform(transform, 1, &output.maxx, &output.maxy, &output.maxz);
    if (ret == 0)
    {
        std::ostringstream msg;
        msg << "Could not project point for reprojectBoxToDD::max" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }

    OCTDestroyCoordinateTransformation(transform);
    OSRDestroySpatialReference(current);
    OSRDestroySpatialReference(dd);

    return output;
}


NitfWriter::NitfWriter()
{
    register_tre_plugins();
}


void NitfWriter::processOptions(const Options& options)
{
    LasWriter::processOptions(options);
    m_nitf.processOptions(options);
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

    BOX3D bounds =  reprojectBoxToDD(m_srs, m_bounds);

    //NITF decimal degree values for corner coordinates only has a
    // precision of 3 after the decimal. This may cause an invalid
    // polygon due to rounding errors with a small tile. Therefore
    // instead of rounding min values will use the floor value and
    // max values will use the ceiling values.
    bounds.minx = (floor(bounds.minx * 1000)) / 1000.0;
    bounds.miny = (floor(bounds.miny * 1000)) / 1000.0;
    bounds.maxx = (ceil(bounds.maxx * 1000)) / 1000.0;
    bounds.maxy = (ceil(bounds.maxy * 1000)) / 1000.0;
    m_nitf.setBounds(bounds);

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
