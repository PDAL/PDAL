/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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

#include <string>
#include <vector>

#include <pdal/Dimension.hpp>
#include <pdal/GDALUtils.hpp>
#include <bpf/BpfHeader.hpp>
#include <las/LasHeader.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/IStream.hpp>

#include "NitfWrap.hpp"

int main(int argc, char* argv[])
{
    std::vector<std::string> args;

    for (int i = 1; i < argc; ++i)
        args.push_back(argv[i]);

    try
    {
        pdal::nitfwrap::NitfWrap nw(args);
    }
    catch (pdal::nitfwrap::error err)
    {
        std::cerr << "nitfwrap: " << err.what() << "\n";
    }
}

namespace pdal
{
namespace nitfwrap
{

NitfWrap::NitfWrap(std::vector<std::string>& args)
{
    BOX3D bounds;

    parseArgs(args);
    m_nitf.setFilename(m_outputFile);
    verify(bounds);
    m_nitf.wrapData(m_inputFile);
    m_nitf.write();
}


void NitfWrap::parseArgs(std::vector<std::string>& argList)
{
    ProgramArgs args;

    try
    {
        m_nitf.setArgs(args);
    }
    catch (arg_error& e)
    {
        throw error(e.m_error);
    }
    args.add("input,i", "Input filename", m_inputFile).setPositional();
    args.add("output,o", "Output filename",
        m_outputFile).setOptionalPositional();

    try
    {
        args.parse(argList);
    }
    catch (arg_error& e)
    {
        throw error(e.m_error);
    }

    if (!FileUtils::fileExists(m_inputFile))
    {
        std::ostringstream oss;

        oss << "Input file '" << m_inputFile << "' doesn't exist.";
        throw error(oss.str());
    }
    if (m_outputFile.empty())
        m_outputFile = FileUtils::stem(m_inputFile) + ".ntf";
}

void NitfWrap::verify(BOX3D& bounds)
{
    std::istream *stream = FileUtils::openFile(m_inputFile);

    if (!stream)
    {
        std::ostringstream oss;

        oss << "Couldn't open input file '" << m_inputFile << "'.";
        throw error(oss.str());
    }

    ILeStream in(stream);
    IStreamMarker mark(in);
    if (!verifyLas(in, bounds))
    {
        mark.rewind();
        if (!verifyBpf(in, bounds))
            throw error("Input file must be LAS/LAZ or BPF in order to "
                "wrap with NITF with nitfwrap.");
    }
}


bool NitfWrap::verifyLas(ILeStream& in, BOX3D& bounds)
{
    LasHeader h;

    try
    {
        in >> h;
    }
    catch (pdal_error&)
    {
        return false;
    }
    bounds = h.getBounds();
    gdal::reprojectBounds(bounds, h.srs().getWKT(), "EPSG:4326");
    return true;
}


bool NitfWrap::verifyBpf(ILeStream& in, BOX3D& bounds)
{
    BpfHeader h;
    BpfDimensionList dims;
    LogPtr l(new Log("nitfwrap", "devnull"));

    h.setLog(l);

    if (!h.read(in))
        return false;
    if (!h.readDimensions(in, dims))
        return false;
    for (auto d : dims)
    {
        if (d.m_id == Dimension::Id::X)
        {
            bounds.minx = d.m_min;
            bounds.maxx = d.m_max;
        }
        if (d.m_id == Dimension::Id::Y)
        {
            bounds.miny = d.m_min;
            bounds.maxy = d.m_max;
        }
        if (d.m_id == Dimension::Id::Z)
        {
            bounds.minz = d.m_min;
            bounds.maxz = d.m_max;
        }
    }
    int32_t zone(abs(h.m_coordId));
    std::string code;
    if (h.m_coordId > 0)
        code = "EPSG:326" + Utils::toString(zone);
    else
        code = "EPSG:327" + Utils::toString(zone);
    gdal::reprojectBounds(bounds, code, "EPSG:4326");
    return true;
}

} //namespace nitfwrap
} //namespace pdal

