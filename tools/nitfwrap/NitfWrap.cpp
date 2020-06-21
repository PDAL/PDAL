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
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/IStream.hpp>
#include <io/BpfHeader.hpp>
#include <io/LasHeader.hpp>
#include <plugins/nitf/io/NitfFileReader.hpp>

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

namespace
{

void outputHelp(ProgramArgs& args)
{
    std::cout << "usage: nitfwrap [options] " << args.commandLine() <<
        std::endl;
    std::cout << "options:" << std::endl;
    args.dump(std::cout, 2, Utils::screenWidth());
}

} // unnamed namespace


NitfWrap::NitfWrap(std::vector<std::string>& args)
{
    if (!parseArgs(args))
        return;

    if (m_unwrap)
        unwrap();
    else
    {
        BOX3D bounds;
        verify(bounds);

        m_nitfWriter.initialize();
        m_nitfWriter.setFilename(m_outputFile);
        m_nitfWriter.setBounds(bounds);
        m_nitfWriter.wrapData(m_inputFile);
        m_nitfWriter.write();
    }
}


void NitfWrap::unwrap()
{
    // Use the NITF reader to get the offset and length
    uint64_t offset, length;
    NitfFileReader reader(m_inputFile);
    reader.open();
    reader.getLasOffset(offset, length);
    reader.close();

    // Open file file and seek to the beginning of the location.
    std::istream *in = FileUtils::openFile(m_inputFile);
    if (!in)
    {
        std::ostringstream oss;

        oss << "Couldn't open input file '" << m_inputFile << "'.";
        throw error(oss.str());
    }
    in->seekg(offset, std::istream::beg);

    // Find out if this is a LAS or BPF file and make the output filename.
    bool compressed;
    BOX3D bounds;
    ILeStream leIn(in);
    IStreamMarker mark(leIn);
    if (verifyLas(leIn, bounds, compressed))
    {
        if (m_outputFile.empty())
        {
            m_outputFile = FileUtils::stem(m_inputFile);
            m_outputFile += (compressed ? ".laz" : ".las");
        }
    }
    else
    {
        mark.rewind();
        if (verifyBpf(leIn, bounds))
        {
            if (m_outputFile.empty())
                m_outputFile = FileUtils::stem(m_inputFile) + ".bpf";
        }
        else
        {
            std::cerr << "Wrapped file isn't BPF or LAS.\n";
            return;
        }
    }

    uint64_t bufsize = 16;
    std::vector<char> buf(bufsize);
    std::ostream *out = FileUtils::createFile(m_outputFile);
    in->seekg(offset, std::istream::beg);
    while (length)
    {
        size_t size = (std::min)(length, bufsize);
        in->read(buf.data(), size);
        out->write(buf.data(), size);
        length -= size;
    }
    FileUtils::closeFile(out);
}


bool NitfWrap::parseArgs(std::vector<std::string>& argList)
{
    ProgramArgs args;

    args.add("input,i", "Input filename", m_inputFile).setPositional();
    args.add("output,o", "Output filename",
        m_outputFile).setOptionalPositional();
    args.add("unwrap,u", "Unwrap NITF file", m_unwrap);
    try
    {
        m_nitfWriter.addArgs(args);
    }
    catch (arg_error& e)
    {
        throw error(e.m_error);
    }

    try
    {
        args.parse(argList);
    }
    catch (arg_error& e)
    {
        std::cerr << "nitfwrap: " << e.m_error << std::endl;
        outputHelp(args);
        return false;
    }

    if (!FileUtils::fileExists(m_inputFile))
    {
        std::ostringstream oss;

        oss << "Input file '" << m_inputFile << "' doesn't exist.";
        throw error(oss.str());
    }
    if (m_outputFile.empty())
        if (!m_unwrap)
            m_outputFile = FileUtils::stem(m_inputFile) + ".ntf";
    return true;
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

    bool compression;
    ILeStream in(stream);
    IStreamMarker mark(in);
    if (!verifyLas(in, bounds, compression))
    {
        mark.rewind();
        if (!verifyBpf(in, bounds))
            throw error("Input file must be LAS/LAZ or BPF.");
    }
}


bool NitfWrap::verifyLas(ILeStream& in, BOX3D& bounds, bool& compressed)
{
    LasHeader h;

    try
    {
        in >> h;
    }
    catch (LasHeader::error&)
    {
        return false;
    }
    compressed = h.compressed();
    bounds = h.getBounds();
    gdal::reprojectBounds(bounds, h.srs(), "EPSG:4326");
    return true;
}


bool NitfWrap::verifyBpf(ILeStream& in, BOX3D& bounds)
{
    BpfHeader h;
    BpfDimensionList dims;
    LogPtr l(Log::makeLog("nitfwrap", "devnull"));

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
    SpatialReference srs = SpatialReference::wgs84FromZone(h.m_coordId);
    gdal::reprojectBounds(bounds, srs, "EPSG:4326");
    return true;
}

} //namespace nitfwrap
} //namespace pdal

