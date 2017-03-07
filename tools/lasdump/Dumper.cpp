/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <laszip/laszip.hpp>
#include <laszip/lasunzipper.hpp>

#include <pdal/util/IStream.hpp>

#include "Dumper.hpp"
#include "Lasdump.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    deque<string> args;

    for (int i = 1; i < argc; ++i)
        args.push_back(argv[i]);
    pdal::lasdump::Dumper d(args);
    if (!d.error())
        d.dump();
}

namespace
{

void usage(const std::string& err = "")
{
    if (!err.empty())
        std::cerr << "Error: " << err << "\n";
    std::cerr << "Usage: lasdump [-o <output filename>] <las/las file>\n";
}

}


namespace pdal
{
namespace lasdump
{

void Dumper::dump()
{
    ILeStream in(m_filename);
    if (!in.good())
    {
        std::cerr << "Error: Couldn't open file \"" << m_filename << "\".\n";
        m_error = -1;
        return;
    }

    try {
        in >> m_header;
    }
    catch (Exception& ex)
    {
        std::cerr << "Error: " << ex << "\n";
        m_error = -1;
        return;
    }
    *m_out << m_header;

    in.seek(m_header.vlrOffset());
    for (uint32_t i = 0; i < m_header.vlrCount(); ++i)
    {
        Vlr vlr;

        in >> vlr;
        if (vlr.matches("laszip encoded", 22204))
            m_zipVlr = vlr;
        *m_out << vlr;
    }
    if (m_header.versionEquals(1, 0))
    {
        uint16_t dataStartSig;
        in >> dataStartSig;
        *m_out << "Data start signature: " << dataStartSig << "\n";
    }

    // Seek to start of points and dump.
    in.seek(m_header.pointOffset());
    if (m_header.compressed())
        readCompressedPoints(in);
    else
        readPoints(in);

    // We're done if this is an extended VLR.
    if (!m_header.versionAtLeast(1, 4))
        return;

    // Seek to start of extended VLRs and dump.
    in.seek(m_header.eVlrOffset());
    for (uint32_t i = 0; i < m_header.eVlrCount(); ++i)
    {
        EVlr vlr;

        in >> vlr;
        *m_out << vlr;
    }
    return;
}


void Dumper::readPoints(ILeStream& in)
{
    vector<char> buf(m_header.pointLen());

    for (uint64_t i = 0; i < m_header.pointCount(); ++i)
    {
        in.get(buf);
        *m_out << cksum(buf) << "\n";
    }
}


void Dumper::readCompressedPoints(ILeStream& in)
{
    LASunzipper unzipper;
    LASzip zip;

    zip.unpack((const unsigned char *)m_zipVlr.data(), m_zipVlr.dataLen());

    // compute the point size
    unsigned pointSize = 0;
    for (unsigned int i = 0; i < zip.num_items; i++)
        pointSize += zip.items[i].size;

    // create the point data
    unsigned char **point = new unsigned char*[zip.num_items];
    std::vector<uint8_t> pointData(pointSize);

    unsigned offset = 0;
    for (unsigned i = 0; i < zip.num_items; i++)
    {
        point[i] = &(pointData[offset]);
        offset += zip.items[i].size;
    }

    unzipper.open(*in.stream(), &zip);

    for (size_t i = 0; i < m_header.pointCount(); ++i)
    {
        unzipper.read(point);
        *m_out << cksum(pointData) << "\n";
    }
}


int Dumper::processArgs(deque<string> args)
{

    if (args.size() == 3)
    {
        if (args[0] != "-o")
        {
            usage();
            return -1;
        }
        args.pop_front();

        m_fout.open(args[0]);
        m_out = &m_fout;
        if (!*m_out)
        {
            usage("Couldn't open output file.");
            return -1;
        }
        args.pop_front();
    }
    if (args.size() != 1)
    {
        usage();
        return -1;
    }

    m_filename = args[0];
    return 0;
}


} // namespace lasdump
} // namespace pdal

