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

#include <pdal/util/IStream.hpp>

#include "Dumper.hpp"
#include "Lasdump.hpp"

int main(int argc, char *argv[])
{
    std::deque<std::string> args;

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
    std::vector<char> buf(m_header.pointLen());

    for (uint64_t i = 0; i < m_header.pointCount(); ++i)
    {
        in.get(buf);
        *m_out << cksum(buf) << "\n";
    }
}


void Dumper::handleLaszip(int result)
{
     if (result)
     {
         char *buf;
         laszip_get_error(m_zip, &buf);
         std::cerr << buf;
         exit(-1);
     }
}


void Dumper::readCompressedPoints(ILeStream& in)
{
    laszip_BOOL compressed;
    laszip_point_struct *zipPoint;

    handleLaszip(laszip_create(&m_zip));
    handleLaszip(laszip_open_reader_stream(m_zip, *in.stream(), &compressed));
    handleLaszip(laszip_get_point_pointer(m_zip, &zipPoint));

    std::vector<char> buf(m_header.pointLen());
    for (size_t i = 0; i < m_header.pointCount(); ++i)
    {
        handleLaszip(laszip_read_point(m_zip));
        loadPoint(zipPoint, buf);
        *m_out << cksum(buf) << "\n";
    }
}


void Dumper::loadPoint(const laszip_point_struct *zipPoint, std::vector<char>& buf)
{
    const char *in = (const char *)zipPoint;
    char *out = buf.data();
    if (m_header.pointFormat() <= 5)
    {
        std::copy(in, in + 20, out);
        out += 20;
        in += 20;
        if (m_header.pointFormat() == 1 || m_header.pointFormat() == 3)
        {
            in = reinterpret_cast<const char *>(&(zipPoint->gps_time));
            std::copy(in, in + 8, out);
            out += 8;
        }
        if (m_header.pointFormat() == 2 || m_header.pointFormat() == 3)
        {
            in = reinterpret_cast<const char *>(&(zipPoint->rgb));
            std::copy(in, in + 6, out);
            out += 6;
        }
    }
    else if (m_header.pointFormat() >= 6 && m_header.pointFormat() <= 10)
    {
        std::copy(in, in + 14, out);
        out += 14;
        in += 14;
        *out++ = zipPoint->extended_return_number |
            (zipPoint->extended_number_of_returns << 4);
        *out++ = zipPoint->extended_classification_flags |
            (zipPoint->extended_scanner_channel << 4) |
            (zipPoint->scan_direction_flag << 6) |
            (zipPoint->edge_of_flight_line << 7);
        *out++ = zipPoint->extended_classification;
        *out++ = zipPoint->user_data;

        in = reinterpret_cast<const char *>(&(zipPoint->extended_scan_angle));
        std::copy(in, in + 2, out);
        out += 2;

        in = reinterpret_cast<const char *>(&(zipPoint->point_source_ID));
        std::copy(in, in + 2, out);
        out += 2;

        in = reinterpret_cast<const char *>(&(zipPoint->gps_time));
        std::copy(in, in + 8, out);
        out += 8;

        if (m_header.pointFormat() == 7)
        {
            in = reinterpret_cast<const char *>(&(zipPoint->rgb));
            std::copy(in, in + 6, out); // RBG
            out += 6;
        }
        if (m_header.pointFormat() == 8)
        {
            in = reinterpret_cast<const char *>(&(zipPoint->rgb));
            std::copy(in, in + 8, out); // RBG, NIR
            out += 8;
        }
    }
}


int Dumper::processArgs(std::deque<std::string> args)
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

