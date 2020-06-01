/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include <fstream>
#include <algorithm>

#include <pdal/util/IStream.hpp>
#include <pdal/util/FileUtils.hpp>

#include "SlpkExtractor.hpp"
#include "EsriUtil.hpp"

namespace pdal
{

#pragma pack(1)
struct zheader
{
    uint16_t m_version;
    uint16_t m_purpose;
    uint16_t m_compression;
    uint32_t m_time;
    uint32_t m_crc;
    uint32_t m_compressedSize;
    uint32_t m_uncompressedSize;
    uint16_t m_nameLen;
    uint16_t m_extraLen;
};
#pragma pack()


void SlpkExtractor::extract() const
{
    if (!FileUtils::directoryExists(m_directory))
        throw i3s::EsriError("Output directory doesn't exist.");

    ILeStream in(m_filename);

    zheader h;
    std::string name;
    std::vector<char> extra;
    int32_t magic;

    in.get(reinterpret_cast<char *>(&magic), sizeof(magic));
    while (magic == 0x04034b50)
    {
        in >> h.m_version;
        in >> h.m_purpose;
        in >> h.m_compression;
        in >> h.m_time;
        in >> h.m_crc;
        in >> h.m_compressedSize;
        in >> h.m_uncompressedSize;
        in >> h.m_nameLen;
        in >> h.m_extraLen;

        in.get(name, h.m_nameLen);
        if (h.m_extraLen)
        {
            extra.resize(h.m_extraLen);
            in.get(extra);
        }

        if (h.m_compression != 0)
            throw i3s::EsriError("Found compressed file in slpk archive.");
        if (h.m_compressedSize != h.m_uncompressedSize)
            throw i3s::EsriError("Compressed and uncompressed sizes don't "
                "match in slpk archive.");
        writeFile(name, in, h.m_compressedSize);
        in.get(reinterpret_cast<char *>(&magic), sizeof(magic));
    }
}

void SlpkExtractor::writeFile(std::string filename, ILeStream& in,
    size_t count) const
{
    std::string dir = FileUtils::getDirectory(filename);
    dir = m_directory + "/" + dir;
    if (dir.size())
        FileUtils::createDirectories(dir);
    filename = m_directory + "/" + filename;

    std::ofstream out(filename, std::ios_base::out | std::ios_base::binary);
    if (!out)
        throw i3s::EsriError("Couldn't open output file '" + filename + "'.");

    const size_t bufsize(100000);
    char buf[bufsize];
    while (count)
    {
        size_t size = (std::min)(bufsize, count);
        in.get(buf, size);
        out.write(buf, size);
        count -= size;
    }
}

} //namespace pdal
