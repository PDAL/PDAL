/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include "SlpkReader.hpp"
#include <pdal/util/IStream.hpp>
#include <pdal/compression/GzipCompression.hpp>

#include "EsriUtil.hpp"

namespace pdal
{

static PluginInfo const slpkInfo
{
    "readers.slpk",
    "SLPK Reader",
    "http://pdal.io/stages/readers.slpk.html"
};

CREATE_SHARED_STAGE(SlpkReader, slpkInfo)

std::string SlpkReader::getName() const { return slpkInfo.name; }

SlpkReader::~SlpkReader()
{
    FileUtils::unmapFile(m_ctx);
}


NL::json SlpkReader::initInfo()
{
    // un-archive the slpk archive
    unarchive();

    size_t fileSize = FileUtils::fileSize(m_filename);
    if (fileSize == 0)
        throwError("Empty or invalid SLPK file '" + m_filename + "'.");
    m_ctx = FileUtils::mapFile(m_filename, true, 0 , fileSize);
    if (m_ctx.addr() == nullptr)
        throwError("Error mapping file SLPK file '" + m_filename +
            "': " + m_ctx.what() + ".");

    std::string json = fetchJson("3dSceneLayer");
    NL::json info = i3s::parse(json, "Invalid JSON in '3dSceneLayer.json.gz'.");

    if (info.empty())
        throwError(std::string("Incorrect Json object"));
    return info;
}


void SlpkReader::unarchive()
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
        m_locMap[name] = { (size_t)in.position(), h.m_compressedSize };
        in.skip(h.m_compressedSize);
        in.get(reinterpret_cast<char *>(&magic), sizeof(magic));
    }
}


std::string SlpkReader::fetchJson(std::string filepath)
{
    filepath += ".json.gz";
    auto li = m_locMap.find(filepath);
    if (li == m_locMap.end())
        throwError("Couldn't find file '" + filepath + "' in SLPK file '" +
            m_filename + "'.");

    Location& loc = li->second;

    std::string output;
    const char *c = reinterpret_cast<const char *>(m_ctx.addr()) + loc.m_pos;
    GzipDecompressor decomp(
        [&output](char *buf, size_t bufsize){output.append(buf, buf + bufsize);});
    decomp.decompress(c, loc.m_length);
    decomp.done();
    return output;
}

// fetch data using arbiter to get a char vector
std::vector<char> SlpkReader::fetchBinary(std::string url, std::string attNum,
    std::string ext) const
{
    std::vector<char> output;

    url += attNum + ext;

    auto li = m_locMap.find(url);
    if (li != m_locMap.end())
    {
        const Location& loc = li->second;

        const char *c = reinterpret_cast<const char *>(m_ctx.addr()) + loc.m_pos;
        if (FileUtils::extension(url) != ".gz")
            output.assign(c, c + loc.m_length);
        else
        {
            GzipDecompressor decomp(
                [&output](char *buf, size_t bufsize)
                    {output.insert(output.end(), buf, buf + bufsize);});
            decomp.decompress(c, loc.m_length);
            decomp.done();
        }
    }
    return output;
}

} //namespace pdal
