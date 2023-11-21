/******************************************************************************
* Copyright (c) 2023, Connor Manning (connor@hobu.co)
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

#include "PointlessLas.hpp"

#include "arbiter/arbiter.hpp"

#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/util/OStream.hpp>

namespace pdal
{

namespace
{

arbiter::http::Headers getRangeHeader(int start, int end = 0)
{
    arbiter::http::Headers h;
    h["Range"] = "bytes=" + std::to_string(start) + "-" +
                 (end ? std::to_string(end - 1) : "");
    return h;
}

} // namespace

std::unique_ptr<arbiter::LocalHandle> getPointlessLasFile(const std::string& path)
{
    assert(Utils::isRemote(path));

    const uint64_t maxHeaderSize(375);

    const uint64_t minorVersionPos(25);
    const uint64_t headerSizePos(94);
    const uint64_t pointOffsetPos(96);
    const uint64_t legacyPointCountPos(107);
    const uint64_t evlrOffsetPos(235);
    const uint64_t evlrNumberPos(evlrOffsetPos + 8);
    const uint64_t pointCountPos(247);

    std::string fileSignature;
    uint8_t minorVersion(0);
    uint16_t headerSize(0);
    uint32_t pointOffset(0);
    uint64_t evlrOffset(0);
    uint32_t evlrNumber(0);

    arbiter::Arbiter a;

    std::string header(a.get(path, getRangeHeader(0, maxHeaderSize)));

    std::stringstream headerStream(
        header,
        std::ios_base::in | std::ios_base::out | std::ios_base::binary);

    pdal::ILeStream is(&headerStream);
    pdal::OLeStream os(&headerStream);

    is.seek(0);
    is.get(fileSignature, 4);

    if (fileSignature != "LASF")
    {
        throw std::runtime_error(
            "Invalid file signature for .las or .laz file: must be LASF");
    }

    is.seek(minorVersionPos);
    is >> minorVersion;

    is.seek(headerSizePos);
    is >> headerSize;

    is.seek(pointOffsetPos);
    is >> pointOffset;

    // Set the legacy point count to 0 since we are removing the point data.
    os.seek(legacyPointCountPos);
    os << (uint32_t)0;

    if (minorVersion >= 4)
    {
        is.seek(evlrOffsetPos);
        is >> evlrOffset;

        is.seek(evlrNumberPos);
        is >> evlrNumber;

        // Modify the header such that the EVLRs come directly after the VLRs -
        // removing the point data itself.
        os.seek(evlrOffsetPos);
        os << pointOffset;

        // Set the 1.4 point count to 0 since we are removing the point data.
        os.seek(pointCountPos);
        os << (uint64_t)0;
    }

    // Extract the modified header, VLRs, and append the EVLRs.
    header = headerStream.str();
    std::vector<char> data(header.data(), header.data() + headerSize);

    const bool hasVlrs = headerSize < pointOffset;
    if (hasVlrs)
    {
        const auto vlrs = a.getBinary(
                              path,
                              getRangeHeader(headerSize, pointOffset));
        data.insert(data.end(), vlrs.begin(), vlrs.end());
    }

    const bool hasEvlrs = evlrNumber && evlrOffset;
    if (hasEvlrs)
    {
        const auto evlrs = a.getBinary(path, getRangeHeader(evlrOffset));
        data.insert(data.end(), evlrs.begin(), evlrs.end());
    }

    const std::string extension(arbiter::getExtension(path));
    const std::string basename(
        std::to_string(arbiter::randomNumber()) +
        (extension.size() ? "." + extension : ""));

    const std::string localPath = arbiter::join(arbiter::getTempPath(), basename);
    a.put(localPath, data);
    return std::make_unique<arbiter::LocalHandle>(localPath, /*isRemote=*/ false);
}

} // namespace pdal
