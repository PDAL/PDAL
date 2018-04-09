/******************************************************************************
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

#pragma once

#include <pdal/Options.hpp>
#include <pdal/pdal_internal.hpp>

#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wunused-private-field"

#include <nitro/c++/import/nitf.hpp>

#pragma GCC diagnostic pop

namespace pdal
{
    class MetadataNode;
}

namespace pdal
{

//
// all the processing that is NITF-file specific goes in here
//
class PDAL_DLL NitfFileReader
{
public:
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

    NitfFileReader(const std::string& filename);
    NitfFileReader(const NitfFileReader&) = delete;
    NitfFileReader& operator=(const NitfFileReader&) = delete;

    void open();
    void close();
    void getLasOffset(uint64_t& offset, uint64_t& length);
    void extractMetadata(MetadataNode& metadata);

private:
    bool locateLidarImageSegment();
    bool locateLidarDataSegment();

    std::unique_ptr<nitf::IOHandle> m_io;
    nitf::Record m_record;

    std::string m_filename;
    bool m_validLidarSegments;
    nitf::Uint32 m_lidarDataSegment;
};


} // namespaces
