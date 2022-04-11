/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#pragma once

#include <vector>

#include <pdal/pdal_export.hpp>
#include <pdal/util/Uuid.hpp>

namespace pdal
{

class OLeStream;
class ILeStream;
class LasVLR;

namespace las
{
    struct Vlr;
}

[[deprecated]] OLeStream& operator<<(OLeStream& out, const LasVLR& v);
[[deprecated]] std::istream& operator>>(std::istream& in, LasVLR& v);
[[deprecated]] std::ostream& operator<<(std::ostream& out, const LasVLR& v);

static const int WKT_RECORD_ID = 2112;
static const uint16_t GEOTIFF_DIRECTORY_RECORD_ID = 34735;
static const uint16_t GEOTIFF_DOUBLES_RECORD_ID = 34736;
static const uint16_t GEOTIFF_ASCII_RECORD_ID = 34737;
static const uint16_t LASZIP_RECORD_ID = 22204;
static const uint16_t EXTRA_BYTES_RECORD_ID = 4;
static const uint16_t PDAL_METADATA_RECORD_ID = 12;
static const uint16_t PDAL_PIPELINE_RECORD_ID = 13;

static const char TRANSFORM_USER_ID[] = "LASF_Projection";
static const char SPEC_USER_ID[] = "LASF_Spec";
static const char LIBLAS_USER_ID[] = "liblas";
static const char LASZIP_USER_ID[] = "laszip encoded";
static const char PDAL_USER_ID[] = "PDAL";

class LasVLR;
typedef std::vector<LasVLR> VlrList;

class PDAL_DLL LasVLR
{
public:
    static const uint16_t MAX_DATA_SIZE = 65535;

    std::string userId() const;
    uint16_t recordId() const;
    std::string description() const;

    bool matches(const std::string& userId) const;
    bool matches(const std::string& userId, uint16_t recordId) const;

    const char* data() const;
    [[deprecated]] char* data();
    bool isEmpty() const;
    uint64_t dataLen() const;
    [[deprecated]] void setDataLen(uint64_t size);
    [[deprecated]] void write(OLeStream& out, uint16_t recordSig);
    [[deprecated]] bool read(ILeStream& in, size_t limit);

    LasVLR(las::Vlr *v);
    LasVLR(const LasVLR& v);
    LasVLR(LasVLR&& v);
    LasVLR& operator=(const LasVLR& src);
    LasVLR& operator=(LasVLR&& src);
    ~LasVLR();

    friend OLeStream& operator<<(OLeStream& out, const LasVLR& v);
    friend std::istream& operator>>(std::istream& in, LasVLR& v);
    friend std::ostream& operator<<(std::ostream& out, const LasVLR& v);

private:
    struct Private;
    std::unique_ptr<Private> d;
};

} // namespace pdal
