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

#ifdef PDAL_HAVE_LASZIP
#include <laszip/laszip.hpp>
#include <laszip/lasunzipper.hpp>
#include <laszip/laszipper.hpp>
#endif

namespace pdal
{

#ifdef PDAL_HAVE_LASZIP

class VariableLengthRecord;

class PDAL_DLL ZipPoint
{
public:
    ZipPoint(VariableLengthRecord *lasHeader);
    ZipPoint(uint8_t format, uint16_t pointLen);
    ~ZipPoint();

    std::vector<uint8_t> vlrData() const;
    LASzip* GetZipper() const
        { return m_zip.get(); }
    
private:
    std::unique_ptr<LASzip> m_zip;

//ABELL - This block should be made private.
public:
    unsigned char** m_lz_point;
    unsigned int m_lz_point_size;
    std::vector<uint8_t> m_lz_point_data;

private:
    void ConstructItems();
};
#else // PDAL_HAVE_LASZIP
// The types here just need to be something suitable for a smart pointer.
// They aren't ever used beyond testing for NULL.
typedef char LASzipper;
typedef char LASunzipper;
typedef char ZipPoint;
#endif

} // namespace pdal
