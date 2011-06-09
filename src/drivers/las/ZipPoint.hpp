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

#ifndef INCLUDED_DRIVERS_LAS_ZIPPOINT_HPP
#define INCLUDED_DRIVERS_LAS_ZIPPOINT_HPP


#include <pdal/drivers/las/Support.hpp>

// liblaszip
class LASzipper;
class LASitem;

namespace pdal { namespace drivers { namespace las {

class VariableLengthRecord;

class ZipPoint
{
public:
    ZipPoint(PointFormat, const std::vector<VariableLengthRecord>& vlrs);
    ~ZipPoint();

    VariableLengthRecord ConstructVLR(PointFormat format) const;

    // this will return false iff we find a laszip VLR and it doesn't match
    // the point format this object was constructed with
    bool ValidateVLR(const VariableLengthRecord& vlr) const;

    bool IsZipVLR(const VariableLengthRecord& vlr) const;

private:
    void ConstructItems(PointFormat);

public: // for now
    // LASzip::pack() allocates/sets vlr_data and vlr_num for us, and deletes it for us  ["his"]
    // LASzip::unpack() just reads from the vlr_data we give it (we allocate and delete)  ["our"]
    int his_vlr_num;
    unsigned char* his_vlr_data;
    int our_vlr_num;
    unsigned char* our_vlr_data;

    unsigned int m_num_items;
    LASitem* m_items;
    unsigned char** m_lz_point;
    unsigned char* m_lz_point_data;
    unsigned int m_lz_point_size;
};

} } } // namespace


#endif // LIBLAS_DETAIL_ZIPPOINT_HPP_INCLUDED
