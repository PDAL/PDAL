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

#include <sstream>
#include <string.h>

#include <pdal/pdal_internal.hpp>

#ifdef PDAL_HAVE_LASZIP

#include "VariableLengthRecord.hpp"
#include "ZipPoint.hpp"

namespace pdal
{

// Read-mode ctor.
ZipPoint::ZipPoint(VariableLengthRecord *vlr) :
    m_zip(new LASzip()), m_lz_point(NULL), m_lz_point_size(0)
{
    if (!vlr || !m_zip->unpack((unsigned char *)vlr->data(), vlr->dataLen()))
    {
        std::ostringstream oss;
        const char* err = m_zip->get_error();
        if (err == NULL) 
            err = "(unknown error)";
        oss << "Error unpacking zip VLR data: " << std::string(err);
        throw pdal_error(oss.str());
    }
    ConstructItems();
}


// Write-mode ctor.
ZipPoint::ZipPoint(uint8_t format, uint16_t pointLen) :
    m_zip(new LASzip()), m_lz_point(NULL), m_lz_point_size(0)
{
    if (!m_zip->setup(format, pointLen))
    {
        std::ostringstream oss;
        const char* err = m_zip->get_error();
        if (err == NULL)
            err = "(unknown error)";
        oss << "Error setting up LASzip for format " << format << ": " <<
            err;
        throw pdal_error(oss.str());
    }
    ConstructItems();
}


ZipPoint::~ZipPoint()
{
    delete[] m_lz_point;
}


void ZipPoint::ConstructItems()
{
    // construct the object that will hold a laszip point

    // compute the point size
    m_lz_point_size = 0;
    for (unsigned int i = 0; i < m_zip->num_items; i++)
        m_lz_point_size += m_zip->items[i].size;

    // create the point data
    unsigned int point_offset = 0;
    m_lz_point = new unsigned char*[m_zip->num_items];

    m_lz_point_data.resize(m_lz_point_size);
    for (unsigned i = 0; i < m_zip->num_items; i++)
    {
        m_lz_point[i] = &(m_lz_point_data[point_offset]);
        point_offset += m_zip->items[i].size;
    }
}


std::vector<uint8_t> ZipPoint::vlrData() const
{
    // This puts a bunch of data into an array of data pointed to by 'data',
    // suitable for storage in a VLR.
    uint8_t* data;
    int num;
    m_zip->pack(data, num);
    return std::vector<uint8_t>(data, data + num);
}

} // namespace pdal

#endif // PDAL_HAVE_LASZIP
