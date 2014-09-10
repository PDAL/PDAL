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

#include <pdal/pdal_internal.hpp>

#ifdef PDAL_HAVE_LASZIP

#include <laszip/laszip.hpp>
#include <laszip/laszipper.hpp>

#include <pdal/drivers/las/VariableLengthRecord.hpp>
#include <pdal/drivers/las/ZipPoint.hpp>
#include <string.h>


// std
//#include <vector>
//#include <fstream>
//#include <stdexcept>
//#include <cstdlib> // std::size_t
//#include <cassert>

namespace pdal
{
namespace drivers
{
namespace las
{

static const char* laszip_userid("laszip encoded");
static boost::uint16_t laszip_recordid = 22204;
static const char* laszip_description = "http://laszip.org";


ZipPoint::ZipPoint(PointFormat format, const LasHeader& lasHeader,
                   bool isReadMode)
    : m_readMode(isReadMode)
    , his_vlr_num(0)
    , his_vlr_data(0)
    , m_lz_point(NULL)
    , m_lz_point_size(0)
{
    std::unique_ptr<LASzip> s(new LASzip());
    m_zip.swap(s);

    const VariableLengthRecord* vlr = NULL;
    auto vlrs = lasHeader.getVLRs().getAll();
    for (std::vector<VariableLengthRecord>::size_type i=0; i<vlrs.size(); i++)
    {
        const VariableLengthRecord& p = vlrs[i];
        if (IsZipVLR(p))
        {
            vlr = &p;
            break;
        }
    }

    if (vlr)
    {
        bool ok(false);
        ok = m_zip->unpack(&(vlr->getBytes()[0]), vlr->getLength());
        if (!ok)
        {
            std::ostringstream oss;
            const char* err = m_zip->get_error();
            if (err == NULL) 
                err = "(unknown error)";
            oss << "Error unpacking zip VLR data: " << std::string(err);
            throw pdal_error(oss.str());
        }

    }
    else
    {
        if (m_readMode)
        {
            std::ostringstream oss;
            oss << "Unable to find LASzip VLR, but the file was opened "
                "in read mode!";
            throw pdal_error(oss.str());
        }

        if (!m_zip->setup((boost::uint8_t)format, lasHeader.getPointDataSize()))
        {
            std::ostringstream oss;
            const char* err = m_zip->get_error();
            if (err == NULL)
                err = "(unknown error)";
            oss << "Error setting up LASzip for format " << format << ": " <<
                err;
            throw pdal_error(oss.str());
        }
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

    boost::scoped_array<boost::uint8_t> d(new boost::uint8_t[ m_lz_point_size ]);
    m_lz_point_data.swap(d);
    for (unsigned i = 0; i < m_zip->num_items; i++)
    {
        m_lz_point[i] = &(m_lz_point_data[point_offset]);
        point_offset += m_zip->items[i].size;
    }

    assert(point_offset == m_lz_point_size);
    return;
}


VariableLengthRecord ZipPoint::ConstructVLR() const
{
    unsigned char* data;
    int num;
    m_zip->pack(data, num);

    if (num > std::numeric_limits<boost::uint16_t>::max())
    {
        std::ostringstream oss;
        std::vector<boost::uint8_t>::size_type overrun = num - static_cast<std::vector<boost::uint8_t>::size_type>(std::numeric_limits<boost::uint16_t>::max());
        oss << "The size of the LASzip VLR, " << num << ", is " << overrun
            << " bytes too large to fit inside the maximum size of a VLR which is "
            << std::numeric_limits<boost::uint16_t>::max() << " bytes.";
        throw std::runtime_error(oss.str());
    }

    VariableLengthRecord vlr(0xAABB,
                             laszip_userid,
                             laszip_recordid,
                             laszip_description,
                             data,
                             (boost::uint16_t)num);

    return vlr;
}


bool ZipPoint::IsZipVLR(const VariableLengthRecord& vlr) const
{
    return (laszip_userid == vlr.getUserId() &&
        laszip_recordid == vlr.getRecordId());
}


}
}
} // namespaces

#endif // PDAL_HAVE_LASZIP
