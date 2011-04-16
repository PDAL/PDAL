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

#include <libpc/libpc.hpp>

#ifdef LIBPC_HAVE_LASZIP

#include "ZipPoint.hpp"

#include <libpc/exceptions.hpp>

// laszip
#include <laszip/laszip.hpp>

// std
//#include <vector>
//#include <fstream>
//#include <stdexcept>
//#include <cstdlib> // std::size_t
//#include <cassert>

namespace libpc { namespace drivers { namespace las {

static std::string laszip_userid("laszip encoded");
static boost::uint16_t laszip_recordid = 22204;
static std::string laszip_description = "encoded for sequential access";


ZipPoint::ZipPoint(PointFormat format) :
    m_num_items(0),
    m_items(NULL),
    m_lz_point(NULL),
    m_lz_point_data(NULL),
    m_lz_point_size(0)
{
    ConstructItems(format);
    return;
}

ZipPoint::~ZipPoint()
{
    m_num_items = 0;
    delete[] m_items;
    m_items = NULL;

    delete[] m_lz_point;
    delete[] m_lz_point_data;

    return;
}

void ZipPoint::ConstructItems(PointFormat format)
{
    switch (format)
    {
    case PointFormat0:
        m_num_items = 1;
        m_items = new LASitem[1];
        m_items[0].set(LASitem::POINT10);
        break;

    case PointFormat1:
        m_num_items = 2;
        m_items = new LASitem[2];
        m_items[0].set(LASitem::POINT10);
        m_items[1].set(LASitem::GPSTIME11);
        break;

    case PointFormat2:
        m_num_items = 2;
        m_items = new LASitem[2];
        m_items[0].set(LASitem::POINT10);
        m_items[1].set(LASitem::RGB12);
        break;

    case PointFormat3:
        m_num_items = 3;
        m_items = new LASitem[3];
        m_items[0].set(LASitem::POINT10);
        m_items[1].set(LASitem::GPSTIME11);
        m_items[2].set(LASitem::RGB12);
        break;

    case PointFormat4:
        m_num_items = 3;
        m_items = new LASitem[3];
        m_items[0].set(LASitem::POINT10);
        m_items[1].set(LASitem::GPSTIME11);
        m_items[2].set(LASitem::WAVEPACKET13);
        break;

    default:
        throw libpc_error("Bad point format in header"); 
    }

    // construct the object that will hold a laszip point

    // compute the point size
    m_lz_point_size = 0;
    for (unsigned int i = 0; i < m_num_items; i++) 
        m_lz_point_size += m_items[i].size;

    // create the point data
    unsigned int point_offset = 0;
    m_lz_point = new unsigned char*[m_num_items];
    m_lz_point_data = new unsigned char[m_lz_point_size];
    for (unsigned i = 0; i < m_num_items; i++)
    {
        m_lz_point[i] = &(m_lz_point_data[point_offset]);
        point_offset += m_items[i].size;
    }

    return;
}

template<class T>
static inline boost::uint8_t* PutBytes(boost::uint8_t* p, T d)
{
    LIBLAS_SWAP_BYTES(d);
    memcpy(p, &d, sizeof(T));
    return p+sizeof(T);
}

template<class T>
static inline boost::uint8_t* GetBytes(boost::uint8_t* p, T& d)
{
    memcpy(&d, p, sizeof(T));
    LIBLAS_SWAP_BYTES(d);
    return p+sizeof(T);
}


#if 0
void ZipPoint::ConstructVLR(VariableRecord& v) const
{
    boost::uint16_t record_length_after_header = (boost::uint16_t)(34+6*m_num_items);

    // set the header
    v.SetReserved(0xAABB);
    v.SetUserId(laszip_userid);
    v.SetRecordId(laszip_recordid);
    v.SetRecordLength(record_length_after_header);
    v.SetDescription(laszip_description);

    // the data following the header of the variable length record is
    //     U32  compression        4 bytes   0
    //     U8   version_major      1 byte    4
    //     U8   version_minor      1 byte    5
    //     U16  version_revision   2 bytes   6
    //     U32  options            4 bytes   8
    //     U32  num_chunks         4 bytes   12
    //     I64  num_points         8 bytes   16
    //     I64  num_bytes          8 bytes   24
    //     U16  num_items          2 bytes   32
    //        U16 type                2 bytes * num_items
    //        U16 size                2 bytes * num_items
    //        U16 version             2 bytes * num_items
    // which totals 34+6*num_items

    boost::uint8_t* data = new boost::uint8_t[record_length_after_header];
    boost::uint8_t* p = data;

    // the header doesn't know what kind of compression the zipwriter 
    // will be doing, but since we only ever use the default we'll just
    // use that for now
    boost::uint32_t compression_type = LASzip::DEFAULT_COMPRESSION;
    p = PutBytes<boost::uint32_t>(p, compression_type);

    boost::uint8_t version_major = LASZIP_VERSION_MAJOR;
    p = PutBytes<boost::uint8_t>(p, version_major);
    boost::uint8_t version_minor = LASZIP_VERSION_MINOR;
    p = PutBytes<boost::uint8_t>(p, version_minor);
    boost::uint16_t version_revision = LASZIP_VERSION_REVISION;
    p = PutBytes<boost::uint16_t>(p, version_revision);
    
    boost::uint32_t options = 0;
    p = PutBytes<boost::uint32_t>(p, options);
    boost::uint32_t num_chunks = 1;
    p = PutBytes<boost::uint32_t>(p, num_chunks);
    boost::int64_t num_points = -1;
    p = PutBytes<boost::int64_t>(p, num_points);
    boost::int64_t num_bytes = -1;
    p = PutBytes<boost::int64_t>(p, num_bytes);
    boost::uint16_t num_items = (boost::uint16_t)m_num_items;
    p = PutBytes<boost::uint16_t>(p, num_items);

    for (boost::uint32_t i = 0; i < num_items; i++)
    {
        boost::uint16_t type = (boost::uint16_t)m_items[i].type;
        p = PutBytes<boost::uint16_t>(p, type);
        boost::uint16_t size = (boost::uint16_t)m_items[i].size;
        p = PutBytes<boost::uint16_t>(p, size);
        boost::uint16_t version = (boost::uint16_t)m_items[i].version;
        p = PutBytes<boost::uint16_t>(p, version);
    }

    assert(p == data + record_length_after_header);

    // Ick.
    std::vector<boost::uint8_t> vdata;
    for (boost::uint32_t i=0; i<record_length_after_header; i++)
    {
        vdata.push_back(data[i]);
    }
        
    v.SetData(vdata);
    
    if (data != 0)
    delete [] data;

    return;
}


bool ZipPoint::ValidateVLR(std::vector<VariableRecord> const& vlrs) const
{
    std::vector<VariableRecord>::const_iterator it;
    for (it = vlrs.begin(); it != vlrs.end(); ++it)
    {
        VariableRecord const& vlr = *it;
        if (IsZipVLR(vlr))
        {
            return ValidateVLR(vlr);
        }
    }

    // if we didn't find one, but we don't care (for now)
    return true;
}

bool ZipPoint::IsZipVLR(const VariableRecord& vlr) const
{
    if (laszip_userid == vlr.GetUserId(true).c_str() && laszip_recordid == vlr.GetRecordId())
    {
        return true;
    }

    return false;
}

bool ZipPoint::ValidateVLR(const VariableRecord& vlr) const
{
    if (laszip_userid != vlr.GetUserId(true).c_str())
        return false;
    
    if (laszip_recordid != vlr.GetRecordId())
        return false;

    // (all validation checking is now done inside the liblas library)

    return true;
}
#endif

} } } // namespaces

#endif // LIBPC_HAVE_LASZIP
