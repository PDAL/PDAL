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

#include <pdal/pdal.hpp>

#ifdef PDAL_HAVE_LASZIP

#include "ZipPoint.hpp"

#include <laszip/laszip.hpp>
#include <laszip/laszipper.hpp>

#include <pdal/exceptions.hpp>
#include <pdal/drivers/las/VariableLengthRecord.hpp>
#include <string.h>


// laszip
#include <laszip/laszip.hpp>

// std
//#include <vector>
//#include <fstream>
//#include <stdexcept>
//#include <cstdlib> // std::size_t
//#include <cassert>

namespace pdal { namespace drivers { namespace las {

static const char* laszip_userid("laszip encoded");
static boost::uint16_t laszip_recordid = 22204;
static const char* laszip_description = "http://laszip.org";


ZipPoint::ZipPoint(PointFormat format, const std::vector<VariableLengthRecord>& vlrs)
    : his_vlr_num(0)
    , his_vlr_data(0)
    , our_vlr_num(0)
    , our_vlr_data(0)
    , m_num_items(0)
    , m_items(NULL)
    , m_lz_point(NULL)
    , m_lz_point_data(NULL)
    , m_lz_point_size(0)
{
    ConstructItems(format);

    const VariableLengthRecord* vlr = NULL;
    for (unsigned int i=0; i<vlrs.size(); i++)
    {
        const VariableLengthRecord& p = vlrs[i];
        if (p.getRecordId() == 22204)
        {
            vlr = &p;
            break;
        }
    }
    if (vlr)
    {
        our_vlr_num = vlr->getLength();
        our_vlr_data = new unsigned char[our_vlr_num];
        for (int i=0; i<our_vlr_num; i++)
        {
            our_vlr_data[i] = vlr->getBytes()[i];
        }
    }

    return;
}

ZipPoint::~ZipPoint()
{
    m_num_items = 0;
    delete[] m_items;
    m_items = NULL;

    delete[] m_lz_point;
    delete[] m_lz_point_data;

    delete[] our_vlr_data;

    return;
}

void ZipPoint::ConstructItems(PointFormat format)
{
    switch (format)
    {
    case PointFormat0:
        m_num_items = 1;
        m_items = new LASitem[1];
        m_items[0].type = LASitem::POINT10;
        m_items[0].size = 20;
        break;

    case PointFormat1:
        m_num_items = 2;
        m_items = new LASitem[2];
        m_items[0].type = LASitem::POINT10;
        m_items[0].size = 8;
        m_items[1].type = LASitem::GPSTIME11;
        m_items[1].size = 8;
        break;

    case PointFormat2:
        m_num_items = 2;
        m_items = new LASitem[2];
        m_items[0].type = LASitem::POINT10;
        m_items[0].size = 20;
        m_items[1].type = LASitem::RGB12;
        m_items[1].size = 6;
        break;

    case PointFormat3:
        m_num_items = 3;
        m_items = new LASitem[3];
        m_items[0].type = LASitem::POINT10;
        m_items[0].size = 20;
        m_items[1].type = LASitem::GPSTIME11;
        m_items[1].size = 8;
        m_items[2].type = LASitem::RGB12;
        m_items[2].size = 6;
        break;

    default:
        throw pdal_error("Bad point format in header"); 
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


VariableLengthRecord ZipPoint::ConstructVLR(PointFormat format) const
{
    unsigned char pointFormat = 0;
    unsigned short pointSize = 0;
    switch (format)
    {
    case PointFormat0:
        pointFormat = 0;
        pointSize = Support::getPointDataSize(format);
        break;
    case PointFormat1:
        pointFormat = 1;
        pointSize = Support::getPointDataSize(format);
        break;
    case PointFormat2:
        pointFormat = 2;
        pointSize = Support::getPointDataSize(format);
        break;
    case PointFormat3:
        pointFormat = 3;
        pointSize = Support::getPointDataSize(format);
        break;
    default:
        throw pdal_error("point format not supported by laszip");
    }

    LASzip laszip;
    laszip.setup(pointFormat, pointSize);

    LASzipper zipper;
    
    unsigned char* data;
    int num;
    laszip.pack(data, num);

    VariableLengthRecord vlr(0xAABB, laszip_userid, laszip_recordid, laszip_description, data, num);

    return vlr;
}


bool ZipPoint::IsZipVLR(const VariableLengthRecord& vlr) const
{
    if (laszip_userid == vlr.getUserId() && laszip_description == vlr.getDescription())
    {
        return true;
    }

    return false;
}


} } } // namespaces

#endif // PDAL_HAVE_LASZIP
