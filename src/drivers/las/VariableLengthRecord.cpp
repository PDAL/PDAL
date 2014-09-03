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

#include <pdal/drivers/las/VariableLengthRecord.hpp>
#include <boost/cstdint.hpp>
#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <pdal/SpatialReference.hpp>
#include "GeotiffSupport.hpp"
#include <sstream>
#include <functional>
#include <cstring>


namespace pdal
{
namespace drivers
{
namespace las
{

static const std::string s_geotiffUserId = "LASF_Projection";
static const boost::uint16_t s_geotiffRecordId_directory = 34735;
static const boost::uint16_t s_geotiffRecordId_doubleparams = 34736;
static const boost::uint16_t s_geotiffRecordId_asciiparams = 34737;
static const std::string s_geotiffDescriptionId_directory = "GeoTIFF GeoKeyDirectoryTag";
static const std::string s_geotiffDescriptionId_doubleparams = "GeoTIFF GeoDoubleParamsTag";
static const std::string s_geotiffDescriptionId_asciiparams = "GeoTIFF GeoAsciiParamsTag";

static const std::string s_wktUserId = "liblas";
static const boost::uint16_t s_wktRecordId = 2112;
static const std::string s_wktDescription = "OGR variant of OpenGIS WKT SRS";

//const int VariableLengthRecord::s_headerLength = 54;

VariableLengthRecord::VariableLengthRecord(boost::uint16_t reserved,
        std::string userId,
        boost::uint16_t recordId,
        std::string description,
        const boost::uint8_t* bytes,
        boost::uint16_t length)
    : m_reserved(0xAABB)
    , m_userId(userId)
    , m_recordId(recordId)
    , m_description(description)
    , m_bytes(0)
    , m_vlr_length(length)
{
    boost::ignore_unused_variable_warning(reserved);

    m_bytes = new boost::uint8_t[m_vlr_length];
    memcpy(m_bytes, bytes, m_vlr_length);

    return;
}


VariableLengthRecord::VariableLengthRecord(const VariableLengthRecord& rhs)
    : m_reserved(rhs.m_reserved)
    , m_userId(rhs.m_userId)
    , m_recordId(rhs.m_recordId)
    , m_description(rhs.m_description)
    , m_bytes(0)
    , m_vlr_length(rhs.m_vlr_length)
{
    m_bytes = new boost::uint8_t[m_vlr_length];
    memcpy(m_bytes, rhs.getBytes(), m_vlr_length);
    return;
}


VariableLengthRecord::~VariableLengthRecord()
{
    delete[] m_bytes;
    m_bytes = 0;
    m_vlr_length = 0;
    return;
}


VariableLengthRecord& VariableLengthRecord::operator=(const VariableLengthRecord& rhs)
{
    if (&rhs != this)
    {
        m_vlr_length = rhs.m_vlr_length;
        delete[] m_bytes;
        m_bytes = new boost::uint8_t[m_vlr_length];
        memcpy(m_bytes, rhs.m_bytes, m_vlr_length);

        m_reserved = rhs.m_reserved;
        m_recordId = rhs.m_recordId;

        m_userId = rhs.m_userId;
        m_description = rhs.m_description;
    }

    return *this;
}


bool VariableLengthRecord::operator==(const VariableLengthRecord& rhs) const
{
    if (m_reserved != rhs.m_reserved) return false;
    if (m_recordId != rhs.m_recordId) return false;

    if (m_userId != rhs.m_userId) return false;
    if (m_description != rhs.m_description) return false;

    if (m_vlr_length != rhs.m_vlr_length) return false;
    for (std::size_t i=0; i<m_vlr_length; i++)
    {
        if (m_bytes[i] != rhs.m_bytes[i]) return false;
    }

    return true;
}


std::string VariableLengthRecord::bytes2string(const boost::uint8_t* bytes, boost::uint32_t len)
{
    std::string s = "";
    for (boost::uint32_t i=0; i<len; i++)
    {
        if (bytes[i]==0) break;
        s += (char)bytes[i];
    }
    return s;
}


boost::uint8_t* VariableLengthRecord::string2bytes(boost::uint32_t len, const std::string& str)
{
    boost::uint8_t* bytes = new boost::uint8_t[len];
    memset(bytes, 0, len);

    assert(str.length() <= len);
    for (boost::uint32_t i=0; i<str.length(); i++)
    {
        bytes[i] = static_cast<boost::uint8_t>(str[i]);
    }
    return bytes;
}


const boost::uint8_t* VariableLengthRecord::getBytes() const
{
    return m_bytes;
}


std::size_t VariableLengthRecord::getLength() const
{
    return m_vlr_length;
}

std::size_t VariableLengthRecord::getTotalSize() const
{
    // Signature 2 bytes
    // UserID 16 bytes
    // RecordID 2 bytes
    // RecordLength after Header 2 bytes
    // Description 32 bytes
    // Data length -- size of the data's vector * the size of uint8_t
    // std::size_t sum = 2 + 16 + 2 + 2 + 32 + m_vlr_length * sizeof(boost::uint8_t);
    std::size_t sum = sizeof(m_reserved) +
                      eUserIdSize +
                      sizeof(m_recordId) +
                      sizeof(m_vlr_length) +
                      eDescriptionSize +
                      m_vlr_length * sizeof(boost::uint8_t);
    return sum;

}


bool VariableLengthRecord::isMatch(const std::string& userId) const
{
    return (userId == m_userId);
}


bool VariableLengthRecord::isMatch(const std::string& userId, boost::uint16_t recordId) const
{
    return (userId == m_userId && recordId == m_recordId);
}


static bool setSRSFromVLRs_wkt(const std::vector<VariableLengthRecord>& vlrs, SpatialReference& srs)
{
    for (std::size_t i = 0; i < vlrs.size(); ++i)
    {
        const VariableLengthRecord& vlr = vlrs[i];

        if (vlr.isMatch(s_wktUserId, s_wktRecordId))
        {
            const boost::uint8_t* data = vlr.getBytes();
            std::size_t length = vlr.getLength();

            const std::string wkt = VariableLengthRecord::bytes2string(data, length);
            srs.setWKT(wkt);
            return true;
        }
    }

    return false;
}


static bool setSRSFromVLRs_geotiff(const std::vector<VariableLengthRecord>& vlrs, SpatialReference& srs)
{
#ifdef PDAL_SRS_ENABLED
    GeotiffSupport geotiff;

    geotiff.resetTags();

    bool gotSomething = false;

    // first we try to get the 2212 VLR

    // nothing is going to happen here if we don't have any vlrs describing
    // srs information on the spatialreference.
    for (std::size_t i = 0; i < vlrs.size(); ++i)
    {
        const VariableLengthRecord& vlr = vlrs[i];

        if (!vlr.isMatch(s_geotiffUserId))
            continue;

        const boost::uint8_t* datax = vlr.getBytes();
        std::size_t length = vlr.getLength();

        // make a writable copy of the array
        boost::uint8_t* data = new boost::uint8_t[length];
        memcpy(data, datax, length);

        switch (vlr.getRecordId())
        {
            case s_geotiffRecordId_directory:
                {
                    int count = length / sizeof(short);
                    // discard invalid "zero" geotags some software emits.
                    while (count > 4
                            && data[count-1] == 0
                            && data[count-2] == 0
                            && data[count-3] == 0
                            && data[count-4] == 0)
                    {
                        count -= 4;
                        data[3] -= 1;
                    }

                    geotiff.setKey(s_geotiffRecordId_directory, count, GeotiffSupport::Geotiff_KeyType_SHORT, data);
                }
                gotSomething = true;
                break;

            case s_geotiffRecordId_doubleparams:
                {
                    int count = length / sizeof(double);
                    geotiff.setKey(s_geotiffRecordId_doubleparams, count, GeotiffSupport::Geotiff_KeyType_DOUBLE, data);
                }
                gotSomething = true;
                break;

            case s_geotiffRecordId_asciiparams:
                {
                    int count = length/sizeof(boost::uint8_t);
                    geotiff.setKey(s_geotiffRecordId_asciiparams, count, GeotiffSupport::Geotiff_KeyType_ASCII, data);
                }
                gotSomething = true;
                break;

            default:
                // ummm....?
                break;
        }

        delete[] data;
    }

    if (!gotSomething)
    {
        return false;
    }

    geotiff.setTags();

    const std::string wkt = geotiff.getWkt(false,false);

    srs.setFromUserInput(wkt);
#else
    boost::ignore_unused_variable_warning(srs);
    boost::ignore_unused_variable_warning(vlrs);
#endif
    return true;
}


void VariableLengthRecord::setSRSFromVLRs(const std::vector<VariableLengthRecord>& vlrs, SpatialReference& srs)
{
    srs.setWKT("");

    if (vlrs.size() == 0)
    {
        return;
    }

    bool ok = setSRSFromVLRs_wkt(vlrs, srs);
    if (ok)
    {
        return;
    }

    ok = setSRSFromVLRs_geotiff(vlrs, srs);
    if (ok)
    {
        return;
    }

    return;
}


void VariableLengthRecord::setVLRsFromSRS(const SpatialReference& srs, std::vector<VariableLengthRecord>& vlrs, SpatialReference::WKTModeFlag modeFlag)
{
#ifdef PDAL_SRS_ENABLED

    int ret = 0;
    short* kdata = 0;
    short kvalue = 0;
    double* ddata = 0;
    double dvalue = 0;
    boost::uint8_t* adata = 0;
    boost::uint8_t avalue = 0;
    int dtype = 0;
    int dcount = 0;
    int ktype = 0;
    int kcount = 0;
    int acount = 0;
    int atype =0;

    GeotiffSupport geotiff;
    {
        const std::string wkt = srs.getWKT(modeFlag, false);
        geotiff.setWkt(wkt);
    }

    ret = geotiff.getKey(s_geotiffRecordId_directory, &kcount, &ktype, (void**)&kdata);
    if (ret)
    {
        boost::uint16_t length = 2 * static_cast<boost::uint16_t>(kcount);

        std::vector<boost::uint8_t> data;

        // Copy the data into the data vector
        for (int i = 0; i < kcount; i++)
        {
            kvalue = kdata[i];

            boost::uint8_t* v = reinterpret_cast<boost::uint8_t*>(&kvalue);

            data.push_back(v[0]);
            data.push_back(v[1]);
        }

        VariableLengthRecord record(0, s_geotiffUserId, s_geotiffRecordId_directory, s_geotiffDescriptionId_directory, &data[0], length);
        vlrs.push_back(record);
    }

    ret = geotiff.getKey(s_geotiffRecordId_doubleparams, &dcount, &dtype, (void**)&ddata);
    if (ret)
    {
        boost::uint16_t length = 8 * static_cast<boost::uint16_t>(dcount);

        std::vector<boost::uint8_t> data;

        // Copy the data into the data vector
        for (int i=0; i<dcount; i++)
        {
            dvalue = ddata[i];

            boost::uint8_t* v =  reinterpret_cast<boost::uint8_t*>(&dvalue);

            data.push_back(v[0]);
            data.push_back(v[1]);
            data.push_back(v[2]);
            data.push_back(v[3]);
            data.push_back(v[4]);
            data.push_back(v[5]);
            data.push_back(v[6]);
            data.push_back(v[7]);
        }

        VariableLengthRecord record(0, s_geotiffUserId, s_geotiffRecordId_doubleparams, s_geotiffDescriptionId_doubleparams, &data[0], length);

        vlrs.push_back(record);
    }

    ret = geotiff.getKey(s_geotiffRecordId_asciiparams, &acount, &atype, (void**)&adata);
    if (ret)
    {
        boost::uint16_t length = static_cast<boost::uint16_t>(acount);

        std::vector<boost::uint8_t> data;

        // whoa.  If the returned count was 0, it is because there
        // was a bug in libgeotiff that existed before r1531 where it
        // didn't calculate the string length for an ASCII geotiff tag.
        // We need to throw an exception in this case because we're
        // screwed, and if we don't we'll end up writing bad GeoTIFF keys.
        if (!acount)
        {
            throw std::runtime_error("GeoTIFF ASCII key with no returned size. "
                                     "Upgrade your libgeotiff to a version greater "
                                     "than r1531 (libgeotiff 1.2.6)");
        }

        // Copy the data into the data vector
        for (int i=0; i<acount; i++)
        {
            avalue = adata[i];
            boost::uint8_t* v =  reinterpret_cast<boost::uint8_t*>(&avalue);
            data.push_back(v[0]);
        }

        VariableLengthRecord record(0, s_geotiffUserId, s_geotiffRecordId_asciiparams, s_geotiffDescriptionId_asciiparams, &data[0], length);


        if (data.size() > (std::numeric_limits<boost::uint16_t>::max()))
        {
            std::ostringstream oss;
            std::vector<boost::uint8_t>::size_type overrun = data.size() - static_cast<std::vector<boost::uint8_t>::size_type>(std::numeric_limits<boost::uint16_t>::max());
            oss << "The size of the GeoTIFF GeoAsciiParamsTag, " << data.size() << ", is " << overrun
                << " bytes too large to fit inside the maximum size of a VLR which is "
                << (std::numeric_limits<boost::uint16_t>::max()) << " bytes.";
            throw std::runtime_error(oss.str());

        }

        vlrs.push_back(record);
    }
#endif
    boost::ignore_unused_variable_warning(modeFlag);

    std::string wkt = srs.getWKT(SpatialReference::eCompoundOK);

    // Add a WKT VLR if we have a WKT definition.
    if (wkt != "")
    {
        const boost::uint8_t* wkt_bytes = reinterpret_cast<const boost::uint8_t*>(wkt.c_str());

        boost::uint32_t len = static_cast<boost::uint32_t>(strlen((const char*)wkt_bytes));

        if (len > std::numeric_limits<boost::uint16_t>::max())
        {
            std::ostringstream oss;
            std::vector<boost::uint8_t>::size_type overrun = len - static_cast<std::vector<boost::uint8_t>::size_type>(std::numeric_limits<boost::uint16_t>::max());
            oss << "The size of the wkt, " << len << ", is " << overrun
                << " bytes too large to fit inside the maximum size of a VLR which is "
                << std::numeric_limits<boost::uint16_t>::max() << " bytes.";
            throw std::runtime_error(oss.str());
        }

        VariableLengthRecord wkt_record(0, s_wktUserId, s_wktRecordId, s_wktDescription, wkt_bytes, static_cast<boost::uint16_t>(len));

        vlrs.push_back(wkt_record);
    }

    return;
}


void VariableLengthRecord::clearVLRs(GeoVLRType eType, std::vector<VariableLengthRecord>& vlrs)
{
    std::vector<VariableLengthRecord>::iterator it;

    for (it = vlrs.begin(); it != vlrs.end();)
    {
        VariableLengthRecord const& vlr = *it;
        bool wipe = false;

        if (eType == eOGRWKT && vlr.isMatch(s_wktUserId, s_wktRecordId))
        {
            wipe = true;
        }
        else if (eType == eGeoTIFF &&
                 (s_geotiffRecordId_directory == vlr.getRecordId() ||
                  s_geotiffRecordId_doubleparams == vlr.getRecordId() ||
                  s_geotiffRecordId_asciiparams == vlr.getRecordId()))
        {
            wipe = true;
        }

        if (wipe)
        {
            it = vlrs.erase(it);
        }
        else
        {
            ++it;
        }
    }

    return;
}


bool VariableLengthRecord::isGeoVLR() const
{
    if (isMatch(s_geotiffUserId))
    {
        if (s_geotiffRecordId_directory == getRecordId())
        {
            return true;
        }

        if (s_geotiffRecordId_doubleparams == getRecordId())
        {
            return true;
        }

        if (s_geotiffRecordId_asciiparams == getRecordId())
        {
            return true;
        }
    }

    if (isMatch(s_wktUserId, s_wktRecordId))
    {
        return true;
    }

    return false;
}


//--------------------------------------------------------------------------------------

void VLRList::add(VariableLengthRecord const& v)
{
    m_list.push_back(v);
}


const VariableLengthRecord& VLRList::get(boost::uint32_t index) const
{
    return m_list[index];
}


VariableLengthRecord& VLRList::get(boost::uint32_t index)
{
    return m_list[index];
}


const std::vector<VariableLengthRecord>& VLRList::getAll() const
{
    return m_list;
}


std::vector<VariableLengthRecord>& VLRList::getAll()
{
    return m_list;
}


void VLRList::remove(boost::uint32_t index)
{
    if (index >= m_list.size())
        throw std::out_of_range("index is out of range");

    std::vector<VariableLengthRecord>::iterator i = m_list.begin() + index;

    m_list.erase(i);

}


static bool sameVLRs(const std::string& userId, boost::uint16_t recordId, const VariableLengthRecord& vlr)
{
    return vlr.isMatch(userId, recordId);
}


void VLRList::remove(const std::string& name, boost::uint16_t id)
{
    m_list.erase(std::remove_if(m_list.begin(),
                                m_list.end(),
                                std::bind(&sameVLRs, name, id, std::placeholders::_1)),
                 m_list.end());

    return;
}


boost::uint32_t VLRList::count() const
{
    return m_list.size();
}


void VLRList::constructSRS(SpatialReference& srs) const
{
    VariableLengthRecord::setSRSFromVLRs(m_list, srs);
    return;
}


void VLRList::addVLRsFromSRS(const SpatialReference& srs, SpatialReference::WKTModeFlag modeFlag)
{
    VariableLengthRecord::setVLRsFromSRS(srs, m_list, modeFlag);
}

}
}
} // namespaces
