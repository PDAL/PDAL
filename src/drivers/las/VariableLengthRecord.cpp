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

#include <libpc/drivers/las/VariableLengthRecord.hpp>

#include <libpc/SpatialReference.hpp>

#include <libpc/exceptions.hpp>


namespace libpc { namespace drivers { namespace las {

VariableLengthRecord::VariableLengthRecord(boost::uint16_t reserved,
                                           std::string userId, 
                                           boost::uint16_t recordId,
                                           std::string description,
                                           const boost::uint8_t* bytes,
                                           std::size_t length)
    : m_reserved(reserved)
    , m_userId(userId)
    , m_recordId(recordId)
    , m_description(description)
    , m_bytes(0)
    , m_length(length)
{
    m_bytes = new boost::uint8_t[m_length];
    memcpy(m_bytes, bytes, m_length);

    return;
}


VariableLengthRecord::VariableLengthRecord(const VariableLengthRecord& rhs)
    : m_reserved(rhs.m_reserved)
    , m_userId(rhs.m_userId)
    , m_recordId(rhs.m_recordId)
    , m_description(rhs.m_description)
    , m_bytes(0)
    , m_length(rhs.m_length)
{
    m_bytes = new boost::uint8_t[m_length];
    memcpy(m_bytes, rhs.getBytes(), m_length);
    return;
}


VariableLengthRecord::~VariableLengthRecord()
{
    delete m_bytes;
    m_bytes = 0;
    m_length = 0;
    return;
}


VariableLengthRecord& VariableLengthRecord::operator=(const VariableLengthRecord& rhs)
{
    if (&rhs != this)
    {
        m_length = rhs.m_length;
        delete[] m_bytes;
        m_bytes = new boost::uint8_t[m_length];

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

    if (m_length != rhs.m_length) return false;
    for (std::size_t i=0; i<m_length; i++)
    {
        if (m_bytes[i] != rhs.m_bytes[i]) return false;
    }

    return true;
}


std::string VariableLengthRecord::bytes2string(boost::uint8_t* bytes, boost::uint32_t len)
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
        bytes[i] = (boost::uint8_t)str[i];
    }
    return bytes;
}


const boost::uint8_t* VariableLengthRecord::getBytes() const
{
    return m_bytes;
}


std::size_t VariableLengthRecord::getLength() const
{
    return m_length;
}


bool VariableLengthRecord::compareUserId(const std::string& userId) const
{
    const std::string& p = m_userId;
    const std::string& q = userId;
           
    return p==q;
}


void VariableLengthRecord::setSRSFromVLRs_X(const std::vector<VariableLengthRecord>& vlrs, SpatialReference& srs)
{
    srs.geotiff_ResetTags();

    const std::string uid("LASF_Projection");
    
    // nothing is going to happen here if we don't have any vlrs describing
    // srs information on the spatialreference.  
    for (std::size_t i = 0; i < vlrs.size(); ++i)
    {
        const VariableLengthRecord& vlr = vlrs[i];
        
        if (!vlr.compareUserId(uid))
            continue;

        const boost::uint8_t* datax = vlr.getBytes();
        std::size_t length = vlr.getLength();

        // make a writable copy of the array
        boost::uint8_t* data = new boost::uint8_t[length];
        memcpy(data, datax, length);

        switch (vlr.getRecordId())
        {
        case 34735:
            {
                int count = length / sizeof(short);
                // discard invalid "zero" geotags some software emits.
                while( count > 4 
                    && data[count-1] == 0
                    && data[count-2] == 0
                    && data[count-3] == 0
                    && data[count-4] == 0 )
                {
                    count -= 4;
                    data[3] -= 1;
                }

                srs.geotiff_ST_SetKey(34735, count, SpatialReference::Geotiff_KeyType_SHORT, data);
            }
            break;

        case 34736:
            {
                int count = length / sizeof(double);
                srs.geotiff_ST_SetKey(34736, count, SpatialReference::Geotiff_KeyType_DOUBLE, data);
            }        
            break;

        case 34737:
            {
                int count = length/sizeof(uint8_t);
                srs.geotiff_ST_SetKey(34737, count, SpatialReference::Geotiff_KeyType_ASCII, data);
            }
            break;

        default:
            // ummm....?
            break;
        }

        delete[] data;
    }

    srs.geotiff_SetTags();
    
    return;
}


void VariableLengthRecord::setVLRsFromSRS_X(const SpatialReference& srs, std::vector<VariableLengthRecord>& vlrs)
{
    //vlrs.clear();

#ifdef LIBPC_SRS_ENABLED

    int ret = 0;
    short* kdata = 0;
    short kvalue = 0;
    double* ddata = 0;
    double dvalue = 0;
    uint8_t* adata = 0;
    uint8_t avalue = 0;
    int dtype = 0;
    int dcount = 0;
    int ktype = 0;
    int kcount = 0;
    int acount = 0;
    int atype =0;

    const std::string userId123 = "LASF_Projection";
    const std::string description1 = "GeoTIFF GeoKeyDirectoryTag";
    const std::string description2 = "GeoTIFF GeoDoubleParamsTag";
    const std::string description3 = "GeoTIFF GeoAsciiParamsTag";
        
    const std::string userId4 = "liblas";
    const std::string description4 = "OGR variant of OpenGIS WKT SRS";

    //GTIFF_GEOKEYDIRECTORY == 34735
    ret = srs.geotiff_ST_GetKey(34735, &kcount, &ktype, (void**)&kdata);
    if (ret)
    {    
        uint16_t length = 2 * static_cast<uint16_t>(kcount);

        std::vector<uint8_t> data;

        // Copy the data into the data vector
        for (int i = 0; i < kcount; i++)
        {
            kvalue = kdata[i];
            
            uint8_t* v = reinterpret_cast<uint8_t*>(&kvalue); 
            
            data.push_back(v[0]);
            data.push_back(v[1]);
        }

        VariableLengthRecord record(0, userId123, 34735, description1, &data[0], length);
        vlrs.push_back(record);
    }

    // GTIFF_DOUBLEPARAMS == 34736
    ret = srs.geotiff_ST_GetKey(34736, &dcount, &dtype, (void**)&ddata);
    if (ret)
    {    
        uint16_t length = 8 * static_cast<uint16_t>(dcount);
        
        std::vector<uint8_t> data;
       
        // Copy the data into the data vector
        for (int i=0; i<dcount;i++)
        {
            dvalue = ddata[i];
            
            uint8_t* v =  reinterpret_cast<uint8_t*>(&dvalue);
            
            data.push_back(v[0]);
            data.push_back(v[1]);
            data.push_back(v[2]);
            data.push_back(v[3]);
            data.push_back(v[4]);
            data.push_back(v[5]);
            data.push_back(v[6]);
            data.push_back(v[7]);   
        }        

        VariableLengthRecord record(0, userId123, 34736, description2, &data[0], length);

        vlrs.push_back(record);
    }
    
    // GTIFF_ASCIIPARAMS == 34737
    ret = srs.geotiff_ST_GetKey(34737, &acount, &atype, (void**)&adata);
    if (ret) 
    {                    
         uint16_t length = static_cast<uint16_t>(acount);

         std::vector<uint8_t> data;

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
         for (int i=0; i<acount;i++)
         {
             avalue = adata[i];
             uint8_t* v =  reinterpret_cast<uint8_t*>(&avalue);
             data.push_back(v[0]);
         }

         VariableLengthRecord record(0, userId123, 34737, description3, &data[0], length);


        if (data.size() > (std::numeric_limits<boost::uint16_t>::max()))
        {
            std::ostringstream oss;
            std::vector<uint8_t>::size_type overrun = data.size() - static_cast<std::vector<uint8_t>::size_type>(std::numeric_limits<boost::uint16_t>::max());
            oss << "The size of the GeoTIFF GeoAsciiParamsTag, " << data.size() << ", is " << overrun 
                << " bytes too large to fit inside the maximum size of a VLR which is " 
                << (std::numeric_limits<boost::uint16_t>::max()) << " bytes.";
            throw std::runtime_error(oss.str());

        }

         vlrs.push_back(record);
    }
#endif

    std::string wkt = srs.getWKT( SpatialReference::eCompoundOK );

    // Add a WKT VLR if we have a WKT definition.
    if( wkt != "" )
    {
        const uint8_t* wkt_bytes = reinterpret_cast<const uint8_t*>(wkt.c_str());

        boost::uint16_t len = static_cast<boost::uint16_t>(strlen((const char*)wkt_bytes));

        if (len > std::numeric_limits<boost::uint16_t>::max())
        {
            std::ostringstream oss;
            std::vector<uint8_t>::size_type overrun = len - static_cast<std::vector<uint8_t>::size_type>(std::numeric_limits<boost::uint16_t>::max());
            oss << "The size of the wkt, " << len << ", is " << overrun 
                << " bytes too large to fit inside the maximum size of a VLR which is " 
                << std::numeric_limits<boost::uint16_t>::max() << " bytes.";
            throw std::runtime_error(oss.str()); 
        }

        VariableLengthRecord wkt_record(0, userId4, 2112, description4, wkt_bytes, len);

        vlrs.push_back( wkt_record );
    }

    return;
}

void VariableLengthRecord::clearVLRs(GeoVLRType eType, std::vector<VariableLengthRecord>& vlrs)
{
    std::vector<VariableLengthRecord>::iterator it;
    std::string const liblas_id("liblas");
    
    for (it = vlrs.begin(); it != vlrs.end(); )
    {
        VariableLengthRecord const& vlr = *it;
        bool wipe = false;

        // for now we can assume all m_vlrs records are LASF_Projection.
        if (eType == eOGRWKT && 
            2112 == vlr.getRecordId() && 
            vlr.compareUserId(liblas_id))
        {
            wipe = true;
        }
        else if (eType == eGeoTIFF && 
                 (34735 == vlr.getRecordId() || 34736 == vlr.getRecordId() || 34737 == vlr.getRecordId()))
        {
            wipe = true;
        }

        if( wipe )
        {
            it = vlrs.erase( it );
        }
        else
        {
            ++it;
        }
    }

//    if( eType == eOGRWKT )
//        m_wkt = "";
//    else if( eType == eGeoTIFF )
//    {
//#ifdef HAVE_LIBGEOTIFF
//        if (m_gtiff != 0)
//        {
//            GTIFFree(m_gtiff);
//            m_gtiff = 0;
//        }
//        if (m_tiff != 0)
//        {
//            ST_Destroy(m_tiff);
//            m_tiff = 0;
//        }
//#endif
//    }

    return;
}



bool VariableLengthRecord::isGeoVLR() const
{
    std::string const las_projid("LASF_Projection");
    std::string const liblas_id("liblas");
    
    if (compareUserId(las_projid))
    {
        // GTIFF_GEOKEYDIRECTORY == 34735
        if (34735 == getRecordId())
        {
            return true;
        }

        // GTIFF_DOUBLEPARAMS == 34736
        if (34736 == getRecordId())
        {
            return true;
        }

        // GTIFF_ASCIIPARAMS == 34737
        if (34737 == getRecordId())
        {
            return true;
        }
    }

    if (compareUserId(liblas_id))
    {
        // OGR_WKT?
        if (2112 == getRecordId())
        {
            return true;
        }
    }

    return false;
}

//--------------------------------------------------------------------------------------

void VLRList::add(VariableLengthRecord const& v) 
{
    m_list.push_back(v);
}


const VariableLengthRecord& VLRList::get(uint32_t index) const 
{
    return m_list[index];
}


VariableLengthRecord& VLRList::get(uint32_t index)
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


void VLRList::remove(uint32_t index) 
{    
    if (index >= m_list.size())
        throw std::out_of_range("index is out of range");

    std::vector<VariableLengthRecord>::iterator i = m_list.begin() + index;

    m_list.erase(i);

}


static bool sameVLRs(const std::string& name, boost::uint16_t id, const VariableLengthRecord& record)
{
    if (record.compareUserId(name)) 
    {
        if (record.getRecordId() == id) 
        {
            return true;
        }
    }
    return false;
}


void VLRList::remove(const std::string& name, boost::uint16_t id)
{
    m_list.erase( std::remove_if( m_list.begin(), 
                                m_list.end(),
                                boost::bind( &sameVLRs, name, id, _1 ) ),
                m_list.end());

    return;
}


boost::uint32_t VLRList::count() const
{
    return m_list.size();
}
    

SpatialReference VLRList::constructSRS() const
{
    SpatialReference srs;
    VariableLengthRecord::setSRSFromVLRs_X(m_list, srs);
    return srs;
}


void VLRList::addVLRsFromSRS(const SpatialReference& srs)
{
    VariableLengthRecord::setVLRsFromSRS_X(srs, m_list);
}

} } } // namespaces
