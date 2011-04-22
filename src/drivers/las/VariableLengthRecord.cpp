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
                                           boost::uint8_t* userId, 
                                           boost::uint16_t recordId,
                                           boost::uint8_t* description,
                                           const boost::uint8_t* bytes, std::size_t len)
    : MetadataRecord(bytes, len)
    , m_reserved(reserved)
    , m_recordId(recordId)
{
    m_userId = new boost::uint8_t[16];
    memcpy(m_userId, userId, 16);
    m_description = new boost::uint8_t[32];
    memcpy(m_description, description, 32);
    return;
}


VariableLengthRecord::VariableLengthRecord(const VariableLengthRecord& vlr)
    : MetadataRecord(vlr)
    , m_reserved(vlr.m_reserved)
    , m_recordId(vlr.m_recordId)
{
    m_userId = new boost::uint8_t[16];
    memcpy(m_userId, vlr.m_userId, 16);
    m_description = new boost::uint8_t[32];
    memcpy(m_description, vlr.m_description, 32);
    return;
}


VariableLengthRecord::~VariableLengthRecord()
{
    delete[] m_userId;
    delete[] m_description;
    return;
}


VariableLengthRecord& VariableLengthRecord::operator=(const VariableLengthRecord& vlr)
{
    (MetadataRecord&)vlr = *(MetadataRecord*)this;

    m_reserved = vlr.m_reserved;
    m_recordId = vlr.m_recordId;

    memcpy(m_userId, vlr.m_userId, 16);
    memcpy(m_description, vlr.m_description, 32);

    return *this;
}


bool VariableLengthRecord::operator==(const VariableLengthRecord& vlr) const
{
    if (m_reserved != vlr.m_reserved) return false;
    if (m_recordId != vlr.m_recordId) return false;

    for (int i=0; i<16; i++)
        if (m_userId[i] != vlr.m_userId[i]) return false;
    for (int i=0; i<32; i++)
        if (m_description[i] != vlr.m_description[i]) return false;

    if (!((MetadataRecord&)vlr == *(MetadataRecord*)this)) return false;

    return true;
}


bool VariableLengthRecord::compareUserId(const std::string& str) const
{
    int len = str.length();
    if (memcmp(getUserId(), str.c_str(), len) == 0)
        return true;
    return false;
}


void VariableLengthRecord::setSRSFromVLRs(const std::vector<VariableLengthRecord>& vlrs, SpatialReference& srs)
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

        boost::shared_array<boost::uint8_t> data = vlr.getBytes();
        std::size_t length = vlr.getLength();

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

                srs.geotiff_ST_SetKey(34735, count, SpatialReference::Geotiff_KeyType_SHORT, data.get());
            }
            break;

        case 34736:
            {
                int count = length / sizeof(double);
                srs.geotiff_ST_SetKey(34736, count, SpatialReference::Geotiff_KeyType_DOUBLE, data.get());
            }        
            break;

        case 34737:
            {
                int count = length/sizeof(uint8_t);
                srs.geotiff_ST_SetKey(34737, count, SpatialReference::Geotiff_KeyType_ASCII, data.get());
            }
            break;

        default:
            // ummm....?
            break;
        }
    }

    srs.geotiff_SetTags();
    
    return;
}


void VariableLengthRecord::setVLRsFromSRS(const SpatialReference& srs, std::vector<VariableLengthRecord>& vlrs)
{
    vlrs.clear();

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
    
    //////if (!m_tiff)
    //////    throw std::invalid_argument("m_tiff was null, cannot reset VLRs without m_tiff");

    //////if (!m_gtiff)
    //////    throw std::invalid_argument("m_gtiff was null, cannot reset VLRs without m_gtiff");

    //GTIFF_GEOKEYDIRECTORY == 34735
    ret = srs.geotiff_ST_GetKey(34735, &kcount, &ktype, (void**)&kdata);
    if (ret)
    {    
        boost::uint8_t userid[16];
        for (int i=0; i<16; i++) userid[i]=0;
        memcpy((char*)userid,(char*)"LASF_Projection",strlen("LASF_Projection"));

        boost::uint8_t description[32];
        for (int i=0; i<32; i++) description[i]=0;
        memcpy((char*)userid,(char*)"GeoTIFF GeoKeyDirectoryTag",strlen("GeoTIFF GeoKeyDirectoryTag"));

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

        VariableLengthRecord record(0, userid, 34735, description, &data[0], length);
        vlrs.push_back(record);
    }

    // GTIFF_DOUBLEPARAMS == 34736
    ret = srs.geotiff_ST_GetKey(34736, &dcount, &dtype, (void**)&ddata);
    if (ret)
    {    
        boost::uint8_t userid[16];
        for (int i=0; i<16; i++) userid[i]=0;
        memcpy((char*)userid,(char*)"LASF_Projection",strlen("LASF_Projection"));

        boost::uint8_t description[32];
        for (int i=0; i<32; i++) description[i]=0;
        memcpy((char*)userid,(char*)"GeoTIFF GeoDoubleParamsTag",strlen("GeoTIFF GeoDoubleParamsTag"));

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

        VariableLengthRecord record(0, userid, 34736, description, &data[0], length);

        vlrs.push_back(record);
    }
    
    // GTIFF_ASCIIPARAMS == 34737
    ret = srs.geotiff_ST_GetKey(34737, &acount, &atype, (void**)&adata);
    if (ret) 
    {                    
        boost::uint8_t userid[16];
        for (int i=0; i<16; i++) userid[i]=0;
        memcpy((char*)userid,(char*)"LASF_Projection",strlen("LASF_Projection"));

        boost::uint8_t description[32];
        for (int i=0; i<32; i++) description[i]=0;
        memcpy((char*)userid,(char*)"GeoTIFF GeoAsciiParamsTag",strlen("GeoTIFF GeoAsciiParamsTag"));
        
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

         VariableLengthRecord record(0, userid, 34737, description, &data[0], length);


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
#endif // ndef HAVE_LIBGEOTIFF


    std::string wkt = srs.getWKT( SpatialReference::eCompoundOK );

    // Add a WKT VLR if we have a WKT definition.
    if( wkt != "" )
    {
        std::vector<uint8_t> data;
        const uint8_t* wkt_bytes = reinterpret_cast<const uint8_t*>(wkt.c_str());

        boost::uint8_t userid[16];
        for (int i=0; i<16; i++) userid[i]=0;
        memcpy((char*)userid,(char*)"liblas",strlen("liblas"));

        boost::uint8_t description[32];
        for (int i=0; i<32; i++) description[i]=0;
        memcpy((char*)description,(char*)"OGR variant of OpenGIS WKT SRS",strlen("OGR variant of OpenGIS WKT SRS"));

        // Would you be surprised if this remarshalling of bytes
        // was annoying to me? FrankW
        while( *wkt_bytes != 0 )
            data.push_back( *(wkt_bytes++) );

        data.push_back( '\0' );

        if (data.size() > std::numeric_limits<boost::uint16_t>::max())
        {
            std::ostringstream oss;
            std::vector<uint8_t>::size_type overrun = data.size() - static_cast<std::vector<uint8_t>::size_type>(std::numeric_limits<boost::uint16_t>::max());
            oss << "The size of the wkt, " << data.size() << ", is " << overrun 
                << " bytes too large to fit inside the maximum size of a VLR which is " 
                << std::numeric_limits<boost::uint16_t>::max() << " bytes.";
            throw std::runtime_error(oss.str());
        }

        VariableLengthRecord wkt_record(0, userid, 2112, description, &data[0], static_cast<boost::uint16_t>(data.size()));

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

} } } // namespaces
