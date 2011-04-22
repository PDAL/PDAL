/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS header class 
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
 * Copyright (c) 2008, Phil Vachon
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
 *     * Neither the name of the Martin Isenburg or Iowa Department 
 *       of Natural Resources nor the names of its contributors may be 
 *       used to endorse or promote products derived from this software 
 *       without specific prior written permission.
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

#include "LasHeaderWriter.hpp"

#include <libpc/drivers/las/Header.hpp>
#include <boost/uuid/uuid_io.hpp>


namespace libpc { namespace drivers { namespace las {


LasHeaderWriter::LasHeaderWriter(LasHeader& header, std::ostream& ostream)
    : m_header(header)
    , m_ostream(ostream)
{
    return;
}



void LasHeaderWriter::write()
{
    using namespace std;

    uint8_t n1 = 0;
    uint16_t n2 = 0;
    uint32_t n4 = 0;
    
    // Figure out how many points we already have.  
    // Figure out if we're in append mode.  If we are, we can't rewrite 
    // any of the VLRs including the Schema and SpatialReference ones.
    bool bAppendMode = false;

    // This test should only be true if we were opened in both 
    // std::ios::in *and* std::ios::out

    // Seek to the beginning
    m_ostream.seekp(0, ios::beg);
    ios::pos_type begin = m_ostream.tellp();

    // Seek to the end
    m_ostream.seekp(0, ios::end);
    ios::pos_type end = m_ostream.tellp();
    if ((begin != end) && (end != static_cast<ios::pos_type>(0))) {
        bAppendMode = true;
    }

    ////// If we are in append mode, we are not touching *any* VLRs. 
    ////if (bAppendMode) 
    ////{
    ////    // We're opened in append mode
    ////    
    ////    if (!Compressed())
    ////    {
    ////        ios::off_type points = end - static_cast<ios::off_type>(GetDataOffset());
    ////        ios::off_type count = points / static_cast<ios::off_type>(GetDataRecordLength());
    ////    
    ////        if (points < 0) {
    ////            std::ostringstream oss;
    ////            oss << "The header's data offset, " << GetDataOffset() 
    ////                <<", is much larger than the size of the file, " << end
    ////                <<", and something is amiss.  Did you use the right header"
    ////                <<" offset value?";
    ////            throw std::runtime_error(oss.str());
    ////        }
    ////        
    ////        m_pointCount = static_cast<uint32_t>(count);

    ////    } else {
    ////        m_pointCount = GetPointRecordsCount();
    ////    }

    ////    // Position to the beginning of the file to start writing the header
    ////    ostream.seekp(0, ios::beg);

    ////} 
    ////else 
    {
        
////        // Rewrite the georeference VLR entries if they exist
////        m_header.DeleteVLRs("liblas", 2112);
////        m_header.SetGeoreference();
////
////        // If we have a custom schema, add the VLR and write it into the 
////        // file.  
////        if (m_header.GetSchema().IsCustom()) {
////            
////            // Wipe any schema-related VLRs we might have, as this is now out of date.
////            m_header.DeleteVLRs("liblas", 7);
////        
////            VariableRecord v = m_header.GetSchema().GetVLR();
////            std::cout <<  m_header.GetSchema()<< std::endl;
////            m_header.AddVLR(v);
////        }
////    
////        // add the laszip VLR, if needed
////        if (m_header.Compressed())
////        {
////#ifdef HAVE_LASZIP
////            m_header.DeleteVLRs("laszip encoded", 22204);
////            ZipPoint zpd(m_header.GetDataFormatId());
////            VariableRecord v;
////            zpd.ConstructVLR(v);
////            m_header.AddVLR(v);
////#else
////            throw configuration_error("LASzip compression support not enabled in this libLAS configuration.");
////#endif
////        }
////        else
////        {
////            m_header.DeleteVLRs("laszip encoded", 22204);
////        }

        int32_t difference = (int32_t)m_header.GetDataOffset() - (int32_t)GetRequiredHeaderSize();

        if (difference <= 0) 
        {
            int32_t d = abs(difference);
            if (m_header.GetVersionMinor()  ==  0) 
            {
                // Add the two extra bytes for the 1.0 pad
                d = d + 2;
            }
            m_header.SetDataOffset(m_header.GetDataOffset() + d );
        }

    }

    
    // 1. File Signature
    std::string const filesig(m_header.GetFileSignature());
    assert(filesig.size() == 4);
    Utils::write_n(m_ostream, filesig.c_str(), 4);
    
    // 2. File SourceId / Reserved
    if (m_header.GetVersionMinor()  ==  0) {
        n4 = m_header.GetReserved();
        Utils::write_n(m_ostream, n4, sizeof(n4));         
    } else if (m_header.GetVersionMinor()  >  0) {
        n2 = m_header.GetFileSourceId();
        Utils::write_n(m_ostream, n2, sizeof(n2));                
        n2 = m_header.GetReserved();
        Utils::write_n(m_ostream, n2, sizeof(n2));        
    } 

    // 3-6. GUID data
    {
        boost::uint8_t d[16];
        boost::uuids::uuid u = m_header.GetProjectId();
        // BUG: this following is to maintain compatability with the liblas driver
        // I have no idea why I need to do this.
        d[0] = u.data[3];
        d[1] = u.data[2];
        d[2] = u.data[1];
        d[3] = u.data[0];
        d[4] = u.data[5];
        d[5] = u.data[4];
        d[6] = u.data[7];
        d[7] = u.data[6];
        for (int i=8; i<16; i++)
            d[i] = u.data[i];
        Utils::write_n(m_ostream, d, 16);
    }

    // 7. Version major
    n1 = m_header.GetVersionMajor();
    assert(1 == n1);
    Utils::write_n(m_ostream, n1, sizeof(n1));
    
    // 8. Version minor
    n1 = m_header.GetVersionMinor();
    Utils::write_n(m_ostream, n1, sizeof(n1));

    // 9. System ID
    std::string sysid(m_header.GetSystemId(true));
    assert(sysid.size() == 32);
    Utils::write_n(m_ostream, sysid.c_str(), 32);
    
    // 10. Generating Software ID
    std::string softid(m_header.GetSoftwareId(true));
    assert(softid.size() == 32);
    Utils::write_n(m_ostream, softid.c_str(), 32);

    // 11. Flight Date Julian
    n2 = m_header.GetCreationDOY();
    Utils::write_n(m_ostream, n2, sizeof(n2));

    // 12. Year
    n2 = m_header.GetCreationYear();
    Utils::write_n(m_ostream, n2, sizeof(n2));

    // 13. Header Size
    n2 = m_header.GetHeaderSize();
    assert(227 <= n2);
    Utils::write_n(m_ostream, n2, sizeof(n2));

    // 14. Offset to data
    n4 = m_header.GetDataOffset();        
    Utils::write_n(m_ostream, n4, sizeof(n4));

    // 15. Number of variable length records
    n4 = m_header.GetRecordsCount();
    Utils::write_n(m_ostream, n4, sizeof(n4));

    // 16. Point Data Format ID
    n1 = static_cast<uint8_t>(m_header.getPointFormat());
    uint8_t n1tmp = n1;
    if (m_header.Compressed()) // high bit set indicates laszip compression
        n1tmp |= 0x80;
    Utils::write_n(m_ostream, n1tmp, sizeof(n1tmp));

    // 17. Point Data Record Length
    n2 = m_header.GetDataRecordLength();
    Utils::write_n(m_ostream, n2, sizeof(n2));

    // 18. Number of point records
    // This value is updated if necessary, see UpdateHeader function.
    n4 = m_header.GetPointRecordsCount();
    Utils::write_n(m_ostream, n4, sizeof(n4));

    // 19. Number of points by return
    std::vector<uint32_t>::size_type const srbyr = 5;
    std::vector<uint32_t> const& vpbr = m_header.GetPointRecordsByReturnCount();
    // TODO: fix this for 1.3, which has srbyr = 7;  See detail/reader/header.cpp for more details
    // assert(vpbr.size() <= srbyr);
    uint32_t pbr[srbyr] = { 0 };
    std::copy(vpbr.begin(), vpbr.begin() + srbyr, pbr); // FIXME: currently, copies only 5 records, to be improved
    Utils::write_n(m_ostream, pbr, sizeof(pbr));

    // 20-22. Scale factors
    Utils::write_n(m_ostream, m_header.GetScaleX(), sizeof(double));
    Utils::write_n(m_ostream, m_header.GetScaleY(), sizeof(double));
    Utils::write_n(m_ostream, m_header.GetScaleZ(), sizeof(double));

    // 23-25. Offsets
    Utils::write_n(m_ostream, m_header.GetOffsetX(), sizeof(double));
    Utils::write_n(m_ostream, m_header.GetOffsetY(), sizeof(double));
    Utils::write_n(m_ostream, m_header.GetOffsetZ(), sizeof(double));

    // 26-27. Max/Min X
    Utils::write_n(m_ostream, m_header.GetMaxX(), sizeof(double));
    Utils::write_n(m_ostream, m_header.GetMinX(), sizeof(double));

    // 28-29. Max/Min Y
    Utils::write_n(m_ostream, m_header.GetMaxY(), sizeof(double));
    Utils::write_n(m_ostream, m_header.GetMinY(), sizeof(double));

    // 30-31. Max/Min Z
    Utils::write_n(m_ostream, m_header.GetMaxZ(), sizeof(double));
    Utils::write_n(m_ostream, m_header.GetMinZ(), sizeof(double));

    // If WriteVLR returns a value, it is because the header's 
    // offset is not large enough to contain the VLRs.  The value 
    // it returns is the number of bytes we must increase the header
    // by in order for it to contain the VLRs.  We do not touch VLRs if we 
    // are in append mode.

    if (!bAppendMode) 
    {
        WriteVLRs();

        // Write the 1.0 pad signature if we need to.
        WriteLAS10PadSignature(); 

    }           
    //////// If we already have points, we're going to put it at the end of the file.  
    //////// If we don't have any points,  we're going to leave it where it is.
    //////if (m_pointCount != 0)
    //////{
    //////    ostream.seekp(0, std::ios::end);
    //////}
    //////else
    //////{
    //////    ostream.seekp(m_header.GetDataOffset(), std::ios::beg);
    //////}

    return;
}

void LasHeaderWriter::WriteVLRs()
{
    // Seek to the end of the public header block (beginning of the VLRs)
    // to start writing
    m_ostream.seekp(m_header.GetHeaderSize(), std::ios::beg);

    int32_t diff = m_header.GetDataOffset() - GetRequiredHeaderSize();
    
    if (diff < 0) {
        std::ostringstream oss;
        oss << "Header is not large enough to contain VLRs.  Data offset is ";
        oss << m_header.GetDataOffset() << " while the required total size ";
        oss << "for the VLRs is " << GetRequiredHeaderSize();
        throw std::runtime_error(oss.str());
    }

    for (uint32_t i = 0; i < m_header.GetRecordsCount(); ++i)
    {
        VariableLengthRecord const &vlr = m_header.getVLRs()[i];

        Utils::write_n(m_ostream, vlr.getReserved(), sizeof(uint16_t));
        Utils::write_n(m_ostream, vlr.getUserId(), 16);
        Utils::write_n(m_ostream, vlr.getRecordId(), sizeof(uint16_t));
        Utils::write_n(m_ostream, vlr.getLength(), sizeof(uint16_t));
        Utils::write_n(m_ostream, vlr.getDescription(), 32);
        boost::shared_array<uint8_t> data(vlr.getBytes());
        Utils::write_n(m_ostream, &(data[0]), vlr.getLength());
    }

    // if we had more room than we need for the VLRs, we need to pad that with 
    // 0's.  We must also not forget to add the 1.0 pad bytes to the end of this
    // but the impl should be the one doing that, not us.
    if (diff > 0) {
        Utils::write_n(m_ostream, "\0", diff);
    }

    return;
}

std::size_t LasHeaderWriter::GetRequiredHeaderSize() const
{
    // if the VLRs won't fit because the data offset is too 
    // small, we need to throw an error.
    std::size_t vlr_total_size = 0;
        
    ////// Calculate a new data offset size
    ////for (uint32_t i = 0; i < GetRecordsCount(); ++i)
    ////{
    ////    VariableRecord const & vlr = m_header.GetVLR(i);
    ////    vlr_total_size += vlr.GetTotalSize();
    ////}
    
    // int32_t difference = m_header.GetDataOffset() - (vlr_total_size + m_header.GetHeaderSize());
    std::size_t size = vlr_total_size + m_header.GetHeaderSize();
    return size;
    
}

void LasHeaderWriter::WriteLAS10PadSignature()
{
    // Only write pad signature bytes for LAS 1.0 files.  Any other files 
    // will not get the pad bytes and we are *not* allowing anyone to 
    // override this either - hobu
    
    if (m_header.GetVersionMinor() > 0) {
        return;
    }

    int32_t diff = (int32_t)m_header.GetDataOffset() - (int32_t)GetRequiredHeaderSize();

    if (diff < 2) {
        std::ostringstream oss;
        oss << "Header is not large enough to write extra 1.0 pad bytes.  Data offset is ";
        oss << m_header.GetDataOffset() << " while the required total size ";
        oss << "for the VLRs is " << GetRequiredHeaderSize();
        throw std::runtime_error(oss.str());
    }    
    
    
    // step back two bytes to write the pad bytes.  We should have already
    // determined by this point if a) they will fit b) they won't overwrite 
    // exiting real data 
    m_ostream.seekp(m_header.GetDataOffset() - 2, std::ios::beg);
    
    // Write the pad bytes.
    uint8_t const sgn1 = 0xCC;
    uint8_t const sgn2 = 0xDD;
    Utils::write_n(m_ostream, sgn1, sizeof(uint8_t));
    Utils::write_n(m_ostream, sgn2, sizeof(uint8_t));
}


} } } // namespaces
