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

#include "ZipPoint.hpp"

#include <pdal/drivers/las/Header.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/scoped_ptr.hpp>


namespace pdal
{
namespace drivers
{
namespace las
{


LasHeaderWriter::LasHeaderWriter(LasHeader& header, std::ostream& ostream, boost::uint64_t firstPos)
    : m_header(header)
    , m_ostream(ostream)
    , m_firstPos(firstPos)
    , m_wktModeFlag(SpatialReference::eCompoundOK)
{
    return;
}


void LasHeaderWriter::write()
{
    using namespace std;

    boost::uint8_t n1 = 0;
    boost::uint16_t n2 = 0;
    boost::uint32_t n4 = 0;

    // Seek to the end
    m_ostream.seekp(0, ios::end);
    // ios::pos_type end = m_ostream.tellp();

    {
        // Rewrite the georeference VLR entries if they exist
        m_header.getVLRs().remove("liblas", 2112);

        // Wipe the GeoTIFF-related VLR records off of the Header
        m_header.getVLRs().remove("LASF_Projection", 34735);
        m_header.getVLRs().remove("LASF_Projection", 34736);
        m_header.getVLRs().remove("LASF_Projection", 34737);

        m_header.getVLRs().addVLRsFromSRS(m_header.getSpatialReference(), m_wktModeFlag);
    }

    {
        m_header.getVLRs().remove("laszip encoded", 22204);

        // add the laszip VLR, if needed
        if (m_header.Compressed())
        {
#ifdef PDAL_HAVE_LASZIP
            ZipPoint zpd(m_header.getPointFormat(), m_header, false);
            VariableLengthRecord v = zpd.ConstructVLR();
            m_header.getVLRs().add(v);
#else
            throw configuration_error("LASzip compression support not enabled in this PDAL configuration.");
#endif
        }
    }

    {
        boost::uint32_t padding_before_calculations = m_header.GetHeaderPadding();
        boost::int32_t existing_padding = m_header.GetDataOffset() -
                                          (m_header.getVLRBlockSize() +
                                           m_header.GetHeaderSize());

        if (existing_padding < 0)
        {
            boost::int32_t d = abs(existing_padding);

            // If our required VLR space is larger than we have
            // room for, we have no padding.  AddVLRs will take care
            // of incrementing up the space it needs.
            if (static_cast<boost::int32_t>(m_header.getVLRBlockSize()) > d)
            {
                m_header.SetHeaderPadding(0);
            }
            else
            {
                m_header.SetHeaderPadding(d - m_header.getVLRBlockSize());
            }
        }
        else
        {
            // cast is safe, we've already checked for < 0
            if (static_cast<boost::uint32_t>(existing_padding) >= m_header.GetHeaderPadding())
            {
                m_header.SetHeaderPadding(existing_padding);
            }
            else
            {
                m_header.SetHeaderPadding(m_header.GetHeaderPadding() + existing_padding);
            }

        }

        m_header.SetDataOffset(m_header.GetHeaderSize() +
                               m_header.getVLRBlockSize() +
                               m_header.GetHeaderPadding() + padding_before_calculations);

    }



    // 1. File Signature
    std::string const filesig(m_header.GetFileSignature());
    assert(filesig.size() == 4);
    Utils::write_n(m_ostream, filesig.c_str(), 4);

    // 2. File SourceId / Reserved
    if (m_header.GetVersionMinor()  ==  0)
    {
        n4 = m_header.GetReserved();
        Utils::write_n(m_ostream, n4, sizeof(n4));
    }
    else if (m_header.GetVersionMinor()  >  0)
    {
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
    n4 = m_header.getVLRs().count();
    Utils::write_n(m_ostream, n4, sizeof(n4));

    // 16. Point Data Format ID
    n1 = static_cast<boost::uint8_t>(m_header.getPointFormat());
    boost::uint8_t n1tmp = n1;
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
    uint32_t pbr[srbyr] = { 0 };
    std::copy(vpbr.begin(), vpbr.begin() + srbyr, pbr);
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

    WriteVLRs();

    // if we have padding, we should write it
    if (m_header.GetHeaderPadding() > 0)
    {
        m_ostream.seekp(m_header.GetHeaderSize() + m_header.getVLRBlockSize(), std::ios::end);
        Utils::write_n(m_ostream, "\0", m_header.GetHeaderPadding());
    }

    // Write the 1.0 pad signature if we need to.
    WriteLAS10PadSignature();

    return;
}


void LasHeaderWriter::WriteVLRs()
{
    // Seek to the end of the public header block (beginning of the VLRs)
    // to start writing
    m_ostream.seekp(m_firstPos + m_header.GetHeaderSize(), std::ios::beg);

    boost::int32_t diff = m_header.GetDataOffset() - GetRequiredHeaderSize();

    if (diff < 0)
    {
        std::ostringstream oss;
        oss << "Header is not large enough to contain VLRs.  Data offset is ";
        oss << m_header.GetDataOffset() << " while the required total size ";
        oss << "for the VLRs is " << GetRequiredHeaderSize();
        throw std::runtime_error(oss.str());
    }

    for (boost::uint32_t i = 0; i < m_header.getVLRs().count(); ++i)
    {
        VariableLengthRecord const &vlr = m_header.getVLRs().get(i);

        boost::uint8_t* userId_data = VariableLengthRecord::string2bytes(16, vlr.getUserId());
        boost::uint8_t* description_data = VariableLengthRecord::string2bytes(32, vlr.getDescription());
        Utils::write_n(m_ostream, vlr.getReserved(), sizeof(boost::uint16_t));
        m_ostream.write((const char*)userId_data, 16); // BUG: move to Utils function
        Utils::write_n(m_ostream, vlr.getRecordId(), sizeof(boost::uint16_t));
        Utils::write_n(m_ostream, vlr.getLength(), sizeof(boost::uint16_t));
        m_ostream.write((const char*)description_data, 32); // BUG: move to Utils::write_array function
        m_ostream.write((const char*)vlr.getBytes(), vlr.getLength());

        delete[] userId_data;
        delete[] description_data;
    }

    // if we had more room than we need for the VLRs, we need to pad that with
    // 0's.  We must also not forget to add the 1.0 pad bytes to the end of this
    // but the impl should be the one doing that, not us.
    if (diff > 0)
    {
        Utils::write_n(m_ostream, "\0", diff);
    }

    return;
}


std::size_t LasHeaderWriter::GetRequiredHeaderSize() const
{
    return m_header.getVLRBlockSize() + m_header.GetHeaderSize();
}


void LasHeaderWriter::WriteLAS10PadSignature()
{
    // Only write pad signature bytes for LAS 1.0 files.  Any other files
    // will not get the pad bytes and we are *not* allowing anyone to
    // override this either - hobu

    if (m_header.GetVersionMinor() > 0)
    {
        return;
    }

    boost::int32_t diff = (boost::int32_t)m_header.GetDataOffset() - (boost::int32_t)GetRequiredHeaderSize();

    if (diff < 2)
    {
        m_header.SetDataOffset(m_header.GetDataOffset() + 2);
        // Seek to the location of the data offset in the header and write a new one.
        m_ostream.seekp(m_firstPos + 96, std::ios::beg);
        Utils::write_n(m_ostream, m_header.GetDataOffset(), sizeof(m_header.GetDataOffset()));
    }

    // step back two bytes to write the pad bytes.  We should have already
    // determined by this point if a) they will fit b) they won't overwrite
    // exiting real data
    m_ostream.seekp(m_firstPos + m_header.GetDataOffset() - 2, std::ios::beg);

    // Write the pad bytes.
    boost::uint8_t const sgn1 = 0xCC;
    boost::uint8_t const sgn2 = 0xDD;
    Utils::write_n(m_ostream, sgn1, sizeof(boost::uint8_t));
    Utils::write_n(m_ostream, sgn2, sizeof(boost::uint8_t));
}


}
}
} // namespaces
