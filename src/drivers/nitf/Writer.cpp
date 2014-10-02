/****************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <memory>
#include <vector>

#include <pdal/drivers/nitf/Writer.hpp>
#include <pdal/drivers/las/Writer.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/GlobalEnvironment.hpp>

#ifdef PDAL_HAVE_NITRO

#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wredundant-decls"
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#  pragma GCC diagnostic ignored "-Wextra"
#  pragma GCC diagnostic ignored "-Wcast-qual"
   // The following pragma doesn't actually work:
   //   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61653
#  pragma GCC diagnostic ignored "-Wliteral-suffix"
#endif

#define IMPORT_NITRO_API
#include <nitro/c++/import/nitf.hpp>

#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic pop
#endif

#endif

// NOTES
//
// is it legal to write a LAZ file?
// syntactically, how do we name all the LAS writer options that we will pass to the las writer?
//

namespace pdal
{
namespace drivers
{
namespace nitf
{


void Writer::processOptions(const Options& options)
{
    las::Writer::processOptions(options);
    m_cLevel = options.getValueOrDefault<std::string>("CLEVEL","03");
    m_sType = options.getValueOrDefault<std::string>("STYPE","BF01");
    m_oStationId = options.getValueOrDefault<std::string>("OSTAID","PDAL");
    m_fileTitle = options.getValueOrDefault<std::string>("FTITLE",m_filename);
    m_fileClass = options.getValueOrDefault<std::string>("FSCLAS","U");
    m_origName = options.getValueOrDefault<std::string>("ONAME","");
    m_origPhone = options.getValueOrDefault<std::string>("OPHONE","");
    m_securityClass = options.getValueOrDefault<std::string>("FSCLAS","U");
    m_imgSecurityClass = options.getValueOrDefault<std::string>("FSCLAS","U");
    m_imgDate = getOptions().getValueOrDefault<std::string>("IDATIM", "");
    m_sic = getOptions().getValueOrDefault<std::string>("FSCLTX", "");
    m_igeolob = getOptions().getValueOrDefault<std::string>("GEOLOB", "");
}


void Writer::done(PointContextRef ctx)
{
    las::Writer::done(ctx);

#ifdef PDAL_HAVE_NITRO
    try
    {
        ::nitf::Record record(NITF_VER_21);
        ::nitf::FileHeader header = record.getHeader();
        header.getFileHeader().set("NITF");
        header.getComplianceLevel().set(m_cLevel);
        header.getSystemType().set(m_sType);
        header.getOriginStationID().set(m_oStationId);
        header.getFileTitle().set(m_fileTitle);
        header.getClassification().set(m_fileClass);
        header.getMessageCopyNum().set("00000");
        header.getMessageNumCopies().set("00000");
        header.getEncrypted().set("0");
        header.getBackgroundColor().setRawData(const_cast<char*>("000"), 3);
        header.getOriginatorName().set(m_origName);
        header.getOriginatorPhone().set(m_origPhone);
        header.getSecurityGroup().getClassificationText().set(m_sic);

        ::nitf::DESegment des = record.newDataExtensionSegment();

        des.getSubheader().getFilePartType().set("DE");
        des.getSubheader().getTypeID().set("LIDARA DES");
        des.getSubheader().getVersion().set("01");
        des.getSubheader().getSecurityClass().set(m_securityClass);
        ::nitf::FileSecurity security = record.getHeader().getSecurityGroup();
        des.getSubheader().setSecurityGroup(security.clone());

        ::nitf::TRE usrHdr("LIDARA DES", "raw_data");
        usrHdr.setField("raw_data", "not");
        ::nitf::Field fld = usrHdr.getField("raw_data");
        fld.setType(::nitf::Field::BINARY);

        flush();
        m_oss.flush();
        std::streambuf *buf = m_oss.rdbuf();
        long size = buf->pubseekoff(0, m_oss.end);
        buf->pubseekoff(0, m_oss.beg);

        std::vector<char> bytes(size);
        buf->sgetn(bytes.data(), size);

        des.getSubheader().setSubheaderFields(usrHdr);

        ::nitf::ImageSegment image = record.newImageSegment();
        ::nitf::ImageSubheader subheader = image.getSubheader();

        subheader.getImageSecurityClass().set(m_imgSecurityClass);
        if (m_imgDate.size())
            subheader.getImageDateAndTime().set(m_imgDate);

        ::nitf::BandInfo info;
        ::nitf::LookupTable lt(0,0);
        info.init("G",    /* The band representation, Nth band */
                  " ",      /* The band subcategory */
                  "N",      /* The band filter condition */
                  "   ",    /* The band standard image filter code */
                  0,        /* The number of look-up tables */
                  0,        /* The number of entries/LUT */
                  lt);     /* The look-up tables */

        std::vector< ::nitf::BandInfo> bands;
        bands.push_back(info);
        subheader.setPixelInformation(
            "INT",      /* Pixel value type */
            8,         /* Number of bits/pixel */
            8,         /* Actual number of bits/pixel */
            "G",       /* Pixel justification */
            "G",     /* Image representation */
            "VIS",     /* Image category */
            1,         /* Number of bands */
            bands);

        subheader.setBlocking(
            8,   /*!< The number of rows */
            8,  /*!< The number of columns */
            8, /*!< The number of rows/block */
            8,  /*!< The number of columns/block */
            "P");                /*!< Image mode */
        subheader.getImageId().set("None");
        // 64 char string
        std::string zeros(64, '0');

        std::unique_ptr< ::nitf::BandSource> band(new ::nitf::MemorySource(
            const_cast<char*>(zeros.c_str()),
            zeros.size() /* memory size */,
            0 /* starting offset */,
            1 /* bytes per pixel */,
            0 /*skip*/));
        ::nitf::ImageSource iSource;
        iSource.addBand(*band);

        ::nitf::Writer writer;
        ::nitf::IOHandle output_io(m_filename.c_str(), NITF_ACCESS_WRITEONLY,
            NITF_CREATE);
        writer.prepare(output_io, record);

        ::nitf::SegmentWriter sWriter = writer.newDEWriter(0);

        ::nitf::SegmentMemorySource sSource(bytes.data(), size, 0, 0, false);
        sWriter.attachSource(sSource);

        ::nitf::ImageWriter iWriter = writer.newImageWriter(0);
        iWriter.attachSource(iSource);

        writer.write();
        output_io.close();
    }
    catch (except::Throwable & t)
    {
        std::ostringstream oss;
        // std::cout << t.getTrace();
        throw pdal_error(t.getMessage());
    }
#endif
}


}
}
} // namespaces

