/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "NitfFileWriter.hpp"

#include "tre_plugins.hpp"


namespace pdal
{

NitfFileWriter::NitfFileWriter()
{
    register_tre_plugins();
}


void NitfFileWriter::setArgs(ProgramArgs& args)
{
    args.add("clevel", "Complexity level", m_cLevel, "03");
    args.add("stype", "Standard type", m_sType, "BF01");
    args.add("ostaid", "Origination station ID", m_oStationId, "PDAL");
    args.add("ftitle", "File title", m_fileTitle);
    args.add("fsclas", "", m_fileClass, "U");
    args.add("oname", "Originator's name", m_origName);
    args.add("ophone", "Originator's phone number", m_origPhone);
    args.add("fsclas", "File security classification", m_securityClass, "U");
    args.add("fsctlh", "File control and handling",
        m_securityControlAndHandling);
    args.add("fsclsy", "File security classification system",
        m_securityClassificationSystem);
    args.add("isclas", "File security classification", m_imgSecurityClass, "U");
    args.add("idatim", "Image date and time", m_imgDate);
    args.add("iid2", "Image identifier 2", m_imgIdentifier2);
    args.add("fscltx", "File classification text", m_sic);
    args.add("aimidb", "Additional (airborne) image ID", m_aimidb);
    args.add("acftb", "Aircraft information", m_acftb);
}


void NitfFileWriter::processOptions(const Options& options)
{
    m_cLevel = options.getValueOrDefault<std::string>("clevel","03");
    m_sType = options.getValueOrDefault<std::string>("stype","BF01");
    m_oStationId = options.getValueOrDefault<std::string>("ostaid", "PDAL");
    m_fileTitle = options.getValueOrDefault<std::string>("ftitle");
    m_fileClass = options.getValueOrDefault<std::string>("fsclas","U");
    m_origName = options.getValueOrDefault<std::string>("oname");
    m_origPhone = options.getValueOrDefault<std::string>("ophone");
    m_securityClass = options.getValueOrDefault<std::string>("fsclas","U");
    m_securityControlAndHandling =
        options.getValueOrDefault<std::string>("fsctlh");
    m_securityClassificationSystem =
        options.getValueOrDefault<std::string>("fsclsy");
    m_imgSecurityClass = options.getValueOrDefault<std::string>("isclas","U");
    m_imgDate = options.getValueOrDefault<std::string>("idatim");
    m_imgIdentifier2 = options.getValueOrDefault<std::string>("iid2");
    m_sic = options.getValueOrDefault<std::string>("fscltx");
    m_aimidb = options.getValueOrDefault<StringList>("aimidb");
    m_acftb = options.getValueOrDefault<StringList>("acftb");
}


//NOTE: Throws except::Throwable.
//
void NitfFileWriter::write()
{
    ::nitf::Record record(NITF_VER_21);
    ::nitf::FileHeader header = record.getHeader();
    header.getFileHeader().set("NITF");

    header.getComplianceLevel().set(m_cLevel);
    header.getSystemType().set(m_sType);
    header.getOriginStationID().set(m_oStationId);
    if (m_fileTitle.empty())
        m_fileTitle = m_filename;
    header.getFileTitle().set(m_fileTitle);
    header.getClassification().set(m_fileClass);
    header.getMessageCopyNum().set("00000");
    header.getMessageNumCopies().set("00000");
    header.getEncrypted().set("0");
    header.getBackgroundColor().setRawData(const_cast<char*>("000"), 3);
    header.getOriginatorName().set(m_origName);
    header.getOriginatorPhone().set(m_origPhone);
    header.getSecurityGroup().getClassificationSystem().set(
        m_securityClassificationSystem);
    header.getSecurityGroup().getControlAndHandling().set(
       m_securityControlAndHandling);
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

    des.getSubheader().setSubheaderFields(usrHdr);

    ::nitf::ImageSegment image = record.newImageSegment();
    ::nitf::ImageSubheader subheader = image.getSubheader();

    double corners[4][2];
    corners[0][0] = m_bounds.maxy;
    corners[0][1] = m_bounds.minx;
    corners[1][0] = m_bounds.maxy;
    corners[1][1] = m_bounds.maxx;
    corners[2][0] = m_bounds.miny;
    corners[2][1] = m_bounds.maxx;
    corners[3][0] = m_bounds.miny;
    corners[3][1] = m_bounds.minx;
    subheader.setCornersFromLatLons(NRT_CORNERS_DECIMAL, corners);
    subheader.getImageSecurityClass().set(m_imgSecurityClass);
    subheader.setSecurityGroup(security.clone());
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

    std::vector<::nitf::BandInfo> bands;
    bands.push_back(info);
    subheader.setPixelInformation(
        "INT",      /* Pixel value type */
        8,         /* Number of bits/pixel */
        8,         /* Actual number of bits/pixel */
        "R",       /* Pixel justification */
        "NODISPLY",     /* Image representation */
        "VIS",     /* Image category */
        1,         /* Number of bands */
        bands);

    subheader.setBlocking(
        8,   /*!< The number of rows */
        8,   /*!< The number of columns */
        8,   /*!< The number of rows/block */
        8,   /*!< The number of columns/block */
        "P");                /*!< Image mode */

    //Image Header fields to set
    subheader.getImageId().set("None");
    subheader.getImageTitle().set(m_imgIdentifier2);

    //AIMIDB
    ::nitf::TRE aimidbTre("AIMIDB");
    for (auto& s : m_aimidb)
    {
        StringList v = Utils::split2(s, ':');
        if (v.size() != 2)
        {
            std::ostringstream oss;
            oss << "Invalid name/value for AIMIDB '" << s <<
                "'.  Format: <name>:<value>.";
            throw oss.str();
        }
        Utils::trim(v[0]);
        Utils::trim(v[1]);
        aimidbTre.setField(v[0], v[1]);
    }
    if (m_aimidb.size())
        subheader.getExtendedSection().appendTRE(aimidbTre);

    //ACFTB
    ::nitf::TRE acftbTre("ACFTB");
    for (auto& s : m_acftb)
    {
        StringList v = Utils::split2(s, ':');
        if (v.size() != 2)
        {
            std::ostringstream oss;
            oss << "Invalid name/value for ACFTB '" << s <<
                "'.  Format: <name>:<value>.";
            throw oss.str();
        }
        Utils::trim(v[0]);
        Utils::trim(v[1]);
        acftbTre.setField(v[0], v[1]);
    }
    if (m_acftb.size())
        subheader.getExtendedSection().appendTRE(acftbTre);

    ::nitf::Writer writer;
    ::nitf::IOHandle output_io(m_filename, NITF_ACCESS_WRITEONLY,
        NITF_CREATE);
    writer.prepare(output_io, record);

    ::nitf::SegmentWriter sWriter = writer.newDEWriter(0);
    sWriter.attachSource(*m_source);

    // 64 char string
    std::string zeros(64, '0');

    ::nitf::MemorySource band(
        const_cast<char*>(zeros.c_str()),
        zeros.size() /* memory size */,
        0 /* starting offset */,
        1 /* bytes per pixel */,
        0 /*skip*/);
    ::nitf::ImageSource iSource;
    iSource.addBand(band);

    ::nitf::ImageWriter iWriter = writer.newImageWriter(0);
    iWriter.attachSource(iSource);

    writer.write();
    output_io.close();
}


void NitfFileWriter::setBounds(const BOX3D& bounds)
{
    m_bounds = bounds;
}


void NitfFileWriter::wrapData(const char *buf, size_t size)
{
    m_source.reset(new ::nitf::SegmentMemorySource(buf, size, 0, 0, false));
}


void NitfFileWriter::wrapData(const std::string& filename)
{
    m_inputHandle.reset(new nitf::IOHandle(filename));
    m_source.reset(new nitf::SegmentFileSource(*m_inputHandle, 0, 0));
}

} // namespace pdal

