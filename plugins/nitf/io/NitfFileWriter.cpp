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

#include <pdal/util/FileUtils.hpp>

#include "tre_plugins.hpp"

namespace pdal
{

void NitfFileWriter::initialize()
{
    try
    {
        register_tre_plugins();
    }
    catch (const pdal_error& err)
    {
        throw error(err.what());
    }
}


void NitfFileWriter::addArgs(ProgramArgs& args)
{
    args.add("clevel", "Complexity level", m_cLevel, "03");
    args.add("stype", "Standard type", m_sType, "BF01");
    args.add("ostaid", "Origination station ID", m_oStationId, "PDAL");
    args.add("ftitle", "File title", m_fileTitle);
    args.add("fsclas", "File/data segment classification", m_fileClass, "U");
    args.add("oname", "Originator's name", m_origName);
    args.add("ophone", "Originator's phone number", m_origPhone);
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


//NOTE: Throws except::Throwable.
//
void NitfFileWriter::write()
{
    if (m_fileTitle.empty())
        m_fileTitle = FileUtils::getFilename(m_filename);

    ::nitf::Record record(NITF_VER_21);
    ::nitf::FileHeader header = record.getHeader();

    // This check would be better in an initialize() function, but we can't
    // because this is a flex writer and we don't know the filename
    // until the execute() step.
    if (m_fileTitle.size() > header.getFileTitle().getLength())
        throw error("Can't write file.  FTITLE field (usually filename) "
            "can't be longer than " +
            Utils::toString(header.getFileTitle().getLength()) +
            ".  Use 'ftitle' option to set appropriately sized FTITLE.");

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
    header.getSecurityGroup().getClassificationSystem().set(
        m_securityClassificationSystem);
    header.getSecurityGroup().getControlAndHandling().set(
       m_securityControlAndHandling);
    header.getSecurityGroup().getClassificationText().set(m_sic);

    ::nitf::DESegment des = record.newDataExtensionSegment();

    des.getSubheader().getFilePartType().set("DE");
    des.getSubheader().getTypeID().set("LIDARA DES");
    des.getSubheader().getVersion().set("01");
    // We currently use the file classification for the subheader, since the
    // only thing we're writing is the data extension.
    des.getSubheader().getSecurityClass().set(m_fileClass);
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
    info.init(" ",    /* The band representation, Nth band */
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
        "B");                /*!< Image mode */

    //Image Header fields to set
    subheader.getImageId().set("None");
    subheader.getImageTitle().set(m_imgIdentifier2);

    //AIMIDB
    ::nitf::TRE aimidbTre("AIMIDB");

    //LIDAR defaults
    if (m_imgDate.size())
        aimidbTre.setField("ACQUISITION_DATE", m_imgDate);
    aimidbTre.setField("MISSION_NO", "UNKN");
    aimidbTre.setField("MISSION_IDENTIFICATION", "NOT AVAIL.");
    aimidbTre.setField("FLIGHT_NO", "00");
    aimidbTre.setField("CURRENT_SEGMENT", "AA");
    aimidbTre.setField("START_TILE_COLUMN", "001");
    aimidbTre.setField("START_TILE_ROW", "00001");
    aimidbTre.setField("END_SEGMENT", "00");
    aimidbTre.setField("END_TILE_COLUMN", "001");
    aimidbTre.setField("END_TILE_ROW", "00001");

    for (auto& s : m_aimidb)
    {
        StringList v = Utils::split2(s, ':');
        if (v.size() != 2)
            throw error("Invalid name/value for AIMIDB '" + s +
                "'.  Format: <name>:<value>.");
        Utils::trim(v[0]);
        Utils::trim(v[1]);
        aimidbTre.setField(v[0], v[1]);
    }
    subheader.getExtendedSection().appendTRE(aimidbTre);

    //if IDATIM is empty set it equal to AIMIDB.ACQUISITION_DATE
    if (!m_imgDate.size())
    {
        m_imgDate = aimidbTre.getField("ACQUISITION_DATE").toString();
        if (m_imgDate.size())
            subheader.getImageDateAndTime().set(m_imgDate);
    }

    //ACFTB
    ::nitf::TRE acftbTre("ACFTB");

    //LIDAR defaults
    acftbTre.setField("AC_MSN_ID", "NOT AVAILABLE");
    acftbTre.setField("SCENE_SOURCE", " ");
    if (m_imgDate.size()>7)
        acftbTre.setField("PDATE", m_imgDate.substr(0,8));
    acftbTre.setField("MPLAN", "999");
    acftbTre.setField("LOC_ACCY", "000.00");
    acftbTre.setField("ROW_SPACING", "0000000");
    acftbTre.setField("ROW_SPACING_UNITS", "u");
    acftbTre.setField("COL_SPACING", "0000000");
    acftbTre.setField("COL_SPACING_UNITS", "u");
    acftbTre.setField("FOCAL_LENGTH", "999.99");

    for (auto& s : m_acftb)
    {
        StringList v = Utils::split2(s, ':');
        if (v.size() != 2)
            throw error("Invalid name/value for ACFTB '" + s +
                "'.  Format: <name>:<value>.");
        Utils::trim(v[0]);
        Utils::trim(v[1]);
        acftbTre.setField(v[0], v[1]);
    }
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

    //NITF decimal degree values for corner coordinates only has a
    // precision of 3 after the decimal. This may cause an invalid
    // polygon due to rounding errors with a small tile. Therefore
    // instead of rounding min values will use the floor value and
    // max values will use the ceiling values.
    m_bounds.minx = (floor(m_bounds.minx * 1000)) / 1000.0;
    m_bounds.miny = (floor(m_bounds.miny * 1000)) / 1000.0;
    m_bounds.maxx = (ceil(m_bounds.maxx * 1000)) / 1000.0;
    m_bounds.maxy = (ceil(m_bounds.maxy * 1000)) / 1000.0;
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

