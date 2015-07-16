/******************************************************************************
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

#include "NitfFile.hpp"

#include <pdal/Metadata.hpp>

#include <boost/algorithm/string.hpp>

#include "MetadataReader.hpp"
#include "tre_plugins.hpp"


// Set to true if you want the metadata to contain
// NITF fields that are empty; if false, those fields
// will be skipped.
static const bool SHOW_EMPTY_FIELDS = true;

// Set to true to if you want an error thrown if the
// NITF file does not have a LAS data segment and a
// corresponding image segment. (Set to false for
// testing robustness of metadata parsing.)
static const bool REQUIRE_LIDAR_SEGMENTS = true;


namespace pdal
{


NitfFile::NitfFile(const std::string& filename) :
    m_reader(NULL),
    m_io(NULL),
    m_filename(filename),
    m_validLidarSegments(false),
    m_lidarImageSegment(0),
    m_lidarDataSegment(0)
{
    register_tre_plugins();
}


NitfFile::~NitfFile()
{
    close();
}


void NitfFile::open()
{
    if (::nitf::Reader::getNITFVersion(m_filename.c_str()) == NITF_VER_UNKNOWN)
        throw pdal_error("Unable to determine NITF file version");

    // read the major NITF data structures, courtesy Nitro
    m_reader = new ::nitf::Reader();
    try
    {
        m_io = new ::nitf::IOHandle(m_filename.c_str());
    }
    catch (::nitf::NITFException& e)
    {
        throw pdal_error("unable to open NITF file (" + e.getMessage() + ")");
    }
    try
    {
        m_record = m_reader->read(*m_io);
    }
    catch (::nitf::NITFException& e)
    {
        throw pdal_error("unable to read NITF file (" + e.getMessage() + ")");
    }


    // find the image segment corresponding the the lidar data, if any
    const bool imageOK = locateLidarImageSegment();
    if (REQUIRE_LIDAR_SEGMENTS && !imageOK)
    {
        throw pdal_error("Unable to find lidar-compatible image "
            "segment in NITF file");
    }

    // find the LAS data hidden in a DE field, if any
    const bool dataOK = locateLidarDataSegment();
    if (REQUIRE_LIDAR_SEGMENTS && !dataOK)
    {
        throw pdal_error("Unable to find LIDARA data extension segment "
            "in NITF file");
    }

    m_validLidarSegments = dataOK && imageOK;
}


void NitfFile::close()
{
    if (m_io)
    {
        m_io->close();
        delete m_io;
        m_io = NULL;
    }

    delete m_reader;
    m_reader = NULL;
}


void NitfFile::getLasOffset(uint64_t& offset,
                            uint64_t& length)
{
    offset = 0;
    length = 0;

    if (!m_validLidarSegments)
        return;

    ::nitf::ListIterator iter = m_record.getDataExtensions().begin();
    const ::nitf::Uint32 numSegs = m_record.getNumDataExtensions();
    for (::nitf::Uint32 segNum=0; segNum<numSegs; segNum++)
    {
        if (segNum == m_lidarDataSegment)
        {
            ::nitf::DESegment seg = *iter;
            const ::nitf::Uint64 seg_offset = seg.getOffset();
            const ::nitf::Uint64 seg_end = seg.getEnd();

            offset = seg_offset;
            length = seg_end - seg_offset;

            return;
        }

        iter++;
    }

    throw pdal_error("error reading nitf (1)");
}


void NitfFile::extractMetadata(MetadataNode& node)
{
    MetadataReader mr(m_record, node, SHOW_EMPTY_FIELDS);
    mr.read();
}


// set the number of the first segment that is likely to be an image
// of the lidar data, and return true iff we found one
bool NitfFile::locateLidarImageSegment()
{
    // as per 3.2.3 (pag 19) and 3.2.4 (page 39)

    ::nitf::ListIterator iter = m_record.getImages().begin();
    const ::nitf::Uint32 numSegs = m_record.getNumImages();

    for (::nitf::Uint32 segNum=0; segNum<numSegs; segNum++)
    {
        ::nitf::ImageSegment imseg = *iter;

        ::nitf::ImageSubheader subheader = imseg.getSubheader();

        ::nitf::Field field = subheader.getImageId();
        ::nitf::Field::FieldType fieldType = field.getType();
        if (fieldType != (::nitf::Field::FieldType)NITF_BCS_A)
            throw pdal_error("error reading nitf (5)");
        std::string iid1 = field.toString();

        // BUG: shouldn't allow "None" here!
        if (iid1 == "INTENSITY " || iid1 == "ELEVATION " ||
            iid1 == "None      ")
        {
            m_lidarImageSegment = segNum;
            return true;
        }

        iter++;
    }

    return false;
}


// set the number of the first segment that is likely to be the LAS
// file, and return true iff we found it
bool NitfFile::locateLidarDataSegment()
{
    // as per 3.2.5, page 59

    ::nitf::ListIterator iter = m_record.getDataExtensions().begin();
    const ::nitf::Uint32 numSegs = m_record.getNumDataExtensions();
    for (::nitf::Uint32 segNum=0; segNum<numSegs; segNum++)
    {
        ::nitf::DESegment seg = *iter;

        ::nitf::DESubheader subheader = seg.getSubheader();

        ::nitf::Field idField = subheader.getTypeID();
        if (idField.getType() != (::nitf::Field::FieldType)NITF_BCS_A)
            throw pdal_error("error reading nitf (6)");

        ::nitf::Field verField = subheader.getVersion();
        if (verField.getType() != (::nitf::Field::FieldType)NITF_BCS_N)
            throw pdal_error("error reading nitf (7)");

        const std::string id = idField.toString();
        const int ver = (int)verField;

        if (id == "LIDARA DES               " && ver == 1)
        {
            m_lidarDataSegment = segNum;
            return true;
        }

        iter++;
    }

    return false;
}


} // namespaces
