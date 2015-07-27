/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

/*

Airborne Topographic Mapper (ATM) Project
NASA, Goddard Space Flight Center, Wallops Flight Facility
Principal Investigator: Bill Krabill (William.B.Krabill@nasa.gov)

Description of ATM QFIT Output Data
(revised 2009-feb-13 sm)

ATM data is generally distributed in the output format of the
processing  program, qfit, which combines airborne laser ranging
data and aircraft attitude from the INS with positioning
information from a processed kinematic  differential GPS
trajectory.  Qfit output files, which usually have names ending
in a .qi extension, are organized as 32-bit (4-byte)
binary  words, equivalent to a C or IDL long integer, which are
scaled to retain the precision of the measurements.  The format
and the scaling factors are presented below.  The qfit program is
run on an Apple PowerPC processor (and formerly Sun/Motorola).
Accordingly, the output is written in a big-endian format and
must be byte-swapped to be read with a PC (Intel processor)
which uses a little-endian format to store 32-bit integers.

The files are organized into fixed-length logical records. The
beginning of the file contains a header of one or more records
followed by a data segment, in which there is one record per
laser shot. It is not necessary to interpret the header
to use the laser data->

The first word of the header (and the file) is a 32-bit  binary
integer giving the number of bytes in each logical record.  Commonly
qfit files have 12 words per record and this integer will be the
number 48.  The remainder of the initial logical record is padded
with blank bytes (in this case 44 blank bytes). 10-word and 14-word
formats have also been used, as described below.

The remainder of the header is generally a series of logical
records  containing the processing history of the file.  In these
logical records, the  initial word contains a 32-bit binary
integer with a value between -9000000 and -9000008.  The
remaining bytes in each header record is filled with a string of
ascii characters containing information on file processing
history.  In this case, the byte offset (as a longword integer)
from the start of file to the start of laser data will be the
second word of the second record of the header.  (Note: The header
records can be removed by eliminating records that begin with a
negative value since the first word of records in the data segment
is always a positive number.)

In the data segment of the file, the information contained in
words 1-11 of the output record pertains to the  laser pulse, its
footprint, and aircraft attitude.  The last word of each record is
always the GPS time of day when the laser measurement was acquired.

Prior to 2008 surveys, the GPS trajectory was edited to restrict PDOP<9
in order to limit GPS errors to be less than roughly 5cm.  The output
survey data would therefore have occasional gaps where the PDOP>9.
Some applications of ATM data have less stringent accuracy requirements
that would be better served by preserving the data in these gaps.
Starting in 2008, the PDOP limit was changed to 20, which could allow
occasional GPS errors up to about 15cm.  The PDOP value is carried in
the qfit output and can be used to edit data for applications requiring
greater precision.   Any file in the 10-word format, or files in the 12-word
format processed prior to January 2009, will have PDOP limited
<9.

The three data formats are described below.  The format is designated by
the logical record length given in the first word of the data file.

The qi 12-word format (in use since 2006):
Word #       Content
   1    Relative Time (msec from start of data file)
   2    Laser Spot Latitude (degrees X 1,000,000)
   3    Laser Spot Longitude (degrees X 1,000,000)
   4    Elevation (millimeters)
   5    Start Pulse Signal Strength (relative)
   6    Reflected Laser Signal Strength (relative)
   7    Scan Azimuth (degrees X 1,000)
   8    Pitch (degrees X 1,000)
   9    Roll (degrees X 1,000)
  10    GPS PDOP (dilution of precision) (X 10)
  11    Laser received pulse width (digitizer samples)
  12    GPS Time packed (example: 153320100 = 15h 33m 20s 100ms)

10-word format (used prior to 2006):
Word #       Content
   1    Relative Time (msec from start of data file)
   2    Laser Spot Latitude (degrees X 1,000,000)
   3    Laser Spot Longitude (degrees X 1,000,000)
   4    Elevation (millimeters)
   5    Start Pulse Signal Strength (relative)
   6    Reflected Laser Signal Strength (relative)
   7    Scan Azimuth (degrees X 1,000)
   8    Pitch (degrees X 1,000)
   9    Roll (degrees X 1,000)
  10    GPS Time packed (example: 153320100 = 15h 33m 20s 100ms)


Between 1997 and 2004 some ATM surveys included
a separate sensor to measure passive brightness.
In the 14-word format, words 10-13 pertain to the
passive brightness signal, which is essentially a relative
measure of radiance reflected from the  earth's surface within
the vicinity of the laser pulse.  The horizontal position of the
passive footprint is determined relative to the laser footprint
by a  delay formulated during ground testing at Wallops.  The
elevation of the footprint is synthesized from surrounding laser
elevation data->  NOTE:  The passive data is not calibrated and
its use, if any, should  be qualitative in nature.  It may aid
the interpretation of terrain features. The measurement capability
was engineered into the ATM sensors to aid in the identification
of the water/beach interface acquired with  the instrument in
coastal mapping applications.

14-word format:
Word #       Content
   1    Relative Time (msec from start of data file)
   2    Laser Spot Latitude (degrees X 1,000,000)
   3    Laser Spot Longitude (degrees X 1,000,000)
   4    Elevation (millimeters)
   5    Start Pulse Signal Strength (relative)
   6    Reflected Laser Signal Strength (relative)
   7    Scan Azimuth (degrees X 1,000)
   8    Pitch (degrees X 1,000)
   9    Roll (degrees X 1,000)
  10    Passive Signal (relative)
  11    Passive Footprint Latitude (degrees X 1,000,000)
  12    Passive Footprint Longitude (degrees X 1,000,000)
  13    Passive Footprint Synthesized Elevation (millimeters)
  14    GPS Time packed (example: 153320100 = 15h 33m 20s 100ms)
*/

#include "QfitReader.hpp"

#include <pdal/PointView.hpp>
#include <pdal/util/Extractor.hpp>
#include <pdal/util/portable_endian.hpp>

#include <algorithm>
#include <map>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif


namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.qfit",
    "QFIT Reader",
    "http://pdal.io/stages/readers.qfit.html" );

CREATE_STATIC_PLUGIN(1, 0, QfitReader, Reader, s_info)

std::string QfitReader::getName() const { return s_info.name; }

QfitReader::QfitReader()
    : pdal::Reader()
    , m_format(QFIT_Format_Unknown)
    , m_size(0)
    , m_littleEndian(false)
    , m_istream()
{}


void QfitReader::initialize()
{
    ISwitchableStream str(m_filename);
    if (!str)
    {
        std::ostringstream oss;
        oss << "Unable to open file '" << m_filename << "'";
        throw qfit_error(oss.str());
    }
    str.seek(0);

    int32_t int4(0);

    str >> int4;

    // They started writting little-endian data->

    /* For years we produced ATM data in big-endian format. With changes in
    computer hardware, we reluctantly changed our standard output to
    little-endian. The transition occurred between the two 2010 campaigns. The
    format of the binary ( .qi, for example) files can be identified by
    examining the first four bytes of the file. Read as a long integer (one
    4-byte word), the value will contain the record length (in bytes) of the
    data records. For example, a .qi file containing 14 words per record will
    have a value 56 (=4*14). If the format of the file matches the format of
    your processor, the value will be reasonable without byte-swapping. If the
    format of the file differs from your processor, then the value is
    interpreted as some very large number, unless you swap the byte order. If
    you use Intel or equivalent processors, then you had to byte-swap files
    from spring 2010 and earlier, you do not swap the ones from fall 2010 and
    later. */

    // If the size comes back something other than 4*no_dimensions, we assume
    // The data were flipped
    if (int4 < 100)
    {
        m_littleEndian = true;
    }
    else
    {
        str.switchToBigEndian();
    }

    if (!m_littleEndian)
        int4 = int32_t(be32toh(uint32_t(int4)));

    if (int4 % 4 != 0)
        throw qfit_error("Base QFIT format is not a multiple of 4, "
            "unrecognized format!");

    m_size = int4;
    m_format = static_cast<QFIT_Format_Type>(m_size / sizeof(m_size));

    // The offset to start reading point data should be here.
    str.seek(m_size + sizeof(int4));

    str >> int4;
    m_offset = static_cast<std::size_t>(int4);

    // Seek to the end
    str.seek(0, std::istream::end);
    std::ios::pos_type end = str.position();

    // First integer is the format of the file
    std::ios::off_type offset = static_cast<std::ios::off_type>(m_offset);
    m_point_bytes = end - offset;
}


Options QfitReader::getDefaultOptions()
{
    Options options;
    Option filename("filename", "", "file to read from");
    Option flip_coordinates("flip_coordinates", true,
        "Flip coordinates from 0-360 to -180-180");
    Option convert_z_units("scale_z", 0.001f,
        "Z scale. Use 0.001 to go from mm to m");
    Option little_endian("little_endian", false,
        "Are data in little endian format?");
    options.add(filename);
    options.add(flip_coordinates);
    options.add(convert_z_units);
    options.add(little_endian);
    return options;
}


void QfitReader::processOptions(const Options& ops)
{
    m_flip_x = ops.getValueOrDefault("flip_coordinates", true);
    m_scale_z = ops.getValueOrDefault("scale_z", 0.001);
}


void QfitReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    m_size = 0;
    layout->registerDim(Id::OffsetTime);
    layout->registerDim(Id::Y);
    layout->registerDim(Id::X);
    layout->registerDim(Id::Z);
    layout->registerDim(Id::StartPulse);
    layout->registerDim(Id::ReflectedPulse);
    layout->registerDim(Id::ScanAngleRank);
    layout->registerDim(Id::Pitch);
    layout->registerDim(Id::Roll);
    m_size += 36;

    if (m_format == QFIT_Format_12)
    {
        layout->registerDim(Id::Pdop);
        layout->registerDim(Id::PulseWidth);
        m_size += 8;
    }
    else if (m_format == QFIT_Format_14)
    {
        layout->registerDim(Id::PassiveSignal);
        layout->registerDim(Id::PassiveY);
        layout->registerDim(Id::PassiveX);
        layout->registerDim(Id::PassiveZ);
        m_size += 16;
    }
    m_size += 4;  // For the GPS time that we currently discard.
}


void QfitReader::ready(PointTableRef)
{
    m_numPoints = m_point_bytes / m_size;
    if (m_point_bytes % m_size)
    {
        std::ostringstream msg;
        msg << "Error calculating file point count.  File size is "
            "inconsistent with point size.";
        throw qfit_error(msg.str());
    }
    m_index = 0;
    m_istream.reset(new IStream(m_filename));
    m_istream->seek(getPointDataOffset());
}


point_count_t QfitReader::read(PointViewPtr data, point_count_t count)
{
    if (!m_istream->good())
    {
        throw pdal_error("QFIT file stream is no good!");
    }
    if (m_istream->stream()->eof())
    {
        throw pdal_error("QFIT file stream is eof!");
    }

    count = std::min(m_numPoints - m_index, count);
    std::vector<char> buf(m_size);
    PointId nextId = data->size();
    point_count_t numRead = 0;
    while (count--)
    {
        m_istream->get(buf);
        SwitchableExtractor extractor(buf.data(), m_size, m_littleEndian);

        // always read the base fields
        {
            int32_t time, y, xi, z, start_pulse, reflected_pulse, scan_angle,
                pitch, roll;
            extractor >> time >> y >> xi >> z >> start_pulse >>
                reflected_pulse >> scan_angle >> pitch >> roll;
            double x = xi / 1000000.0;
            if (m_flip_x && x > 180)
                x -= 360;

            data->setField(Dimension::Id::OffsetTime, nextId, time);
            data->setField(Dimension::Id::Y, nextId, y / 1000000.0);
            data->setField(Dimension::Id::X, nextId, x);
            data->setField(Dimension::Id::Z, nextId, z * m_scale_z);
            data->setField(Dimension::Id::StartPulse, nextId, start_pulse);
            data->setField(Dimension::Id::ReflectedPulse, nextId,
                reflected_pulse);
            data->setField(Dimension::Id::ScanAngleRank, nextId,
                scan_angle / 1000.0);
            data->setField(Dimension::Id::Pitch, nextId, pitch / 1000.0);
            data->setField(Dimension::Id::Roll, nextId, roll / 1000.0);
        }

        if (m_format == QFIT_Format_12)
        {
            int32_t pdop, pulse_width;
            extractor >> pdop >> pulse_width;
            data->setField(Dimension::Id::Pdop, nextId, pdop / 10.0);
            data->setField(Dimension::Id::PulseWidth, nextId, pulse_width);
        }
        else if (m_format == QFIT_Format_14)
        {
            int32_t passive_signal, passive_y, passive_x, passive_z;
            extractor >> passive_signal >> passive_y >> passive_x >> passive_z;
            double x = passive_x / 1000000.0;
            if (m_flip_x && x > 180)
                x -= 360;
            data->setField(Dimension::Id::PassiveSignal, nextId, passive_signal);
            data->setField(Dimension::Id::PassiveY, nextId,
                passive_y / 1000000.0);
            data->setField(Dimension::Id::PassiveX, nextId, x);
            data->setField(Dimension::Id::PassiveZ, nextId,
                passive_z * m_scale_z);
        }
        // GPS time is really a GPS offset from the start of the GPS day
        // encoded in this odd way: 153320100 = 15 hours 33 minutes
        // 20 seconds 100 milliseconds.
        // Not sure why we have that AND the other offset time.  For now
        // we'll just extract this time and drop it.
        int32_t gpstime;
        extractor >> gpstime;

        if (m_cb)
            m_cb(*data, nextId);

        numRead++;
        nextId++;
    }
    m_index += numRead;

    return numRead;
}


Dimension::IdList QfitReader::getDefaultDimensions()
{
    Dimension::IdList ids;

    ids.push_back(Dimension::Id::OffsetTime);
    ids.push_back(Dimension::Id::Y);
    ids.push_back(Dimension::Id::X);
    ids.push_back(Dimension::Id::Z);
    ids.push_back(Dimension::Id::StartPulse);
    ids.push_back(Dimension::Id::ReflectedPulse);
    ids.push_back(Dimension::Id::ScanAngleRank);
    ids.push_back(Dimension::Id::Pitch);
    ids.push_back(Dimension::Id::Roll);
    ids.push_back(Dimension::Id::Pdop);
    ids.push_back(Dimension::Id::PulseWidth);
    ids.push_back(Dimension::Id::PassiveSignal);
    ids.push_back(Dimension::Id::PassiveY);
    ids.push_back(Dimension::Id::PassiveX);
    ids.push_back(Dimension::Id::PassiveZ);

    return ids;
}


void QfitReader::done(PointTableRef)
{
    m_istream.reset();
}

} // namespace pdal
