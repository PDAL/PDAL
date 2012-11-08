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
to use the laser data.

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
elevation data.  NOTE:  The passive data is not calibrated and
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


#include <pdal/drivers/qfit/Reader.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>


#include <map>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{
namespace drivers
{
namespace qfit
{

PointDimensions::PointDimensions(const Schema& schema, std::string const& ns)
{

    Time = &schema.getDimension("Time", ns);

    X = &schema.getDimension("X", ns);
    Y = &schema.getDimension("Y", ns);
    Z = &schema.getDimension("Z", ns);

    StartPulse = &schema.getDimension("StartPulse", ns);
    ReflectedPulse = &schema.getDimension("ReflectedPulse", ns);
    ScanAngleRank = &schema.getDimension("ScanAngleRank", ns);
    Pitch = &schema.getDimension("Pitch", ns);
    Roll = &schema.getDimension("Roll", ns);

    try
    {
        GPSTime = &schema.getDimension("GPSTime", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        GPSTime = 0;
    }

    try
    {
        PulseWidth = &schema.getDimension("PulseWidth", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        PulseWidth = 0;
    }

    try
    {
        PDOP = &schema.getDimension("PDOP", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        PDOP = 0;
    }

    try
    {
        PassiveSignal = &schema.getDimension("PassiveSignal", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        PassiveSignal = 0;
    }

    try
    {
        PassiveY = &schema.getDimension("PassiveY", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        PassiveY = 0;
    }

    try
    {
        PassiveX = &schema.getDimension("PassiveX", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        PassiveX = 0;
    }

    try
    {
        PassiveZ = &schema.getDimension("PassiveZ", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        PassiveZ = 0;
    }


    return;
}

Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_format(QFIT_Format_Unknown)
    , m_size(0)
    , m_flip_x(true)
    , m_scale_z(1.0)
    , m_littleEndian(false)
{

    std::string filename= getFileName();

    m_flip_x = getOptions().getValueOrDefault("flip_coordinates", true);
    m_scale_z = getOptions().getValueOrDefault("scale_z", 1.0);

    std::istream* str = FileUtils::openFile(filename);

    if (str == 0)
    {
        std::ostringstream oss;
        oss << "Unable to open file '" << filename << "'";
        throw qfit_error(oss.str());
    }
    str->seekg(0);

    boost::int32_t int4(0);

    Utils::read_n(int4, *str, sizeof(int4));



    // They started writting little-endian data.

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

    if (!m_littleEndian)
        QFIT_SWAP_BE_TO_LE(int4);

    if (int4 % 4 != 0)
        throw qfit_error("Base QFIT format is not a multiple of 4, unrecognized format!");

    m_size = int4;

    m_format = static_cast<QFIT_Format_Type>(m_size/sizeof(m_size));
    // std::cout << "QFIT Point format " << m_format << " with size: " << m_size << std::endl;

    // The offset to start reading point data should be here.
    str->seekg(m_size+sizeof(int4));

    Utils::read_n(int4, *str, sizeof(int4));
    if (!m_littleEndian)
        QFIT_SWAP_BE_TO_LE(int4);

    m_offset = static_cast<std::size_t>(int4);

    registerFields();

    const Schema& schema = getSchema();

    // Seek to the beginning
    str->seekg(0, std::ios::beg);
    std::ios::pos_type beginning = str->tellg();

    // Seek to the end
    str->seekg(0, std::ios::end);
    std::ios::pos_type end = str->tellg();
    std::ios::off_type size = end - beginning;

    // First integer is the format of the file
    std::ios::off_type offset = static_cast<std::ios::off_type>(m_offset);
    std::ios::off_type length = static_cast<std::ios::off_type>(schema.getByteSize());
    std::ios::off_type point_bytes = end - offset;

    // Figure out how many points we have and whether or not we have
    // extra slop in there.
    std::ios::off_type count = point_bytes / length;
    std::ios::off_type remainder = point_bytes % length;

    // std::cout << "count: " << count << std::endl;
    // std::cout <<" point_bytes: " << point_bytes << std::endl;
    // std::cout <<" length: " << length << std::endl;
    // std::cout <<" offset: " << offset << std::endl;
    // std::cout <<" remainder: " << remainder << std::endl;
    // std::cout <<" beginning: " << beginning << std::endl;

    if (remainder != 0)
    {
        std::ostringstream msg;
        msg <<  "The number of points in the header that was set "
            "by the software does not match the actual number of points in the file "
            "as determined by subtracting the data offset ("
            << m_offset << ") from the file length ("
            << size <<  ") and dividing by the point record length ("
            << schema.getByteSize() << ")."
            " It also does not perfectly contain an exact number of"
            " point data and we cannot infer a point count."
            " Calculated number of points: " << count <<
            " Point data remainder: " << remainder;
        throw qfit_error(msg.str());

    }
    setPointCountType(PointCount_Fixed);
    setNumPoints(count);

    if (str != 0)
        delete str;
}


void Reader::initialize()
{
    pdal::Reader::initialize();

    std::ostringstream oss;
    oss << "flipping coordinates?: " << m_flip_x;
    log()->get(logDEBUG) << oss.str() << std::endl;
    oss.str("");
    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    oss.precision(Utils::getStreamPrecision(m_scale_z));
    oss << "setting z scale to: " << m_scale_z;
    log()->get(logDEBUG) << oss.str() << std::endl;
}


Options Reader::getDefaultOptions()
{
    Options options;
    Option filename("filename", "", "file to read from");
    Option flip_coordinates("flip_coordinates", true, "Flip coordinates from 0-360 to -180-180");
    Option convert_z_units("scale_z", 1.0f, "Z scale. Use 0.001 to go from mm to m");
    Option little_endian("little_endian", false, "Are data in little endian format?");
    options.add(filename);
    options.add(flip_coordinates);
    options.add(convert_z_units);
    options.add(little_endian);
    return options;
}


std::string Reader::getFileName() const
{
    return getOptions().getOption("filename").getValue<std::string>();
}

void Reader::registerFields()
{
    Schema& schema = getSchemaRef();

    Schema dimensions(getDefaultDimensions());

    schema.appendDimension(dimensions.getDimension("Time"));
    schema.appendDimension(dimensions.getDimension("Y"));
    schema.appendDimension(dimensions.getDimension("X"));

    Dimension z = dimensions.getDimension("Z");
    z.setNumericScale(m_scale_z);
    schema.appendDimension(z);

    schema.appendDimension(dimensions.getDimension("StartPulse"));
    schema.appendDimension(dimensions.getDimension("ReflectedPulse"));
    schema.appendDimension(dimensions.getDimension("ScanAngleRank"));
    schema.appendDimension(dimensions.getDimension("Pitch"));
    schema.appendDimension(dimensions.getDimension("Roll"));

    if (m_format == QFIT_Format_12)
    {
        schema.appendDimension(dimensions.getDimension("PDOP"));
        schema.appendDimension(dimensions.getDimension("PulseWidth"));
        schema.appendDimension(dimensions.getDimension("GPSTime"));

    }
    else if (m_format == QFIT_Format_14)
    {
        schema.appendDimension(dimensions.getDimension("PassiveSignal"));
        schema.appendDimension(dimensions.getDimension("PassiveY"));
        schema.appendDimension(dimensions.getDimension("PassiveX"));
        Dimension z = dimensions.getDimension("PassiveZ");
        z.setNumericScale(m_scale_z);
        schema.appendDimension(z);
        schema.appendDimension(dimensions.getDimension("GPSTime"));
    }
    else
    {
        schema.appendDimension(dimensions.getDimension("GPSTime"));
    }

    return;
}


Reader::~Reader()
{

    return;
}

boost::uint32_t Reader::processBuffer(PointBuffer& data, std::istream& stream, boost::uint64_t numPointsLeft) const
{
    // we must not read more points than are left in the file
    const boost::uint64_t numPoints64 = std::min<boost::uint64_t>(data.getCapacity(), numPointsLeft);
    const boost::uint32_t numPoints = (boost::uint32_t)std::min<boost::uint64_t>(numPoints64, std::numeric_limits<boost::uint32_t>::max());

    const Schema& schema = data.getSchema();

    const int pointByteCount = getPointDataSize();
    const PointDimensions dimensions(schema, getName());

    Dimension const* dimX = &schema.getDimension("X", getName());
    Dimension const* dimPassiveX(0);
    try
    {
        dimPassiveX = &schema.getDimension("PassiveX", getName());
    }
    catch (pdal::dimension_not_found&)
    {
        dimPassiveX = 0;
    }
    boost::uint8_t* buf = new boost::uint8_t[pointByteCount * numPoints];


    if (!stream.good())
    {
        throw pdal_error("QFIT Reader::processBuffer stream is no good!");
    }
    if (stream.eof())
    {
        throw pdal_error("QFIT Reader::processBuffer stream is eof!");
    }

    Utils::read_n(buf, stream, pointByteCount * numPoints);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        boost::uint8_t* p = buf + pointByteCount * pointIndex;

        // always read the base fields
        {

            boost::int32_t time = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(time);
            
            if (dimensions.Time)
                data.setField<boost::int32_t>(*dimensions.Time, pointIndex, time);

            boost::int32_t y = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(y);
            if (dimensions.Y)
                data.setField<boost::int32_t>(*dimensions.Y, pointIndex, y);

            boost::int32_t x = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(x);


            if (m_flip_x)
            {
                double xd = dimX->applyScaling(x);
                if (xd > 180)
                {
                    xd = xd - 360;
                    x = dimX->removeScaling<boost::int32_t>(xd);
                }
            }
            if (dimensions.X)
                data.setField<boost::int32_t>(*dimensions.X, pointIndex, x);

            boost::int32_t z = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(z);
            if (dimensions.Z)
                data.setField<boost::int32_t>(*dimensions.Z, pointIndex, z);

            boost::int32_t start_pulse = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(start_pulse);
            if (dimensions.StartPulse)
                data.setField<boost::int32_t>(*dimensions.StartPulse, pointIndex, start_pulse);

            boost::int32_t reflected_pulse = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(reflected_pulse);
            if (dimensions.ReflectedPulse)
                data.setField<boost::int32_t>(*dimensions.ReflectedPulse, pointIndex, reflected_pulse);

            boost::int32_t scan_angle = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(scan_angle);
            if (dimensions.ScanAngleRank)
                data.setField<boost::int32_t>(*dimensions.ScanAngleRank, pointIndex, scan_angle);

            boost::int32_t pitch = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(pitch);
            if (dimensions.Pitch)
                data.setField<boost::int32_t>(*dimensions.Pitch, pointIndex, pitch);

            boost::int32_t roll = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(roll);
            if (dimensions.Roll)
                data.setField<boost::int32_t>(*dimensions.Roll, pointIndex, roll);

        }

        if (m_format == QFIT_Format_12)
        {
            boost::int32_t pdop = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(pdop);
            if (dimensions.PDOP)
                data.setField<boost::int32_t>(*dimensions.PDOP, pointIndex, pdop);

            boost::int32_t pulse_width = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(pulse_width);
            if (dimensions.PulseWidth)
                data.setField<boost::int32_t>(*dimensions.PulseWidth, pointIndex, pulse_width);

            boost::int32_t gpstime = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(gpstime);
            if (dimensions.GPSTime)
                data.setField<boost::int32_t>(*dimensions.GPSTime, pointIndex, gpstime);

        }

        else if (m_format == QFIT_Format_14)
        {
            boost::int32_t passive_signal = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(passive_signal);
            if (dimensions.PassiveSignal)
                data.setField<boost::int32_t>(*dimensions.PassiveSignal, pointIndex, passive_signal);

            boost::int32_t passive_y = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(passive_y);
            if (dimensions.PassiveY)
                data.setField<boost::int32_t>(*dimensions.PassiveY, pointIndex, passive_y);

            boost::int32_t passive_x = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(passive_x);

            if (m_flip_x)
            {
                double xd = dimX->applyScaling(passive_x);
                if (xd > 180)
                {
                    xd = xd - 360;
                    passive_x = dimPassiveX->removeScaling<boost::int32_t>(xd);
                }
            }
            if (dimensions.PassiveX)
                data.setField<boost::int32_t>(*dimensions.PassiveX, pointIndex, passive_x);

            boost::int32_t passive_z = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(passive_z);
            if (dimensions.PassiveZ)
                data.setField<boost::int32_t>(*dimensions.PassiveZ, pointIndex, passive_z);

            boost::int32_t gpstime = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(gpstime);
            if (dimensions.GPSTime)
                data.setField<boost::int32_t>(*dimensions.GPSTime, pointIndex, gpstime);


        }
        else
        {
            boost::int32_t gpstime = Utils::read_field<boost::int32_t>(p);
            if (!m_littleEndian)
                QFIT_SWAP_BE_TO_LE(gpstime);
            if (dimensions.GPSTime)
                data.setField<boost::int32_t>(*dimensions.GPSTime, pointIndex, gpstime);

        }
        data.setNumPoints(pointIndex+1);
    }

    delete[] buf;

    return numPoints;
}

pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::qfit::iterators::sequential::Reader(*this, buffer);
}


pdal::StageRandomIterator* Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::qfit::iterators::random::Reader(*this, buffer);
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

std::vector<Dimension> Reader::getDefaultDimensions()
{
    std::vector<Dimension> output;
    
    Dimension time("Time", dimension::SignedInteger, 4,
                   "Relative Time (msec from start of data file)");
    time.setUUID("adfc310a-111d-437b-b76b-5a1805eab8ad");
    time.setNamespace(s_getName());
    output.push_back(time);

    double xy_scale = 1/1000000.0;

    Dimension y("Y", dimension::SignedInteger, 4,
                "Laser Spot Latitude (degrees X 1,000,000).");
    y.setUUID("bf8e431c-86a6-4b6e-90d8-c23c3ad1fb90");
    y.setNumericScale(xy_scale);
    y.setNamespace(s_getName());
    output.push_back(y);

    Dimension x("X", dimension::SignedInteger, 4,
                "Laser Spot Longitude (degrees X 1,000,000).");
    x.setUUID("1e524a69-e239-4e7a-aea1-ef697895a3c0");
    x.setNumericScale(xy_scale);
    x.setNamespace(s_getName());
    output.push_back(x);

    Dimension z("Z", dimension::SignedInteger, 4,
                "Elevation (millimeters)");
    z.setUUID("cadb2807-c14f-4ce3-8b7a-b393a91a9f10");
    z.setNamespace(s_getName());
    output.push_back(z);

    Dimension start_pulse("StartPulse", dimension::SignedInteger, 4,
                          "Start Pulse Signal Strength (relative)");
    start_pulse.setUUID("4830d76f-c736-4b09-9121-e751323438f3");
    start_pulse.setNamespace(s_getName());
    output.push_back(start_pulse);

    Dimension reflected_pulse("ReflectedPulse", dimension::SignedInteger, 4,
                              "Start Pulse Signal Strength (relative)");
    reflected_pulse.setUUID("8526cc21-3fe6-4876-84c7-3303384f56b1");
    reflected_pulse.setNamespace(s_getName());
    output.push_back(reflected_pulse);

    Dimension scan_angle("ScanAngleRank", dimension::SignedInteger, 4,
                         "Scan Azimuth (degrees X 1,000)");
    scan_angle.setUUID("81e73eec-d342-471c-bcb2-6a534a7334ec");
    scan_angle.setNumericScale(1.0/1000.0);
    scan_angle.setNamespace(s_getName());
    output.push_back(scan_angle);

    Dimension pitch("Pitch", dimension::SignedInteger, 4,
                    "Pitch (degrees X 1,000)");
    pitch.setUUID("6aa8f3b8-1c88-49b4-844c-cc76a7186e8c");
    pitch.setNumericScale(1.0/1000.0);
    pitch.setNamespace(s_getName());
    output.push_back(pitch);

    Dimension roll("Roll", dimension::SignedInteger, 4,
                   "Roll (degrees X 1,000)");
    roll.setUUID("bec65174-21a9-40e4-a7a9-d5d207f72464");
    roll.setNumericScale(1.0/1000.0);
    roll.setNamespace(s_getName());
    output.push_back(roll);

    Dimension pdop("PDOP", dimension::SignedInteger, 4,
                   "GPS PDOP (dilution of precision) (X 10)");
    pdop.setUUID("03d1b370-0d96-4c9a-ba3e-bfd5baffc926");
    pdop.setNumericScale(1.0/10.0);
    pdop.setNamespace(s_getName());
    output.push_back(pdop);

    Dimension width("PulseWidth", dimension::SignedInteger, 4,
                    "Laser received pulse width (digitizer samples)");
    width.setUUID("1ec1ec5d-b77f-4a3b-8382-d83d0f8e96b8");
    width.setNamespace(s_getName());
    output.push_back(width);

    Dimension gpstime("GPSTime", dimension::SignedInteger, 4,
                      "GPS Time packed (example: 153320100 = 15h 33m 20s 100ms)");
    gpstime.setUUID("bcd3e53f-c869-438a-9da9-b5a1ee0d324a");
    gpstime.setNamespace(s_getName());
    output.push_back(gpstime);

    Dimension passive_signal("PassiveSignal", dimension::SignedInteger, 4,
                             "Passive Signal (relative)");
    passive_signal.setUUID("015a27b2-2fe1-44b6-b5ab-0ee16c2d43e2");
    passive_signal.setNamespace(s_getName());
    output.push_back(passive_signal);

    Dimension passive_y("PassiveY", dimension::SignedInteger, 4,
                        "Passive Footprint Latitude (degrees X 1,000,000)");
    passive_y.setUUID("b2a4863b-b580-40a5-b5e7-117d80221240");
    passive_y.setNumericScale(xy_scale);
    passive_y.setNamespace(s_getName());
    output.push_back(passive_y);

    Dimension passive_x("PassiveX", dimension::SignedInteger, 4,
                        "Passive Footprint Longitude (degrees X 1,000,000)");
    passive_x.setUUID("6884be08-c9ee-4d7f-83a7-cb4c9cad4745");
    passive_x.setNumericScale(xy_scale);
    passive_x.setNamespace(s_getName());
    output.push_back(passive_x);

    Dimension passive_z("PassiveZ", dimension::SignedInteger, 4,
                        "Passive Footprint Synthesized Elevation (millimeters)");
    passive_z.setUUID("0d3dec08-188d-4c2e-84d2-bb6881d40c7e");
    passive_z.setNamespace(s_getName());
    output.push_back(passive_z);
    
    return output;
}


namespace iterators
{

namespace sequential
{


Reader::Reader(const pdal::drivers::qfit::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderSequentialIterator(reader, buffer)
    , m_reader(reader)
    , m_istream(NULL)
{
    m_istream = FileUtils::openFile(m_reader.getFileName());
    m_istream->seekg(m_reader.getPointDataOffset());
    return;
}


Reader::~Reader()
{
    FileUtils::closeFile(m_istream);
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{
    m_istream->seekg(m_reader.getPointDataSize() * count, std::ios::cur);
    return count;
}


bool Reader::atEndImpl() const
{
    return getIndex() >= getStage().getNumPoints();
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return m_reader.processBuffer(data, *m_istream, getStage().getNumPoints()-this->getIndex());
}

} // sequential

namespace random
{


Reader::Reader(const pdal::drivers::qfit::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderRandomIterator(reader, buffer)
    , m_reader(reader)
    , m_istream(NULL)
{
    m_istream = FileUtils::openFile(m_reader.getFileName());
    m_istream->seekg(m_reader.getPointDataOffset());
    return;
}


Reader::~Reader()
{
    FileUtils::closeFile(m_istream);
    return;
}


boost::uint64_t Reader::seekImpl(boost::uint64_t count)
{

    if (!m_istream->good())
        throw pdal_error("QFIT RandomIterator::seekImpl stream is no good before seeking!");

    m_istream->seekg(m_reader.getPointDataSize() * count + m_reader.getPointDataOffset(), std::ios::beg);

    if (m_istream->eof())
        throw pdal_error("Seek past the end of the file!");

    if (!m_istream->good())
        throw pdal_error("QFIT RandomIterator::seekImpl stream is no good!");

    return count;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    boost::uint64_t numpoints = getStage().getNumPoints();
    boost::uint64_t index = this->getIndex();

    return m_reader.processBuffer(data, *m_istream, numpoints-index);
}


} // random
} // iterators

}
}
} // namespace pdal::driver::oci
