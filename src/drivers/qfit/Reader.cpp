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
#include <pdal/drivers/qfit/Iterator.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>


#include <map>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal { namespace drivers { namespace qfit {

PointIndexes::PointIndexes(const Schema& schema, QFIT_Format_Type format)
{
    Time = schema.getDimensionIndex(DimensionId::Qfit_Time);
    X = schema.getDimensionIndex(DimensionId::X_i32);
    Y = schema.getDimensionIndex(DimensionId::Y_i32);
    Z = schema.getDimensionIndex(DimensionId::Z_i32);
    
    StartPulse = schema.getDimensionIndex(DimensionId::Qfit_StartPulse);
    ReflectedPulse = schema.getDimensionIndex(DimensionId::Qfit_ReflectedPulse);
    ScanAngleRank = schema.getDimensionIndex(DimensionId::Qfit_ScanAngleRank);
    Pitch = schema.getDimensionIndex(DimensionId::Qfit_Pitch);
    Roll = schema.getDimensionIndex(DimensionId::Qfit_Roll);

    if (format == QFIT_Format_14) 
    {
        PassiveSignal = schema.getDimensionIndex(DimensionId::Qfit_PassiveSignal);
        PassiveX = schema.getDimensionIndex(DimensionId::Qfit_PassiveX);
        PassiveY = schema.getDimensionIndex(DimensionId::Qfit_PassiveY);
        PassiveZ = schema.getDimensionIndex(DimensionId::Qfit_PassiveZ);
        GPSTime = schema.getDimensionIndex(DimensionId::Qfit_GpsTime);
        
    } else if (format == QFIT_Format_12)
    {
        PDOP = schema.getDimensionIndex(DimensionId::Qfit_PDOP);
        PulseWidth = schema.getDimensionIndex(DimensionId::Qfit_PulseWidth);
        GPSTime = schema.getDimensionIndex(DimensionId::Qfit_GpsTime);

    } else
    {
        GPSTime = schema.getDimensionIndex(DimensionId::Qfit_GpsTime);
    }
        
    return;
}


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_format(QFIT_Format_Unknown)
    , m_size(0)
    , m_flip_x(true)
    , m_convert_z(true)
{
    std::string filename= getFileName();
    
    m_flip_x = getOptions().getValueOrDefault("flip_coordinates", true);
    m_convert_z = getOptions().getValueOrDefault("convert_z_units", true);
    
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
    QFIT_SWAP_BE_TO_LE(int4);
    
    if ( int4 % 4 != 0)
        throw qfit_error("Base QFIT format is not a multiple of 4, unrecognized format!");
    
    m_size = int4;
    m_format = static_cast<QFIT_Format_Type>(m_size/sizeof(m_size));
    // std::cout << "QFIT Point format " << m_format << " with size: " << m_size << std::endl;
    
    // The offset to start reading point data should be here.
    str->seekg(m_size+sizeof(int4));

    Utils::read_n(int4, *str, sizeof(int4));
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

    if (remainder != 0 )
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
}


const Options Reader::getDefaultOptions() const
{
    Options options;
    Option filename("filename", "", "file to read from");
    Option flip_coordinates("flip_coordinates", true, "Flip coordinates from 0-360 to -180-180");
    Option convert_z_units("convert_z_units", true, "Convert Z units from mm to m");
    options.add(filename);
    options.add(flip_coordinates);
    options.add(convert_z_units);
    return options;
}


std::string Reader::getFileName() const
{
    return getOptions().getOption("filename").getValue<std::string>();
}

void Reader::registerFields()
{
    Schema& schema = getSchemaRef();

    double xy_scale = 1/1000000.0;

    Dimension time(DimensionId::Qfit_Time);
    schema.appendDimension(time);

    Dimension y(DimensionId::Y_i32);
    y.setNumericScale(xy_scale);
    schema.appendDimension(y);

    Dimension x(DimensionId::X_i32);
    x.setNumericScale(xy_scale);
    schema.appendDimension(x);

    Dimension z(DimensionId::Z_i32);
    schema.appendDimension(z);

    Dimension start_pulse(DimensionId::Qfit_StartPulse);
    schema.appendDimension(start_pulse);

    Dimension reflected_pulse(DimensionId::Qfit_ReflectedPulse);
    schema.appendDimension(reflected_pulse);

    Dimension scan_angle(DimensionId::Qfit_ScanAngleRank);
    scan_angle.setNumericScale(1.0/1000.0);
    schema.appendDimension(scan_angle);

    Dimension pitch(DimensionId::Qfit_Pitch);
    pitch.setNumericScale(1.0/1000.0);
    schema.appendDimension(pitch);

    Dimension roll(DimensionId::Qfit_Roll);
    roll.setNumericScale(1.0/1000.0);
    schema.appendDimension(roll);

    if (m_format == QFIT_Format_12) 
    {
        Dimension pdop(DimensionId::Qfit_PDOP);
        pdop.setNumericScale(1.0/10.0);
        schema.appendDimension(pdop);

        Dimension pulse_width(DimensionId::Qfit_PulseWidth);
        schema.appendDimension(pulse_width);

        Dimension gpstime(DimensionId::Qfit_GpsTime);
        schema.appendDimension(gpstime);
    
    } 
    else if (m_format == QFIT_Format_14)
    {
        Dimension passive_signal(DimensionId::Qfit_PassiveSignal);
        schema.appendDimension(passive_signal);

        Dimension passive_y(DimensionId::Qfit_PassiveY);
        passive_y.setNumericScale(xy_scale);
        schema.appendDimension(passive_y);

        Dimension passive_x(DimensionId::Qfit_PassiveX);
        passive_x.setNumericScale(xy_scale);
        schema.appendDimension(passive_x);

        Dimension passive_z(DimensionId::Qfit_PassiveZ);
        schema.appendDimension(passive_z);

        Dimension gpstime(DimensionId::Qfit_GpsTime);
        schema.appendDimension(gpstime);
    }
    else
    {
        Dimension gpstime(DimensionId::Qfit_GpsTime);
        schema.appendDimension(gpstime);
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
    const PointIndexes indexes(schema, m_format);
    
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
            QFIT_SWAP_BE_TO_LE(time);
            data.setField<boost::int32_t>(pointIndex, indexes.Time, time);

            boost::int32_t y = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(y);
            data.setField<boost::int32_t>(pointIndex, indexes.Y, y);

            boost::int32_t x = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(x);
            
            if (m_flip_x) {
                if (x > 180) 
                {
                    x = x - 360;
                }
            }
            data.setField<boost::int32_t>(pointIndex, indexes.X, x);

            boost::int32_t z = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(z);
            
            if (m_convert_z)
            {
                z = static_cast<boost::int32_t>(Utils::sround(static_cast<double>(z)/100.0));
            }
            data.setField<boost::int32_t>(pointIndex, indexes.Z, z);

            boost::int32_t start_pulse = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(start_pulse);
            data.setField<boost::int32_t>(pointIndex, indexes.StartPulse, start_pulse);

            boost::int32_t reflected_pulse = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(reflected_pulse);
            data.setField<boost::int32_t>(pointIndex, indexes.ReflectedPulse, reflected_pulse);

            boost::int32_t scan_angle = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(scan_angle);
            data.setField<boost::int32_t>(pointIndex, indexes.ScanAngleRank, scan_angle);

            boost::int32_t pitch = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(pitch);
            data.setField<boost::int32_t>(pointIndex, indexes.Pitch, pitch);

            boost::int32_t roll = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(roll);
            data.setField<boost::int32_t>(pointIndex, indexes.Roll, roll);

        }

        if (m_format == QFIT_Format_12) 
        {
            boost::int32_t pdop = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(pdop);
            data.setField<boost::int32_t>(pointIndex, indexes.PDOP, pdop);

            boost::int32_t pulse_width = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(pulse_width);
            data.setField<boost::int32_t>(pointIndex, indexes.PulseWidth, pulse_width);

            boost::int32_t gpstime = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(gpstime);
            data.setField<boost::int32_t>(pointIndex, indexes.GPSTime, gpstime);

        }

        else if (m_format == QFIT_Format_14)
        {
            boost::int32_t passive_signal = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(passive_signal);
            data.setField<boost::int32_t>(pointIndex, indexes.PassiveSignal, passive_signal);

            boost::int32_t passive_y = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(passive_y);
            data.setField<boost::int32_t>(pointIndex, indexes.PassiveY, passive_y);

            boost::int32_t passive_x = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(passive_x);
            data.setField<boost::int32_t>(pointIndex, indexes.PassiveX, passive_x);

            boost::int32_t passive_z = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(passive_z);
            data.setField<boost::int32_t>(pointIndex, indexes.PassiveZ, passive_z);

            boost::int32_t gpstime = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(gpstime);
            data.setField<boost::int32_t>(pointIndex, indexes.GPSTime, gpstime);

     
        }
        else {
            boost::int32_t gpstime = Utils::read_field<boost::int32_t>(p);
            QFIT_SWAP_BE_TO_LE(gpstime);
            data.setField<boost::int32_t>(pointIndex, indexes.GPSTime, gpstime);
            
        }
        data.setNumPoints(pointIndex+1);
    }

    delete[] buf;

    // data.setSpatialBounds( lasHeader.getBounds() );

    return numPoints;
}

pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new pdal::drivers::qfit::SequentialIterator(*this);
}


pdal::StageRandomIterator* Reader::createRandomIterator() const
{
    return new pdal::drivers::qfit::RandomIterator(*this);
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}


}}} // namespace pdal::driver::oci
