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

#include <pdal/drivers/las/Support.hpp>

#include <pdal/drivers/las/SummaryData.hpp>

#include <pdal/exceptions.hpp>


namespace pdal { namespace drivers { namespace las {

void Support::registerFields(Schema& schema, PointFormat format)
{
    std::ostringstream text;

    Dimension x(Dimension::Field_X, Dimension::Int32);
    text << "x coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    x.setDescription(text.str());
    schema.addDimension(x);
    text.str("");

    Dimension y(Dimension::Field_Y, Dimension::Int32);
    text << "y coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    y.setDescription(text.str());
    schema.addDimension(y);
    text.str("");

    Dimension z(Dimension::Field_Z, Dimension::Int32);
    text << "z coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    z.setDescription(text.str());
    schema.addDimension(z);
    text.str("");

    Dimension intensity(Dimension::Field_Intensity, Dimension::Uint16);
    text << "The intensity value is the integer representation of the pulse "
         "return magnitude. This value is optional and system specific. "
         "However, it should always be included if available.";
    intensity.setDescription(text.str());
    schema.addDimension(intensity);
    text.str("");

    Dimension return_no(Dimension::Field_ReturnNumber, Dimension::Uint8); // 3 bits only
    text << "Return Number: The Return Number is the pulse return number for "
         "a given output pulse. A given output laser pulse can have many "
         "returns, and they must be marked in sequence of return. The first "
         "return will have a Return Number of one, the second a Return "
         "Number of two, and so on up to five returns.";
    return_no.setDescription(text.str());
    schema.addDimension(return_no);
    text.str("");

    Dimension no_returns(Dimension::Field_NumberOfReturns, Dimension::Uint8); // 3 bits only
    text << "Number of Returns (for this emitted pulse): The Number of Returns "
         "is the total number of returns for a given pulse. For example, "
         "a laser data point may be return two (Return Number) within a "
         "total number of five returns.";
    no_returns.setDescription(text.str());
    schema.addDimension(no_returns);
    text.str("");

    Dimension scan_dir(Dimension::Field_ScanDirectionFlag, Dimension::Uint8); // 1 bit only
    text << "The Scan Direction Flag denotes the direction at which the "
         "scanner mirror was traveling at the time of the output pulse. "
         "A bit value of 1 is a positive scan direction, and a bit value "
         "of 0 is a negative scan direction (where positive scan direction "
         "is a scan moving from the left side of the in-track direction to "
         "the right side and negative the opposite). ";
    scan_dir.setDescription(text.str());
    schema.addDimension(scan_dir);
    text.str("");

    Dimension edge(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8); // 1 bit only
    text << "The Edge of Flight Line data bit has a value of 1 only when "
         "the point is at the end of a scan. It is the last point on "
         "a given scan line before it changes direction.";
    edge.setDescription(text.str());
    schema.addDimension(edge);
    text.str("");

    Dimension classification(Dimension::Field_Classification, Dimension::Uint8);
    text << "Classification in LAS 1.0 was essentially user defined and optional. "
         "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
         "the field is now mandatory. If a point has never been classified, this "
         "byte must be set to zero. There are no user defined classes since "
         "both point format 0 and point format 1 supply 8 bits per point for "
         "user defined operations. Note that the format for classification is a "
         "bit encoded field with the lower five bits used for class and the "
         "three high bits used for flags.";
    classification.setDescription(text.str());
    schema.addDimension(classification);
    text.str("");

    Dimension scan_angle(Dimension::Field_ScanAngleRank, Dimension::Int8);
    text << "The Scan Angle Rank is a signed one-byte number with a "
         "valid range from -90 to +90. The Scan Angle Rank is the "
         "angle (rounded to the nearest integer in the absolute "
         "value sense) at which the laser point was output from the "
         "laser system including the roll of the aircraft. The scan "
         "angle is within 1 degree of accuracy from +90 to –90 degrees. "
         "The scan angle is an angle based on 0 degrees being nadir, "
         "and –90 degrees to the left side of the aircraft in the "
         "direction of flight.";
    scan_angle.setDescription(text.str());
    schema.addDimension(scan_angle);
    text.str("");

    Dimension user_data(Dimension::Field_UserData, Dimension::Uint8);
    text << "This field may be used at the user’s discretion";
    user_data.setDescription(text.str());
    schema.addDimension(user_data);
    text.str("");

    Dimension point_source_id(Dimension::Field_PointSourceId, Dimension::Uint16);
    text << "This value indicates the file from which this point originated. "
         "Valid values for this field are 1 to 65,535 inclusive with zero "
         "being used for a special case discussed below. The numerical value "
         "corresponds to the File Source ID from which this point originated. "
         "Zero is reserved as a convenience to system implementers. A Point "
         "Source ID of zero implies that this point originated in this file. "
         "This implies that processing software should set the Point Source "
         "ID equal to the File Source ID of the file containing this point "
         "at some time during processing. ";
    point_source_id.setDescription(text.str());
    schema.addDimension(point_source_id);
    text.str("");

    if (hasTime(format))
    {
        Dimension t(Dimension::Field_Time, Dimension::Double);
        text << "The GPS Time is the double floating point time tag value at "
            "which the point was acquired. It is GPS Week Time if the "
            "Global Encoding low bit is clear and Adjusted Standard GPS "
            "Time if the Global Encoding low bit is set (see Global Encoding "
            "in the Public Header Block description).";
        t.setDescription(text.str());
        schema.addDimension(t);
        text.str("");
    }

    if (hasColor(format))
    {
        Dimension red(Dimension::Field_Red, Dimension::Uint16);
        text << "The red image channel value associated with this point";
        red.setDescription(text.str());
        schema.addDimension(red);
        text.str("");

        Dimension green(Dimension::Field_Green, Dimension::Uint16);
        text << "The green image channel value associated with this point";
        green.setDescription(text.str());
        schema.addDimension(green);
        text.str("");

        Dimension blue(Dimension::Field_Blue, Dimension::Uint16);
        text << "The blue image channel value associated with this point";
        blue.setDescription(text.str());
        schema.addDimension(blue);
        text.str("");
    }

    if (hasWave(format))
    {
        schema.addDimension(Dimension(Dimension::Field_WavePacketDescriptorIndex, Dimension::Uint8));
        schema.addDimension(Dimension(Dimension::Field_WaveformDataOffset, Dimension::Uint64));
        schema.addDimension(Dimension(Dimension::Field_ReturnPointWaveformLocation, Dimension::Uint32));
        schema.addDimension(Dimension(Dimension::Field_WaveformXt, Dimension::Float));
        schema.addDimension(Dimension(Dimension::Field_WaveformYt, Dimension::Float));
        schema.addDimension(Dimension(Dimension::Field_WaveformZt, Dimension::Float));
    }
    
    return;
}


void Support::setScaling(Schema& schema, double scaleX, double scaleY, double scaleZ, double offsetX, double offsetY, double offsetZ)
{
    const int idxX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int idxY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int idxZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);

    schema.getDimension(idxX).setNumericScale(scaleX);
    schema.getDimension(idxY).setNumericScale(scaleY);
    schema.getDimension(idxZ).setNumericScale(scaleZ);

    schema.getDimension(idxX).setNumericOffset(offsetX);
    schema.getDimension(idxY).setNumericOffset(offsetY);
    schema.getDimension(idxZ).setNumericOffset(offsetZ);

    return;
}


bool Support::hasTime(PointFormat format)
{
    return (format == PointFormat1) || (format == PointFormat3) || (format == PointFormat4) || (format == PointFormat5);
}


bool Support::hasColor(PointFormat format)
{
    return (format == PointFormat2) || (format == PointFormat3) || (format == PointFormat5);
}


bool Support::hasWave(PointFormat format)
{
    return (format == PointFormat4) || (format == PointFormat5);
}


boost::uint16_t Support::getPointDataSize(PointFormat pointFormat)
{
    switch (pointFormat)
    {
    case PointFormat0: return 20;
    case PointFormat1: return 28;
    case PointFormat2: return 26;
    case PointFormat3: return 34;
    default:
        throw invalid_format("point format unsupported");
    }

}


// --------------------------------------------------------------------------
// LasPointIndexes
// --------------------------------------------------------------------------

PointIndexes::PointIndexes(const Schema& schema, PointFormat format)
{
    X = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    Y = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    Z = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    
    Intensity = schema.getDimensionIndex(Dimension::Field_Intensity, Dimension::Uint16);
    ReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber, Dimension::Uint8);
    NumberOfReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns, Dimension::Uint8);
    ScanDirectionFlag = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag, Dimension::Uint8);
    EdgeOfFlightLine = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8);
    Classification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);
    ScanAngleRank = schema.getDimensionIndex(Dimension::Field_ScanAngleRank, Dimension::Int8);
    UserData = schema.getDimensionIndex(Dimension::Field_UserData, Dimension::Uint8);
    PointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId, Dimension::Uint16);
    
    Time = (Support::hasTime(format) ? schema.getDimensionIndex(Dimension::Field_Time, Dimension::Double) : 0);
    
    Red = (Support::hasColor(format) ? schema.getDimensionIndex(Dimension::Field_Red, Dimension::Uint16) : 0);
    Green = (Support::hasColor(format) ? schema.getDimensionIndex(Dimension::Field_Green, Dimension::Uint16) : 0);
    Blue = (Support::hasColor(format) ? schema.getDimensionIndex(Dimension::Field_Blue, Dimension::Uint16) : 0);
        
    // WavePacketDescriptorIndex = (Support::hasWave(format) ? schema.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    // WaveformDataOffset = (Support::hasWave(format) ? schema.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    // ReturnPointWaveformLocation = (Support::hasWave(format) ? schema.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    // WaveformXt = (Support::hasWave(format) ? schema.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    // WaveformYt = (Support::hasWave(format) ? schema.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    // WaveformZt = (Support::hasWave(format) ? schema.getDimensionIndex(Dimension::Field_WaveformZt) : 0);

    return;
}


void Support::rewriteHeader(std::ostream& stream, const SummaryData& data)
{
    // move from header start to "number of point records" field
    stream.seekp(107, std::ios_base::cur);

    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        Utils::write_field<boost::uint32_t>(p, data.getTotalNumPoints());

        for (int i=1; i<=SummaryData::s_maxNumReturns; i++)
        {
            Utils::write_field<boost::uint32_t>(p, data.getReturnCount(i));
        }

        Utils::write_n(stream, buf, 4 + 4*SummaryData::s_maxNumReturns);
    }

    // skip over scale/offset fields
    stream.seekp(8*6, std::ios_base::cur);

    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        double minX, minY, minZ, maxX, maxY, maxZ;
        data.getBounds(minX, minY, minZ, maxX, maxY, maxZ);
        Utils::write_field<double>(p, maxX);
        Utils::write_field<double>(p, minX);
        Utils::write_field<double>(p, maxY);
        Utils::write_field<double>(p, minY);
        Utils::write_field<double>(p, maxZ);
        Utils::write_field<double>(p, minZ);
        Utils::write_n(stream, buf, 6*8);
    }

    stream.seekp(0, std::ios_base::end);

    return;
}


} } } // namespaces
