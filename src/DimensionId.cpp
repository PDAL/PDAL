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

#include <pdal/dimension/Dimension.hpp>
#include <pdal/Utils.hpp>

#include <map>

#include <boost/algorithm/string.hpp>

namespace pdal
{

    // --------------------------------------------------------------------------
    
class KnownDimension
{
public:
    DimensionId::Id id;
    Dimension::DataType datatype;
    std::string name;
    std::string description;
};

KnownDimension s_knownDimensions[] =
{
    //
    // common stuff
    //
    { 
        DimensionId::X_i32, Dimension::Int32, "X", 
        "x coordinate as a long integer. You must use the scale and offset information of the header to determine the double value." 
    },
    {
        DimensionId::Y_i32, Dimension::Int32, "Y", 
        "y coordinate as a long integer. You must use the scale and offset information of the header to determine the double value."
    },
    { 
        DimensionId::Z_i32, Dimension::Int32, "Z",
        "z coordinate as a long integer. You must use the scale and offset information of the header to determine the double value." 
    },
    { 
        DimensionId::X_f64, Dimension::Double, "X",
        "x coordinate as a double"
    },
    { 
        DimensionId::Y_f64, Dimension::Double, "Y", 
        "y coordinate as a double" 
    },
    { 
        DimensionId::Z_f64, Dimension::Double, "Z",
        "z coordinate as a double"
    },

    {
        DimensionId::Time_u64, Dimension::Uint64, "Time",
        "Time value" 
    },

    { 
        DimensionId::Red_u16, Dimension::Uint16, "Red",
        "The red image channel value associated with this point" 
    },
    {
        DimensionId::Green_u16, Dimension::Uint16, "Green", 
        "The green image channel value associated with this point" 
    },
    {
        DimensionId::Blue_u16, Dimension::Uint16, "Blue",
        "The blue image channel value associated with this point"
    },
    {
        DimensionId::Red_u8, Dimension::Uint8, "Red",
        "The red image channel value associated with this point"
    },
    {
        DimensionId::Green_u8, Dimension::Uint8, "Green",
        "The green image channel value associated with this point"
    },
    {
        DimensionId::Blue_u8, Dimension::Uint8, "Blue",
        "The blue image channel value associated with this point"
    },

    //
    // las
    // 
    { 
        DimensionId::Las_Intensity, Dimension::Uint16, "Intensity", 
        "The intensity value is the integer representation of the pulse "
        "return magnitude. This value is optional and system specific. "
        "However, it should always be included if available." 
    },
    {
        DimensionId::Las_ReturnNumber, Dimension::Uint8, "ReturnNumber",
        "Return Number: The Return Number is the pulse return number for "
        "a given output pulse. A given output laser pulse can have many "
        "returns, and they must be marked in sequence of return. The first "
        "return will have a Return Number of one, the second a Return "
        "Number of two, and so on up to five returns." 
        },
    { 
        DimensionId::Las_NumberOfReturns, Dimension::Uint8, "NumberOfReturns",
        "Number of Returns (for this emitted pulse): The Number of Returns "
        "is the total number of returns for a given pulse. For example, "
        "a laser data point may be return two (Return Number) within a "
        "total number of five returns." 
        },
    { 
        DimensionId::Las_ScanDirectionFlag, Dimension::Uint8, "ScanDirectionFlag",
        "The Scan Direction Flag denotes the direction at which the "
        "scanner mirror was traveling at the time of the output pulse. "
        "A bit value of 1 is a positive scan direction, and a bit value "
        "of 0 is a negative scan direction (where positive scan direction "
        "is a scan moving from the left side of the in-track direction to "
        "the right side and negative the opposite)." 
    },
    {
        DimensionId::Las_EdgeOfFlightLine, Dimension::Uint8, "EdgeOfFlightLine",
        "The Edge of Flight Line data bit has a value of 1 only when "
        "the point is at the end of a scan. It is the last point on "
        "a given scan line before it changes direction." 
    },
    {
        DimensionId::Las_Classification, Dimension::Uint8, "Classification",
        "Classification in LAS 1.0 was essentially user defined and optional. "
        "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
        "the field is now mandatory. If a point has never been classified, this "
        "byte must be set to zero. There are no user defined classes since "
        "both point format 0 and point format 1 supply 8 bits per point for "
        "user defined operations. Note that the format for classification is a "
        "bit encoded field with the lower five bits used for class and the "
        "three high bits used for flags." 
    },
    {
        DimensionId::Las_ScanAngleRank, Dimension::Int8, "ScanAngleRank",
        "The Scan Angle Rank is a signed one-byte number with a "
        "valid range from -90 to +90. The Scan Angle Rank is the "
        "angle (rounded to the nearest integer in the absolute "
        "value sense) at which the laser point was output from the "
        "laser system including the roll of the aircraft. The scan "
        "angle is within 1 degree of accuracy from +90 to –90 degrees. "
        "The scan angle is an angle based on 0 degrees being nadir, "
        "and –90 degrees to the left side of the aircraft in the "
        "direction of flight." 
    },
    {
        DimensionId::Las_UserData, Dimension::Uint8, "UserData", 
        "This field may be used at the user’s discretion" 
    },
    {
        DimensionId::Las_PointSourceId, Dimension::Uint16, "PointSourceId",
        "This value indicates the file from which this point originated. "
        "Valid values for this field are 1 to 65,535 inclusive with zero "
        "being used for a special case discussed below. The numerical value "
        "corresponds to the File Source ID from which this point originated. "
        "Zero is reserved as a convenience to system implementers. A Point "
        "Source ID of zero implies that this point originated in this file. "
        "This implies that processing software should set the Point Source "
        "ID equal to the File Source ID of the file containing this point "
        "at some time during processing. " 
    },
    {
        DimensionId::Las_WavePacketDescriptorIndex, Dimension::Uint8, "WavePacketDescriptorIndex",
    },
    { 
        DimensionId::Las_WaveformDataOffset, Dimension::Uint64, "WaveformDataOffset",
    },
    { 
        DimensionId::Las_ReturnPointWaveformLocation, Dimension::Uint32, "ReturnPointWaveformLocation",
    },
    { 
        DimensionId::Las_WaveformXt, Dimension::Float, "WaveformXt" , ""
    },
    { 
        DimensionId::Las_WaveformYt, Dimension::Float, "WaveformYt", ""
    },
    { 
        DimensionId::Las_WaveformZt, Dimension::Float, "WaveformZt", ""
    },
    {
        DimensionId::Las_Time, Dimension::Double, "Time",
        "The GPS Time is the double floating point time tag value at "
        "which the point was acquired. It is GPS Week Time if the "
        "Global Encoding low bit is clear and Adjusted Standard GPS "
        "Time if the Global Encoding low bit is set (see Global Encoding "
        "in the Public Header Block description)." 
    },

    //
    // terrasolid
    //
    // X: "Easting"
    // Y: "Northing"
    // Z: "Elevation"
    { 
        DimensionId::TerraSolid_Alpha, Dimension::Uint8, "Alpha", 
        "The alpha image channel value associated with this point" 
    },
    {
        DimensionId::TerraSolid_Classification, Dimension::Uint8, "Classification", 
        "Classification code 0-255"
    },
    {
        DimensionId::TerraSolid_PointSourceId_u8, Dimension::Uint8, "PointSourceId_u8",
        "Flightline number 0-255" 
    },
    {
        DimensionId::TerraSolid_PointSourceId_u16, Dimension::Uint16, "PointSourceId_u16",
        "Flightline number 0-255"
    },
    {
        DimensionId::TerraSolid_ReturnNumber_u8, Dimension::Uint8, "ReturnNumber",
        "Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo."
    },
    {
        DimensionId::TerraSolid_ReturnNumber_u16, Dimension::Uint16, "ReturnNumber",
        "Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo."
    },
    {
        DimensionId::TerraSolid_Flag, Dimension::Uint8, "Flag",
        "Runtime flag (view visibility)"
    },
    {
        DimensionId::TerraSolid_Mark, Dimension::Uint8, "Mark",
        "Runtime flag"
    },
    { 
        DimensionId::TerraSolid_Intensity, Dimension::Uint16, "Intensity", 
        "The intensity value is the integer representation of the pulse "
        "return magnitude. This value is optional and system specific. "
        "However, it should always be included if available."
    },
    {
        DimensionId::TerraSolid_Time, Dimension::Uint32, "TerraSolid Time",
        "32 bit integer time stamps. Time stamps are assumed to be GPS week seconds. The storage format is a 32 bit unsigned integer where each integer step is 0.0002 seconds."
    },

    //
    // chipper
    //
    {
        DimensionId::Chipper_1, Dimension::Int32, "Chipper:PointID", ""
    },
    {
        DimensionId::Chipper_2, Dimension::Int32, "Chipper:BlockID", ""
    },

    //
    // qfit
    //
    // X: "Longitude coordinate with 1/1000000 decimals of precision"
    // Y: "Latitude coordinate with 1/1000000 decimals of precision"
    // Z: "z coordinate as a long integer.  You must use the scale and offset information of the header to determine the double value."
    {
        DimensionId::Qfit_StartPulse, Dimension::Int32, "QFIT:StartPulse", "Start Pulse Signal Strength (relative)" 
     },
    {
        DimensionId::Qfit_ReflectedPulse, Dimension::Int32, "QFIT:ReflectedPulse", "Reflected Laser Signal Strength (relative)" 
    },
    { 
        DimensionId::Qfit_ScanAngleRank, Dimension::Int32, "QFIT:ScanAngleRank", "Scan Azimuth (degrees X 1,000)" 
    },
    {
        DimensionId::Qfit_Pitch, Dimension::Int32, "QFIT:Pitch", "Pitch (degrees X 1,000)" 
    },
    { 
        DimensionId::Qfit_Roll, Dimension::Int32, "QFIT:Roll", "Roll (degrees X 1,000)" 
    },
    { 
        DimensionId::Qfit_Time, Dimension::Int32, "QFIT:Time", "Relative Time (msec from start of data file)" 
    },
    { 
        DimensionId::Qfit_PassiveSignal, Dimension::Int32, "QFIT:PassiveSignal", "Passive Signal (relative)" 
    },
    { 
        DimensionId::Qfit_PassiveX, Dimension::Int32, "QFIT:PassiveX", "Passive Footprint Longitude (degrees X 1,000,000)" 
    },
    { 
        DimensionId::Qfit_PassiveY, Dimension::Int32, "QFIT:PassiveY", "Passive Footprint Latitude (degrees X 1,000,000)"
    },
    { 
        DimensionId::Qfit_PassiveZ, Dimension::Int32, "QFIT:PassiveZ", "Passive Footprint Synthesized Elevation (millimeters)" 
    },
    { 
        DimensionId::Qfit_GpsTime, Dimension::Int32, "QFIT:GpsTime", "GPS Time packed (example: 153320100 = 15h 33m 20s 100ms)" 
    },
    { 
        DimensionId::Qfit_PDOP, Dimension::Int32, "QFIT:PDOP", "GPS PDOP (dilution of precision) (X 10)" 
    },
    { 
        DimensionId::Qfit_PulseWidth, Dimension::Int32, "QFIT:PulseWdith", "Laser received pulse width (digitizer samples)" 
    },

    // eof
    { DimensionId::Undefined, Dimension::Undefined, "" }
};


typedef std::map<DimensionId::Id, boost::uint32_t> Map;
typedef std::pair<DimensionId::Id, boost::uint32_t> Pair;

static Map map;

static void buildMap()
{
    // only do this once
    if (map.size() != 0)
        return;

    boost::uint32_t i=0;
    while (s_knownDimensions[i].id != DimensionId::Undefined)
    {
        assert(map.find(s_knownDimensions[i].id) == map.end());
        Pair pair(s_knownDimensions[i].id,i);
        map.insert(pair);
        ++i;
    }

    return;
}


void DimensionId::lookupKnownDimension(const Id& id, boost::uint32_t/*Dimension::DataType*/& datatype, std::string& name, std::string& description)
{
    buildMap();

    Map::const_iterator iter = map.find(id);
    if (iter == map.end())
    {
        throw pdal_error("Dimension not found");
    }
    assert(iter->first == id);
    int index = iter->second;
    const KnownDimension& kd = s_knownDimensions[index];
    assert(kd.id == id);
    
    datatype = kd.datatype;
    name = kd.name;
    description = kd.description;

    return;
}


bool DimensionId::hasKnownDimension(const Id& id)
{
    buildMap();

    Map::const_iterator iter = map.find(id);
    if (iter == map.end())
    {
        return false;
    }
    return true;
}


DimensionId::Id DimensionId::getIdFromName(std::string const& name)
{
    // BUG: should we be checking for the Double datatype version of X,Y,Z too?
    if (boost::iequals(name, "X"))   
        return DimensionId::X_i32;
    
    if (boost::iequals(name, "Y"))
        return DimensionId::Y_i32;

    if (boost::iequals(name, "Z"))
        return DimensionId::Z_i32;

    if (boost::iequals(name, "Intensity"))
        return DimensionId::Las_Intensity;

    if (boost::iequals(name, "Return Number") ||
        boost::iequals(name, "ReturnNumber"))
        return DimensionId::Las_ReturnNumber;

    if (boost::iequals(name, "Number of Returns") ||
        boost::iequals(name, "NumberOfReturns"))
        return DimensionId::Las_NumberOfReturns;

    if (boost::iequals(name, "Number of Returns"))
        return DimensionId::Las_NumberOfReturns;

    if (boost::iequals(name, "Scan Direction") ||
        boost::iequals(name, "ScanDirectionFlag") ||
        boost::iequals(name, "ScanDirection"))
        return DimensionId::Las_ScanDirectionFlag;

    if (boost::iequals(name, "Flightline Edge") ||
        boost::iequals(name, "EdgeOfFlightLine") ||
        boost::iequals(name, "FlightlineEdge"))
        return DimensionId::Las_EdgeOfFlightLine;

    if (boost::iequals(name, "Classification"))
        return DimensionId::Las_Classification;

    if (boost::iequals(name, "Scan Angle Rank") ||
        boost::iequals(name, "ScanAngle") ||
        boost::iequals(name, "ScanAngleRank"))
        return DimensionId::Las_ScanAngleRank;

    if (boost::iequals(name, "User Data") ||
        boost::iequals(name, "UserData"))
        return DimensionId::Las_UserData;

    if (boost::iequals(name, "Point Source ID")||
        boost::iequals(name, "PointSourceId"))
        return DimensionId::Las_PointSourceId;

    if (boost::iequals(name, "Time"))
        return DimensionId::Las_Time;

    if (boost::iequals(name, "Red"))
        return DimensionId::Red_u16;

    if (boost::iequals(name, "Green"))
        return DimensionId::Green_u16;

    if (boost::iequals(name, "Blue"))
        return DimensionId::Blue_u16;

    if (boost::iequals(name, "Alpha"))
        return DimensionId::TerraSolid_Alpha;
    
    if (boost::iequals(name, "Chipper:PointID"))
        return DimensionId::Chipper_1;

    if (boost::iequals(name, "Chipper:BlockID"))
        return DimensionId::Chipper_2;

    if (boost::iequals(name, "QFIT:StartPulse"))
        return DimensionId::Qfit_StartPulse;

    if (boost::iequals(name, "QFIT:ReflectedPulse"))
        return DimensionId::Qfit_ReflectedPulse;

    if (boost::iequals(name, "QFIT:ScanAngleRank"))
        return DimensionId::Qfit_ScanAngleRank;

    if (boost::iequals(name, "QFIT:Pitch"))
        return DimensionId::Qfit_Pitch;

    if (boost::iequals(name, "QFIT:Roll"))
        return DimensionId::Qfit_Roll;

    if (boost::iequals(name, "QFIT:Time"))
        return DimensionId::Qfit_Time;

    if (boost::iequals(name, "QFIT:PassiveSignal"))
        return DimensionId::Qfit_PassiveSignal;

    if (boost::iequals(name, "QFIT:PassiveX"))
        return DimensionId::Qfit_PassiveX;

    if (boost::iequals(name, "QFIT:PassiveY"))
        return DimensionId::Qfit_PassiveY;

    if (boost::iequals(name, "QFIT:PassiveZ"))
        return DimensionId::Qfit_PassiveZ;

    if (boost::iequals(name, "QFIT:GpsTime"))
        return DimensionId::Qfit_GpsTime;

    if (boost::iequals(name, "QFIT:PDOP"))
        return DimensionId::Qfit_PDOP;

    if (boost::iequals(name, "QFIT:PulseWdith"))
        return DimensionId::Qfit_PulseWidth;
        
    // Yes, this is scary.  What else can we do?
    throw pdal_error("unknown field name: " + name);
}

DimensionId::Id DimensionId::getIdForDimension(Dimension const& d)
{
    // BUG: should we be checking for the Double datatype version of X,Y,Z too?
    if (boost::iequals(d.getName(), "X"))
    {
        switch (d.getInterpretation())
        {
            case dimension::SignedInteger:
                if (d.getByteSize() == 4)
                {
                    return DimensionId::X_i32;
                } 
                break;
            case dimension::Float:
                return DimensionId::X_f64;
                break;
            default:
                return DimensionId::X_i32;
        }
    }   

    if (boost::iequals(d.getName(), "Y"))
    {
        switch (d.getInterpretation())
        {
            case dimension::SignedInteger:
                if (d.getByteSize() == 4)
                {
                    return DimensionId::Y_i32;
                } 
                break;
            case dimension::Float:
                return DimensionId::Y_f64;
                break;
            default:
                return DimensionId::Y_i32;
        }
    }   

    if (boost::iequals(d.getName(), "Z"))
    {
        switch (d.getInterpretation())
        {
            case dimension::SignedInteger:
                if (d.getByteSize() == 4)
                {
                    return DimensionId::Z_i32;
                } 
                break;
            case dimension::Float:
                return DimensionId::Z_f64;
                break;
            default:
                return DimensionId::Z_i32;
        }
    }   


    if (boost::iequals(d.getName(), "Intensity"))
        return DimensionId::Las_Intensity;

    if (boost::iequals(d.getName(), "Return Number") ||
        boost::iequals(d.getName(), "ReturnNumber"))
        return DimensionId::Las_ReturnNumber;

    if (boost::iequals(d.getName(), "Number of Returns") ||
        boost::iequals(d.getName(), "NumberOfReturns"))
        return DimensionId::Las_NumberOfReturns;

    if (boost::iequals(d.getName(), "Number of Returns"))
        return DimensionId::Las_NumberOfReturns;

    if (boost::iequals(d.getName(), "Scan Direction") ||
        boost::iequals(d.getName(), "ScanDirectionFlag") ||
        boost::iequals(d.getName(), "ScanDirection"))
        return DimensionId::Las_ScanDirectionFlag;

    if (boost::iequals(d.getName(), "Flightline Edge") ||
        boost::iequals(d.getName(), "EdgeOfFlightLine") ||
        boost::iequals(d.getName(), "FlightlineEdge"))
        return DimensionId::Las_EdgeOfFlightLine;

    if (boost::iequals(d.getName(), "Classification"))
        return DimensionId::Las_Classification;

    if (boost::iequals(d.getName(), "Scan Angle Rank") ||
        boost::iequals(d.getName(), "ScanAngle") ||
        boost::iequals(d.getName(), "ScanAngleRank"))
        return DimensionId::Las_ScanAngleRank;

    if (boost::iequals(d.getName(), "User Data") ||
        boost::iequals(d.getName(), "UserData"))
        return DimensionId::Las_UserData;

    if (boost::iequals(d.getName(), "Point Source ID")||
        boost::iequals(d.getName(), "PointSourceId"))
        return DimensionId::Las_PointSourceId;

    if (boost::iequals(d.getName(), "Time"))
        return DimensionId::Time_u64;

    if (boost::iequals(d.getName(), "Red"))
        return DimensionId::Red_u8;

    if (boost::iequals(d.getName(), "Green"))
        return DimensionId::Green_u8;

    if (boost::iequals(d.getName(), "Blue"))
        return DimensionId::Blue_u8;

    if (boost::iequals(d.getName(), "Alpha"))
        return DimensionId::TerraSolid_Alpha;
    
    if (boost::iequals(d.getName(), "Chipper:PointID"))
        return DimensionId::Chipper_1;

    if (boost::iequals(d.getName(), "Chipper:BlockID"))
        return DimensionId::Chipper_2;

    if (boost::iequals(d.getName(), "QFIT:StartPulse"))
        return DimensionId::Qfit_StartPulse;

    if (boost::iequals(d.getName(), "QFIT:ReflectedPulse"))
        return DimensionId::Qfit_ReflectedPulse;

    if (boost::iequals(d.getName(), "QFIT:ScanAngleRank"))
        return DimensionId::Qfit_ScanAngleRank;

    if (boost::iequals(d.getName(), "QFIT:Pitch"))
        return DimensionId::Qfit_Pitch;

    if (boost::iequals(d.getName(), "QFIT:Roll"))
        return DimensionId::Qfit_Roll;

    if (boost::iequals(d.getName(), "QFIT:Time"))
        return DimensionId::Qfit_Time;

    if (boost::iequals(d.getName(), "QFIT:PassiveSignal"))
        return DimensionId::Qfit_PassiveSignal;

    if (boost::iequals(d.getName(), "QFIT:PassiveX"))
        return DimensionId::Qfit_PassiveX;

    if (boost::iequals(d.getName(), "QFIT:PassiveY"))
        return DimensionId::Qfit_PassiveY;

    if (boost::iequals(d.getName(), "QFIT:PassiveZ"))
        return DimensionId::Qfit_PassiveZ;

    if (boost::iequals(d.getName(), "QFIT:GpsTime"))
        return DimensionId::Qfit_GpsTime;

    if (boost::iequals(d.getName(), "QFIT:PDOP"))
        return DimensionId::Qfit_PDOP;

    if (boost::iequals(d.getName(), "QFIT:PulseWdith"))
        return DimensionId::Qfit_PulseWidth;
        
    // Yes, this is scary.  What else can we do?
    throw pdal_error("unknown field name: " + d.getName());
}

} // namespace pdal
