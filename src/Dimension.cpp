/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Dimension implementation for C++ libLAS
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Howard Butler
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

#include <pdal/Dimension.hpp>
#include <pdal/external/boost/uuid/nil_generator.hpp>

#include <map>

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
        DimensionId::Chipper_1, Dimension::Int32, "Chipper Point ID", ""
    },
    {
        DimensionId::Chipper_2, Dimension::Int32, "Chipper Block ID", ""
    },

    //
    // qfit
    //
    // X: "Longitude coordinate with 1/1000000 decimals of precision"
    // Y: "Latitude coordinate with 1/1000000 decimals of precision"
    // Z: "z coordinate as a long integer.  You must use the scale and offset information of the header to determine the double value."
    { DimensionId::Qfit_StartPulse, Dimension::Int32, "StartPulse", "Start Pulse Signal Strength (relative)" 
        },
    {
        DimensionId::Qfit_ReflectedPulse, Dimension::Int32, "ReflectedPulse", "Reflected Laser Signal Strength (relative)" 
    },
    { 
        DimensionId::Qfit_ScanAngleRank, Dimension::Int32, "ScanAngleRank", "Scan Azimuth (degrees X 1,000)" 
    },
    {
        DimensionId::Qfit_Pitch, Dimension::Int32, "Pitch", "Pitch (degrees X 1,000)" 
    },
    { 
        DimensionId::Qfit_Roll, Dimension::Int32, "Roll", "Roll (degrees X 1,000)" 
    },
    { 
        DimensionId::Qfit_Time, Dimension::Int32, "Qfit Time", "Relative Time (msec from start of data file)" 
    },
    { 
        DimensionId::Qfit_PassiveSignal, Dimension::Int32, "PassiveSignal", "Passive Signal (relative)" 
    },
    { 
        DimensionId::Qfit_PassiveX, Dimension::Int32, "PassiveX", "Passive Footprint Longitude (degrees X 1,000,000)" 
    },
    { 
        DimensionId::Qfit_PassiveY, Dimension::Int32, "PassiveY", "Passive Footprint Latitude (degrees X 1,000,000)"
    },
    { 
        DimensionId::Qfit_PassiveZ, Dimension::Int32, "PassiveZ", "Passive Footprint Synthesized Elevation (millimeters)" 
    },
    { 
        DimensionId::Qfit_GpsTime, Dimension::Int32, "GpsTime", "GPS Time packed (example: 153320100 = 15h 33m 20s 100ms)" 
    },
    { 
        DimensionId::Qfit_PDOP, Dimension::Int32, "PDOP", "GPS PDOP (dilution of precision) (X 10)" 
    },
    { 
        DimensionId::Qfit_PulseWidth, Dimension::Int32, "PulseWdith", "Laser received pulse width (digitizer samples)" 
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


// BUG: this is too slow
static const KnownDimension& lookupKnownDimension(const DimensionId::Id& id)
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
    return kd;
}

static bool hasKnownDimension(const DimensionId::Id& id)
{
    buildMap();

    Map::const_iterator iter = map.find(id);
    if (iter == map.end())
    {
        return false;
    }
    return true;
}


// --------------------------------------------------------------------------

Dimension::Dimension(DimensionId::Id id)
    : m_dataType(Undefined)
    , m_id(id)
    , m_name(std::string(""))
    , m_flags(0)
    , m_endian(pdal::Endian_Little)
    , m_byteSize(0)
    , m_description(std::string(""))
    , m_min(0.0)
    , m_max(0.0)
    , m_precise(false)
    , m_numericScale(0.0)
    , m_numericOffset(0.0)
{
    const KnownDimension& kd = lookupKnownDimension(id);
    m_dataType = kd.datatype;
    m_name = kd.name;
    m_description = kd.description;

    m_byteSize = getDataTypeSize(m_dataType);
}


Dimension::Dimension(DimensionId::Id id, DataType dataType, std::string name, std::string description)
    : m_dataType(dataType)
    , m_id(id)
    , m_name(name)
    , m_flags(0)
    , m_endian(pdal::Endian_Little)
    , m_byteSize(0)
    , m_description(description)
    , m_min(0.0)
    , m_max(0.0)
    , m_precise(false)
    , m_numericScale(0.0)
    , m_numericOffset(0.0)
{
    assert(!hasKnownDimension(id));
    
    m_byteSize = getDataTypeSize(m_dataType);
}

/// copy constructor
Dimension::Dimension(Dimension const& other) 
    : m_dataType(other.m_dataType)
    , m_id(other.m_id)
    , m_name(other.m_name)
    , m_flags(other.m_flags)
    , m_endian(other.m_endian)
    , m_byteSize(other.m_byteSize)
    , m_description(other.m_description)
    , m_min(other.m_min)
    , m_max(other.m_max)
    , m_precise(other.m_precise)
    , m_numericScale(other.m_numericScale)
    , m_numericOffset(other.m_numericOffset)
{
    return;
}

/// assignment operator
Dimension& Dimension::operator=(Dimension const& rhs)
{
    if (&rhs != this)
    {
        m_dataType = rhs.m_dataType;
        m_id = rhs.m_id;
        m_name = rhs.m_name;
        m_flags = rhs.m_flags;
        m_endian = rhs.m_endian;
        m_byteSize = rhs.m_byteSize;
        m_description = rhs.m_description;
        m_min = rhs.m_min;
        m_max = rhs.m_max;
        m_precise = rhs.m_precise;
        m_numericScale = rhs.m_numericScale;
        m_numericOffset = rhs.m_numericOffset;
    }

    return *this;
}


bool Dimension::operator==(const Dimension& other) const
{
    if (m_dataType == other.m_dataType &&
        m_id == other.m_id &&
        m_name == other.m_name &&
        m_flags == other.m_flags &&
        m_endian == other.m_endian &&
        m_byteSize == other.m_byteSize &&
        m_description == other.m_description &&
        Utils::compare_approx(m_min, other.m_min, (std::numeric_limits<double>::min)()) &&
        Utils::compare_approx(m_max, other.m_max, (std::numeric_limits<double>::min)()) &&
        m_precise == other.m_precise &&
        Utils::compare_approx(m_numericScale, other.m_numericScale, (std::numeric_limits<double>::min)()) &&
        Utils::compare_approx(m_numericOffset, other.m_numericOffset, (std::numeric_limits<double>::min)()) 
        
        )
    {
        return true;
    }

    return false;
}


bool Dimension::operator!=(const Dimension& other) const
{
  return !(*this==other);
}


std::string Dimension::getDataTypeName(DataType type)
{
    switch (type)
    {
    case Int8:
        return "Int8";
    case Uint8:
        return "Uint8";
    case Int16:
        return "Int16";
    case Uint16:
        return "Uint16";
    case Int32:
        return "Int32";
    case Uint32:
        return "Uint32";
    case Pointer:
        return "Pointer";
    case Int64:
        return "Int64";
    case Uint64:
        return "Uint64";
    case Float:
        return "Float";
    case Double:
        return "Double";
    case Undefined:
        return "Undefined";
    }
    throw;
}


std::size_t Dimension::getDataTypeSize(DataType type)
{
    switch (type)
    {
    case Int8:
        return 1;
    case Uint8:
        return 1;
    case Int16:
        return 2;
    case Uint16:
        return 2;
    case Int32:
        return 4;
    case Uint32:
        return 4;
    case Pointer:
        return sizeof(void*);
    case Int64:
        return 8;
    case Uint64:
        return 8;
    case Float:
        return 4;
    case Double:
        return 8;
    case Undefined:
        throw;
    }
    throw;
}


bool Dimension::getDataTypeIsNumeric(DataType type)
{
    switch (type)
    {
    case Int8:
    case Uint8:
    case Int16:
    case Uint16:
    case Int32:
    case Uint32:
    case Int64:
    case Uint64:
        return true;
    case Pointer:
        return false;
    case Float:
    case Double:
        return true;
    case Undefined:
        throw;
    }
    throw;
}


bool Dimension::getDataTypeIsSigned(DataType type)
{
    switch (type)
    {
    case Uint8:
    case Uint16:
    case Uint32:
    case Uint64:
        return false;
    case Int8:
    case Int16:
    case Int32:
    case Int64:
        return true;
    case Pointer:
        return false;
    case Float:
    case Double:
        return true;
    case Undefined:
        throw;
        
    }
    throw;
}


bool Dimension::getDataTypeIsInteger(DataType type)
{
    switch (type)
    {
    case Uint8:
    case Uint16:
    case Uint32:
    case Uint64:
        return true;
    case Int8:
    case Int16:
    case Int32:
    case Int64:
        return true;
    case Pointer:
        return false;
    case Float:
    case Double:
        return false;
    case Undefined:
        throw;

    }
    throw;
}


Dimension::DataType Dimension::getDataTypeFromString(const std::string& s)
{
    if (s == "Int8") return Int8;
    if (s == "Uint8") return Uint8;
    if (s == "Int16") return Int16;
    if (s == "Uint16") return Uint16;
    if (s == "Int32") return Int32;
    if (s == "Uint32") return Uint32;
    if (s == "Int64") return Int64;
    if (s == "Uint64") return Uint64;
    if (s == "Pointer") return Pointer;
    if (s == "Float") return Float;
    if (s == "Double") return Double;
    throw;
}


std::string const& Dimension::getName() const
{
    return m_name;
}





boost::property_tree::ptree Dimension::toPTree() const
{
    using boost::property_tree::ptree;
    ptree dim;
    dim.put("name", getName());
    dim.put("datatype", getDataTypeName(getDataType()));
    dim.put("description", getDescription());
    dim.put("bytesize", getByteSize());
    
    std::string e("little");
    if (getEndianness() == Endian_Big) 
        e = std::string("big");
    dim.put("endianness", e);


    if (! (Utils::compare_distance(getMinimum(), getMaximum()) && 
           Utils::compare_distance(0.0, getMaximum())))
    {
        dim.put("minimum", getMinimum());
        dim.put("maximum", getMaximum());
    }
    if (! (Utils::compare_distance(getNumericScale(), 0.0)))
    {
        dim.put("scale", getNumericScale());
    }
    if (! (Utils::compare_distance(getNumericOffset(), 0.0)))
    {
        dim.put("offset", getNumericOffset());
    }
    
    dim.put("scale", getNumericScale());

    dim.put("isValid", isValid());

    return dim;
}


void Dimension::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& os, pdal::Dimension const& d)
{
    using boost::property_tree::ptree;
    ptree tree = d.toPTree();

    std::string const name = tree.get<std::string>("name");

    std::ostringstream quoted_name;
    quoted_name << "'" << name << "'";
    std::ostringstream pad;
    std::string const& cur = quoted_name.str();
    std::string::size_type size = cur.size();
    std::string::size_type pad_size = 24 - size;

    for (std::string::size_type i=0; i != pad_size; i++ )
    {
        pad << " ";
    }
    os << quoted_name.str() << pad.str() <<" -- "<< " size: " << tree.get<boost::uint32_t>("bytesize");

    try {
        double value = tree.get<double>("scale");
        boost::uint32_t precision = Utils::getStreamPrecision(value);
        os.setf(std::ios_base::fixed, std::ios_base::floatfield);
        os.precision(precision);
        os << " scale: " << value;
    }
    catch (boost::property_tree::ptree_bad_path const& ) {
    }    

    try {
        double value = tree.get<double>("offset");
        boost::uint32_t precision = Utils::getStreamPrecision(value);
        os.setf(std::ios_base::fixed, std::ios_base::floatfield);
        os.precision(precision);
        os << " offset: " << value;
    }
    catch (boost::property_tree::ptree_bad_path const& ) {
    }    
    
    //os << " offset: " << tree.get<boost::uint32_t>("byteoffset");
    os << std::endl;

    return os;
}


} // namespace pdal
