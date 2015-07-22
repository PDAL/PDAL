/******************************************************************************
 * Copyright (c) 2014, Hobu Inc.
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

#pragma once

#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <pdal/pdal_internal.hpp>

//NOTE: How to add a predefined dimension.
//
// A dimension is easily added to PDAL by doing the following:
// 1) Add an entry to the enumeration pdal::Dimension::Id::Enum with an
//   appropriate name.
// 2) Add an appropriate entry to the switch statement in the function
//   pdal::Dimension::name() to supply the string name of the dimension.
// 3) Add an appropriate entry to the switch statement in the function
//   pdal::Dimension::id() to return the dimension ID given a matching
//   name.  Make sure that names don't map to more than one dimension.
// 4) Add an appropriate entry to the switch statement in the function
//   pdal::Dimension::description() to return a description of the dimension.
// 5) Add an appropriate entry to the switch statement in the function
//   pdal::Dimension::defaultType() to return a type sufficiently
//   large to hold values of the dimension for all relevant formats.

//This should be generated from another format - JSON?
namespace pdal
{
namespace Dimension
{

namespace BaseType
{
enum Enum
{
    Signed = 0x100,
    Unsigned = 0x200,
    Floating = 0x400
};
}

inline BaseType::Enum fromName(std::string name)
{
    if (name == "signed")
        return BaseType::Signed;
    else if (name == "unsigned")
        return BaseType::Unsigned;
    else if (name == "floating")
        return BaseType::Floating;
    else
        throw pdal_error("Invalid BaseType name");
}

inline std::string toName(BaseType::Enum b)
{
    switch (b)
    {
    case BaseType::Signed:
        return "signed";
    case BaseType::Unsigned:
        return "unsigned";
    case BaseType::Floating:
        return "floating";
    default:
        return "";
    }
}

namespace Type
{
enum Enum
{
    None = 0,
    Unsigned8 = BaseType::Unsigned | 1,
    Signed8 = BaseType::Signed | 1,
    Unsigned16 = BaseType::Unsigned | 2,
    Signed16 = BaseType::Signed | 2,
    Unsigned32 = BaseType::Unsigned | 4,
    Signed32 = BaseType::Signed | 4,
    Unsigned64 = BaseType::Unsigned | 8,
    Signed64 = BaseType::Signed | 8,
    Float = BaseType::Floating | 4,
    Double = BaseType::Floating | 8
};
}

inline size_t size(Type::Enum t)
{
    return t & 0xFF;
}

inline BaseType::Enum base(Type::Enum t)
{
    return BaseType::Enum(t & 0xFF00);
}

namespace Id
{
enum Enum
{
    Unknown,
    X,
    Y,
    Z,
    Intensity,
    Amplitude,
    Reflectance,
    ReturnNumber,
    NumberOfReturns,
    ScanDirectionFlag,
    EdgeOfFlightLine,
    Classification,
    ScanAngleRank,
    UserData,
    PointSourceId,
    Red,
    Green,
    Blue,
    GpsTime,
    InternalTime,
    OffsetTime,
    IsPpsLocked,
    StartPulse,
    ReflectedPulse,
    Pdop,
    Pitch,
    Roll,
    PulseWidth,
    Deviation,
    PassiveSignal,
    BackgroundRadiation,
    PassiveX,
    PassiveY,
    PassiveZ,
    XVelocity,
    YVelocity,
    ZVelocity,
    PlatformHeading,
    WanderAngle,
    XBodyAccel,
    YBodyAccel,
    ZBodyAccel,
    XBodyAngRate,
    YBodyAngRate,
    ZBodyAngRate,
    Flag,
    Mark,
    Alpha,
    EchoRange,
    ScanChannel,
    Infrared,
    HeightAboveGround
};
} // namespace Id
typedef std::vector<Id::Enum> IdList;

static const int COUNT = std::numeric_limits<uint16_t>::max();
static const int PROPRIETARY = 0xFF00;

/// Get a description of a predefined dimension.
/// \param[in] id  Dimension ID.
/// \return  Dimension description.
inline std::string description(Id::Enum id)
{
    switch (id)
    {
    case Id::X:
        return "X coordinate";
    case Id::Y:
        return "Y coordinate";
    case Id::Z:
        return "Z coordinate";
    case Id::Intensity:
        return "Representation of the pulse return magnitude";
    case Id::Amplitude:
        return "This is the ratio of the received power to the "
            "power received at the detection limit expressed in dB";
    case Id::Reflectance:
        return "This is the ratio of the received power to the "
            "power that would be received from a white diffuse target "
            "at the same distance expressed in dB. The reflectance "
            "represents a range independent property of the target. "
            "The surface normal of this target is assumed to be in "
            "parallel to the laser beam direction.";
    case Id::ReturnNumber:
        return "Pulse return number for a given output pulse. A given output "
            "laser pulse can have many returns, and they must be marked in "
            "order, starting with 1";
    case Id::NumberOfReturns:
        return "Total number of returns for a given pulse.";
    case Id::ScanDirectionFlag:
        return "Direction at which the scanner mirror was traveling at the "
            "time of the output pulse. A value of 1 is a positive scan "
            "direction, and a bit value of 0 is a negative scan direction, "
            "where positive scan direction is a scan moving from the left "
            "side of the in-track direction to the right side and negative "
            "the opposite";
    case Id::EdgeOfFlightLine:
        return "Indicates the end of scanline before a direction change "
            "with a value of 1 - 0 otherwise";
    case Id::Classification:
        return "ASPRS classification.  0 for no classification.  See "
            "LAS specification for details";
    case Id::ScanAngleRank:
        return "Angle degree at which the laster point was output from "
            "the system, including the roll of the aircraft.  The scan "
            "angle is based on being nadir, and -90 the left side of the "
            "aircraft in the direction of flight";
    case Id::UserData:
        return "Unspecified user data";
    case Id::PointSourceId:
        return "File source ID from which the point originated.  Zero "
            "indicates that the point originated in the current file";
    case Id::GpsTime:
        return "GPS time that the point was acquired";
    case Id::InternalTime:
        return "Scanner's internal time when the point was aquired, in seconds";
    case Id::OffsetTime:
        return "Milliseconds from first acquired point";
    case Id::IsPpsLocked:
        return "The external PPS signal was found to be synchronized at the "
            "time of the current laser shot.";
    case Id::Red:
        return "Red image channel value";
    case Id::Green:
        return "Green image channel value";
    case Id::Blue:
        return "Blue image channel value";
    case Id::Alpha:
        return "Alpha image channel value";
    case Id::StartPulse:
        return "Relative pulse signal strength";
    case Id::ReflectedPulse:
        return "Relative reflected pulse signal strength";
    case Id::Pitch:
        return "Pitch in degrees";
    case Id::Roll:
        return "Roll in degrees";
    case Id::Pdop:
        return "GPS PDOP (dilution of precision)";
    case Id::PulseWidth:
        return "Laser received pulse width (digitizer samples)";
    case Id::Deviation:
        return "A larger value for deviation indicates larger distortion.";
    case Id::PassiveSignal:
        return "Relative passive signal";
    case Id::BackgroundRadiation:
        return "A measure of background radiation.";
    case Id::PassiveX:
        return "Passive X footprint";
    case Id::PassiveY:
        return "Passive Y footprint";
    case Id::PassiveZ:
        return "Passive Z footprint";
    case Id::XVelocity:
        return "X Velocity";
    case Id::YVelocity:
        return "Y Velocity";
    case Id::ZVelocity:
        return "Z Velocity";
    case Id::PlatformHeading:
        return "Platform Heading";
    case Id::WanderAngle:
        return "Wander Angle";
    case Id::XBodyAccel:
        return "X Body Acceleration";
    case Id::YBodyAccel:
        return "Y Body Acceleration";
    case Id::ZBodyAccel:
        return "Z Body Acceleration";
    case Id::XBodyAngRate:
        return "X Body Angle Rate";
    case Id::YBodyAngRate:
        return "Y Body Angle Rate";
    case Id::ZBodyAngRate:
        return "Z Body Angle Rate";
    case Id::Mark:
        return "Mark";
    case Id::Flag:
        return "Flag";
    case Id::EchoRange:
        return "The distance from the laser origin to the target.";
    case Id::ScanChannel:
        return "Scan Channel";
    case Id::Infrared:
        return "Near Infrared";
    case Id::HeightAboveGround:
        return "Height above ground";
    case Id::Unknown:
        return "";
    }
    return "";
}

/// Get a predefined dimension ID given a dimension name.  Multiple names
/// may map to the same dimension for convenience.  Names are case-insensitive.
/// \param[in] s  Name of dimension.
/// \return  Dimension ID associated with the name.  Id::Unknown is returned
///    if the name doesn't map to a predefined dimension.
inline Id::Enum id(std::string s)
{
    boost::to_upper(s);
    if (s == "X")
        return Id::X;
    else if (s == "Y")
        return Id::Y;
    else if (s == "Z")
        return Id::Z;
    else if (s == "INTENSITY")
        return Id::Intensity;
    else if (s == "AMPLITUDE")
        return Id::Amplitude;
    else if (s == "REFLECTANCE")
        return Id::Reflectance;
    else if (s == "RETURNNUMBER")
        return Id::ReturnNumber;
    else if (s == "NUMBEROFRETURNS")
        return Id::NumberOfReturns;
    else if (s == "SCANDIRECTIONFLAG")
        return Id::ScanDirectionFlag;
    else if (s == "EDGEOFFLIGHTLINE")
        return Id::EdgeOfFlightLine;
    else if (s == "CLASSIFICATION")
        return Id::Classification;
    else if (s == "SCANANGLERANK" || s == "SCANANGLE")
        return Id::ScanAngleRank;
    else if (s == "USERDATA")
        return Id::UserData;
    else if (s == "POINTSOURCEID")
        return Id::PointSourceId;
    else if (s == "RED")
        return Id::Red;
    else if (s == "GREEN")
        return Id::Green;
    else if (s == "BLUE")
        return Id::Blue;
    else if (s == "ALPHA")
        return Id::Alpha;
    else if (s == "GPSTIME")
        return Id::GpsTime;
    else if (s == "INTERNALTIME")
        return Id::InternalTime;
    else if (s == "TIME" || s == "OFFSETTIME")
        return Id::OffsetTime;
    else if (s == "ISPPSLOCKED")
        return Id::IsPpsLocked;
    else if (s == "STARTPULSE")
        return Id::StartPulse;
    else if (s == "RELFECTEDPULSE")
        return Id::ReflectedPulse;
    else if (s == "PITCH")
        return Id::Pitch;
    else if (s == "ROLL")
        return Id::Roll;
    else if (s == "PDOP")
        return Id::Pdop;
    else if (s == "PULSEWIDTH")
        return Id::PulseWidth;
    else if (s == "DEVIATION")
        return Id::Deviation;
    else if (s == "PASSIVESIGNAL")
        return Id::PassiveSignal;
    else if (s == "BACKGROUNDRADIATION")
        return Id::BackgroundRadiation;
    else if (s == "PASSIVEX")
        return Id::PassiveX;
    else if (s == "PASSIVEY")
        return Id::PassiveY;
    else if (s == "PASSIVEZ")
        return Id::PassiveZ;
    else if (s == "XVELOCITY")
        return Id::XVelocity;
    else if (s == "YVELOCITY")
        return Id::YVelocity;
    else if (s == "ZVELOCITY")
        return Id::ZVelocity;
    else if (s == "PLATFORMHEADING")
        return Id::PlatformHeading;
    else if (s == "WANDERANGLE")
        return Id::WanderAngle;
    else if (s == "XBODYACCEL")
        return Id::XBodyAccel;
    else if (s == "YBODYACCEL")
        return Id::YBodyAccel;
    else if (s == "ZBODYACCEL")
        return Id::ZBodyAccel;
    else if (s == "XBODYANGRATE")
        return Id::XBodyAngRate;
    else if (s == "YBODYANGRATE")
        return Id::YBodyAngRate;
    else if (s == "ZBODYANGRATE")
        return Id::ZBodyAngRate;
    else if (s == "MARK")
        return Id::Mark;
    else if (s == "FLAG")
        return Id::Flag;
    else if (s == "ECHORANGE")
        return Id::EchoRange;
    else if (s == "SCANCHANNEL")
        return Id::ScanChannel;
    else if (s == "INFRARED" || s == "NEARINFRARED")
        return Id::Infrared;
    else if (s == "HEIGHTABOVEGROUND")
        return Id::HeightAboveGround;
    return Id::Unknown;
}

/// Get the name of a predefined dimension.
/// \param[in] id  Dimension ID
/// \return  Dimension name.
inline std::string name(Id::Enum id)
{
    switch (id)
    {
    case Id::X:
        return "X";
    case Id::Y:
        return "Y";
    case Id::Z:
        return "Z";
    case Id::Intensity:
        return "Intensity";
    case Id::Amplitude:
        return "Amplitude";
    case Id::Reflectance:
        return "Reflectance";
    case Id::ReturnNumber:
        return "ReturnNumber";
    case Id::NumberOfReturns:
        return "NumberOfReturns";
    case Id::ScanDirectionFlag:
        return "ScanDirectionFlag";
    case Id::EdgeOfFlightLine:
        return "EdgeOfFlightLine";
    case Id::Classification:
        return "Classification";
    case Id::ScanAngleRank:
        return "ScanAngleRank";
    case Id::UserData:
        return "UserData";
    case Id::PointSourceId:
        return "PointSourceId";
    case Id::Red:
        return "Red";
    case Id::Green:
        return "Green";
    case Id::Blue:
        return "Blue";
    case Id::Alpha:
        return "Alpha";
    case Id::GpsTime:
        return "GpsTime";
    case Id::InternalTime:
        return "InternalTime";
    case Id::OffsetTime:
        return "OffsetTime";
    case Id::IsPpsLocked:
        return "IsPpsLocked";
    case Id::StartPulse:
        return "StartPulse";
    case Id::ReflectedPulse:
        return "ReflectedPulse";
    case Id::Pitch:
        return "Pitch";
    case Id::Roll:
        return "Roll";
    case Id::Pdop:
        return "Pdop";
    case Id::PulseWidth:
        return "PulseWidth";
    case Id::Deviation:
        return "Deviation";
    case Id::PassiveSignal:
        return "PassiveSignal";
    case Id::BackgroundRadiation:
        return "BackgroundRadiation";
    case Id::PassiveX:
        return "PassiveX";
    case Id::PassiveY:
        return "PassiveY";
    case Id::PassiveZ:
        return "PassiveZ";
    case Id::XVelocity:
        return "XVelocity";
    case Id::YVelocity:
        return "YVelocity";
    case Id::ZVelocity:
        return "ZVelocity";
    case Id::PlatformHeading:
        return "PlatformHeading";
    case Id::WanderAngle:
        return "WanderAngle";
    case Id::XBodyAccel:
        return "XBodyAccel";
    case Id::YBodyAccel:
        return "YBodyAccel";
    case Id::ZBodyAccel:
        return "ZBodyAccel";
    case Id::XBodyAngRate:
        return "XBodyAngRate";
    case Id::YBodyAngRate:
        return "YBodyAngRate";
    case Id::ZBodyAngRate:
        return "ZBodyAngRate";
    case Id::Mark:
        return "Mark";
    case Id::Flag:
        return "Flag";
    case Id::EchoRange:
        return "EchoRange";
    case Id::ScanChannel:
        return "ScanChannel";
    case Id::Infrared:
        return "Infrared";
    case Id::HeightAboveGround:
        return "HeightAboveGround";
    case Id::Unknown:
        return "";
    }
    return "";
}


/// Get the default storage type of a predefined dimension.
/// \param[in] id  ID of the predefined dimension.
/// \return  The dimension's default storage type.  An exception is thrown if
///   the id doesn't represent a predefined dimension.
inline Type::Enum defaultType(Id::Enum id)
{
    using namespace Type;

    switch (id)
    {
    case Id::X:
        return Double;
    case Id::Y:
        return Double;
    case Id::Z:
        return Double;
    case Id::Intensity:
        return Unsigned16;
    case Id::Amplitude:
        return Float;
    case Id::Reflectance:
        return Float;
    case Id::ReturnNumber:
        return Unsigned8;
    case Id::NumberOfReturns:
        return Unsigned8;
    case Id::ScanDirectionFlag:
        return Unsigned8;
    case Id::EdgeOfFlightLine:
        return Unsigned8;
    case Id::Classification:
        return Unsigned8;
    case Id::ScanAngleRank:
        return Float;
    case Id::UserData:
        return Unsigned8;
    case Id::PointSourceId:
        return Unsigned16;
    case Id::GpsTime:
        return Double;
    case Id::InternalTime:
        return Double;
    case Id::OffsetTime:
        return Unsigned32;
    case Id::IsPpsLocked:
        return Unsigned8;
    case Id::Red:
        return Unsigned16;
    case Id::Green:
        return Unsigned16;
    case Id::Blue:
        return Unsigned16;
    case Id::Alpha:
        return Unsigned16;
    case Id::StartPulse:
        return Signed32;
    case Id::ReflectedPulse:
        return Signed32;
    case Id::Pitch:
        return Float;
    case Id::Roll:
        return Float;
    case Id::Pdop:
        return Float;
    case Id::PulseWidth:
        return Float;
    case Id::Deviation:
        return Float;
    case Id::PassiveSignal:
        return Signed32;
    case Id::BackgroundRadiation:
        return Float;
    case Id::PassiveX:
        return Double;
    case Id::PassiveY:
        return Double;
    case Id::PassiveZ:
        return Double;
    case Id::XVelocity:
        return Double;
    case Id::YVelocity:
        return Double;
    case Id::ZVelocity:
        return Double;
    case Id::PlatformHeading:
        return Double;
    case Id::WanderAngle:
        return Double;
    case Id::XBodyAccel:
        return Double;
    case Id::YBodyAccel:
        return Double;
    case Id::ZBodyAccel:
        return Double;
    case Id::XBodyAngRate:
        return Double;
    case Id::YBodyAngRate:
        return Double;
    case Id::ZBodyAngRate:
        return Double;
    case Id::Mark:
        return Unsigned8;
    case Id::Flag:
        return Unsigned8;
    case Id::EchoRange:
        return Double;
    case Id::ScanChannel:
        return Unsigned8;
    case Id::Infrared:
        return Unsigned16;
    case Id::HeightAboveGround:
        return Double;
    case Id::Unknown:
        throw pdal_error("No type for undefined dimension ID.");
    }
    throw pdal_error("No type for undefined dimension ID.");
}

/// Get a string reresentation of a datatype.
/// \param[in] dimtype  Dimension type.
/// \return  String representation of dimension type.
inline std::string interpretationName(Type::Enum dimtype)
{
    switch (dimtype)
    {
    case Type::None:
        return "unknown";
    case Type::Signed8:
        return "int8_t";
    case Type::Signed16:
        return "int16_t";
    case Type::Signed32:
        return "int32_t";
    case Type::Signed64:
        return "int64_t";
    case Type::Unsigned8:
        return "uint8_t";
    case Type::Unsigned16:
        return "uint16_t";
    case Type::Unsigned32:
        return "uint32_t";
    case Type::Unsigned64:
        return "uint64_t";
    case Type::Float:
        return "float";
    case Type::Double:
        return "double";
    }
    return "unknown";
}


/// Get the type corresponding to a type name.
/// \param[in] s  Name of type.
/// \return  Corresponding type enumeration value.
inline Type::Enum type(std::string s)
{
    boost::to_lower(s);

    if (s == "int8_t" || s == "int8")
       return Type::Signed8;
    if (s == "int16_t" || s == "int16")
       return Type::Signed16;
    if (s == "int32_t" || s == "int32")
       return Type::Signed32;
    if (s == "int64_t" || s == "int64")
       return Type::Signed64;
    if (s == "uint8_t" || s == "uint8")
        return Type::Unsigned8;
    if (s == "uint16_t" || s == "uint16")
        return Type::Unsigned16;
    if (s == "uint32_t" || s == "uint32")
        return Type::Unsigned32;
    if (s == "uint64_t" || s == "uint64")
        return Type::Unsigned64;
    if (s == "float")
        return Type::Float;
    if (s == "double")
        return Type::Double;
    return Type::None;
}

class Detail
{
public:
    Detail() : m_id(Id::Unknown), m_offset(-1), m_type(Type::None)
    {}
    //NOTE - This is strange, but for some reason things run faster with
    // this NOOP virtual dtor.  Perhaps it has something to do with
    // an inlining optimization or perhaps alignment (though a void * doesn't
    // cause the same performance improvement) It may help on no machine
    // except mine, but it doesn't hurt anything, either.
    virtual ~Detail()
    {}

    void setOffset(int offset)
        { m_offset = offset; }
    void setType(Type::Enum type)
        { m_type = type; }
    void setId(Id::Enum id)
        { m_id = id; }
    Id::Enum id() const
        { return m_id; }
    int offset() const
        { return m_offset; }
    Type::Enum type() const
        { return m_type; }
    size_t size() const
        { return Dimension::size(m_type); }
    BaseType::Enum base() const
        { return Dimension::base(m_type); }

private:
    Id::Enum m_id; 
    int m_offset;
    Type::Enum m_type;
};
typedef std::vector<Detail> DetailList;

} // namespace Dimension

struct DimType
{
    DimType() : m_id(Dimension::Id::Unknown), m_type(Dimension::Type::None)
    {}
    DimType(Dimension::Id::Enum id, Dimension::Type::Enum type,
        double scale = 1.0, double offset = 0.0) :
        m_id(id), m_type(type), m_xform(scale, offset)
    {}
    DimType(Dimension::Id::Enum id, Dimension::Type::Enum type, XForm xform) :
        m_id(id), m_type(type), m_xform(xform)
    {}

    Dimension::Id::Enum m_id;
    Dimension::Type::Enum m_type;
    XForm m_xform;  // A convenience for some formats.
};
typedef std::vector<DimType> DimTypeList;

} // namespace pdal

