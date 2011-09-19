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
#if 0
Dimension::Id Dimension_X_i32 =                           { 0xab, 0xca, 0x4c, 0x06, 0xef, 0x6c, 0x4c, 0x38, 0xbb, 0x30, 0x0c, 0x8c, 0x43, 0x58, 0x6f, 0xcc };
Dimension::Id Dimension_Y_i32 =                           { 0x6E, 0xE5, 0xCA, 0xCB, 0x35, 0x68, 0x4F, 0x3F, 0x80, 0xAA, 0xE9, 0x96, 0x5B, 0x7E, 0x4A, 0x23 };
Dimension::Id Dimension_Z_i32 =                           { 0x93, 0xE1, 0x61, 0x79, 0xC0, 0xC5, 0x43, 0x7E, 0xB9, 0x06, 0x60, 0x01, 0xC4, 0x96, 0xA3, 0x35 };
Dimension::Id Dimension_X_f64 =                           { 0x14, 0x96, 0x0E, 0x7A, 0x11, 0xBD, 0x49, 0xD9, 0xB2, 0x4B, 0xE0, 0xBB, 0xB7, 0x59, 0xCE, 0x32 };
Dimension::Id Dimension_Y_f64 =                           { 0xAD, 0xF4, 0xD3, 0x4F, 0x6B, 0x34, 0x47, 0x64, 0x9F, 0x35, 0xFC, 0x42, 0x28, 0x2D, 0x5A, 0x76 };
Dimension::Id Dimension_Z_f64 =                           { 0x2D, 0xB0, 0x00, 0x14, 0x05, 0xFE, 0x4C, 0x0A, 0x83, 0xAC, 0xE5, 0xB9, 0x87, 0xC6, 0x0F, 0x6D };

Dimension::Id Dimension_Red_u8 =                          { 0xD9, 0x2F, 0x5F, 0x32, 0xB7, 0x0E, 0x4E, 0x98, 0xB3, 0x97, 0xB6, 0xF0, 0xAA, 0x87, 0x73, 0x2A };
Dimension::Id Dimension_Green_u8 =                        { 0xD7, 0x3D, 0xDA, 0x07, 0x83, 0x43, 0x44, 0x5C, 0xA1, 0x4F, 0xB0, 0x30, 0xB7, 0xB5, 0x00, 0x4E };
Dimension::Id Dimension_Blue_u8 =                         { 0xBC, 0x2F, 0xEC, 0x3D, 0x69, 0xA7, 0x43, 0x10, 0xB5, 0xAF, 0xE7, 0xFF, 0xBF, 0xD4, 0x8C, 0x8F };
Dimension::Id Dimension_Red_u16 =                         { 0xBB, 0xA4, 0xCF, 0x23, 0x28, 0xA4, 0x48, 0x07, 0x91, 0xAC, 0xC6, 0xF5, 0x9A, 0xE3, 0x5D, 0xDC };
Dimension::Id Dimension_Green_u16 =                       { 0x55, 0xC7, 0x02, 0xC7, 0x14, 0xB6, 0x43, 0x94, 0x9F, 0xDB, 0x4D, 0x43, 0x24, 0x0A, 0x17, 0xE7 };
Dimension::Id Dimension_Blue_u16 ;//=                        { 0xBA, 0xC2, 0x52, 0x5E, 0x43, 0x54, 0x4B, 0x1E, 0x98, 0x31, 0xDC, 0xE1, 0xBD, 0xEC, 0x58, 0x81 };

Dimension::Id Dimension_Time_u64 =                        { 0xF0, 0xC1, 0x92, 0xFA, 0x60, 0x7D, 0x42, 0xD7, 0x96, 0xA2, 0x16, 0x0A, 0x63, 0x78, 0x6C, 0x45 };

// f64, gps
Dimension::Id Dimension_Las_Intensity =                   { 0xEF, 0x7C, 0x70, 0xC9, 0x2F, 0xCC, 0x4D, 0x8F, 0x82, 0x2C, 0x1C, 0x95, 0x73, 0xCF, 0x55, 0xEE };
Dimension::Id Dimension_Las_ReturnNumber =                { 0xB7, 0xED, 0x85, 0x45, 0x49, 0xD0, 0x47, 0x2F, 0xBE, 0xF9, 0xAF, 0x10, 0x82, 0x2A, 0x3A, 0x27 };
Dimension::Id Dimension_Las_NumberOfReturns =             { 0x50, 0x8E, 0x08, 0x60, 0x56, 0xF7, 0x4C, 0x67, 0xB5, 0xCE, 0xD1, 0x94, 0x2A, 0xCF, 0xBE, 0xC6 };
Dimension::Id Dimension_Las_ScanDirectionFlag =           { 0x53, 0x52, 0x00, 0xF6, 0xBD, 0xE1, 0x49, 0x43, 0xB6, 0x32, 0x38, 0x7A, 0x4F, 0x09, 0x55, 0xFD };
Dimension::Id Dimension_Las_EdgeOfFlightLine =            { 0x44, 0x50, 0x43, 0xD6, 0x43, 0xED, 0x44, 0x69, 0x9C, 0x53, 0x32, 0xCE, 0x65, 0x63, 0x60, 0x11 };
Dimension::Id Dimension_Las_Classification =              { 0x88, 0x38, 0xCA, 0xBA, 0xCF, 0x54, 0x43, 0xED, 0x98, 0xD5, 0xE1, 0x66, 0x5C, 0x6B, 0xBB, 0xD6 };
Dimension::Id Dimension_Las_ScanAngleRank =               { 0x79, 0x16, 0x19, 0xE1, 0x36, 0x3B, 0x40, 0x56, 0xAD, 0x4B, 0x20, 0x74, 0xD3, 0x62, 0x8C, 0x4B };
Dimension::Id Dimension_Las_UserData =                    { 0x7F, 0x50, 0xAD, 0xCF, 0x09, 0x9D, 0x42, 0xFC, 0x8D, 0x1C, 0xF3, 0xFF, 0x68, 0x70, 0x42, 0xB3 };
Dimension::Id Dimension_Las_PointSourceId =               { 0x0F, 0x8B, 0xDF, 0x1F, 0x78, 0x6E, 0x4B, 0xF9, 0xB2, 0x77, 0x47, 0x12, 0xE1, 0x22, 0x0D, 0x25 };
Dimension::Id Dimension_Las_WavePacketDescriptorIndex =   { 0xB4, 0xC9, 0x39, 0x72, 0x3C, 0x19, 0x4C, 0x70, 0x86, 0x63, 0xA6, 0xC5, 0xF5, 0xF1, 0x10, 0x87 };
Dimension::Id Dimension_Las_WaveformDataOffset =          { 0x9A, 0xDE, 0x88, 0xC0, 0xB8, 0x1E, 0x47, 0x2F, 0xAD, 0xF6, 0x3F, 0x4B, 0x85, 0xB1, 0xE0, 0xEC };
Dimension::Id Dimension_Las_ReturnPointWaveformLocation = { 0xDB, 0x1D, 0x7F, 0x9E, 0x03, 0x23, 0x4B, 0x60, 0xB8, 0x4B, 0x09, 0x8C, 0x20, 0x1B, 0x3A, 0x8A };
Dimension::Id Dimension_Las_WaveformXt =                  { 0xF4, 0x90, 0xF2, 0xAC, 0xCB, 0xAC, 0x41, 0x56, 0x93, 0x9D, 0x4E, 0xD6, 0x1C, 0x6A, 0x1C, 0x6B };
Dimension::Id Dimension_Las_WaveformYt =                  { 0x90, 0x1F, 0x76, 0x27, 0xF5, 0xC3, 0x48, 0x8D, 0x99, 0xD8, 0xCF, 0x4B, 0x3E, 0xDA, 0x42, 0x87 };
Dimension::Id Dimension_Las_WaveformZt =                  { 0x5F, 0xFC, 0x07, 0x62, 0x0B, 0xA2, 0x4A, 0x64, 0x91, 0xCF, 0x1C, 0x62, 0x85, 0x76, 0x21, 0x90 };
Dimension::Id Dimension_Las_Time =                        { 0xE3, 0x32, 0xB3, 0x93, 0xC3, 0x44, 0x49, 0x25, 0xA1, 0x45, 0x3C, 0x72, 0xA9, 0x1D, 0xC5, 0xD1 };

//
// TerraSolid
//
// 
// X: "Easting"
// Y": "Northing"
// Z: "Elevation"

//u8 The alpha image channel value associated with this point";    //u8
Dimension::Id Dimension_TerraSolid_Alpha =                { 0xEC, 0x91, 0x99, 0xEC, 0xC5, 0x4D, 0x40, 0x78, 0x84, 0x85, 0x72, 0xFC, 0x25, 0x73, 0x61, 0x8F };
//"Classification code 0-255" //u8
Dimension::Id Dimension_TerraSolid_Classification =       { 0xDB, 0x65, 0x44, 0xFC, 0x25, 0x8D, 0x46, 0x09, 0x8A, 0x59, 0x55, 0x13, 0x29, 0x3A, 0x9C, 0xCF };
//"Flightline number 0-255"
Dimension::Id Dimension_TerraSolid_PointSourceId_u8 =     { 0xB6, 0x66, 0xEF, 0x8B, 0xB6, 0x13, 0x4F, 0x61, 0xBD, 0xFB, 0x40, 0x1F, 0x4A, 0xC1, 0x40, 0xF9 };
//"Flightline number 0-255"
Dimension::Id Dimension_TerraSolid_PointSourceId_u16 =    { 0x7B, 0x56, 0x73, 0x4C, 0xC3, 0x9C, 0x49, 0xC3, 0x96, 0x1A, 0x71, 0x0A, 0x7C, 0x70, 0xE4, 0x81 };
//"Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo."
Dimension::Id Dimension_TerraSolid_ReturnNumber_u8 =      { 0x50, 0x23, 0x0B, 0x8D, 0x89, 0xAC, 0x4D, 0xEC, 0x95, 0x8A, 0x28, 0x45, 0x9C, 0x35, 0x78, 0xB9 };
Dimension::Id Dimension_TerraSolid_ReturnNumber_u16 =     { 0xEA, 0x17, 0x26, 0x34, 0x72, 0xA5, 0x43, 0x34, 0x95, 0x1E, 0x58, 0x20, 0x13, 0x64, 0x8A, 0x19 };
//"Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo."
Dimension::Id Dimension_TerraSolid_Flag =                 { 0xA2, 0x9A, 0x22, 0xD2, 0xF0, 0x6E, 0x4E, 0x07, 0x9D, 0x41, 0x6F, 0x69, 0xBC, 0xBB, 0xE8, 0xCD };
//"Runtime flag"//u8
Dimension::Id Dimension_TerraSolid_Mark =                 { 0x95, 0xBA, 0xC8, 0x1D, 0x97, 0x4D, 0x42, 0x11, 0x84, 0xAD, 0xA9, 0x10, 0x03, 0x9E, 0x18, 0xDC };
//"Intensity bits 0-13, echo bits 14-15" //u16
Dimension::Id Dimension_TerraSolid_Intensity =            { 0xB5, 0x0A, 0x89, 0xFB, 0x14, 0x2D, 0x45, 0xC2, 0xBC, 0xDE, 0x29, 0x8F, 0x78, 0x3D, 0xE0, 0xED };
//"32 bit integer time stamps. Time stamps are assumed to be GPS week seconds. The storage format is a 32 bit unsigned integer where each integer step is 0.0002 seconds.";//i32
Dimension::Id Dimension_TerraSolid_Time =                 { 0xA1, 0x56, 0x63, 0xBB, 0xB4, 0xC2, 0x41, 0x92, 0xBB, 0xE8, 0x55, 0x91, 0x35, 0xDA, 0x5A, 0x3A };

//
// Chipper
//
Dimension::Id Dimension_Chipper_1 =                       { 0xF2, 0x52, 0xF1, 0x5A, 0x2F, 0x96, 0x48, 0x68, 0xA3, 0x5C, 0x9F, 0xDC, 0x6D, 0xA7, 0x6B, 0x22 };
Dimension::Id Dimension_Chipper_2 =                       { 0x2E, 0xB2, 0x64, 0xCB, 0x9F, 0x18, 0x45, 0xB8, 0xA3, 0x4F, 0x16, 0x60, 0xD8, 0x6C, 0xDC, 0x74 };

//
// Qfit
// 

// //"Longitude coordinate with 1/1000000 decimals of precision"
////"Latitude coordinate with 1/1000000 decimals of precision"
////"z coordinate as a long integer.  You must use the scale and offset information of the header to determine the double value."
//"Start Pulse Signal Strength (relative)"
Dimension::Id Dimension_Qfit_StartPulse =                 { 0xAC, 0x04, 0x01, 0xA2, 0x45, 0xC5, 0x4F, 0x5A, 0x93, 0x70, 0x51, 0x91, 0xBD, 0x40, 0x33, 0x6A };
//"Reflected Laser Signal Strength (relative)"
Dimension::Id Dimension_Qfit_ReflectedPulse =             { 0xF6, 0x51, 0x99, 0x72, 0x8C, 0xF9, 0x40, 0xF0, 0x9A, 0x30, 0x16, 0x8A, 0x1E, 0x02, 0xB3, 0x40 };
//"Scan Azimuth (degrees X 1,000)"
Dimension::Id Dimension_Qfit_ScanAngleRank =              { 0xA2, 0xCD, 0x54, 0x4B, 0x63, 0x12, 0x46, 0x20, 0xB7, 0x19, 0x6A, 0xAF, 0x95, 0x4A, 0xAF, 0x00 };
//"Pitch (degrees X 1,000)"
Dimension::Id Dimension_Qfit_Pitch =                      { 0xAA, 0x8B, 0xE1, 0xA8, 0x34, 0xFA, 0x41, 0x8E, 0xA2, 0xEB, 0x11, 0x43, 0x16, 0xC2, 0x24, 0xC8 };
//"Roll (degrees X 1,000)"
Dimension::Id Dimension_Qfit_Roll =                       { 0xFC, 0xCE, 0x23, 0x04, 0xED, 0xED, 0x43, 0x4D, 0xA0, 0x41, 0x3B, 0xBA, 0x29, 0x75, 0xC7, 0x75 };
// "Relative Time (msec from start of data file)";// i32
Dimension::Id Dimension_Qfit_Time =                       { 0x5F, 0xFC, 0x07, 0x62, 0x0B, 0xA2, 0x4A, 0x64, 0x91, 0xCF, 0x1C, 0x62, 0x85, 0x76, 0x21, 0x90 };
//"Passive Signal (relative)" // i32
Dimension::Id Dimension_Qfit_PassiveSignal =              { 0xEA, 0x96, 0x37, 0x54, 0x6B, 0x9C, 0x4B, 0x5F, 0xB7, 0x5E, 0xD6, 0xF9, 0xCC, 0x55, 0x44, 0xD7 };
//"Passive Footprint Longitude (degrees X 1,000,000)" // i32
Dimension::Id Dimension_Qfit_PassiveX =                   { 0x01, 0x6A, 0x10, 0x07, 0xC3, 0x9D, 0x48, 0x6E, 0xAA, 0x91, 0x1A, 0x50, 0xBB, 0x62, 0xDF, 0x9D };
//"Passive Footprint Latitude (degrees X 1,000,000)"  // i32
Dimension::Id Dimension_Qfit_PassiveY =                   { 0x66, 0x6D, 0x81, 0x0B, 0xB7, 0xED, 0x41, 0xC8, 0x8E, 0xA2, 0xEA, 0x6E, 0x46, 0x3B, 0xAE, 0xCD };
//"Passive Footprint Synthesized Elevation (millimeters)" // i32
Dimension::Id Dimension_Qfit_PassiveZ =                   { 0x5D, 0xC5, 0xFB, 0x8F, 0xC4, 0x24, 0x4F, 0x2E, 0xA1, 0x93, 0xF9, 0xB1, 0x0B, 0xE8, 0x1C, 0xD7 };
//"GPS Time packed (example: 153320100 = 15h 33m 20s 100ms)" // i32
Dimension::Id Dimension_Qfit_GpsTime =                    { 0x4E, 0x87, 0x33, 0x04, 0xBB, 0x38, 0x4F, 0x26, 0x8E, 0x52, 0x68, 0x33, 0xDE, 0x59, 0xFB, 0xB9 };
//"GPS PDOP (dilution of precision) (X 10)  // i32
Dimension::Id Dimension_Qfit_PDOP =                       { 0x2A, 0x12, 0xC6, 0x49, 0xBC, 0xF0, 0x48, 0x47, 0x88, 0x83, 0xD5, 0x1E, 0xEC, 0xC5, 0x82, 0xB4 };
//"Laser received pulse width (digitizer samples) // i32
Dimension::Id Dimension_Qfit_PulseWidth =                 { 0x00, 0xD3, 0x7C, 0x5C, 0x6A, 0xAF, 0x44, 0x53, 0x97, 0xD2, 0xB6, 0x1E, 0xCE, 0xEB, 0x88, 0x05 };
#endif
class KnownDimension
{
public:
    Dimension::Id id;
    Dimension::DataType datatype;
    std::string name;
};

KnownDimension s_knownDimensions[] =
{
    // common x,y,z
    { Dimension::Id_X_i32, Dimension::Int32, "X" },
    { Dimension::Id_Y_i32, Dimension::Int32, "Y" },
    { Dimension::Id_Z_i32, Dimension::Int32, "Z" },
    { Dimension::Id_X_f64, Dimension::Double, "X" },
    { Dimension::Id_Y_f64, Dimension::Double, "Y" },
    { Dimension::Id_Z_f64, Dimension::Double, "Z" },

    { Dimension::Id_Time_u64, Dimension::Uint64, "Time" },

    { Dimension::Id_Red_u16, Dimension::Uint16, "Red" },
    { Dimension::Id_Green_u16, Dimension::Uint16, "Green" },
    { Dimension::Id_Blue_u16, Dimension::Uint16, "Blue" },
    { Dimension::Id_Red_u8, Dimension::Uint8, "Red" },
    { Dimension::Id_Green_u8, Dimension::Uint8, "Green" },
    { Dimension::Id_Blue_u8, Dimension::Uint8, "Blue" },

    // las
    { Dimension::Id_Las_Intensity, Dimension::Uint16, "Intensity" },
    { Dimension::Id_Las_ReturnNumber, Dimension::Uint8, "ReturnNumber" },
    { Dimension::Id_Las_NumberOfReturns, Dimension::Uint8, "NumberOfReturns" },
    { Dimension::Id_Las_ScanDirectionFlag, Dimension::Uint8, "ScanDirectionFlag" },
    { Dimension::Id_Las_EdgeOfFlightLine, Dimension::Uint8, "EdgeOfFlightLine" },
    { Dimension::Id_Las_Classification, Dimension::Uint8, "Classification" },
    { Dimension::Id_Las_ScanAngleRank, Dimension::Int8, "ScanAngleRank" },
    { Dimension::Id_Las_UserData, Dimension::Uint8, "UserData" },
    { Dimension::Id_Las_PointSourceId, Dimension::Uint16, "PointSourceId" },
    { Dimension::Id_Las_WavePacketDescriptorIndex, Dimension::Uint8, "WavePacketDescriptorIndex" },
    { Dimension::Id_Las_WaveformDataOffset, Dimension::Uint64, "WaveformDataOffset" },
    { Dimension::Id_Las_ReturnPointWaveformLocation, Dimension::Uint32, "ReturnPointWaveformLocation" },
    { Dimension::Id_Las_WaveformXt, Dimension::Float, "WaveformXt" },
    { Dimension::Id_Las_WaveformYt, Dimension::Float, "WaveformYt" },
    { Dimension::Id_Las_WaveformZt, Dimension::Float, "WaveformZt" },
    { Dimension::Id_Las_Time, Dimension::Double, "Time" },

    // terrasolid
    { Dimension::Id_TerraSolid_Alpha, Dimension::Uint8, "Alpha" },
    { Dimension::Id_TerraSolid_Classification, Dimension::Uint8, "Classification" },
    { Dimension::Id_TerraSolid_PointSourceId_u8, Dimension::Uint8, "PointSourceId_u8" },
    { Dimension::Id_TerraSolid_PointSourceId_u16, Dimension::Uint16, "PointSourceId_u16" },
    { Dimension::Id_TerraSolid_ReturnNumber_u8, Dimension::Uint8, "ReturnNumber" },
    { Dimension::Id_TerraSolid_ReturnNumber_u16, Dimension::Uint16, "ReturnNumber" },
    { Dimension::Id_TerraSolid_Flag, Dimension::Uint8, "Flag" },
    { Dimension::Id_TerraSolid_Mark, Dimension::Uint8, "Mark" },
    { Dimension::Id_TerraSolid_Intensity, Dimension::Uint16, "Intensity" },
    { Dimension::Id_TerraSolid_Time, Dimension::Uint32, "TerraSolid Time" },

    // chipper
    { Dimension::Id_Chipper_1, Dimension::Int32, "Chipper1" },
    { Dimension::Id_Chipper_2, Dimension::Int32, "Chipper2" },

    // qfit
    { Dimension::Id_Qfit_StartPulse, Dimension::Int32, "StartPulse" },
    { Dimension::Id_Qfit_ReflectedPulse, Dimension::Int32, "ReflectedPulse" },
    { Dimension::Id_Qfit_ScanAngleRank, Dimension::Int32, "ScanAngleRank" },
    { Dimension::Id_Qfit_Pitch, Dimension::Int32, "Pitch" },
    { Dimension::Id_Qfit_Roll, Dimension::Int32, "Roll" },
    { Dimension::Id_Qfit_Time, Dimension::Int32, "Qfit Time" },
    { Dimension::Id_Qfit_PassiveSignal, Dimension::Int32, "PassiveSignal" },
    { Dimension::Id_Qfit_PassiveX, Dimension::Int32, "PassiveX" },
    { Dimension::Id_Qfit_PassiveY, Dimension::Int32, "PassiveY" },
    { Dimension::Id_Qfit_PassiveZ, Dimension::Int32, "PassiveZ" },
    { Dimension::Id_Qfit_GpsTime, Dimension::Int32, "GpsTime" },
    { Dimension::Id_Qfit_PDOP, Dimension::Int32, "PDOP" },
    { Dimension::Id_Qfit_PulseWidth, Dimension::Int32, "PulseWdith" },

    // eof
    { Dimension::Id_Undefined, Dimension::Undefined, "" }
};


static void validate();

// BUG: this is too slow
static const KnownDimension& lookupKnownDimension(const Dimension::Id& id)
{
    validate();

    int i=0;
    while (s_knownDimensions[i].id != Dimension::Id_Undefined)
    {
        const KnownDimension& kd = s_knownDimensions[i];
        if (kd.id == id) return kd;
        ++i;
    }
    throw pdal_error("Dimension not found");
}

static bool hasKnownDimension(const Dimension::Id& id)
{
    int i=0;
    while (s_knownDimensions[i].id != Dimension::Id_Undefined)
    {
        const KnownDimension& kd = s_knownDimensions[i];
        if (kd.id == id) return true;
        ++i;
    }
    return false;
}

static void validate()
{
    std::map<Dimension::Id, int> map;

    int i=0;
    while (s_knownDimensions[i].id != Dimension::Id_Undefined)
    {
        assert(map.find(s_knownDimensions[i].id) == map.end());
        map.insert(std::pair<Dimension::Id,int>(s_knownDimensions[i].id,1) );
        ++i;
    }
}

// --------------------------------------------------------------------------

Dimension::Dimension(const Id& id)
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
    m_description = "";

    m_byteSize = getDataTypeSize(m_dataType);
}


Dimension::Dimension(const Id& id, DataType dataType, std::string name)
    : m_dataType(dataType)
    , m_id(id)
    , m_name(name)
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
