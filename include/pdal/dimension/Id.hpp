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

#ifndef PDAL_DIMENSIONID_HPP_INCLUDED
#define PDAL_DIMENSIONID_HPP_INCLUDED

#include <pdal/pdal_internal.hpp>

namespace pdal
{

class Dimension;

class PDAL_DLL DimensionId
{
public:

    enum Id
    {
    //
    // common field types: 0..999
    // 
    X_i32 = 0,
    Y_i32,
    Z_i32,
    X_f64,
    Y_f64,
    Z_f64,

    Red_u8,
    Green_u8,
    Blue_u8,
    Red_u16,
    Green_u16,
    Blue_u16,

    Time_u64,

    //
    // LAS: 1000..1999
    //
    Las_Intensity = 1000,
    Las_ReturnNumber,
    Las_NumberOfReturns,
    Las_ScanDirectionFlag,
    Las_EdgeOfFlightLine,
    Las_Classification,
    Las_ScanAngleRank,
    Las_UserData,
    Las_PointSourceId,
    Las_WavePacketDescriptorIndex,
    Las_WaveformDataOffset,
    Las_ReturnPointWaveformLocation,
    Las_WaveformXt,
    Las_WaveformYt,
    Las_WaveformZt,
    Las_Time,

    //
    // terrasolid: 2000..2999
    // 
    TerraSolid_Alpha = 2000,
    TerraSolid_Classification,
    TerraSolid_PointSourceId_u8,
    TerraSolid_PointSourceId_u16,
    TerraSolid_ReturnNumber_u8,
    TerraSolid_ReturnNumber_u16,
    TerraSolid_Flag,
    TerraSolid_Mark,
    TerraSolid_Intensity,
    TerraSolid_Time,

    //
    // chipper stuff: 3000..3999
    // 
    Chipper_1 = 3000,
    Chipper_2,

    //
    // qfit: 4000..4999
    // 
    Qfit_StartPulse = 4000,
    Qfit_ReflectedPulse,
    Qfit_ScanAngleRank,
    Qfit_Pitch,
    Qfit_Roll,
    Qfit_Time,
    Qfit_PassiveSignal,
    Qfit_PassiveX,
    Qfit_PassiveY,
    Qfit_PassiveZ,
    Qfit_GpsTime,
    Qfit_PDOP,
    Qfit_PulseWidth,

    // user fields are 100,000..199,999

    Undefined = 200000
    };

public:
    static Id getIdFromName(std::string const& name);
    static void lookupKnownDimension(const Id& id, boost::uint32_t/*Dimension::DataType*/& datatype, std::string& name, std::string& description);
    static bool hasKnownDimension(const Id& id);
    static Id getIdForDimension(Dimension const& d);

};


} // namespace pdal

#endif // PDAL_DIMENSIONID_HPP_INCLUDED
