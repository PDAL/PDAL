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

#ifndef PDAL_DIMENSIONTYPE_HPP_INCLUDED
#define PDAL_DIMENSIONTYPE_HPP_INCLUDED

#include <pdal/pdal_internal.hpp>

namespace pdal
{

namespace dimension
{
    
class PDAL_DLL DimensionType
{
public:

    enum Type
    {
    //
    // common field types: 0..999
    // 
    X = 0,
    Y,
    Z,

    Red,
    Green,
    Blue,
    Alpha,
    Time,

    //
    // LAS: 1000..1999
    //
    Intensity = 1000,
    ReturnNumber,
    NumberOfReturns,
    ScanDirectionFlag,
    EdgeOfFlightLine,
    Classification,
    ScanAngleRank,
    UserData,
    PointSourceId,
    WavePacketDescriptorIndex,
    WaveformDataOffset,
    ReturnPointWaveformLocation,
    WaveformXt,
    WaveformYt,
    WaveformZt,

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
    StartPulse = 4000,
    ReflectedPulse,
    Pitch,
    Roll,
    PassiveSignal,
    PassiveX,
    PassiveY,
    PassiveZ,
    GpsTime,
    PDOP,
    PulseWidth


    };

};


}} // namespace pdal::dimension

#endif // PDAL_DIMENSIONID_HPP_INCLUDED
