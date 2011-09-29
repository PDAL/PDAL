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

#include <pdal/DimensionId.hpp>
#include <pdal/Utils.hpp>

namespace pdal
{


DimensionId::Id DimensionId::getIdFromName(std::string const& name)
{
    // BUG: should we be checking for the Double datatype version of X,Y,Z too?
    if (!Utils::compare_no_case(name, "X"))   
        return DimensionId::X_i32;
    
    if (!Utils::compare_no_case(name, "Y"))
        return DimensionId::Y_i32;

    if (!Utils::compare_no_case(name, "Z"))
        return DimensionId::Z_i32;

    if (!Utils::compare_no_case(name, "Intensity"))
        return DimensionId::Las_Intensity;

    if (!Utils::compare_no_case(name, "Return Number") ||
        !Utils::compare_no_case(name, "ReturnNumber"))
        return DimensionId::Las_ReturnNumber;

    if (!Utils::compare_no_case(name, "Number of Returns") ||
        !Utils::compare_no_case(name, "NumberOfReturns"))
        return DimensionId::Las_NumberOfReturns;

    if (!Utils::compare_no_case(name, "Number of Returns"))
        return DimensionId::Las_NumberOfReturns;

    if (!Utils::compare_no_case(name, "Scan Direction") ||
        !Utils::compare_no_case(name, "ScanDirectionFlag") ||
        !Utils::compare_no_case(name, "ScanDirection"))
        return DimensionId::Las_ScanDirectionFlag;

    if (!Utils::compare_no_case(name, "Flightline Edge") ||
        !Utils::compare_no_case(name, "EdgeOfFlightLine") ||
        !Utils::compare_no_case(name, "FlightlineEdge"))
        return DimensionId::Las_EdgeOfFlightLine;

    if (!Utils::compare_no_case(name, "Classification"))
        return DimensionId::Las_Classification;

    if (!Utils::compare_no_case(name, "Scan Angle Rank") ||
        !Utils::compare_no_case(name, "ScanAngle") ||
        !Utils::compare_no_case(name, "ScanAngleRank"))
        return DimensionId::Las_ScanAngleRank;

    if (!Utils::compare_no_case(name, "User Data") ||
        !Utils::compare_no_case(name, "UserData"))
        return DimensionId::Las_UserData;

    if (!Utils::compare_no_case(name, "Point Source ID")||
        !Utils::compare_no_case(name, "PointSourceId"))
        return DimensionId::Las_PointSourceId;

    if (!Utils::compare_no_case(name, "Time"))
        return DimensionId::Las_Time;

    if (!Utils::compare_no_case(name, "Red"))
        return DimensionId::Red_u16;

    if (!Utils::compare_no_case(name, "Green"))
        return DimensionId::Green_u16;

    if (!Utils::compare_no_case(name, "Blue"))
        return DimensionId::Blue_u16;

    if (!Utils::compare_no_case(name, "Alpha"))
        return DimensionId::TerraSolid_Alpha;
    
    if (!Utils::compare_no_case(name, "Chipper Point ID"))
        return DimensionId::Chipper_1;

    if (!Utils::compare_no_case(name, "Chipper Block ID"))
        return DimensionId::Chipper_2;

    // Yes, this is scary.  What else can we do?
    throw pdal_error("unknown field name: " + name);
}


} // namespace pdal
