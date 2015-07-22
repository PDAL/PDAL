/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include <pdal/util/Georeference.hpp>

namespace pdal
{


class optech_error : public pdal_error
{
public:
    optech_error(const std::string& msg)
        : pdal_error(msg)
    {
    }
};


// Optech csd files contain misalignment angles and IMU offsets.
// Misalignment angles and IMU offsets combine to form the boresight matrix.
typedef struct
{
    char signature[4];
    char vendorId[64];
    char softwareVersion[32];
    float formatVersion;
    uint16_t headerSize;
    uint16_t gpsWeek;
    double minTime; // seconds
    double maxTime; // seconds
    uint32_t numRecords;
    uint16_t numStrips;
    uint32_t stripPointers[256];
    double misalignmentAngles[3]; // radians
    double imuOffsets[3];         // radians
    double temperature;           // degrees
    double pressure;              // mbar
    char freeSpace[830];
} CsdHeader;


typedef struct
{
    double gpsTime;
    uint8_t returnCount;
    float range[4]; // metres
    uint16_t intensity[4];
    float scanAngle;  // radians
    float roll;       // radians
    float pitch;      // radians
    float heading;    // radians
    double latitude;  // radians
    double longitude; // radians
    float elevation;  // metres
} CsdPulse;


// Optech does it like R3(heading) * R1(-pitch) * R2(-roll)
inline pdal::georeference::RotationMatrix
createOptechRotationMatrix(double roll, double pitch, double heading)
{
    return georeference::RotationMatrix(
        std::cos(roll) * std::cos(heading) +
            std::sin(pitch) * std::sin(roll) * std::sin(heading), // m00
        std::cos(pitch) * std::sin(heading),                      // m01
        std::cos(heading) * std::sin(roll) -
            std::cos(roll) * std::sin(pitch) * std::sin(heading), // m02
        std::cos(heading) * std::sin(pitch) * std::sin(roll) -
            std::cos(roll) * std::sin(heading), // m10
        std::cos(pitch) * std::cos(heading),    // m11
        -std::sin(roll) * std::sin(heading) -
            std::cos(roll) * std::cos(heading) * std::sin(pitch), // m12
        -std::cos(pitch) * std::sin(roll),                        // m20
        std::sin(pitch),                                          // m21
        std::cos(pitch) * std::cos(roll)                          // m22
        );
}
}
