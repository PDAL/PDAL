/******************************************************************************
* Copyright (c) 2015-2016, Bradley J Chambers, brad.chambers@gmail.com
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

#pragma once

#include <pdal/Writer.hpp>
#include <pdal/plugin.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

extern "C" int32_t DerivativeWriter_ExitFunc();
extern "C" PF_ExitFunc DerivativeWriter_InitPlugin();

namespace pdal
{

class BOX2D;

class PDAL_DLL DerivativeWriter : public Writer
{
    enum PrimitiveType
    {
        SLOPE_D8,
        SLOPE_FD,
        ASPECT_D8,
        ASPECT_FD,
        HILLSHADE,
        CONTOUR_CURVATURE,
        PROFILE_CURVATURE,
        TANGENTIAL_CURVATURE,
        TOTAL_CURVATURE
    };

    struct TypeOutput
    {
        PrimitiveType m_type;
        std::string m_filename;
    };

public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    DerivativeWriter()
    {}

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void write(const PointViewPtr view);

    std::string generateFilename(const std::string& primName,
                                 std::string::size_type hashPos) const;

    std::string m_filename;
    std::string m_driver;
    std::string m_slopeUnit;
    double m_edgeLength;
    double m_illumAltDeg;
    double m_illumAzDeg;
    StringList m_primTypesSpec;
    std::vector<TypeOutput> m_primitiveTypes;

    const double c_PI = std::acos(-1.0);
    const double c_rad2deg = 180.0 / c_PI;
    double percentSlopeToDegrees(double slope)
    {
        return std::atan(slope / 100.0) * c_rad2deg;
    }

    DerivativeWriter& operator=(const DerivativeWriter&); // not implemented
    DerivativeWriter(const DerivativeWriter&); // not implemented
};

} // namespace pdal
