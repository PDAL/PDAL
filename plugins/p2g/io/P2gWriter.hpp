/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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
#include <pdal/StageFactory.hpp>

#include <memory>

#include <boost/tuple/tuple.hpp>

#include <points2grid/config.h>
#include <points2grid/Interpolation.hpp>
#include <points2grid/Global.hpp>
#include <points2grid/OutCoreInterp.hpp>

namespace pdal
{


class p2g_error : public pdal_error
{
public:
    p2g_error(std::string const& msg)
        : pdal_error(msg)
    {}
};



class CoreInterp;

class PDAL_DLL P2gWriter : public Writer
{
public:
    P2gWriter() : m_outputTypes(0), m_outputFormat(OUTPUT_FORMAT_ARC_ASCII)
        {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

private:
    P2gWriter& operator=(const P2gWriter&) = delete;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);

    std::unique_ptr<OutCoreInterp> m_interpolator;

    uint32_t m_GRID_SIZE_X;
    uint32_t m_GRID_SIZE_Y;

    double m_GRID_DIST_X;
    double m_GRID_DIST_Y;

    double m_RADIUS;
    StringList m_outputTypeSpec;
    std::string m_outputFormatSpec;
    unsigned int m_outputTypes;
    uint32_t m_fill_window_size;
    BOX3D m_bounds;

    std::string m_filename;
    int m_outputFormat;

    typedef struct
    {
        double x;
        double y;
        double z;
    } Coordinate;

    std::vector<Coordinate> m_coordinates;
};

} // namespaces
