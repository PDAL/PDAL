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

#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#ifdef PDAL_HAVE_P2G
#include <points2grid/config.h>
#include <points2grid/Interpolation.hpp>
#include <points2grid/Global.hpp>
#include <points2grid/OutCoreInterp.hpp>
#endif

namespace pdal
{
namespace drivers
{
namespace p2g
{


class p2g_error : public pdal_error
{
public:
    p2g_error(std::string const& msg)
        : pdal_error(msg)
    {}
};



class CoreInterp;

class PDAL_DLL P2gWriter : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.p2g.writer", "Points2Grid Writer")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.p2g.writer.html")
#ifdef PDAL_HAVE_P2G
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif
 
    P2gWriter(const Options& options) : Writer(options), m_outputTypes(0), m_outputFormat(OUTPUT_FORMAT_ARC_ASCII) {};
    ~P2gWriter() {};

    static Options getDefaultOptions();

private:
    P2gWriter& operator=(const P2gWriter&); // not implemented
    
    virtual void processOptions(const Options& options);
    virtual void ready(PointContext ctx) {};
    virtual void write(const PointBuffer& buf);
    virtual void done(PointContext ctx) {};
    virtual void initialize() {};

    boost::scoped_ptr<OutCoreInterp> m_interpolator;
    boost::uint64_t m_pointCount;

    boost::uint32_t m_GRID_SIZE_X;
    boost::uint32_t m_GRID_SIZE_Y;

    double m_GRID_DIST_X;
    double m_GRID_DIST_Y;

    double m_RADIUS_SQ;
    unsigned int m_outputTypes;
    boost::uint32_t m_fill_window_size;
    Bounds<double> m_bounds;

    std::string m_filename;
    int m_outputFormat;

    std::vector<boost::tuple<double, double, double> > m_coordinates;
};

}
}
} // namespaces

