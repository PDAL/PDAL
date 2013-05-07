/******************************************************************************
* Copyright (c) 2013, Bradley J Chambers, brad.chambers@gmail.com
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

#include <pdal/drivers/prc/oPRCFile.hpp>
#include <pdal/drivers/prc/Writer.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/pdal_macros.hpp>

//#include <iostream>
//#include <algorithm>
//#include <map>

//#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string/erase.hpp>


#ifdef USE_PDAL_PLUGIN_PRC
MAKE_WRITER_CREATOR(prcWriter, pdal::drivers::prc::Writer)
CREATE_WRITER_PLUGIN(prc, pdal::drivers::prc::Writer)
#endif


namespace pdal
{
namespace drivers
{
namespace prc
{


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_prcFile(options.getOption("filename").getValue<std::string>())
{

    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();

    grpopt.no_break = true;
    grpopt.do_break = false;
    grpopt.tess = true;

    return;
}



Options Writer::getDefaultOptions()
{
    Options options;

    Option filename("filename", "", "Filename to write PRC file to");

    options.add(filename);

    return options;
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{ 
    m_prcFile.begingroup("points",&grpopt);
    
    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    m_prcFile.endgroup();
    m_prcFile.finish();

    return;
}

boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{
    boost::uint32_t numPoints = 0;

    pdal::Schema const& schema = data.getSchema();
    
    pdal::Dimension const& dimX = schema.getDimension("X");
    pdal::Dimension const& dimY = schema.getDimension("Y");
    pdal::Dimension const& dimZ = schema.getDimension("Z");
    
    double **points;
    points = (double**) malloc(data.getNumPoints()*sizeof(double*));
    for (boost::uint32_t i = 0; i < data.getNumPoints(); ++i)
    {
        points[i] = (double*) malloc(3*sizeof(double));
    }

    double xd(0.0);
    double yd(0.0);
    double zd(0.0);

    for(boost::uint32_t i = 0; i < data.getNumPoints(); ++i)
    {
        boost::int32_t x = data.getField<boost::int32_t>(dimX, i);
        boost::int32_t y = data.getField<boost::int32_t>(dimY, i);
        boost::int32_t z = data.getField<boost::int32_t>(dimZ, i);

/*
        xd = dimX.applyScaling<boost::int32_t>(x);
        yd = dimY.applyScaling<boost::int32_t>(y);
        zd = dimZ.applyScaling<boost::int32_t>(z);
*/
        points[i][0] = x;
        points[i][1] = y;
        points[i][2] = z;
        
        numPoints++;
    }

    m_prcFile.addPoints(numPoints, const_cast<const double**>(points), RGBAColour(1.0,1.0,0.0,1.0),1.0);
    
    for (boost::uint32_t i = 0; i < data.getNumPoints(); ++i)
    {
        free(points[i]);
    }
    free(points);

    return numPoints;
}


}
}
} // namespaces
