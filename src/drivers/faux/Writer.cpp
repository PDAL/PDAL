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

#include <pdal/drivers/faux/Writer.hpp>

#include <pdal/PointBuffer.hpp>


namespace pdal { namespace drivers { namespace faux {


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();
}


const Options Writer::getDefaultOptions() const
{
    Options options;
    return options;
}



void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    m_minimumX = m_minimumY = m_minimumZ = std::numeric_limits<double>::max();
    m_maximumX = m_maximumY = m_maximumZ = std::numeric_limits<double>::min();
    m_averageX = m_averageY = m_averageZ = 0;

    return;
}


void Writer::writeEnd(boost::uint64_t actualNumPointsWritten)
{
    m_averageX /= (double)actualNumPointsWritten;
    m_averageY /= (double)actualNumPointsWritten;
    m_averageZ /= (double)actualNumPointsWritten;

    //cout << "FauxWriter::writeEnd()" << endl;
    //cout << "  wrote " << actualNumPointsWritten << " points" << endl;

    //cout << "  min X: " << m_minimumX << endl;
    //cout << "  min Y: " << m_minimumY << endl;
    //cout << "  min Z: " << m_minimumZ << endl;
    //cout << "  max X: " << m_maximumX << endl;
    //cout << "  max Y: " << m_maximumY << endl;
    //cout << "  max Z: " << m_maximumZ << endl;
    //
    //cout << endl;

    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& PointBuffer)
{
    const boost::uint32_t numPoints = PointBuffer.getNumPoints();

    const Schema& schema = PointBuffer.getSchema();

    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {

        double x = PointBuffer.getField<double>(dimX, pointIndex);
        double y = PointBuffer.getField<double>(dimY, pointIndex);
        double z = PointBuffer.getField<double>(dimZ, pointIndex);

        m_minimumX = std::min(m_minimumX, x);
        m_minimumY = std::min(m_minimumY, y);
        m_minimumZ = std::min(m_minimumZ, z);
        m_maximumX = std::max(m_maximumX, x);
        m_maximumY = std::max(m_maximumY, y);
        m_maximumZ = std::max(m_maximumZ, z);

        m_averageX += x;
        m_averageY += y;
        m_averageZ += z;
    }

    return numPoints;
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

} } } // namespaces
