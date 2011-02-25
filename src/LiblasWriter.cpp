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

#include "libpc/LiblasWriter.hpp"

#include <cassert>

#include <liblas/Writer.hpp>

#include "libpc/LiblasHeader.hpp"

namespace libpc
{


LiblasWriter::LiblasWriter(Stage& prevStage, std::ostream& ostream)
    : Writer(prevStage)
    , m_ostream(ostream)
    , m_writer(NULL)
{
    liblas::Header extHeader;
    extHeader.SetCompressed(false);

    m_writer = new liblas::Writer(m_ostream, extHeader);


    // make our own header
    LiblasHeader* myHeader = new LiblasHeader;
    setHeader(myHeader);

    //myHeader->setNumPoints( extHeader.GetPointRecordsCount() );

    //const liblas::Bounds<double>& extBounds = extHeader.GetExtent();
    //const Bounds<double> bounds(extBounds.minx(), extBounds.miny(), extBounds.minz(), extBounds.maxx(), extBounds.maxy(), extBounds.maxz());
    //myHeader->setBounds(bounds);

    //Schema& schema = myHeader->getSchema();
    //schema.addDimension(Dimension(Dimension::Field_X, Dimension::Double));
    //schema.addDimension(Dimension(Dimension::Field_Y, Dimension::Double));
    //schema.addDimension(Dimension(Dimension::Field_Z, Dimension::Double));

    return;
}


LiblasWriter::~LiblasWriter()
{
    delete m_writer;
}


void LiblasWriter::writeBegin()
{
    return;
}


void LiblasWriter::writeEnd()
{
    return;
}


boost::uint32_t LiblasWriter::writeBuffer(const PointData& pointData)
{
    boost::uint32_t numPoints = pointData.getNumPoints();
    boost::uint32_t i = 0;

    const std::size_t indexX = pointData.getDimensionIndex(Dimension::Field_X);
    const std::size_t indexY = pointData.getDimensionIndex(Dimension::Field_Y);
    const std::size_t indexZ = pointData.getDimensionIndex(Dimension::Field_Z);
    const std::size_t indexT = pointData.getDimensionIndex(Dimension::Field_Time);

    liblas::Point pt;

    for (i=0; i<numPoints; i++)
    {
        const double x = pointData.getField<float>(i, indexX);
        const double y = pointData.getField<float>(i, indexY);
        const double z = pointData.getField<float>(i, indexZ);
        const boost::uint64_t t = pointData.getField<boost::uint64_t>(i, indexT);

        pt.SetCoordinates(x,y,z);
        pt.SetTime((double)t);

        bool ok = m_writer->WritePoint(pt);
        assert(ok);
    }

    return numPoints;
}

} // namespace libpc
