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

#include "libpc/LasReader.hpp"

namespace libpc
{


LasReader::LasReader(std::istream& istream)
    : Reader()
    , m_istream(istream)
{
    LasHeader* lasHeader = new LasHeader;
    lasHeader->read(istream);

    setHeader(lasHeader);

    return;
}


const LasHeader& LasReader::getLasHeader() const
{
    return (const LasHeader&)getHeader();
}


LasHeader& LasReader::getLasHeader()
{
    return (LasHeader&)getHeader();
}


void LasReader::seekToPoint(boost::uint64_t& pointNum)
{
    reset();

    boost::uint32_t chunk = (boost::uint32_t)pointNum; // BUG: this needs to be done in blocks if pointNum is large

    PointData pointData(getHeader().getSchema(), chunk);
    readPoints(pointData);

    // just drop the points on the floor and return
    return;
}


void LasReader::reset()
{
    m_currentPointIndex = 0;
    m_numPointsRead = 0;
}


void LasReader::readPoints(PointData& pointData)
{

    //Utils::read_n(m_point->GetData().front(), m_ifs, m_record_size);

    return;
}

} // namespace libpc
