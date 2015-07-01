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

#include "SummaryData.hpp"

namespace pdal
{

SummaryData::SummaryData() :
    m_minX((std::numeric_limits<double>::max)()),
    m_minY((std::numeric_limits<double>::max)()),
    m_minZ((std::numeric_limits<double>::max)()),
    m_maxX(std::numeric_limits<double>::lowest()),
    m_maxY(std::numeric_limits<double>::lowest()),
    m_maxZ(std::numeric_limits<double>::lowest()),
    m_totalNumPoints(0)
{
    m_returnCounts.fill(0);
}


void SummaryData::addPoint(double x, double y, double z, int returnNumber)
{
    ++m_totalNumPoints;
    m_minX = (std::min)(m_minX, x);
    m_minY = (std::min)(m_minY, y);
    m_minZ = (std::min)(m_minZ, z);
    m_maxX = (std::max)(m_maxX, x);
    m_maxY = (std::max)(m_maxY, y);
    m_maxZ = (std::max)(m_maxZ, z);

    // Returns numbers are indexed from one, but the array indexes from 0.
    returnNumber--;
    if (returnNumber >= 0 && (size_t)returnNumber < m_returnCounts.size())
        m_returnCounts[returnNumber]++;
}


BOX3D SummaryData::getBounds() const
{
    BOX3D output(m_minX, m_minY, m_minZ, m_maxX, m_maxY, m_maxZ);
    return output;
}


point_count_t SummaryData::getReturnCount(int returnNumber) const
{
    if (returnNumber < 0 || (size_t)returnNumber >= m_returnCounts.size())
        throw pdal_error("getReturnCount: point returnNumber is out of range");
    return m_returnCounts[returnNumber];
}


void SummaryData::dump(std::ostream& str) const
{
    str << "MinX: " << m_minX << "\n";
    str << "MinY: " << m_minY << "\n";
    str << "MinZ: " << m_minZ << "\n";
    str << "MaxX: " << m_maxX << "\n";
    str << "MaxY: " << m_maxY << "\n";
    str << "MaxZ: " << m_maxZ << "\n";

    str << "Number of returns:";
    for (size_t i = 0; i < m_returnCounts.size(); ++i)
        str << " " << m_returnCounts[i];
    str << "\n";

    str << "Total number of points: " << m_totalNumPoints << "\n";
}


std::ostream& operator<<(std::ostream& ostr, const SummaryData& data)
{
    data.dump(ostr);
    return ostr;
}

} // namespace pdal
