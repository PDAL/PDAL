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

#include <pdal/drivers/las/SummaryData.hpp>


namespace pdal
{
namespace drivers
{
namespace las
{


SummaryData::SummaryData()
    : m_isFirst(true)
    , m_minX(0.0)
    , m_minY(0.0)
    , m_minZ(0.0)
    , m_maxX(0.0)
    , m_maxY(0.0)
    , m_maxZ(0.0)
    , m_totalNumPoints(0)
{
    reset();
    return;
}


SummaryData::~SummaryData()
{
}


void SummaryData::reset()
{
    m_isFirst = true;

    m_minX = 0.0;
    m_minY = 0.0;
    m_minZ = 0.0;
    m_maxX = 0.0;
    m_maxY = 0.0;
    m_maxZ = 0.0;

    m_returnCounts.assign(MAXRETURNCOUNT,0);

    m_totalNumPoints = 0;

    return;
}


void SummaryData::addPoint(double x, double y, double z, int returnNumber)
{
    bool bAddReturn(true);
    if (returnNumber < 0 || returnNumber > MAXRETURNCOUNT - 1)
        bAddReturn = false;
//         throw invalid_point_data("addPoint: point returnNumber is out of range", 0);

    if (m_isFirst)
    {
        m_isFirst = false;
        m_minX = x;
        m_minY = y;
        m_minZ = z;
        m_maxX = x;
        m_maxY = y;
        m_maxZ = z;
    }
    else
    {
        m_minX = std::min(m_minX, x);
        m_minY = std::min(m_minY, y);
        m_minZ = std::min(m_minZ, z);
        m_maxX = std::max(m_maxX, x);
        m_maxY = std::max(m_maxY, y);
        m_maxZ = std::max(m_maxZ, z);
    }

    if (bAddReturn)
        m_returnCounts[returnNumber] = m_returnCounts[returnNumber] + 1;

    ++m_totalNumPoints;

    return;
}


boost::uint32_t SummaryData::getTotalNumPoints() const
{
    return m_totalNumPoints;
}


pdal::Bounds<double> SummaryData::getBounds() const
{
    pdal::Bounds<double> output;
    output.setMinimum(0, m_minX);
    output.setMinimum(1, m_minY);
    output.setMinimum(2, m_minZ);

    output.setMaximum(0, m_maxX);
    output.setMaximum(1, m_maxY);
    output.setMaximum(2, m_maxZ);
    return output;
}


boost::uint32_t SummaryData::getReturnCount(int returnNumber) const
{
    if (returnNumber < 0 || returnNumber > MAXRETURNCOUNT-1)
        return 0;
//         throw invalid_point_data("getReturnCount: point returnNumber is out of range", 0);

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
    for (std::vector<boost::uint32_t>::size_type i=0; i<m_returnCounts.size(); i++)
    {
        str << " " << m_returnCounts[i];
    }
    str << "\n";

    str << "Total number of points: " << m_totalNumPoints << "\n";

    return;
}


std::ostream& operator<<(std::ostream& ostr, const SummaryData& data)
{
    data.dump(ostr);

    return ostr;
}


}
}
} // namespaces
