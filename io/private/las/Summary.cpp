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

#include "Summary.hpp"

namespace pdal
{
namespace las
{

Summary::Summary()
{
    clear();
}

void Summary::clear()
{
    m_totalNumPoints = 0;
    m_returnCounts.fill(0);
    m_bounds.clear();
}

void Summary::addPoint(double x, double y, double z, int returnNumber)
{
    ++m_totalNumPoints;
    m_bounds.grow(x, y, z);

    // Returns numbers are indexed from one, but the array indexes from 0.
    if (returnNumber >= 1 && returnNumber <= (int)m_returnCounts.size())
        m_returnCounts[returnNumber - 1]++;
}


BOX3D Summary::getBounds() const
{
    return m_bounds;
}


point_count_t Summary::getReturnCount(int returnNumber) const
{
    return m_returnCounts[returnNumber];
}


void Summary::dump(std::ostream& str) const
{
    str << m_bounds;
    str << "Number of returns:";
    for (size_t i = 0; i < m_returnCounts.size(); ++i)
        str << " " << m_returnCounts[i];
    str << "\n";

    str << "Total number of points: " << m_totalNumPoints << "\n";
}


std::ostream& operator<<(std::ostream& ostr, const Summary& data)
{
    data.dump(ostr);
    return ostr;
}

} // namespace las
} // namespace pdal
