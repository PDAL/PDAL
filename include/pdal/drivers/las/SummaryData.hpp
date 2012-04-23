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

#ifndef INCLUDED_DRIVERS_LAS_SUMMARYDATA_HPP
#define INCLUDED_DRIVERS_LAS_SUMMARYDATA_HPP

#include <pdal/pdal_internal.hpp>

#include <ostream>

namespace pdal
{
namespace drivers
{
namespace las
{

class PDAL_DLL SummaryData
{
public:
    SummaryData();
    ~SummaryData();

    void reset();

    // note that returnNumber is in the range [0..4]
    void addPoint(double x, double y, double z, int returnNumber);

    boost::uint32_t getTotalNumPoints() const;

    void getBounds(double& minX, double& minY, double& minZ, double& maxX, double& maxY, double& maxZ) const;

    // note that returnNumber is in the range [0..4]
    boost::uint32_t getReturnCount(int returnNumber) const;

    void dump(std::ostream&) const;

    static const int s_maxNumReturns = 5;

private:
    bool m_isFirst;
    double m_minX;
    double m_minY;
    double m_minZ;
    double m_maxX;
    double m_maxY;
    double m_maxZ;
    boost::uint32_t m_returnCounts[s_maxNumReturns];
    boost::uint32_t m_totalNumPoints;

    SummaryData& operator=(const SummaryData&); // not implemented
    SummaryData(const SummaryData&); // not implemented
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const SummaryData&);


}
}
} // namespaces

#endif
