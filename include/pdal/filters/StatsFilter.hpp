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

#ifndef INCLUDED_FILTERS_STATSFILTER_HPP
#define INCLUDED_FILTERS_STATSFILTER_HPP

#include <pdal/pdal.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Range.hpp>
#include <pdal/PointBuffer.hpp>


namespace pdal { namespace filters {

class StatsFilterSequentialIterator;

// this is just a pass-thorugh filter, which collects some stats about the points
// that are fed through it
class PDAL_DLL StatsFilter : public Filter
{
public:
    SET_STAGE_NAME("filters.stats", "Statistics Filter")

    StatsFilter(Stage& prevStage, const Options&);
    StatsFilter(Stage& prevStage);

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        if (t == StageIterator_Random ) return true;
        if (t == StageIterator_Block ) return true;
        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator() const;
    pdal::StageRandomIterator* createRandomIterator() const { return 0; } // BUG: add this
    pdal::StageBlockIterator* createBlockIterator() const { return 0; } // BUG: add this

    void processBuffer(PointBuffer& data) const;

    // clears the counters
    void reset();
    void getData(boost::uint64_t& count, 
                 double& minx, double& miny, double& minz, 
                 double& maxx, double& maxy, double& maxz,
                 double& avgx, double& avgy, double& avgz) const;

private:
    // BUG: not threadsafe, these should maybe live in the iterator
    mutable boost::uint64_t m_totalPoints;
    mutable double m_minimumX, m_minimumY, m_minimumZ;
    mutable double m_maximumX, m_maximumY, m_maximumZ;
    mutable double m_sumX, m_sumY, m_sumZ;

    StatsFilter& operator=(const StatsFilter&); // not implemented
    StatsFilter(const StatsFilter&); // not implemented
};


} } // namespaces

#endif
