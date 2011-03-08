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

#ifndef INCLUDED_CACHEFILTER_HPP
#define INCLUDED_CACHEFILTER_HPP

#include "libpc/Filter.hpp"

namespace libpc
{

// This is just a very simple MRU filter -- future versions will be smarter.
// This cache has the following constraints:
//   - only one block is cached
//   - read chunk sizes are assumed to be just 1 point
// If more than one point is read, the cache is skipped.
class LIBPC_DLL CacheFilter : public Filter
{
public:
    CacheFilter(Stage& prevStage);
    ~CacheFilter();

    const std::string& getName() const;

    // number of points requested from this filter via read()
    boost::uint64_t getNumPointsRequested() const;

    // num points this filter read from the previous stage
    boost::uint64_t getNumPointsRead() const;

    // override
    boost::uint64_t getCurrentPointIndex() const;

    // override
    void seekToPoint(boost::uint64_t index);

private:
    boost::uint32_t readBuffer(PointData&);

    boost::uint64_t m_currentPointIndex;
    boost::uint64_t m_storedPointIndex;
    PointData* m_storedPointData;
    boost::uint64_t m_numPointsRequested;
    boost::uint64_t m_numPointsRead;

    CacheFilter& operator=(const CacheFilter&); // not implemented
    CacheFilter(const CacheFilter&); // not implemented
};

} // namespace libpc

#endif
