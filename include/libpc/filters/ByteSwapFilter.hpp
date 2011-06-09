/******************************************************************************
* Copyright (c) 2011, Howard Butler <hobu.inc@gmail.com>
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

#ifndef INCLUDED_FILTERS_CROPFILTER_HPP
#define INCLUDED_FILTERS_CROPFILTER_HPP

#include <pdal/pdal.hpp>

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>
#include <pdal/Bounds.hpp>

namespace pdal
{
    class PointBuffer;
}

namespace pdal { namespace filters {

class ByteSwapFilterSequentialIterator;

// removes any points outside of the given range
// updates the header accordingly
class LIBPC_DLL ByteSwapFilter : public Filter
{
public:
    ByteSwapFilter(const Stage& prevStage);

    const std::string& getDescription() const;
    const std::string& getName() const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;

        return false;
    }
    
    pdal::SequentialIterator* createSequentialIterator() const;
    pdal::RandomIterator* createRandomIterator() const { return NULL; }

    // returns number of points accepted into the data buffer (which may be less than data.getNumPoints(),
    // if we're calling this routine multiple times with the same buffer
    boost::uint32_t processBuffer(PointBuffer& dstData, const PointBuffer& srcData) const;


private:


    ByteSwapFilter& operator=(const ByteSwapFilter&); // not implemented
    ByteSwapFilter(const ByteSwapFilter&); // not implemented
};


} } // namespaces

#endif
