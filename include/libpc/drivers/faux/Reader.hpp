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

#ifndef INCLUDED_DRIVERS_FAUX_READER_HPP
#define INCLUDED_DRIVERS_FAUX_READER_HPP

#include <libpc/libpc.hpp>

#include <libpc/Stage.hpp>
#include <libpc/Bounds.hpp>
#include <libpc/Dimension.hpp>

namespace libpc
{
    class PointBuffer;
}

namespace libpc { namespace drivers { namespace faux {


// The FauxReader doesn't read from disk, but instead just makes up data for its
// points.  The reader is constructed with a given bounding box and a given 
// number of points.
//
// This reader knows about 4 fields (Dimensions):
//    X,Y,Z - floats
//    Time  - uint64
//
// It supports a few modes: 
//   - "random" generates points that are randomly distributed within the given bounding box
//   - "constant" generates its points to always be at the minimum of the bounding box
//   - "ramp" generates its points as a linear ramp from the minimum of the bbox to the maximum
// In all these modes, however, the Time field is always set to the point number.
//
class LIBPC_DLL Reader : public libpc::Stage
{
public:
    enum Mode
    {
        Constant,
        Random,
        Ramp
    };

public:
    Reader(const Bounds<double>&, int numPoints, Mode mode);
    Reader(const Bounds<double>&, int numPoints, Mode mode, const std::vector<Dimension>& dimensions);

    const std::string& getDescription() const;
    const std::string& getName() const;
    
    Mode getMode() const;
    
    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        if (t == StageIterator_Random ) return true;
        
        return false;
    }

    libpc::SequentialIterator* createSequentialIterator() const;
    libpc::RandomIterator* createRandomIterator() const;

    // this is called by the stage's iterator
    boost::uint32_t processBuffer(PointBuffer& data, boost::uint64_t index) const;

private:

    Mode m_mode;

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented
};


} } } // namespaces


#endif
