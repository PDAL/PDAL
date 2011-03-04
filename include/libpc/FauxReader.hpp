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

#ifndef INCLUDED_FAUXREADER_HPP
#define INCLUDED_FAUXREADER_HPP

#include "libpc/Reader.hpp"

namespace libpc
{

// The FauxReader doesn't read from disk, but instead just makes up data for its
// points.  The reader is constructed with a given bounding box and a given 
// number of points.
//
// This reader knows about 4 fields (Dimensions):
//    X,Y,Z - floats
//    Time  - uint64
//
// It supports two modes: "random" generates points that are randomly
// distributed within the given bounding box, and "constant" generates its
// points to always be at the minimum of the bounding box.  The Time field
// is always set to the point number.
//
class LIBPC_DLL FauxReader : public Reader
{
public:
    enum Mode
    {
        Constant,
        Random
    };

public:
    FauxReader(const Bounds<double>&, int numPoints, Mode mode);
    FauxReader(const Bounds<double>&, int numPoints, Mode mode, const std::vector<Dimension>& dimensions);

    const std::string& getName() const;

    void seekToPoint(boost::uint64_t);

private:
    boost::uint32_t readBuffer(PointData& data);

    Mode m_mode;

    FauxReader& operator=(const FauxReader&); // not implemented
    FauxReader(const FauxReader&); // not implemented
};

} // namespace libpc

#endif
