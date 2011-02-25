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

#ifndef INCLUDED_STAGE_HPP
#define INCLUDED_STAGE_HPP

// boost
#include <boost/cstdint.hpp>

#include "libpc/PointData.hpp"
#include "libpc/Header.hpp"

    
namespace libpc
{

// every stage owns its own header, they are not shared
class LIBPC_DLL Stage
{
public:
    Stage();
    virtual ~Stage();

    // This reads a set of points at the current position in the file.
    //
    // The schema of the PointData buffer we are given here might
    // not match our own header's schema.  That's okay, though: all
    // that matters is that the buffer we are given has the fields
    // we need to write into.
    //
    // Returns the number of valid points read.
    virtual boost::uint32_t readPoints(PointData&) = 0;

    // advance (or retreat) to the Nth point in the file (absolute, 
    // not relative).  In some cases, this might be a very slow, painful
    // function to call.
    virtual void seekToPoint(boost::uint64_t pointNum) = 0;

    // resets the object's state such that it is positioned to the beginning
    // of the file, as if no reads had yet been done
    virtual void reset() = 0;

    // returns true after we've read all the points available to this stage
    virtual bool atEnd() const = 0;

    const Header& getHeader() const;
    Header& getHeader();

protected:
    void setHeader(Header*); // stage takes ownership

private:
    Header* m_header;

    Stage& operator=(const Stage&); // not implemented
    Stage(const Stage&); // not implemented
};

} // namespace libpc

#endif
