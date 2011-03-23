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

#ifndef INCLUDED_ITERATOR_HPP
#define INCLUDED_ITERATOR_HPP

#include <libpc/Bounds.hpp>

namespace libpc
{
class Stage;
class PointBuffer;

class LIBPC_DLL Iterator
{
public:
    Iterator(const Stage& stage);
    virtual ~Iterator();

    const Stage& getStage() const;

    // This reads a set of points at the current position in the file.
    //
    // The schema of the PointBuffer buffer we are given here might
    // not match our own header's schema.  That's okay, though: all
    // that matters is that the buffer we are given has the fields
    // we need to write into.
    //
    // Returns the number of valid points read.
    virtual boost::uint32_t read(PointBuffer&) = 0;

    // advance N points ahead in the file
    //
    // In some cases, this might be a very slow, painful function to call because
    // it might entail physically reading the N points (and dropping the data on the
    // floor)
    virtual void skip(boost::uint64_t pointNum) = 0;

    // returns true after we've read all the points available to this stage
    virtual bool atEnd() const = 0;

    // Returns the current point number.  The first point is 0.
    // If this number if > getNumPoints(), then no more points
    // may be read (and atEnd() should be true).
    //
    // All stages have the notion of a current point number, even for stages
    // that are not really "oredered", in that the index just starts at zero 
    // and increments by N every time another N points are read
    boost::uint64_t getIndex() const;

    // used to control intermediate buffering needed by some stages
    void setChunkSize(boost::uint32_t size);
    boost::uint32_t getChunkSize() const;

protected:
    // advance the index by N, e.g. for use by skip()
    void incrementIndex(boost::uint64_t count);

private:
    const Stage& m_stage;
    boost::uint64_t m_index;
    boost::uint32_t m_chunkSize;

    Iterator& operator=(const Iterator&); // not implemented
    Iterator(const Iterator&); // not implemented
};


} // namespace libpc

#endif
