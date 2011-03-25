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

#include <libpc/libpc.hpp>

#include <string>

    
namespace libpc
{

class Header;
class Iterator;
class SequentialIterator;
class RandomIterator;

// every stage owns its own header, they are not shared
class LIBPC_DLL Stage
{
public:
    Stage();
    virtual ~Stage();

    // Implement this in your concrete classes to return a constant string
    // as the name of the stage.  Use upper camel case, with spaces between
    // words.  The last word should generally be "Reader" or "Filter".
    virtual const std::string& getName() const = 0;

    // returns the number of points this stage has available
    // (actually a convenience function that gets it from the header)
    boost::uint64_t getNumPoints() const;

    const Header& getHeader() const;
    Header& getHeader();

    virtual bool supportsIterator (StageIteratorType) const { return false; }
    
    virtual bool supportsSequentialIterator() const { return false; }
    virtual bool supportsRandomIterator() const { return false; }
    virtual SequentialIterator* createSequentialIterator() const { return NULL; }
    virtual RandomIterator* createRandomIterator() const  { return NULL; }

protected:
    void setHeader(Header*); // stage takes ownership

private:
    Header* m_header;

    Stage& operator=(const Stage&); // not implemented
    Stage(const Stage&); // not implemented
};

} // namespace libpc

#endif
