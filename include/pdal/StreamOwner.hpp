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

#ifndef INCLUDED_STREAMOWNER_HPP
#define INCLUDED_STREAMOWNER_HPP

#include <pdal/pdal.hpp>

#include <string>
#include <cassert>
#include <stdexcept>
#include <cmath>
#include <ostream>
#include <istream>

namespace pdal
{

// Many of our reader & writer classes want to take a filename or a stream
// in their ctors, which means that we need a common piece of code that
// creates and takes ownership of the stream, if needed.
//
// This could, I suppose, evolve into something that even takes in things
// other than std streams or filenames.

class PDAL_DLL StreamOwnerBase
{
public:
    StreamOwnerBase(const std::string& filename);
    virtual ~StreamOwnerBase() {}

    // returns "" if stream-based ctor was used
    virtual const std::string& getFileName() const;

private:
    std::string m_filename;

    StreamOwnerBase(const StreamOwnerBase&); // nope
    StreamOwnerBase operator=(const StreamOwnerBase&); // nope
};


class PDAL_DLL IStreamOwner : public StreamOwnerBase
{
public:
    IStreamOwner(const std::string& filename);
    IStreamOwner(std::istream&);
    ~IStreamOwner();

    std::istream& istream();

private:
    std::istream* m_pistream; // not NULL iff we own the stream
    std::istream& m_istream;
};


class PDAL_DLL OStreamOwner : public StreamOwnerBase
{
public:
    OStreamOwner(const std::string& filename);
    OStreamOwner(std::ostream&);
    ~OStreamOwner();

    std::ostream& ostream();

private:
    std::ostream* m_postream;
    std::ostream& m_ostream;
};

} // namespace pdal

#endif
