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

#ifndef INCLUDED_STREAMFACTORY_HPP
#define INCLUDED_STREAMFACTORY_HPP

#include <pdal/pdal_internal.hpp>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file.hpp>
#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4512)  // assignment operator could not be generated
#endif
#include <boost/iostreams/restrict.hpp>
#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif

#include <string>
#include <cassert>
#include <stdexcept>
#include <cmath>
#include <ostream>
#include <istream>
#include <vector>

namespace pdal
{


class PDAL_DLL StreamFactory
{
public:
    StreamFactory() {}
    virtual ~StreamFactory() {}

    // returns the stream in the open state
    virtual std::istream& allocate() = 0;

    // will close the stream for you
    virtual void deallocate(std::istream&) = 0;

private:
    StreamFactory(const StreamFactory&); // nope
    StreamFactory& operator=(const StreamFactory&); // nope
};


// this one can't clone its stream!
class PDAL_DLL PassthruStreamFactory : public StreamFactory
{
public:
    PassthruStreamFactory(std::istream& s);
    virtual ~PassthruStreamFactory();

    virtual std::istream& allocate();
    virtual void deallocate(std::istream&);

private:
    std::istream& m_istream;
    bool m_allocated;
};


class PDAL_DLL FilenameStreamFactory : public StreamFactory
{
public:
    FilenameStreamFactory(const std::string& file);
    virtual ~FilenameStreamFactory();

    virtual std::istream& allocate();
    virtual void deallocate(std::istream&);

private:
    const std::string m_filename;
    std::vector<std::istream*> m_istreams;
};

class PDAL_DLL FilenameSubsetStreamFactory : public StreamFactory
{
public:
    FilenameSubsetStreamFactory(const std::string& file, boost::uint64_t offset, boost::uint64_t length);
    virtual ~FilenameSubsetStreamFactory();

    virtual std::istream& allocate();
    virtual void deallocate(std::istream&);

private:
    const std::string m_filename;
    const boost::uint64_t m_offset;
    const boost::uint64_t m_length;

    typedef boost::iostreams::stream<boost::iostreams::file_source> FStream;
    typedef boost::iostreams::restriction<FStream> FStreamSlice;
    typedef boost::iostreams::stream<FStreamSlice> FStreamSliceStream;

    struct StreamSet
    {
       StreamSet(std::istream*,FStreamSlice*,FStreamSliceStream*);
       ~StreamSet();
       bool match(std::istream&) const;
       std::istream* stream;
       FStreamSlice* slice;
       FStreamSliceStream* streamslice;
    };
    std::vector<StreamSet*> m_streams;
};


// Many of our writer classes want to take a filename or a stream
// in their ctors, which means that we need a common piece of code that
// creates and takes ownership of the stream, if needed.
//
// You must always call open(), regardless of what kind of ctor you call,
// i.e. stream or filename.  Calling close() is optional (the dtor will do
// it for you.)

class PDAL_DLL OutputStreamManager
{
public:
    OutputStreamManager(const std::string& filename);
    OutputStreamManager(std::ostream*); // may not be NULL
    ~OutputStreamManager();

    void open(); // throws
    void close();

    std::ostream& ostream();

    boost::uint64_t firstPos() const;

private:
    const bool m_isFileBased;
    bool m_isOpen;
    std::string m_filename;
    std::ostream* m_ostream;
    boost::uint64_t m_firstPos;

    OutputStreamManager(const OutputStreamManager&); // nope
    OutputStreamManager& operator=(const OutputStreamManager&); // nope
};


} // namespace pdal

#endif
