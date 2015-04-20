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

#include <ostream>
#include <istream>
#include <vector>
#include <map>
#include <set>

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

class PDAL_DLL FilenameStreamFactory : public StreamFactory
{
public:
    FilenameStreamFactory(const std::string& file);
    virtual ~FilenameStreamFactory();

    virtual std::istream& allocate();
    virtual void deallocate(std::istream&);

private:
    const std::string m_filename;

    typedef std::set<std::istream*> Set;
    Set m_streams;
};

class PDAL_DLL FilenameSubsetStreamFactory : public StreamFactory
{
public:
    FilenameSubsetStreamFactory(const std::string& file, uint64_t offset, uint64_t length);
    virtual ~FilenameSubsetStreamFactory();

    virtual std::istream& allocate();
    virtual void deallocate(std::istream&);

private:
    const std::string m_filename;
    const uint64_t m_offset;
    const uint64_t m_length;

    struct StreamSet
    {
    public:
        StreamSet(const std::string& filename, uint64_t offset, uint64_t length);
        ~StreamSet();
        std::istream* stream()
        {
            return m_streamslice;
        }
    private:
        typedef boost::iostreams::stream<boost::iostreams::file_source> Stream;
        typedef boost::iostreams::restriction<Stream> StreamSlice;
        typedef boost::iostreams::stream<StreamSlice> StreamSliceStream;
        std::istream* m_stream;
        StreamSlice* m_slice;
        StreamSliceStream* m_streamslice;
    };
    typedef std::map<std::istream*,StreamSet*> Map;  // maps streamslice -> set
    Map m_streams;
};

} // namespace pdal

#endif
