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

#include <libpc/Utils.hpp>

#include <cassert>

#ifdef LIBPC_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4702)  // unreachable code
#endif
#include <boost/iostreams/device/file.hpp>
#ifdef LIBPC_COMPILER_MSVC
#  pragma warning(pop)
#endif
#include <boost/iostreams/stream.hpp>
#include <boost/filesystem.hpp>

#include <libpc/exceptions.hpp>


#ifdef LIBPC_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace libpc
{


void Utils::random_seed(unsigned int seed)
{
    srand(seed);
}


double Utils::random(double minimum, double maximum)
{
    double r = (double)rand();  // [0..32767]
    double v = (maximum - minimum) / (double)RAND_MAX;
    double s = r * v; // [0..(max-min)]
    double t = minimum + s; // [min..max]

    assert(t >= minimum);
    assert(t <= maximum);

    return t;
}


std::istream* Utils::openFile(std::string const& filename, bool asBinary)
{
    if (!Utils::fileExists(filename))
        throw libpc_error("File not found: " + filename);

    std::ios::openmode mode = std::ios::in;
    if (asBinary)
      mode |= std::ios::binary;

    namespace io = boost::iostreams;
    io::stream<io::file_source>* ifs = new io::stream<io::file_source>();
    ifs->open(filename.c_str(), mode);
    if (ifs->is_open() == false) return NULL;
    return ifs;
}


std::ostream* Utils::createFile(std::string const& filename, bool asBinary)
{
    std::ios::openmode mode = std::ios::out;
    if (asBinary)
      mode  |= std::ios::binary;

    namespace io = boost::iostreams;
    io::stream<io::file_sink>* ofs = new io::stream<io::file_sink>();
    ofs->open(filename.c_str(), mode);
    if (ofs->is_open() == false) return NULL;
    return ofs;
}


void Utils::closeFile(std::ostream* ofs)
{
    namespace io = boost::iostreams;

    // An ofstream is closeable and deletable, but
    // an ostream like &std::cout isn't.
    if (!ofs) return;
    io::stream<io::file_sink>* sink = dynamic_cast<io::stream<io::file_sink>*>(ofs);
    if (sink)
    {
        sink->close();
        delete sink;
    }
}


void Utils::closeFile(std::istream* ifs)
{
    namespace io = boost::iostreams;

    // An ifstream is closeable and deletable, but
    // an istream like &std::cin isn't.
    if (!ifs) return;
    io::stream<io::file_source>* source = dynamic_cast<io::stream<io::file_source>*>(ifs);
    if (source)
    {
        source->close();
        delete source;
    }
}


bool Utils::deleteFile(const std::string& file)
{
  return boost::filesystem::remove(file);
}


void Utils::renameFile(const std::string& dest, const std::string& src)
{
  boost::filesystem::rename(src, dest);
}


bool Utils::fileExists(const std::string& file)
{
  return boost::filesystem::exists(file);
}


boost::uintmax_t Utils::fileSize(const std::string& file)
{
  return boost::filesystem::file_size(file);
}


char* Utils::getenv(const char* env)
{
    return ::getenv(env);
}


int Utils::putenv(const char* env)
{
#ifdef LIBPC_COMPILER_MSVC
    return ::_putenv(env);
#else
    return ::putenv(const_cast<char*>(env));
#endif
}


void Utils::eatwhitespace(std::istream& s)
{
    while (true)
    {
        const char c = (char)s.peek();
        if (!isspace(c)) break;

        // throw it away
        s.get();
    }
    return;
}
    

bool Utils::eatcharacter(std::istream& s, char x)
{
    const char c = (char)s.peek();
    if (c != x) return false;

    // throw it away
    s.get();

    return true;
}


} // namespace libpc
