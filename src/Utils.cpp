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

#include <cassert>

// boost
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>

#include "libpc/Utils.hpp"


namespace libpc
{

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


std::istream* Utils::Open(std::string const& filename, std::ios::openmode mode)
{
    namespace io = boost::iostreams;
    io::stream<io::file_source>* ifs = new io::stream<io::file_source>();
    ifs->open(filename.c_str(), mode);
    if (ifs->is_open() == false) return NULL;
    return ifs;
}


std::ostream* Utils::Create(std::string const& filename, std::ios::openmode mode)
{
    namespace io = boost::iostreams;
    io::stream<io::file_sink>* ofs = new io::stream<io::file_sink>();
    ofs->open(filename.c_str(), mode);
    if (ofs->is_open() == false) return NULL;
    return ofs;
}


void Utils::Cleanup(std::ostream* ofs)
{
    namespace io = boost::iostreams;

    // An ofstream is closeable and deletable, but
    // an ostream like &std::cout isn't.
    if (!ofs) return;
    if (static_cast<io::stream<io::file_sink>&>(*ofs))
    {
        static_cast<io::stream<io::file_sink>&>(*ofs).close();
        delete ofs;
    }
}


void Utils::Cleanup(std::istream* ifs)
{
    namespace io = boost::iostreams;

    // An ifstream is closeable and deletable, but
    // an istream like &std::cin isn't.
    if (!ifs) return;
    if (static_cast<io::stream<io::file_source>&>(*ifs))
    {
        static_cast<io::stream<io::file_source>&>(*ifs).close();
        delete ifs;
    }
}

} // namespace libpc
