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

#include <pdal/Utils.hpp>

#include <cassert>
#include <cstdlib>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
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


char* Utils::getenv(const char* env)
{
    return ::getenv(env);
}


int Utils::putenv(const char* env)
{
#ifdef PDAL_PLATFORM_WIN32
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


std::string Utils::trim(const std::string& str)
{
    // Trim Both leading and trailing spaces
    std::size_t startpos = str.find_first_not_of(" \t\n"); // Find the first character position after excluding leading blank spaces
    std::size_t endpos = str.find_last_not_of(" \t\n"); // Find the first character position from reverse af
 
    // if all spaces or empty return an empty string
    if((std::string::npos == startpos ) || (std::string::npos == endpos))
    {
        return "";
    }
    
    return str.substr( startpos, endpos-startpos+1 );
}

boost::uint32_t Utils::getStreamPrecision(double scale)
{
    double frac = 0;
    double integer = 0;
    
    frac = std::modf(scale, &integer);
    double precision = std::fabs(std::floor(std::log10(frac)));
    
    // FIXME: This should test that precision actually ends up being a 
    // whole number
    boost::uint32_t output = static_cast<boost::uint32_t>(precision);
    return output;
}



} // namespace pdal
