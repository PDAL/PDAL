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

#ifndef INCLUDED_UTILS_HPP
#define INCLUDED_UTILS_HPP

#include <pdal/pdal_internal.hpp>

#include <string>
#include <cassert>
#include <stdexcept>
#include <cmath>
#include <ostream>
#include <istream>
#include <limits>
#include <cstring>
#include <sstream>
#include <vector>

#include <boost/numeric/conversion/cast.hpp>

namespace pdal
{

class PDAL_DLL Utils
{
public:
    static void random_seed(unsigned int seed);
    static double random(double minimum, double maximum);

    // compares two values to within the datatype's epsilon
    template<class T>
    static bool compare_distance(const T& actual, const T& expected)
    {
        const T epsilon = std::numeric_limits<T>::epsilon();
        return compare_approx<T>(actual, expected, epsilon);
    }

    // compares two values to within a given tolerance
    // the value |tolerance| is compared to |actual - expected|
    template<class T>
    static bool compare_approx(const T& actual, const T& expected,
        const T& tolerance)
    {
        double diff = std::abs((double)actual - (double)expected);
        return diff <= std::abs((double) tolerance);
    }

    // Copy v into *t and increment dest by the sizeof v
    template<class T>
    static inline void write_field(boost::uint8_t*& dest, T v)
    {
        *(T*)(void*)dest = v;
        dest += sizeof(T);
    }

    // Return a 'T' from a stream and increment src by the sizeof 'T'
    template<class T>
    static inline T read_field(boost::uint8_t*& src)
    {
        T tmp = *(T*)(void*)src;
        src += sizeof(T);
        return tmp;
    }

    // Copy data from dest to src and increment dest by the copied size.
    template<class T>
    static inline void read_array_field(boost::uint8_t*& src, T* dest,
        std::size_t count)
    {
        memcpy((boost::uint8_t*)dest, (boost::uint8_t*)(T*)src,
            sizeof(T)*count);
        src += sizeof(T) * count;
    }

    // Read 'num' items from the source stream to the dest location
    template <typename T>
    static inline void read_n(T& dest, std::istream& src,
        std::streamsize const& num)
    {
        if (!src.good())
            throw pdal::invalid_stream("pdal::Utils::read_n<T> input stream is "
                "not readable");

        char* p = as_buffer(dest);
        src.read(p, num);

        assert(check_stream_state(src));
    }

    template <typename T>
    static inline void write_n(std::ostream& dest, T const& src,
        std::streamsize const& num)
    {
        if (!dest.good())
            throw std::runtime_error("pdal::Utils::write_n<T>: output stream "
                "is not writable");

        T& tmp = const_cast<T&>(src);

        char const* p = as_bytes(tmp);
        dest.rdbuf()->sputn(p, num);

        assert(check_stream_state(dest));
    }

    // From http://stackoverflow.com/questions/485525/round-for-float-in-c
    static inline double sround(double r)
    {
        return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
    }
    
    static inline std::vector<boost::uint8_t>hex_string_to_binary(std::string const& source)
    {
        // Stolen from http://stackoverflow.com/questions/7363774/c-converting-binary-data-to-a-hex-string-and-back
        static int nibbles[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0, 10, 11, 12, 13, 14, 15 };
        std::vector<unsigned char> retval;
        for (std::string::const_iterator it = source.begin(); it < source.end(); it += 2) {
            unsigned char v = 0;
            if (::isxdigit(*it))
                v = (unsigned char)nibbles[::toupper(*it) - '0'] << 4;
            if (it + 1 < source.end() && ::isxdigit(*(it + 1)))
                v += (unsigned char)nibbles[::toupper(*(it + 1)) - '0'];
            retval.push_back(v);
        }
        return retval;
    }

    static inline void binary_to_hex_stream(unsigned char* source,
        std::ostream& destination, int start, int end )
    {
        static char syms[] = "0123456789ABCDEF";
        for (int i = start; i != end; i++)
            destination << syms[((source[i] >> 4) & 0xf)] <<
                           syms[source[i] & 0xf];
    }

    static inline std::string binary_to_hex_string(
        const std::vector<unsigned char>& source)
    {
        static char syms[] = "0123456789ABCDEF";
        std::stringstream ss;
        for (std::vector<unsigned char>::const_iterator it = source.begin();
                it != source.end(); it++)
            ss << syms[((*it >> 4) & 0xf)] << syms[*it & 0xf];

        return ss.str();
    }

    template<typename Target, typename Source>
    static inline Target saturation_cast(Source const& src)
    {
        try
        {
            return boost::numeric_cast<Target>(src);
        }
        catch (boost::numeric::negative_overflow const&)
        {
            return std::numeric_limits<Target>::min();
        }
        catch (boost::numeric::positive_overflow const&)
        {
            return std::numeric_limits<Target>::max();
        }
    }
    
    static void* registerPlugin( void* stageFactoryPtr, 
                                std::string const& filename, 
                                std::string const& registerMethodName,
                                std::string const& versionMethodName);

    static char* getenv(const char* env);
    static std::string getenv(std::string const& name);
    static int putenv(const char* env);

    // aid to operator>> parsers
    static void eatwhitespace(std::istream& s);

    // aid to operator>> parsers
    // if char found, eats it and returns true; otherwise, don't eat it and
    // then return false
    static bool eatcharacter(std::istream& s, char x);

    static boost::uint32_t getStreamPrecision(double scale);

    static boost::uint32_t safeconvert64to32(boost::uint64_t x64);

    // Generates a random temporary filename
    static std::string generate_filename();
    static std::string generate_tempfile();

    static void* getDLLSymbol(std::string const& library,
       std::string const& name);
    static std::string base64_encode(std::vector<boost::uint8_t> const& bytes);
    static std::vector<boost::uint8_t> base64_decode(std::string const& input);
    
    static FILE* portable_popen(const std::string& command,
        const std::string& mode);
    static int portable_pclose(FILE* fp);
    static int run_shell_command(const std::string& cmd, std::string& output);
   
    static std::string replaceAll(std::string result, 
                                  const std::string& replaceWhat, 
                                  const std::string& replaceWithWhat);
    static void wordWrap(std::string const& inputString, 
                         std::vector<std::string>& outputString, 
                         unsigned int lineLength);
    static std::string& escapeJSON(std::string &s);
    static std::string demangle(const std::string& s);

    template<typename T>
    static std::string typeidName()
        { return Utils::demangle(typeid(T).name()); }
 
private:
    template<typename T>
    static inline char* as_buffer(T& data)
    {
        return static_cast<char*>(static_cast<void*>(&data));
    }

    template<typename T>
    static inline char* as_buffer(T* data)
    {
        return static_cast<char*>(static_cast<void*>(data));
    }

    template<typename T>
    static inline char const* as_bytes(T const& data)
    {
        return static_cast<char const*>(static_cast<void const*>(&data));
    }

    template<typename T>
    static inline char const* as_bytes(T const* data)
    {
        return static_cast<char const*>(static_cast<void const*>(data));
    }

    template <typename C, typename T>
    static inline bool check_stream_state(std::basic_ios<C, T>& srtm)
    {
        // Test stream state bits
        if (srtm.eof())
            throw std::out_of_range("end of file encountered");
        else if (srtm.fail())
            throw std::runtime_error("non-fatal I/O error occured");
        else if (srtm.bad())
            throw std::runtime_error("fatal I/O error occured");
        return true;
    }
};

} // namespace pdal

#endif
