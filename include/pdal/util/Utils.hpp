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

#pragma once

#include <pdal/pdal_internal.hpp>

#include <algorithm>
#include <string>
#include <cassert>
#include <cctype>
#include <stdexcept>
#include <cmath>
#include <fstream>
#include <istream>
#include <limits>
#include <cstring>
#include <sstream>
#include <vector>
#include <map>

#include <boost/numeric/conversion/cast.hpp>
#include <boost/lexical_cast.hpp>

namespace pdal
{

namespace Utils
{
    template<typename T>
    char *as_buffer(T& data)
        { return static_cast<char*>(static_cast<void*>(&data)); }

    template<typename T>
    char *as_buffer(T* data)
        { return static_cast<char*>(static_cast<void*>(data)); }

    template <typename C, typename T>
    bool check_stream_state(std::basic_ios<C, T>& srtm)
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

    PDAL_DLL void random_seed(unsigned int seed);
    PDAL_DLL double random(double minimum, double maximum);
    PDAL_DLL double uniform(const double& minimum=0.0f,
        const double& maximum=1.0f, uint32_t seed=0);
    PDAL_DLL double normal(const double& mean=0.0f, const double& sigma=1.0f,
        uint32_t seed=0);

    // compares two values to within a given tolerance
    // the value |tolerance| is compared to |actual - expected|
    template<class T>
    bool compare_approx(const T& actual, const T& expected, const T& tolerance)
    {
        double diff = std::abs((double)actual - (double)expected);
        return diff <= std::abs((double) tolerance);
    }

    // compares two values to within the datatype's epsilon
    template<class T>
    bool compare_distance(const T& actual, const T& expected)
    {
        const T epsilon = std::numeric_limits<T>::epsilon();
        return compare_approx<T>(actual, expected, epsilon);
    }

    // From http://stackoverflow.com/questions/485525/round-for-float-in-c
    inline double sround(double r)
        { return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5); }

    inline std::vector<uint8_t>
    hex_string_to_binary(std::string const& source)
    {
        // Stolen from http://stackoverflow.com/questions/7363774/  ...
        //    c-converting-binary-data-to-a-hex-string-and-back
        static int nibbles[] =
            { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0,
              10, 11, 12, 13, 14, 15 };
        std::vector<unsigned char> retval;
        for (auto it = source.begin(); it < source.end(); it += 2) {
            unsigned char v = 0;
            if (::isxdigit(*it))
                v = (unsigned char)nibbles[::toupper(*it) - '0'] << 4;
            if (it + 1 < source.end() && ::isxdigit(*(it + 1)))
                v += (unsigned char)nibbles[::toupper(*(it + 1)) - '0'];
            retval.push_back(v);
        }
        return retval;
    }

    inline void binary_to_hex_stream(unsigned char* source,
        std::ostream& destination, int start, int end )
    {
        static char syms[] = "0123456789ABCDEF";
        for (int i = start; i != end; i++)
            destination << syms[((source[i] >> 4) & 0xf)] <<
                           syms[source[i] & 0xf];
    }

    inline std::string binary_to_hex_string(
        const std::vector<unsigned char>& source)
    {
        static char syms[] = "0123456789ABCDEF";
        std::stringstream ss;
        for (std::vector<unsigned char>::const_iterator it = source.begin();
                it != source.end(); it++)
            ss << syms[((*it >> 4) & 0xf)] << syms[*it & 0xf];

        return ss.str();
    }

    inline std::string tolower(const std::string& s)
    {
        std::string out;
        for (size_t i = 0; i < s.size(); ++i)
            out += (char)std::tolower(s[i]);
        return out;
    }

    inline std::string toupper(const std::string& s)
    {
        std::string out;
        for (size_t i = 0; i < s.size(); ++i)
            out += (char)std::toupper(s[i]);
        return out;
    }

    inline bool iequals(const std::string& s, const std::string& s2)
    {
        if (s.length() != s2.length())
            return false;
        for (size_t i = 0; i < s.length(); ++i)
            if (std::toupper(s[i]) != std::toupper(s2[i]))
                return false;
        return true;
    }

    inline bool startsWith(const std::string& s, const std::string& prefix)
    {
        if (prefix.size() > s.size())
            return false;
        return (strncmp(prefix.data(), s.data(), prefix.size()) == 0);
    }

    inline int cksum(char *buf, size_t size)
    {
        int i = 0;
        while (size--)
            i += *buf++;
        return i;
    }

    PDAL_DLL void *registerPlugin(void *stageFactoryPtr,
        std::string const& filename, std::string const& registerMethodName,
        std::string const& versionMethodName);

    PDAL_DLL char *getenv(const char *env);
    PDAL_DLL std::string getenv(std::string const& name);
    PDAL_DLL int putenv(const char *env);

    // aid to operator>> parsers
    PDAL_DLL void eatwhitespace(std::istream& s);
    PDAL_DLL void trimLeading(std::string& s);
    PDAL_DLL void trimTrailing(std::string& s);
    inline void trim(std::string& s)
    {
        trimLeading(s);
        trimTrailing(s);
    }
    // if char found, eats it and returns true; otherwise, don't eat it and
    // then return false
    PDAL_DLL bool eatcharacter(std::istream& s, char x);
    PDAL_DLL uint32_t getStreamPrecision(double scale);
    PDAL_DLL void *getDLLSymbol(std::string const& library,
        std::string const& name);
    PDAL_DLL std::string base64_encode(const unsigned char *buf, size_t size);
    inline std::string base64_encode(std::vector<uint8_t> const& bytes)
        { return base64_encode(bytes.data(), bytes.size()); }
    PDAL_DLL std::vector<uint8_t>
    base64_decode(std::string const& input);

    PDAL_DLL FILE* portable_popen(const std::string& command,
        const std::string& mode);
    PDAL_DLL int portable_pclose(FILE* fp);
    PDAL_DLL int run_shell_command(const std::string& cmd, std::string& output);
    PDAL_DLL std::string replaceAll(std::string result,
        const std::string& replaceWhat, const std::string& replaceWithWhat);
    PDAL_DLL StringList wordWrap(std::string const& inputString,
        size_t lineLength);
    PDAL_DLL std::string escapeJSON(const std::string &s);
    PDAL_DLL std::string demangle(const std::string& s);
    PDAL_DLL int screenWidth();
    PDAL_DLL std::string escapeNonprinting(const std::string& s);
    PDAL_DLL std::string hexDump(const char *buf, size_t count);

    /// Split a string into substrings.  Characters matching the predicate are
    ///   discarded.
    /// \param[in] s  String to split.
    /// \param[in] p  Predicate returns true if a char in a string is a split
    ///   location.
    /// \return  Vector of substrings.
    template<typename PREDICATE>
    PDAL_DLL std::vector<std::string> split(const std::string& s, PREDICATE p)
    {
        std::vector<std::string> result;

        if (s.empty())
            return result;

        auto it = s.begin();
        decltype(it) endIt;
        do
        {
            endIt = std::find_if(it, s.end(), p);
            result.push_back(std::string(it, endIt));
            it = endIt + 1;
        } while (endIt != s.end());
        return result;
    }

    /// Split a string into substrings.  Characters matching the predicate are
    ///   discarded, as are empty strings otherwise produced by split().
    /// \param[in] s  String to split.
    /// \param[in] p  Predicate returns true if a char in a string is a split
    ///   location.
    /// \return  Vector of substrings.
    template<typename PREDICATE>
    PDAL_DLL std::vector<std::string> split2(const std::string& s, PREDICATE p)
    {
        std::vector<std::string> result;

        if (s.empty())
            return result;

        auto it = s.begin();
        decltype(it) endIt;
        do
        {
            endIt = std::find_if(it, s.end(), p);
            if (it != endIt)
                result.push_back(std::string(it, endIt));
            it = endIt + 1;
        } while (endIt != s.end());
        return result;
    }

    inline PDAL_DLL std::vector<std::string>
    split(const std::string& s, char tChar)
    {
        auto pred = [tChar](char c){ return(c == tChar); };
        return split(s, pred);
    }


    inline PDAL_DLL std::vector<std::string>
    split2(const std::string& s, char tChar)
    {
        auto pred = [tChar](char c){ return(c == tChar); };
        return split2(s, pred);
    }

    template<typename T>
    std::string typeidName()
        { return Utils::demangle(typeid(T).name()); }

    template<typename KEY, typename VALUE>
    bool contains(const std::map<KEY, VALUE>& c, const KEY& v)
        { return c.find(v) != c.end(); }

    template<typename COLLECTION, typename VALUE>
    bool contains(const COLLECTION& c, const VALUE& v)
        { return (std::find(c.begin(), c.end(), v) != c.end()); }

    struct RedirectStream
    {
        std::ofstream *m_out;
        std::streambuf *m_buf;
    };

    /// Redirect a stream to some file, by default /dev/null.
    /// \param[in] out   Stream to redirect.
    /// \param[in] file  Name of file where stream should be redirected.
    /// \return  Context for stream restoration (see restore()).
    inline RedirectStream redirect(std::ostream& out,
        const std::string& file = "/dev/null")
    {
        RedirectStream redir;

        redir.m_out = new std::ofstream(file);
        redir.m_buf = out.rdbuf();
        out.rdbuf(redir.m_out->rdbuf());
        return redir;
    }

    /// Restore a stream redirected with redirect().
    /// \param[in] out  Stream to be restored.
    /// \param[in] redir RedirectStream returned from corresponding
    /// redirect() call.
    inline void restore(std::ostream& out, RedirectStream redir)
    {
        out.rdbuf(redir.m_buf);
        redir.m_out->close();
    }

    //ABELL - This is certainly not as efficient as boost::numeric_cast, but
    //  has the advantage of not requiring an exception to indicate an error.
    //  We should investigate incorporating a version of boost::numeric_cast
    //  that avoids the exception for an error.
    // Determine whether a value of a given input type may be safely
    // statically casted to the given output type without over/underflow.  If
    // the output type is integral, inRange() will determine whether the
    // rounded input value, rather than truncated, may be safely converted.
    template<typename T_OUT>
    bool inRange(double in)
    {
        if (std::is_integral<T_OUT>::value)
        {
            in = sround((double)in);
        }

        return std::is_same<double, T_OUT>::value ||
           (in >= static_cast<double>(std::numeric_limits<T_OUT>::lowest()) &&
            in <= static_cast<double>(std::numeric_limits<T_OUT>::max()));
    }

    template<typename T_IN, typename T_OUT>
    bool inRange(T_IN in)
    {
        return std::is_same<T_IN, T_OUT>::value ||
            inRange<T_OUT>(static_cast<double>(in));
    }

    template<typename T_IN, typename T_OUT>
    bool numericCast(T_IN in, T_OUT& out)
    {
        if (std::is_same<T_IN, T_OUT>::value)
        {
            out = static_cast<T_OUT>(in);
            return true;
        }
        if (std::is_integral<T_OUT>::value)
            in = static_cast<T_IN>(sround((double)in));
        if ((std::is_same<T_OUT, double>::value) ||
            (in <= static_cast<double>(std::numeric_limits<T_OUT>::max()) &&
             in >= static_cast<double>(std::numeric_limits<T_OUT>::lowest())))
        {
            out = static_cast<T_OUT>(in);
            return true;
        }
        return false;
    }

    template<typename T>
    std::string toString(const T& from)
    {
        std::ostringstream oss;
        oss << from;
        return oss.str();
    }

    // There is an overload of std::to_string() for float and double, but
    // its behavior is different from streaming and doesn't match what
    // we have been doing historically.

    inline std::string toString(long long from)
        { return std::to_string(from); }

    inline std::string toString(unsigned long from)
        { return std::to_string(from); }

    inline std::string toString(long from)
        { return std::to_string(from); }

    inline std::string toString(unsigned int from)
        { return std::to_string(from); }

    inline std::string toString(int from)
        { return std::to_string(from); }

    inline std::string toString(unsigned short from)
        { return std::to_string((int)from); }

    inline std::string toString(short from)
        { return std::to_string((int)from); }

    inline std::string toString(char from)
        { return std::to_string((int)from); }

    inline std::string toString(unsigned char from)
        { return std::to_string((int)from); }

    inline std::string toString(signed char from)
        { return std::to_string((int)from); }

    template<typename T>
    bool fromString(const std::string& from, T& to)
    {
        try
        {
            to = boost::lexical_cast<T>(from);
        }
        catch (boost::bad_lexical_cast&)
        {
            return false;
        }
        return true;
    }

    template<>
    inline bool fromString<char>(const std::string& s, char& to)
    {
        int i = std::stoi(s);
        if (i >= std::numeric_limits<char>::lowest() &&
            i <= std::numeric_limits<char>::max())
        {
            to = static_cast<char>(i);
            return true;
        }
        return false;
    }

    template<>
    inline bool fromString<unsigned char>(const std::string& s,
        unsigned char& to)
    {
        int i = std::stoi(s);
        if (i >= std::numeric_limits<unsigned char>::lowest() &&
            i <= std::numeric_limits<unsigned char>::max())
        {
            to = static_cast<unsigned char>(i);
            return true;
        }
        return false;
    }

    template<>
    inline bool fromString<signed char>(const std::string& s, signed char& to)
    {
        int i = std::stoi(s);
        if (i >= std::numeric_limits<signed char>::lowest() &&
            i <= std::numeric_limits<signed char>::max())
        {
            to = static_cast<signed char>(i);
            return true;
        }
        return false;
    }

} // namespace Utils
} // namespace pdal

