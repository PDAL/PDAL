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

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <istream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <vector>

#include <iostream>

#include "NullOStream.hpp"
#include "pdal_util_export.hpp"

namespace pdal
{

namespace Utils
{

#if defined(__APPLE__) && defined(__MACH__)
    const char dynamicLibExtension[] = ".dylib";
    const char dirSeparator = '/';
    const char pathListSeparator = ':';
#elif defined _WIN32
    const char dynamicLibExtension[] = ".dll";
    const char dirSeparator = '\\';
    const char pathListSeparator = ';';
#else
    const char dynamicLibExtension[] = ".so";
    const char dirSeparator = '/';
    const char pathListSeparator = ':';
#endif

    class StatusWithReason
    {
    public:
        StatusWithReason() : m_code(0)
        {}
        StatusWithReason(bool ok)
        {
            if (ok)
                m_code = 0;
            else
                m_code = -1;
        }
        StatusWithReason(int code);  // Not defined
        StatusWithReason(int code, const std::string& what) :
            m_code(code), m_what(what)
        {}

        int code() const
        { return m_code; }

        operator bool () const
        { return (m_code == 0); }

        std::string what() const
        { return m_what; }

    private:
        int m_code;
        std::string m_what;
    };


    /**
     * \brief Clamp value to given bounds.
     *
     * Clamps the input value t to bounds specified by min and max.
     * Used to ensure that row and column indices remain within valid bounds.
     *
     * \param t the input value.
     * \param min the lower bound.
     * \param max the upper bound.
     * \return the value to clamped to the given bounds.
     */
    template <class T>
    PDAL_DLL const T& clamp(const T& t, const T& minimum, const T& maximum)
    {
        return ((t < minimum) ? minimum : ((t > maximum) ? maximum : t));
    }

    /**
      Set a seed for random number generation.

      \param seed  Seed value.
    */
    PDAL_DLL void random_seed(unsigned int seed);

    /**
      Generate a random value in the range [minimum, maximum].

      \param minimum  Lower value of range for random number generation.
      \param maximum  Upper value of range for random number generation.
    */
    PDAL_DLL double random(double minimum, double maximum);

    /**
      Determine if two values are within a particular range of each other.

      \param v1  First value to compare.
      \param v2  Second value to compare.
      \param tolerance  Maximum difference between \ref v1 and \ref v2
    */
    PDAL_DLL inline bool compare_approx(double v1, double v2, double tolerance)
    {
        double diff = std::abs(v1 - v2);
        return diff <= std::abs(tolerance);
    }

    /**
      Round double value to nearest integral value.

      \param r  Value to round
      \return  Rounded value
    */
    inline double sround(double r)
        { return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5); }

    /**
      Convert a string to lowercase.

      \return  Converted string.
    **/
    inline std::string tolower(const std::string& s)
    {
        std::string out;
        for (size_t i = 0; i < s.size(); ++i)
            out += (char)std::tolower(s[i]);
        return out;
    }

    /**
      Convert a string to uppercase.

      \return  Converted string.
    */
    inline std::string toupper(const std::string& s)
    {
        std::string out;
        for (size_t i = 0; i < s.size(); ++i)
            out += (char)std::toupper(s[i]);
        return out;
    }

    /**
      Compare strings in a case-insensitive manner.

      \param s  First string to compare.
      \param s2  Second string to compare.
      \return  Whether the strings are equal.
    */
    inline bool iequals(const std::string& s, const std::string& s2)
    {
        if (s.length() != s2.length())
            return false;
        for (size_t i = 0; i < s.length(); ++i)
            if (std::toupper(s[i]) != std::toupper(s2[i]))
                return false;
        return true;
    }

    /**
      Determine if a string starts with a particular prefix.

      \param s  String to check for prefix.
      \param prefix  Prefix to search for.
      \return  Whether the string begins with the prefix.
    */
    inline bool startsWith(const std::string& s, const std::string& prefix)
    {
        if (prefix.size() > s.size())
            return false;
        return (strncmp(prefix.data(), s.data(), prefix.size()) == 0);
    }

    /**
      Determine if a string ends with a particular postfix.

      \param s  String to check for postfix.
      \param postfix Postfix to search for.
      \return  Whether the string ends with the postfix.
    */
    inline bool endsWith(const std::string& s, const std::string& postfix)
    {
        if (postfix.size() > s.size())
            return false;
        return (strcmp(postfix.data(),
                    s.data() + s.size() - postfix.size()) == 0);
    }

    /**
      Generate a checksum that is the integer sum of the values of the bytes
      in a buffer.

      \param buf  Pointer to buffer.
      \param size  Size of buffer.
      \return  Generated checksum.
    */
    inline int cksum(char *buf, size_t size)
    {
        int i = 0;
        while (size--)
            i += *buf++;
        return i;
    }

    /**
      Fetch the value of an environment variable.

      \param name  Name of environment variable.
      \param name  Value of the environment variable if it exists, empty
        otherwise.
      \return  0 on success, -1 on failure
    */
    PDAL_DLL int getenv(std::string const& name, std::string& val);

    /**
      Set the value of an environment variable.

      \param env  Name of environment variable.
      \param val  Value of environment variable.
      \return  0 on success, -1 on failure
    */
    PDAL_DLL int setenv(const std::string& env, const std::string& val);

    /**
      Clear the value of an environment variable.

      \param env  Name of the environment variable to clear.
      \return  0 on success, -1 on failure
    */
    PDAL_DLL int unsetenv(const std::string& env);

    /**
      Skip stream input until a non-space character is found.

      \param s  Stream to process.
    */
    PDAL_DLL void eatwhitespace(std::istream& s);

    /**
      Remove whitespace from the beginning of a string.

      \param s  String to be trimmed.
    */
    PDAL_DLL void trimLeading(std::string& s);

    /**
      Remove whitespace from the end of a string.

      \param s  String to be trimmed.
    */
    PDAL_DLL void trimTrailing(std::string& s);

    /**
      Remove whitespace from the beginning and end of a string.

      \param s  String to be trimmed.
    */
    inline void trim(std::string& s)
    {
        trimLeading(s);
        trimTrailing(s);
    }

    /**
      If specified character is at the current stream position, advance the
      stream position by 1.

      \param s  Stream to insect.
      \param x  Character to check for.
      \return \c true if the character is at the current stream position,
        \c false otherwise.
    */
    PDAL_DLL bool eatcharacter(std::istream& s, char x);

    /**
      Convert a buffer to a string using base64 encoding.

      \param buf  Pointer to buffer to encode.
      \param size  Size of buffer.
      \return  Encoded buffer.
    */
    PDAL_DLL std::string base64_encode(const unsigned char *buf, size_t size);

    /**
      Convert a buffer to a string using base64 encoding.

      \param bytes  Pointer to buffer to encode.
      \return  Encoded buffer.
    */
    inline std::string base64_encode(std::vector<uint8_t> const& bytes)
        { return base64_encode(bytes.data(), bytes.size()); }

    /**
      Decode a base64-encoded string into a buffer.

      \param input  String to decode.
      \return  Buffer containing decoded string.
    */
    PDAL_DLL std::vector<uint8_t>
    base64_decode(std::string const& input);

    /**
      Start a process to run a command and open a pipe to which input can
      be written and from which output can be read.

      \param command  Command to run in subprocess.
      \mode  Either 'r', 'w' or 'r+' to specify if the pipe should be opened
        as read-only, write-only or read-write.
      \return  Pointer to FILE for input/output from the subprocess.
    */
    PDAL_DLL FILE* portable_popen(const std::string& command,
        const std::string& mode);
    /**
      Close file opened with \ref portable_popen.

      \param fp  Pointer to file to close.
      \return  0 on success, -1 on failure.
    */
    PDAL_DLL int portable_pclose(FILE* fp);

    /**
      Create a subprocess and set the standard output of the command into the
      provided output string.

      \param cmd  Command to run.
      \param output  String to which output from the command should be written,
    */
    PDAL_DLL int run_shell_command(const std::string& cmd, std::string& output);

    /**
      Replace all instances of one string found in the input with another value.

      \param input  Input string to modify.
      \param replaceWhat  Token to locate in input string.
      \param replaceWithWhat  Replacement for found tokens.
      \return  Modified version of input string.
    */
    PDAL_DLL std::string replaceAll(std::string input,
        const std::string& replaceWhat , const std::string& replaceWithWhat);

    /**
      Break a string into a list of strings to not exceed a specified length.
      Whitespace is condensed to a single space and each string is free of
      whitespace at the beginning and end when possible.  Optionally, a line
      length for the first line can be different from subsequent lines.

      \param inputString  String to split into substrings.
      \param lineLength  Maximum length of substrings.
      \param firstLength  When non-zero, the maximum length of the first
        substring.  When zero, the first firstLength is assigned the value
        provided in lineLength.
      \return  List of substrings generated from the input string.
    */
    PDAL_DLL std::vector<std::string> wordWrap(std::string const& inputString,
        size_t lineLength, size_t firstLength = 0);

    /**
      Break a string into a list of strings to not exceed a specified length.
      The concatanation of the returned substrings will yield the original
      string.  The algorithm attempts to break the original string such that
      each substring begins with a word.

      \param inputString  String to split into substrings.
      \param lineLength  Maximum length of substrings.
      \param firstLength  When non-zero, the maximum length of the first
        substring.  When zero, the first firstLength is assigned the value
        provided in lineLength.
      \return  List of substrings generated from the input string.
    */
    PDAL_DLL std::vector<std::string> wordWrap2(std::string const& inputString,
        size_t lineLength, size_t firstLength = 0);

    /**
      Add escape characters or otherwise transform an input string so as to
      be a valid JSON string.

      \param s  Input string.
      \return  Valid JSON version of input string.
    */
    PDAL_DLL std::string escapeJSON(const std::string &s);

    /**
      Demangle a C++ symbol into readable form.

      \param s  String to demangle.
      \return  Demangled symbol.
    */
    PDAL_DLL std::string demangle(const std::string& s);

    /**
      Return the screen width of an associated tty.

      \return  The tty screen width or 80 if on Windows or it can't be
        determined.
    */
    PDAL_DLL int screenWidth();

    /**
      Escape non-printing characters by using standard notation (i.e. \n)
      or hex notation (\x10) as as necessary.

      \param s  String to modify.
      \return  Copy of input string with non-printing characters converted
        to printable notation.
    */
    PDAL_DLL std::string escapeNonprinting(const std::string& s);

    /**
      Normalize longitude so that it's between (-180, 180].

      \param longitude  Longitude to normalize.
      \return  Normalized longitude.
    */
    PDAL_DLL double normalizeLongitude(double longitude);

    /**
      Convert an input buffer to a hexadecimal string representation similar
      to the output of the UNIX command 'od'.  This is mostly used as an
      occasional debugging aid.

      \param buf  Point to buffer to dump.
      \param count  Size of buffer.
      \return  Buffer converted to hex string.
    */
    PDAL_DLL std::string hexDump(const char *buf, size_t count);

    /**
      Count the number of characters in a string that meet a predicate.

      \param s  String in which to start counting characters.
      \param p  Position in input string at which to start counting.
      \param pred  Unary predicate that tests a character.
      \return  Then number of characters matching the predicate.
    */
    template<typename PREDICATE>
    PDAL_DLL std::string::size_type
    extract(const std::string& s, std::string::size_type p, PREDICATE pred)
    {
        std::string::size_type count = 0;
        while (p < s.size() && pred(s[p++]))
            count++;
        return count;
    }

    /**
      Count the number of characters spaces in a string at a position.

      \param s  String in which to start counting characters.
      \param p  Position in input string at which to start counting.
      \return  Then number of space-y characters matching the predicate.
    */
    PDAL_DLL inline std::string::size_type
    extractSpaces(const std::string& s, std::string::size_type p)
        { return extract(s, p, (int(*)(int))std::isspace); }

    /**
      Split a string into substrings based on a predicate.  Characters
      matching the predicate are discarded.

      \param s  String to split.
      \param p  Unary predicate that returns true to indicate that a character
        is a split location.
      \return  Substrings.
    */
    template<typename PREDICATE>
    PDAL_DLL std::vector<std::string> split(const std::string& s, PREDICATE p)
    {
        std::vector<std::string> result;

        if (s.empty())
            return result;

        auto it = s.cbegin();
        auto const end = s.cend();
        decltype(it) nextIt;
        do
        {
            nextIt = std::find_if(it, end, p);
            result.push_back(std::string(it, nextIt));

            // Avoid advancing the iterator past the end to avoid UB.
            if (nextIt != end)
                it = nextIt + 1;
        } while (nextIt != end);

        return result;
    }

    /**
      Split a string into substrings.  Characters matching the predicate are
      discarded, as are empty strings otherwise produced by \ref split().

      \param s  String to split.
      \param p  Predicate returns true if a char in a string is a split
        location.
      \return  Vector of substrings.
    */
    template<typename PREDICATE>
    PDAL_DLL std::vector<std::string> split2(const std::string& s, PREDICATE p)
    {
        std::vector<std::string> result;

        if (s.empty())
            return result;

        auto it = s.cbegin();
        auto const end = s.cend();
        decltype(it) nextIt;
        do
        {
            nextIt = std::find_if(it, end, p);
            if (it != nextIt)
                result.push_back(std::string(it, nextIt));

            // Avoid advancing the iterator past the end to avoid UB.
            if (nextIt != end)
                it = nextIt + 1;
        } while (nextIt != end);

        return result;
    }

    /**
      Split a string into substrings based a splitting character.  The
      splitting characters are discarded.

      \param s  String to split.
      \param p  Character indicating split positions.
      \return  Substrings.
    */
    inline PDAL_DLL std::vector<std::string>
    split(const std::string& s, char tChar)
    {
        auto pred = [tChar](char c){ return(c == tChar); };
        return split(s, pred);
    }

    /**
      Split a string into substrings based a splitting character.  The
      splitting characters are discarded as are empty strings otherwise
      produced by \ref split().

      \param s  String to split.
      \param p  Character indicating split positions.
      \return  Substrings.
    */
    inline PDAL_DLL std::vector<std::string>
    split2(const std::string& s, char tChar)
    {
        auto pred = [tChar](char c){ return(c == tChar); };
        return split2(s, pred);
    }

    /**
      Perform shell-style word expansion (break a string into arguments).
      This only does simple handling of quoted values and backslashes
      and doesn't support fancier shell behavior.  Use the real wordexp()
      if you need all that.  The behavior of escaped values in a string
      was surprising to me, so try the shell first if you think you've
      found a problem.

      \param s  Input string to parse.
      \return  List of arguments.
    */
    PDAL_DLL std::vector<std::string>
    simpleWordexp(const std::string& s);

    /**
      Return a string representation of a type specified by the template
      argument.

      \return  String representation of the type.
    */
    template<typename T>
    std::string typeidName()
        { return Utils::demangle(typeid(T).name()); }

    struct RedirectStream
    {
        RedirectStream() : m_out(NULL), m_buf(NULL), m_null(new NullOStream)
        {}

        std::ofstream *m_out;
        std::streambuf *m_buf;
        std::unique_ptr<NullOStream> m_null;
    };

    /**
      Redirect a stream to some other stream, by default a null stream.

      \param out   Stream to redirect.
      \param dst   Destination stream.
      \return  Context for stream restoration (see \ref restore()).
    */
    inline RedirectStream redirect(std::ostream& out, std::ostream& dst)
    {
        RedirectStream redir;

        redir.m_buf = out.rdbuf();
        out.rdbuf(dst.rdbuf());
        return redir;
    }

    /**
      Redirect a stream to a null stream.

      \param out   Stream to redirect.
      \return  Context for stream restoration (see \ref restore()).
    */
    inline RedirectStream redirect(std::ostream& out)
    {
        RedirectStream redir;

        redir.m_buf = out.rdbuf();
        out.rdbuf(redir.m_null->rdbuf());
        return redir;
    }

    /**
      Redirect a stream to some file.

      \param out   Stream to redirect.
      \param file  Name of file where stream should be redirected.
      \return  Context for stream restoration (see \ref restore()).
    */
    inline RedirectStream redirect(std::ostream& out, const std::string& file)
    {
        RedirectStream redir;

        redir.m_out = new std::ofstream(file);
        redir.m_buf = out.rdbuf();
        out.rdbuf(redir.m_out->rdbuf());
        return redir;
    }

    /**
      Restore a stream redirected with redirect().

      \param out  Stream to be restored.
      \param redir RedirectStream returned from corresponding
        \ref redirect() call.
    */
    inline void restore(std::ostream& out, RedirectStream& redir)
    {
        out.rdbuf(redir.m_buf);
        if (redir.m_out)
            redir.m_out->close();
        redir.m_out = NULL;
        redir.m_buf = NULL;
    }

    //ABELL - This is certainly not as efficient as boost::numeric_cast, but
    //  has the advantage of not requiring an exception to indicate an error.
    //  We should investigate incorporating a version of boost::numeric_cast
    //  that avoids the exception for an error.
    /**
      Determine whether a double value may be safely converted to the given
      output type without over/underflow.  If the output type is integral the
      input will be rounded before being tested.

      \param in  Value to range test.
      \return  Whether value can be safely converted to template type.
    */
    template<typename T_OUT>
    bool inRange(double in)
    {
        if (std::is_integral<T_OUT>::value)
        {
            in = sround((double)in);
        }

        return std::is_same<double, T_OUT>::value ||
            (in >= static_cast<double>(std::numeric_limits<T_OUT>::lowest()) &&
             in <= static_cast<double>((std::numeric_limits<T_OUT>::max)()));
    }

    /**
      Determine whether a value may be safely converted to the given
      output type without over/underflow.  If the output type is integral and
      different from the input time, the value will be rounded before being
      tested.

      \param in  Value to range test.
      \return  Whether value can be safely converted to template type.
    */
    template<typename T_IN, typename T_OUT>
    bool inRange(T_IN in)
    {
        return std::is_same<T_IN, T_OUT>::value ||
            inRange<T_OUT>(static_cast<double>(in));
    }

    /**
      Convert a numeric value from one type to another.  Floating point
      values are rounded to the nearest integer before a conversion is
      attempted.

      \param in  Value to convert.
      \param out  Converted value.
      \return  \c true if the conversion was successful, \c false if the
        datatypes/input value don't allow conversion.
    */
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
            (in <= static_cast<double>((std::numeric_limits<T_OUT>::max)()) &&
             in >= static_cast<double>(std::numeric_limits<T_OUT>::lowest())))
        {
            out = static_cast<T_OUT>(in);
            return true;
        }
        return false;
    }

    /**
      Convert a numeric value from double to float.  Specialization to handle
      NaN.

      \param in  Value to convert.
      \param out  Converted value.
      \return  \c true if the conversion was successful, \c false if the
        datatypes/input value don't allow conversion.
    */
    template<>
    inline bool numericCast(double in, float& out)
    {
        if ((in <= static_cast<double>((std::numeric_limits<float>::max)()) &&
            in >= static_cast<double>(std::numeric_limits<float>::lowest())) ||
            std::isnan(in))
        {
            out = static_cast<float>(in);
            return true;
        }
        return false;
    }

    /**
      Convert a value to its string representation by writing to a stringstream.

      \param from  Value to convert.
      \return  String representation.
    */
    template<typename T>
    std::string toString(const T& from)
    {
        std::ostringstream oss;
        oss << from;
        return oss.str();
    }

    /**
      Convert a bool to a string.
    */
    inline std::string toString(bool from)
    {
        return from ? "true" : "false";
    }

    /**
      Convert a double to string with a precision of 10 decimal places.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(double from, size_t precision = 10)
    {
        std::ostringstream oss;
        // Standardize nan/inf output to the JAVA property names because
        // when we convert to a string, we usually convert to JSON.
        if (std::isnan(from))
            return "NaN";
        if (std::isinf(from))
            return (from < 0 ? "-Infinity" : "Infinity");
        oss << std::setprecision(precision) << from;
        return oss.str();
    }

    /**
      Convert a float to string with a precision of 10 decimal places.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(float from)
    {
        std::ostringstream oss;
        oss << std::setprecision(8) << from;
        return oss.str();
    }

    /**
      Convert a long long int to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(long long from)
        { return std::to_string(from); }

    /**
      Convert an unsigned long long int to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(unsigned long from)
        { return std::to_string(from); }

    /**
      Convert a long int to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(long from)
        { return std::to_string(from); }

    /**
      Convert an unsigned int to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(unsigned int from)
        { return std::to_string(from); }

    /**
      Convert an int to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(int from)
        { return std::to_string(from); }

    /**
      Convert an unsigned short to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(unsigned short from)
        { return std::to_string((int)from); }

    /**
      Convert a short int to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(short from)
        { return std::to_string((int)from); }

    /**
      Convert a char (treated as numeric) to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(char from)
        { return std::to_string((int)from); }

    /**
      Convert an unsigned char (treated as numeric) to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(unsigned char from)
        { return std::to_string((int)from); }

    /**
      Convert a signed char (treated as numeric) to string.

      \param from  Value to convert.
      \return  String representation of numeric value.
    */
    inline std::string toString(signed char from)
        { return std::to_string((int)from); }


    template<typename T>
    StatusWithReason fromString(const std::string& from, T* & to)
    {
        void *v;
        // Uses sscanf instead of operator>>(istream, void*&) as a workaround
        // for https://bugs.llvm.org/show_bug.cgi?id=19740, which presents with
        // clang-800.0.42.1 for x86_64-apple-darwin15.6.0.
        int result = sscanf(from.c_str(), "%p", &v);
        if (result != 1) {
            return false;
        }
        to = reinterpret_cast<T*>(v);
        return true;
    }

    /**
      Convert a string to a value by reading from a string stream.

      \param from  String to convert.
      \param to  Converted value.
      \return  \c true if the conversion was successful, \c false otherwise.
    */
    template<typename T>
    StatusWithReason fromString(const std::string& from, T& to)
    {
        std::istringstream iss(from);

        iss >> to;
        return !iss.fail();
    }

    // Optimization of above.
    template<>
    inline StatusWithReason fromString(const std::string& from, std::string& to)
    {
        to = from;
        return true;
    }

    /**
      Convert a numeric string to a char numeric value.

      \parm s  String to convert.
      \param to  Converted numeric value.
      \return  \c true if the conversion was successful, \c false otherwise.
    */
    template<>
    inline StatusWithReason fromString(const std::string& s, char& to)
    {
        try
        {
            int i = std::stoi(s);
            if (i >= std::numeric_limits<char>::lowest() &&
                    i <= (std::numeric_limits<char>::max)())
            {
                to = static_cast<char>(i);
                return true;
            }
        }
        catch (std::invalid_argument&) // Character that isn't a number?
        {
            if (s.length() == 1)
            {
                to = s[0];
                return true;
            }
        }
        return false;
    }

    /**
      Convert a numeric string to an unsigned char numeric value.

      \parm s  String to convert.
      \param to  Converted numeric value.
      \return  \c true if the conversion was successful, \c false otherwise.
    */
    template<>
    inline StatusWithReason fromString(const std::string& s, unsigned char& to)
    {
        try
        {
            int i  = std::stoi(s);
            if (i >= std::numeric_limits<unsigned char>::lowest() &&
                i <= (std::numeric_limits<unsigned char>::max)())
            {
                to = static_cast<unsigned char>(i);
                return true;
            }
        }
        catch (std::invalid_argument&) // Character that isn't a number?
        {
            if (s.length() == 1)
            {
                to = s[0];
                return true;
            }
        }

        return false;
    }

    /**
      Convert a numeric string to a signed char numeric value.

      \parm s  String to convert.
      \param to  Converted numeric value.
      \return  \c true if the conversion was successful, \c false otherwise.
    */
    template<>
    inline StatusWithReason fromString(const std::string& s, signed char& to)
    {
        try
        {
            int i = std::stoi(s);
            if (i >= std::numeric_limits<signed char>::lowest() &&
                i <= (std::numeric_limits<signed char>::max)())
            {
                to = static_cast<signed char>(i);
                return true;
            }
        }
        catch (std::invalid_argument&) // Character that isn't a number?
        {
            if (s.length() == 1)
            {
                to = s[0];
                return true;
            }
        }
        return false;
    }

    /**
      Specialization conversion from string to double to handle Nan.

      \param s  String to be converted.
      \param d  Converted value.
      \return  \c true if the conversion was successful, \c false otherwise.
    */
    template<>
    inline StatusWithReason fromString(const std::string& s, double& d)
    {
        if (s == "nan" || s == "NaN")
        {
            d = std::numeric_limits<double>::quiet_NaN();
            return true;
        }

        std::istringstream iss(s);

        iss >> d;
        return !iss.fail();
    }

    /**
      Return the argument cast to its underlying type.  Typically used on
      an enum.

      \param e  Variable for which to find the underlying type.
      \return  Converted variable.
    */
    template<typename E>
    typename std::underlying_type<E>::type toNative(E e)
    {
        return static_cast<typename std::underlying_type<E>::type>(e);
    }

} // namespace Utils
} // namespace pdal
