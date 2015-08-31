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

#include <boost/algorithm/string/erase.hpp>
#include <boost/filesystem.hpp>

#include <pdal/util/Utils.hpp>

#include <cassert>
#include <cstdlib>
#include <cctype>
#include <random>

#ifndef _WIN32
#include <cxxabi.h>
#include <sys/ioctl.h>
#endif

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

#if !defined(_WIN32)
#include <dlfcn.h>
#define DLL_LOAD_UNIX
#else
#include <windows.h>
#define DLL_LOAD_WINDOWS
#endif

#include <stdio.h>
#include <iomanip>

using namespace std;

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


double Utils::uniform(const double& minimum, const double& maximum,
    uint32_t seed)
{
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> dist(minimum, maximum);

    return dist(gen);
}


double Utils::normal(const double& mean, const double& sigma, uint32_t seed)
{
    std::mt19937 gen(seed);
    std::normal_distribution<double> dist(mean, sigma);

    return dist(gen);
}


void* Utils::registerPlugin(void* stageFactoryPtr, string const& filename,
    string const& registerMethod, string const& versionMethod)
{
    void* pRegister;
    void* pVersion;

    pVersion = Utils::getDLLSymbol(filename, versionMethod);

    int plugins_version = ((int (*)()) pVersion)();

    if (plugins_version != PDAL_PLUGIN_VERSION)
    {
        ostringstream oss;
        oss << "Unable to register shared library '" << filename <<
            "' with method name '" << registerMethod <<
            "' version of plugin, '" << plugins_version <<
            "' did not match PDALs version '" << PDAL_PLUGIN_VERSION << "'";
        throw pdal_error(oss.str());
    }


    pRegister = Utils::getDLLSymbol(filename, registerMethod);
    if (pRegister != NULL)
    {
        ((void (*)(void*)) pRegister)(stageFactoryPtr);
    }
    else
    {
        ostringstream oss;
        oss << "Unable to register shared library '" << filename <<
            "' with method name '" << registerMethod << "'";
        throw pdal_error(oss.str());
    }

    return pRegister;
}


char* Utils::getenv(const char* env)
{
    return ::getenv(env);
}


string Utils::getenv(string const& name)
{
    char* value = ::getenv(name.c_str());
    return value ? string(value) : string();
}

int Utils::putenv(const char* env)
{
#ifdef _WIN32
    return ::_putenv(env);
#else
    return ::putenv(const_cast<char*>(env));
#endif
}

void Utils::eatwhitespace(istream& s)
{
    while (true)
    {
        const char c = (char)s.peek();
        if (!isspace(c))
            break;

        // throw it away
        s.get();
    }
    return;
}


void Utils::trimLeading(std::string& s)
{
    size_t pos = 0;
    // Note, that this should be OK in C++11, which guarantees a NULL.
    while (isspace(s[pos]))
        pos++;
    s = s.substr(pos);
}


void Utils::trimTrailing(std::string& s)
{
    size_t pos = s.size() - 1;
    while (isspace(s[pos]))
    {
        if (pos == 0)
        {
            s.clear();
            return;
        }
        else
            pos--;
    }
    s = s.substr(0, pos + 1);
}


bool Utils::eatcharacter(istream& s, char x)
{
    const char c = (char)s.peek();
    if (c != x)
        return false;

    // throw it away
    s.get();

    return true;
}

uint32_t Utils::getStreamPrecision(double scale)
{
    double frac = 0;
    double integer = 0;

    frac = modf(scale, &integer);
    return abs(floorl(log10(frac)));
}

void* Utils::getDLLSymbol(string const& library, string const& name)
{
    // Completely stolen from GDAL.
    /***************************************************************************
     * $Id: cplgetsymbol.cpp 16702 2009-04-01 20:42:49Z rouault $
     *
     * Project:  Common Portability Library
     * Purpose:  Fetch a function pointer from a shared library / DLL.
     * Author:   Frank Warmerdam, warmerdam@pobox.com
     *
     ***************************************************************************
     * Copyright (c) 1999, Frank Warmerdam
     *
     * Permission is hereby granted, free of charge, to any person obtaining a
     * copy of this software and associated documentation files
     * (the "Software"),
     * to deal in the Software without restriction, including without limitation
     * the rights to use, copy, modify, merge, publish, distribute, sublicense,
     * and/or sell copies of the Software, and to permit persons to whom the
     * Software is furnished to do so, subject to the following conditions:
     *
     * The above copyright notice and this permission notice shall be included
     * in all copies or substantial portions of the Software.
     *
     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
     * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
     * MERCHANTABILITY,
     * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
     * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
     * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
     * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
     * OTHER DEALINGS IN THE SOFTWARE.
     ****************************************************************************/

    void* pLibrary = NULL;
    void* pSymbol = NULL;

#ifdef DLL_LOAD_UNIX
    pLibrary = dlopen(library.c_str(), RTLD_LAZY);
    if (pLibrary == NULL)
    {
        ostringstream oss;
        oss << "Unable to open '" << library <<"' with error " << dlerror();
        throw pdal_error(oss.str());
    }

    pSymbol = dlsym(pLibrary, name.c_str());
#if (defined(__APPLE__) && defined(__MACH__))
    /* On mach-o systems, C symbols have a leading underscore and depending
     * on how dlcompat is configured it may or may not add the leading
     * underscore.  So if dlsym() fails add an underscore and try again.
     */
    if (pSymbol == NULL)
    {
        ostringstream prefixed;
        prefixed << "_" << name;
        pSymbol = dlsym(pLibrary, prefixed.str().c_str());
    }
#endif

    if (pSymbol == NULL)
    {
        ostringstream oss;
        oss << "Opened library '" << library << "', but unable to open symbol "
            "'" << name << "' with error " << dlerror();
        throw pdal_error(oss.str());
    }

#endif

#ifdef DLL_LOAD_WINDOWS

    pLibrary = LoadLibrary(library.c_str());
    if (pLibrary == NULL)
    {
        LPVOID      lpMsgBuf = NULL;
        int         nLastError = GetLastError();

        FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER
                      | FORMAT_MESSAGE_FROM_SYSTEM
                      | FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL, nLastError,
                      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                      (LPTSTR) &lpMsgBuf, 0, NULL);

        ostringstream oss;
        oss << "Can't load requested DLL '" << library <<
            " with error code " << nLastError <<
            " and message \"" << (const char *) lpMsgBuf <<"\"";
        throw pdal_error(oss.str());
    }

    pSymbol = (void *) GetProcAddress((HINSTANCE) pLibrary, name.c_str());

    if (pSymbol == NULL)
    {
        ostringstream oss;
        oss << "Can't find requested entry point '" << name <<"'";
        throw pdal_error(oss.str());
    }


#endif
    return pSymbol;
}


string Utils::base64_encode(const unsigned char *bytes_to_encode, size_t in_len)
{

    /*
        base64.cpp and base64.h

        Copyright (C) 2004-2008 René Nyffenegger

        This source code is provided 'as-is', without any express or implied
        warranty. In no event will the author be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:

        1. The origin of this source code must not be misrepresented;
           you must not claim that you wrote the original source code. If you
           use this source code in a product, an acknowledgment in the product
           documentation would be appreciated but is not required.

        2. Altered source versions must be plainly marked as such, and must
           not be misrepresented as being the original source code.

        3. This notice may not be removed or altered from any source
           distribution.

        René Nyffenegger rene.nyffenegger@adp-gmbh.ch
    */

    if (in_len == 0)
        return string();

    const string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    string ret;
    int i = 0;
    int j = 0;
    uint8_t char_array_3[3];
    uint8_t char_array_4[4];

    while (in_len--)
    {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) +
                ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) +
                ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i <4) ; i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) +
            ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) +
            ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while ((i++ < 3))
            ret += '=';
    }
    return ret;
}


static inline bool is_base64(unsigned char c)
{
    return (isalnum(c) || (c == '+') || (c == '/'));
}


vector<uint8_t> Utils::base64_decode(string const& encoded_string)
{
    const string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    string::size_type in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    vector<uint8_t> ret;

    while (in_len-- && (encoded_string[in_] != '=') &&
        is_base64(encoded_string[in_]))
    {
        char_array_4[i++] = encoded_string[in_];
        in_++;
        if (i == 4)
        {
            for (i = 0; i <4; i++)
                char_array_4[i] = static_cast<unsigned char>(
                    base64_chars.find(char_array_4[i]));

            char_array_3[0] = (char_array_4[0] << 2) +
                ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) +
                ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                ret.push_back(char_array_3[i]);
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j <4; j++)
            char_array_4[j] = 0;

        for (j = 0; j <4; j++)
            char_array_4[j] =
                static_cast<unsigned char>(base64_chars.find(char_array_4[j]));

        char_array_3[0] = (char_array_4[0] << 2) +
            ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) +
            ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) +
            char_array_4[3];

        for (j = 0; (j < i - 1); j++)
            ret.push_back(char_array_3[j]);
    }
    return ret;
}

FILE* Utils::portable_popen(const string& command, const string& mode)
{
#ifdef _WIN32
    const string dos_command = Utils::replaceAll(command, "/", "\\");
    return _popen(dos_command.c_str(), mode.c_str());
#else
    return popen(command.c_str(), mode.c_str());
#endif
}

int Utils::portable_pclose(FILE* fp)
{
    int status = 0;

#ifdef _WIN32
    status = _pclose(fp);
#else
    status = pclose(fp);
    if (status == -1)
    {
        throw runtime_error("error executing command");
    }
    if (WIFEXITED(status) != 0)
    {
        status = WEXITSTATUS(status);
    }
    else
    {
        status = 0;
    }
#endif

    return status;
}


// BUG:
// Under unix, the pclose() operation causes the boost unit test system
// to produce a fatal error iff the process started by popen returns a
// nonzero status code.  For this reason, I've put all the "negative"
// cmd line app tests under #ifdef PDAL_COMPILER_MSVC.
//
// This problem shows up on mpg's Ubuntu 11.4 machine (gcc 4.5.2, boost 1.47.0)
// as well as on Hobu's machine.

// Boost's unit test system has a flag on the execution monitor that catches
// all signals --  p_catch_system_errors, and throws unittest errors when
// it sees them. We can use --catch_system_errors=no as part of the invocation,
// or manually turn them off in the execution monitor. -- hobu 7/12/2012
//  boost::unit_test::unit_test_monitor.p_catch_system_errors.set (false);
// #include <boost/test/unit_test_monitor.hpp>

int Utils::run_shell_command(const string& cmd, string& output)
{
    const int maxbuf = 4096;
    char buf[maxbuf];

    output = "";

    const char* gdal_debug = ::pdal::Utils::getenv("CPL_DEBUG");
    if (gdal_debug == 0)
    {
        pdal::Utils::putenv("CPL_DEBUG=OFF");
    }

    FILE* fp = portable_popen(cmd.c_str(), "r");

    while (!feof(fp))
    {
        if (fgets(buf, maxbuf, fp) == NULL)
        {
            if (feof(fp)) break;
            if (ferror(fp)) break;
        }
        output += buf;
    }
    return portable_pclose(fp);
}


string Utils::replaceAll(string result, const string& replaceWhat,
    const string& replaceWithWhat)
{
    size_t pos = 0;
    while (1)
    {
        pos = result.find(replaceWhat, pos);
        if (pos == string::npos)
            break;
        result.replace(pos, replaceWhat.size(), replaceWithWhat);
        pos += replaceWithWhat.size();
        if (pos >= result.size())
            break;
    }
    return result;
}


// Adapted from http://stackoverflow.com/a/11969098.
std::string Utils::escapeJSON(const string &str)
{
    std::string escaped(str);

    escaped.erase
    (
        remove_if
        (
            escaped.begin(),
            escaped.end(),
            [](const char c)
            {
                return (c <= 31);
            }
        ),
        escaped.end()
    );

    size_t pos(0);

    while((pos = escaped.find_first_of("\"\\/", pos)) != string::npos)
    {
        escaped.insert(pos, "\\");
        pos += 2;
    }

    return escaped;
}

/// Break a string into a list of strings, none of which exceeds a specified
/// length.
/// \param[in] inputString  String to split
/// \param[in] lineLength  Maximum length of any of the output strings
/// \return  List of string split from input.
///
StringList Utils::wordWrap(string const& inputString, size_t lineLength)
{
    // stolen from http://stackoverflow.com/questions/5815227/fix-improve-word-wrap-function

    StringList output;

    istringstream iss(inputString);
    string line;
    do
    {
        string word;
        iss >> word;

        if (line.length() + word.length() > lineLength)
        {
            output.push_back(line);
            line.clear();
        }
        line += word + " ";

    } while (iss);

    if (!line.empty())
        output.push_back(line);
    return output;
}


/// Demangle strings using the compiler-provided demangle function.
/// \param[in] s  String to be demangled.
/// \return  Demangled string
std::string Utils::demangle(const std::string& s)
{
#ifndef _WIN32
    int status;
    std::unique_ptr<char[], void (*)(void*)> result(
            abi::__cxa_demangle(s.c_str(), 0, 0, &status), std::free);
    return std::string(result.get());
#else
    return s;
#endif
}


int Utils::screenWidth()
{
#ifdef WIN32
    return 80;
#else
    struct winsize ws;
    ioctl(0, TIOCGWINSZ, &ws);

    return ws.ws_col;
#endif
}


std::string Utils::escapeNonprinting(const std::string& s)
{
    std::string out;

    for (size_t i = 0; i < s.size(); ++i)
    {
        if (s[i] == '\n')
            out += "\\n";
        else if (s[i] == '\a')
            out += "\\a";
        else if (s[i] == '\b')
            out += "\\b";
        else if (s[i] == '\r')
            out += "\\r";
        else if (s[i] == '\v')
            out += "\\v";
        else if (s[i] < 32)
        {
            std::stringstream oss;
            oss << std::hex << std::setfill('0') << std::setw(2) << (int)s[i];
            out += "\\x" + oss.str();
        }
        else
            out += s[i];
    }
    return out;
}

// Useful for debug on occasion.
std::string Utils::hexDump(const char *buf, size_t count)
{
   const unsigned char *cp = reinterpret_cast<const unsigned char *>(buf);
   char foo[80];
   int bytes, i, address = 0;
   std::string out;

   bytes = (count > 16) ? 16 : count;

   while (bytes) {
      sprintf(foo, "0x%06x ", address);
      address += 16;
      for (i = 0; i < 16; i++) {
         if (i < bytes) {
            sprintf(foo, "%02X ", cp[i]);
            out += foo;
         }
         else
            out += "   ";
      }
      out += "|";
      for (i = 0; i < bytes; i++) {
         sprintf(foo, "%c", isprint(cp[i]) ? cp[i] : '.');
         out += foo;
      }
      out += "|\n";
      count -= bytes;
      cp += bytes;
      bytes = (count > 16) ? 16 : count;
   }
   return (out);
}

} // namespace pdal
