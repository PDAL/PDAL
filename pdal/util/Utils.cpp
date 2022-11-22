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

#include <pdal/util/Utils.hpp>

#include <array>
#include <cassert>
#include <cstdlib>
#include <cctype>
#include <memory>
#include <random>
#include <sstream>

#ifndef _WIN32
#include <cxxabi.h>
#include <sys/ioctl.h>
#include <sys/wait.h>  // WIFEXITED, WEXITSTATUS
#endif

#pragma warning(disable: 4127)  // conditional expression is constant

#include <stdio.h>
#include <iomanip>

#include "private/BacktraceImpl.hpp"

typedef std::vector<std::string> StringList;

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


int Utils::getenv(const std::string& name, std::string& val)
{
    char* value = ::getenv(name.c_str());
    if (value)
        val = value;
    else
        val.clear();
    return value ?  0 : -1;
}


int Utils::setenv(const std::string& env, const std::string& val)
{
#ifdef _WIN32
    return ::_putenv_s(env.c_str(), val.c_str()) ? -1 : 0;
#else
    return ::setenv(env.c_str(), val.c_str(), 1);
#endif
}


int Utils::unsetenv(const std::string& env)
{
#ifdef _WIN32
    return ::_putenv_s(env.c_str(), "") ? -1 : 0;
#else
    return ::unsetenv(env.c_str());
#endif
}


void Utils::eatwhitespace(std::istream& s)
{
    while (true)
    {
        const char c = (char)s.peek();
        if (!isspace(c))
            break;

        // throw it away
        s.get();
    }
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
    if (s.empty())
        return;

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


bool Utils::eatcharacter(std::istream& s, char x)
{
    const char c = (char)s.peek();
    if (c != x)
        return false;

    // throw it away
    s.get();

    return true;
}


std::string Utils::base64_encode(const unsigned char *bytes_to_encode,
    size_t in_len)
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
        return std::string();

    const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    std::string ret;
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


std::vector<uint8_t> Utils::base64_decode(std::string const& encoded_string)
{
    const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    std::string::size_type in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::vector<uint8_t> ret;

    while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_]))
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
            char_array_4[j] = static_cast<unsigned char>(base64_chars.find(char_array_4[j]));

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++)
            ret.push_back(char_array_3[j]);
    }
    return ret;
}


FILE* Utils::portable_popen(const std::string& command, const std::string& mode)
{
#ifdef _WIN32
    const std::string dos_command = Utils::replaceAll(command, "/", "\\");
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
        throw std::runtime_error("Error closing pipe for subprocess");
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


int Utils::run_shell_command(const std::string& cmd, std::string& output)
{
    const int maxbuf = 4096;
    char buf[maxbuf];

    output = "";

    FILE* fp = portable_popen(cmd.c_str(), "r");

    if (fp == NULL)
        return 1;

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


std::string Utils::replaceAll(std::string result,
    const std::string& replaceWhat, const std::string& replaceWithWhat)
{
    size_t pos = 0;
    while (1)
    {
        pos = result.find(replaceWhat, pos);
        if (pos == std::string::npos)
            break;
        result.replace(pos, replaceWhat.size(), replaceWithWhat);
        pos += replaceWithWhat.size();
        if (pos >= result.size())
            break;
    }
    return result;
}


std::string Utils::escapeJSON(const std::string &str)
{
    std::string s(str);

    std::array<std::string, 35> replacements {
      { "\\u0000", "\\u0001", "\\u0002", "\\u0003", "\\u0004",
        "\\u0005", "\\u0006", "\\u0007", "\\u0008", "\\t",
        "\\n", "\\b", "\\f", "\\r", "\\u000E",
        "\\u000F", "\\u0010", "\\u0011", "\\u0012", "\\u0013",
        "\\u0014", "\\u0015", "\\u0016", "\\u0017", "\\u0018",
        "\\u0019", "\\u001A", "\\u001B", "\\u001C", "\\u001D",
        "\\u001E", "\\u001F", " ", "!", "\\\"" }
    };
    for (std::string::size_type i = 0; i < s.size();)
    {
        unsigned char val = s[i];
        if (val < replacements.size())
        {
            s.replace(i, 1, replacements[val]);
            i += replacements[val].size();
        }
        else if (val == '\\')
        {
            s.replace(i, 1, "\\\\");
            i += 2;
        }
        else
            i++;
    }

    return s;
}


StringList Utils::wordWrap(std::string const& s, size_t lineLength,
    size_t firstLength)
{
    std::vector<std::string> output;
    if (s.empty())
        return output;

    if (firstLength == 0)
        firstLength = lineLength;

    size_t len = firstLength;

    std::istringstream iss(s);
    std::string line;
    do
    {
        std::string word;
        iss >> word;

        if ((line.length() + word.length() > len) && line.length())
        {
            trimTrailing(line);
            output.push_back(line);
            len = lineLength;
            line.clear();
        }
        while (word.length() > len)
        {
            output.push_back(word.substr(0, len));
            word = word.substr(len);
            len = lineLength;
        }
        line += word + " ";
    } while (iss);
    trimTrailing(line);
    if (!line.empty())
        output.push_back(line);
    return output;
}


StringList Utils::wordWrap2(std::string const& s, size_t lineLength,
    size_t firstLength)
{
    std::vector<std::string> output;
    if (s.empty())
        return output;

    if (firstLength == 0)
        firstLength = lineLength;

    auto pushWord = [&s, &output](size_t start, size_t end)
    {
        if (start != end)
            output.push_back(s.substr(start, end - start + 1));
    };

    size_t len = firstLength;
    size_t startPos = 0;
    while (true)
    {
        size_t endPos = (std::min)(startPos + len - 1, s.size() - 1);
        if (endPos + 1 == s.size())
        {
            pushWord(startPos, endPos);
            return output;
        }
        size_t pos = endPos;
        while (pos > startPos)
        {
            if (std::isspace(s[pos]) && !std::isspace(s[pos + 1]))
            {
                endPos = pos;
                break;
            }
            pos--;
        }
        pushWord(startPos, endPos);
        len = lineLength;
        startPos = endPos + 1;
    }
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
    if (status == 0)
        return std::string(result.get());
#endif

    return s;
}


int Utils::screenWidth()
{
#ifdef _WIN32
    return 80;
#else
    struct winsize ws;
    if (ioctl(0, TIOCGWINSZ, &ws))
        return 80;

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


double Utils::normalizeLongitude(double longitude)
{
    longitude = fmod(longitude, 360.0);
    if (longitude <= -180)
        longitude += 360;
    else if (longitude > 180)
        longitude -= 360;
    return longitude;
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


std::vector<std::string> Utils::simpleWordexp(const std::string& cmdline)
{
    std::string temp;
    bool instring = false;
    bool escape = false;
    std::vector<std::string> cmdArgs;
    for (size_t i = 0; i < cmdline.size(); ++i)
    {
        if (instring)
        {
            if (escape)
            {
                if (cmdline[i] != '"' && cmdline[i] != '\\')
                    temp += '\\';
                escape = false;
                temp += cmdline[i];
            }
            else if (cmdline[i] == '"')
                instring = false;
            else if (cmdline[i] == '\\')
                escape = true;
            else
                temp += cmdline[i];
        }
        else
        {
            if (escape)
            {
                escape = false;
                temp += cmdline[i];
            }
            else if (cmdline[i] == '"')
                instring = true;
            else if (cmdline[i] ==  '\\')
                escape = true;
            else if (std::isspace(cmdline[i]))
            {
                if (temp.size())
                {
                    cmdArgs.push_back(temp);
                    temp.clear();
                }
            }
            else
                temp += cmdline[i];
        }
    }
    if (!instring && temp.size())
        cmdArgs.push_back(temp);
    return cmdArgs;
}

} // namespace pdal
