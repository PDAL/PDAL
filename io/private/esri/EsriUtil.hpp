/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include <sstream>
#include <stdexcept>

#include <pdal/JsonFwd.hpp>

namespace lepcc
{
    struct Point3D;
    struct RGB_t;
}

namespace pdal
{
namespace i3s
{

class Version
{
private:
    int major = 0;
    int minor = 0;
    int patch = 0;

public:
    Version()
    {}

    Version(std::string vString)
    {
        std::istringstream iss(vString);
        std::string token;
        if(std::getline(iss, token, '.'))
            if(!token.empty())
                major = std::stoi(token);
        if(std::getline(iss, token, '.'))
            if(!token.empty())
                minor = std::stoi(token);
        if(std::getline(iss, token, '.'))
            if(!token.empty())
                patch = std::stoi(token);
    }

    bool operator<(const Version& other)
    {
        if(this->major < other.major)
            return true;
        if(this->minor < other.minor && this->major == other.major)
            return true;
        if(this->patch < other.patch &&
                this->major == other.major &&
                this->minor == other.minor)
            return true;
        return false;
    }
    bool operator==(const Version& other)
    {
        return (this->patch == other.patch && this->major == other.major &&
                this->minor == other.minor);
    }
    bool operator <=(const Version& other)
    { return *this < other || *this == other; }
    bool operator >=(const Version& other)
    { return !(*this < other); }
    bool operator > (const Version& other)
    { return !(*this < other) && !(*this == other); }

    friend std::ostream& operator<<(std::ostream& out, const Version & v);
};
inline std::ostream& operator<<(std::ostream& out, const Version & v)
{
    out << v.major << "." << v.minor << "." << v.patch;
    return out;
}

class EsriError : public std::runtime_error
{
public:
    EsriError(const std::string txt) : std::runtime_error(txt)
    {}
};

NL::json parse(const std::string& data, const std::string& error);
std::vector<lepcc::Point3D> decompressXYZ(std::vector<char>* compData);
std::vector<lepcc::RGB_t> decompressRGB(std::vector<char>* compData);
std::vector<uint16_t> decompressIntensity(std::vector<char>* compData);

} // namespace i3s
} // namespace pdal
