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

#include <pdal/Metadata.hpp>
#include <pdal/Utils.hpp>

#include <sstream>
#include <cstring>

#include <sstream>
#include <string>

namespace pdal
{





Metadata::Metadata( std::string const& name, 
                    std::string const& ns,
                    pdal::metadata::Type t) 
    : m_name(name)
    , m_namespace(ns) 
    , m_type(t)
{
    return;
}


Metadata::Metadata(const Metadata& other)
    // :
{
    return;
}


Metadata::~Metadata()
{

}

std::vector<std::string> Metadata::getAttributeNames() const
{
    std::vector<std::string> output;
    metadata::MetadataAttributeM::const_iterator i = m_attributes.begin();
    while (i != m_attributes.end())
    {
        output.push_back(i->first);
        ++i;
    }
    
    return output;
}


void Metadata::addAttribute(std::string const& name, std::string const value)
{
    std::pair<std::string, std::string> p(name, value);
    m_attributes.insert(p);
}

std::string Metadata::getAttribute(std::string const& name) const
{
    metadata::MetadataAttributeM::const_iterator p = m_attributes.find(name);
    if (p != m_attributes.end())
        return p->second;
    else
        return std::string("");
}

Metadata& Metadata::operator=(Metadata const& rhs)
{
    if (&rhs != this)
    {
    }
    return *this;
}


bool Metadata::operator==(Metadata const& rhs) const
{
    return false;
}

std::ostream& operator<<(std::ostream& ostr, const Metadata& metadata)
{
    ostr << metadata.getVariant() << std::endl;
    return ostr;
}


} // namespace pdal


namespace std
{

    std::ostream& operator<<(std::ostream& ostr, const pdal::metadata::ByteArray& data)
    {
        
        std::string output = pdal::Utils::base64_encode(data.get());
        
        ostr << output;
        return ostr;
    }
    
}
