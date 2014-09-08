/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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
#include <string>
#include <map>

#include <boost/property_tree/xml_parser.hpp>

namespace pdal
{

std::string MetadataNodeImpl::toJSON() const
{
    std::ostringstream o;

    o << "{" << std::endl;
    if (m_name.empty())
        subnodesToJSON(o, 1);
    else
    {
        o << "  \"" << m_name << "\":" << std::endl;
        o << "  {" << std::endl;
        toJSON(o, 2);
        o << "  }" << std::endl;
    }
    o << "}" << std::endl;
    return o.str();
}


void MetadataNodeImpl::toJSON(std::ostream& o, int level) const
{
    std::string indent(level * 2, ' ');

    std::string escaped_description(m_descrip);
    escaped_description = Utils::escapeJSON(escaped_description);
    std::string escaped_value(m_value);
    escaped_value = Utils::escapeJSON(escaped_value);

    o << indent << "\"description\":\"" << escaped_description << "\"," << std::endl;
    o << indent << "\"type\":\"" << m_type << "\"," << std::endl;
    o << indent << "\"value\":\"" << escaped_value << "\"";

    if (m_subnodes.size())
        o << ",";
    o << std::endl;
    subnodesToJSON(o, level);
}


void MetadataNodeImpl::subnodesToJSON(std::ostream& o, int level) const
{
    std::string indent(level * 2, ' ');

    for (auto si = m_subnodes.begin(); si != m_subnodes.end(); ++si)
    {
        const MetadataImplList& subnodes = si->second;
        if (subnodes.empty())
            continue;
        MetadataNodeImplPtr node = *subnodes.begin();
        if (node->m_kind == MetadataType::Array)
        {
            o << indent << "\"" << node->m_name << "\": [" << std::endl;
            for (auto si = subnodes.begin(); si != subnodes.end(); ++si)
            {
                MetadataNodeImplPtr node = *si;

                o << indent << "{" << std::endl;
                node->toJSON(o, level + 1);
                o << indent << "}";
                if (si != subnodes.rbegin().base() - 1)
                    o << ",";
                o << std::endl;
            }
            o << indent << "]";
        }
        else
        {
            o << indent << "\"" << node->m_name << "\":" << std::endl;
            o << indent << "{" << std::endl;
            node->toJSON(o, level + 1);
            o << indent << "}";
        }
        auto sii = si;
        sii++;
        if (sii != m_subnodes.end() && sii->second.size())
            o << ",";
        o << std::endl;
    }
}


std::string MetadataNode::toJSON() const
    { return m_impl->toJSON(); }


} // close namespace


namespace std
{

std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& data)
{
    ostr << pdal::Utils::base64_encode(data.get());
    return ostr;
}

std::istream& operator>>(std::istream& istr, pdal::ByteArray& output)
{
    std::string data;
    istr >> data;
    output.set(pdal::Utils::base64_decode(data));
    return istr;
}

} // namespace std;


