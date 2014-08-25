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
        o << "  " << m_name << ":" << std::endl;
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

    o << indent << "\"description\":\"" << m_descrip << "\"," << std::endl;
    o << indent << "\"type\":\"" << m_type << "\"," << std::endl;
    o << indent << "\"value\":\"" << m_value << "\"";
    if (m_subnodes.size())
        o << ",";
    o << std::endl;
    subnodesToJSON(o, level);
}


void MetadataNodeImpl::subnodesToJSON(std::ostream& o, int level) const
{
    std::string indent(level * 2, ' ');

    std::map<std::string, MetadataImplList> nodes;
    for (auto mi = m_subnodes.begin(); mi != m_subnodes.end(); ++mi)
    {
        MetadataNodeImplPtr mp = *mi;
        nodes[mp->m_name].push_back(mp);
    }
    
    for (auto ni = nodes.begin(); ni != nodes.end(); ++ni)
    {
        MetadataImplList& subnodes = ni->second;
        if (subnodes.size() > 1)
        {
            o << indent << "\"" << ni->first << "\": [" << std::endl;
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
            o << indent << "\"" << ni->first << "\":" << std::endl;
            o << indent << "{" << std::endl;
            MetadataNodeImplPtr node = *subnodes.begin();
            node->toJSON(o, level + 1);
            o << indent << "}";
        }
        auto nii = ni;
        nii++;
        if (nii != nodes.end())
            o << ",";
        o << std::endl;
    }
}


boost::property_tree::ptree MetadataNodeImpl::toPTree() const
{
    using namespace boost::property_tree;
    typedef ptree::path_type path;
        
    ptree tree;
    tree.put("name", m_name);
    tree.put("description", m_descrip);
    tree.put("type", m_type);
    tree.put("value", m_value);
    
    typedef std::map<std::string, uint32_t>::iterator NameIterator;
    std::map<std::string, uint32_t> names;
    
    for (auto mi = m_subnodes.begin(); mi != m_subnodes.end(); ++mi)
    {
        std::string name = (*mi)->m_name;
        NameIterator i = names.find(name);
        if (i == names.end())
            names.insert(std::make_pair(name, 1));
        else
            i->second++;
    }

    for (auto mi = m_subnodes.begin(); mi != m_subnodes.end(); ++mi)
    {
        std::string name = (*mi)->m_name;
        boost::property_tree::ptree const& node = (*mi)->toPTree();        
        NameIterator cnt = names.find(name);

        if (cnt->second > 1 && name.size())
        {
            std::string plural(name);

            boost::optional<ptree&> opt = tree.get_child_optional(path(plural, '/'));
            if (opt)
            {
                opt->push_back(std::make_pair("", node));               
            } else
            {
                tree.push_back(ptree::value_type(plural, ptree()));
                auto& p = tree.get_child(path(plural, '/'));
                p.push_back(std::make_pair("", node));
                
            }

        } else
        {
            if (name.size())
                tree.push_back(std::make_pair(name, node));

        }
    }    
    return tree;
}

}
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


