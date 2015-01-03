/******************************************************************************
* Copyright (c) 2014, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/PDALUtils.hpp>

using namespace std;

namespace pdal
{

namespace {

void toJSON(const MetadataNode& m, std::ostream& o, int level);

void subnodesToJSON(const MetadataNode& parent, std::ostream& o, int level)
{
    const std::string indent(level * 2, ' ');

    std::vector<std::string> names = parent.childNames();

    for (auto ni = names.begin(); ni != names.end(); ++ni)
    {
        MetadataNodeList children = parent.children(*ni);

        MetadataNode& node = *children.begin();
        if (node.kind() == MetadataType::Array)
        {
            o << indent << "\"" << node.name() << "\": [" << std::endl;
            for (auto ci = children.begin(); ci != children.end(); ++ci)
            {
                MetadataNode& m = *ci;

                o << indent << "{" << std::endl;
                toJSON(m, o, level + 1);
                o << indent << "}";
                if (ci != children.rbegin().base() - 1)
                    o << ",";
                o << std::endl;
            }
            o << indent << "]";
        }
        else
        {
            o << indent << "\"" << node.name() << "\":" << std::endl;
            o << indent << "{" << std::endl;
            toJSON(node, o, level + 1);
            o << indent << "}";
        }
        if (ni != names.rbegin().base() - 1)
            o << ",";
        o << std::endl;
    }
}

void toJSON(const MetadataNode& m, std::ostream& o, int level)
{
    std::string indent(level * 2, ' ');
    std::string description(Utils::escapeJSON(m.description()));
    std::string value(Utils::escapeJSON(m.value()));
    std::string type(m.type());
    MetadataNodeList children = m.children();

    if (description.size())
    {
        o << indent << "\"description\":\"" << description << "\"";
        if (type.size() || value.size() || children.size())
            o << ",";
        o << std::endl;
    }

    if (type.size())
    {
        o << indent << "\"type\":\"" << type << "\"";
        if (value.size() || children.size())
            o << ",";
        o << std::endl;
    }

    if (value.size())
    {
        o << indent << "\"value\":\"" << value << "\"";
        if (m.hasChildren())
            o << ",";
        o << std::endl;
    }
    subnodesToJSON(m, o, level);
}

} // unnamed namespace

namespace utils
{

std::string toJSON(const MetadataNode& m)
{
    std::ostringstream o;

    toJSON(m, o);
    return o.str();
}

void toJSON(const MetadataNode& m, std::ostream& o)
{
    o << "{" << std::endl;

    if (m.name().empty())
        subnodesToJSON(m, o, 1);
    else
    {
        o << "  \"" << m.name() << "\":" << std::endl;
        o << "  {" << std::endl;
        pdal::toJSON(m, o, 2);
        o << "  }" << std::endl;
    }

    o << "}" << std::endl;
}

namespace reST
{
    
using namespace boost::property_tree;

static std::string indent(int level)
{
    std::string s;
    for (int i=0; i<level; i++) s += "    ";
    return s;
}


void write_rst(std::ostream& ost,
               const boost::property_tree::ptree& pt,
               int level)
{
    using boost::property_tree::ptree;

    if (pt.empty())
    {
        ost << pt.data();
        ost << endl << endl;
    }
    else
    {
        if (level) ost << endl << endl;
        for (ptree::const_iterator pos = pt.begin(); pos != pt.end();)
        {
            ost << indent(level+1) << "- " << pos->first << ": ";
            write_rst(ost, pos->second, level + 1);
            ++pos;
            //ost << endl << endl;
        }
    }
}


std::ostream& toRST(const ptree& pt, std::ostream& os)
{
    write_rst(os, pt);
    return os;
}

} // namespace reST
} // namespace utils
} // namespace pdal
