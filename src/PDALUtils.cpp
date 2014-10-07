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
namespace utils
{
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
