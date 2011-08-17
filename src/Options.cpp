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

#include <pdal/Options.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/concept_check.hpp> // ignore_unused_variable_warning
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <sstream>
#include <iostream>


namespace pdal
{


Options::Options(const Options& rhs)
    : m_tree(rhs.getPTree())
{
    return;
}


Options::Options(boost::property_tree::ptree t) 
    : m_tree(t)
{
    return;
}


Options::Options(const Option<std::string>& opt1)
{
    add(opt1);
    return;
}


Options::Options(const Option<std::string>& opt1, const Option<std::string>& opt2)
{
    add(opt1);
    add(opt2);
    return;
}


Options::Options(const Option<std::string>& opt1, const Option<std::string>& opt2, const Option<std::string>& opt3)
{
    add(opt1);
    add(opt2);
    add(opt3);
    return;
}


Options::Options(const Option<std::string>& opt1, const Option<std::string>& opt2, const Option<std::string>& opt3, const Option<std::string>& opt4)
{
    add(opt1);
    add(opt2);
    add(opt3);
    add(opt4);
    return;
}


Options::Options(std::istream& istr)
{
    boost::property_tree::xml_parser::read_xml(istr, m_tree);

#if DEBUG
    boost::property_tree::ptree::const_iterator iter = m_tree.begin();
    while (iter != m_tree.end())
    {
        std::string g = (*iter).first;
        assert(g == "Option");
        boost::property_tree::ptree h = (*iter).second;
        Option<std::string> hopt(h);
        ++iter;
    }
#endif

    return;
}


// the empty options set
static const Options s_none;
const Options& Options::none()
{
    return s_none;
}


const boost::property_tree::ptree Options::getOptionPTree(const std::string& name) const
{
    using boost::property_tree::ptree;

    BOOST_FOREACH(ptree::value_type v, m_tree)
    {
        if (v.first == "Option")
        {
            // v.second is <option><name>..</name><value>..</value><desc>..</desc></option>
            const std::string s = v.second.get_child("Name").get_value<std::string>();
            if (name == s)
            {
                return v.second;
            }
        }
    }

    throw option_not_found(name);
}


const boost::property_tree::ptree& Options::getPTree() const
{
    return m_tree;
}


void Options::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& ostr, const Options& options)
{
    const boost::property_tree::ptree& tree = options.getPTree();

    boost::property_tree::write_json(ostr, tree);

    return ostr;
}


} // namespace pdal
