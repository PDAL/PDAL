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
#include <boost/concept_check.hpp> // ignore_unused_variable_warning
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <sstream>
#include <iostream>

#include <pdal/exceptions.hpp>


namespace pdal
{

OptionsOld::OptionsOld()
{
    m_tree.put("is3d", false);

}    


Options::Options(boost::property_tree::ptree t)
{
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, t.get_child("option"))
    {
        v;
    }


    //boost::property_tree::ptree tt = t.get_value<std::string>("option");
    
    //Option<std::string> opt = t.get<std::string>("option");

    std::string name = t.get<std::string>("name");
    std::string value = t.get<std::string>("value");
    std::string description = t.get<std::string>("description");
}


Options::Options(std::istream& istr)
{
    boost::property_tree::xml_parser::read_xml(istr, m_tree);


    boost::property_tree::ptree::const_iterator iter = m_tree.begin();
    while (iter != m_tree.end())
    {
        std::string g = (*iter).first;
        assert(g == "option");
        boost::property_tree::ptree h = (*iter).second;
        Option<std::string> hopt(h);
        ++iter;
    }


    return;
}


static Options s_noOptions;
const Options& Options::none()
{
    return s_noOptions;
}


const boost::property_tree::ptree Options::getOptionPTree(const std::string& name) const
{
    using boost::property_tree::ptree;

    BOOST_FOREACH(ptree::value_type v, m_tree)
    {
        if (v.first == "option")
        {
            // v.second is <option><name>..</name><value>..</value><desc>..</desc></option>
            const std::string s = v.second.get_child("name").get_value<std::string>();
            if (name == s)
            {
                return v.second;
            }
        }
    }

    throw option_not_found(name);
}



boost::property_tree::ptree const& Options::getPTree() const
{
    return m_tree;
}



std::ostream& operator<<(std::ostream& ostr, const OptionsOld& /*options*/)
{
//     ostr << "  Num points: " << stage.getNumPoints() << std::endl;
// 
//     ostr << "  Bounds:" << std::endl;
//     ostr << "    " << stage.getBounds() << std::endl;
// 
//     ostr << "  Schema: " << std::endl;
//     ostr << "    Num dims: " << stage.getSchema().getDimensions().size() << std::endl;
// //    ostr << "    Size in bytes: " << header.getSchema().getByteSize() << std::endl;
// 
//     ostr << "  Spatial Reference:" << std::endl;
//     ostr << "    WKT: " << stage.getSpatialReference().getWKT() << std::endl;

    return ostr;
}


} // namespace pdal
