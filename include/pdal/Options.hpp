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

#ifndef INCLUDED_OPTIONS_HPP
#define INCLUDED_OPTIONS_HPP

#include <pdal/pdal.hpp>
#include <pdal/Bounds.hpp>
#include <boost/property_tree/ptree.hpp>


namespace pdal
{

    
class PDAL_DLL OptionsOld
{

private:
    boost::property_tree::ptree m_tree;

public:

    OptionsOld();
    OptionsOld(boost::property_tree::ptree const& tree) { m_tree = tree; }
    //template<class T> void add(Option<T> const& option) { m_tree.put_child(option.getTree()); }
    //template<class T> Option<T> const& get(std::string const& name) { return m_tree.get<T>(name); }
    
    boost::property_tree::ptree& GetPTree() { return m_tree; }
    boost::property_tree::ptree const& GetPTree() const { return m_tree; }
};


// An Option is just a record with three fields: name, value, and description.
//
// Dumped as XML, it looks like this:
//     <?xml...>
//     <name>my name</name>
//     <description>my descr</description>
//     <value>17</value>
// although of course that's not valid XML, since it has no single root element.

template <typename T>
class PDAL_DLL Option
{
public:
    Option(std::string const& name, T value, std::string const& description) 
        : m_name(name)
        , m_description(description)
        , m_value(value)
    {}
    
    std::string const& getName() const { return m_name; }
    std::string const& getDescription() const { return m_description; }
    T const& getValue() const { return m_value; }
    
    boost::property_tree::ptree getPTree() const 
    {
        boost::property_tree::ptree t;
        t.put("name", getName());
        t.put("description", getDescription());
        t.put("value", getValue());
        return t;
    }
    
private:
    std::string m_name;
    std::string m_description;
    T m_value;
};




// An Options object is just a tree of Option objects.
//
// Dumped as XML, an Options object with two Option objects looks like this:
//     <?xml...>
//     <option>
//       <name>my name</name>
//       <description>my descr</description>
//       <value>17</value>
//     </option>
//     <option>
//       <name>my name2</name>
//       <description>my descr2</description>
//       <value>13</value>
//     </option>
// although of course that's not valid XML, since it has no single root element.
class PDAL_DLL Options
{
public:
    Options();

    // add an option
    template<class T> void add(Option<T> const& option)
    {
       boost::property_tree::ptree fields = option.getPTree();
       m_tree.add_child("option", fields);
    }

    // add an option (shortcut version, bypass need for an Option object)
    template<class T> void add(const std::string& name, T value, const std::string& description)
    {
        Option<T> opt(name, value, description);
        add(opt);
    }

    // get value of an option
    template<class T> T getValue(std::string const& name) const
    {
        boost::property_tree::ptree optionTree = getOptionPTree(name);
        return optionTree.get_child("value").get_value<T>();
    }

    // get description of an option
    std::string getDescription(std::string const& name) const;
    
    boost::property_tree::ptree const& getPTree() const;
   
    // return the empty options list
    static const Options& none();

private:
    boost::property_tree::ptree getOptionPTree(std::string const& name) const;

    boost::property_tree::ptree m_tree;
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const OptionsOld&);


} // namespace pdal

#endif
