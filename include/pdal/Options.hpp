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
#include <pdal/exceptions.hpp>
#include <pdal/Bounds.hpp>
#include <boost/property_tree/ptree.hpp>


namespace pdal
{

    
class PDAL_DLL OptionsOld
{
private:
    boost::property_tree::ptree m_tree;

public:
    OptionsOld()
    {
        m_tree.put("is3d", false);
    }

    OptionsOld(boost::property_tree::ptree const& tree) { m_tree = tree; }
    boost::property_tree::ptree& GetPTree() { return m_tree; }
    boost::property_tree::ptree const& GetPTree() const { return m_tree; }
};


// An Option is just a record with three fields: name, value, and description.
//
// Dumped as XML, it looks like this:
//     <?xml...>
//     <Name>myname</Name>
//     <Description>my descr</Description>
//     <Value>17</Value>
// although of course that's not valid XML, since it has no single root element.

template <typename T>
class PDAL_DLL Option
{
public:
    // construct it manually
    Option(std::string const& name, T value, std::string const& description="")
        : m_name(name)
        , m_description(description)
        , m_value(value)
    {}
    
    // construct it from an xml stream
    Option(std::istream& istr)
    {
        boost::property_tree::ptree t;
        boost::property_tree::xml_parser::read_xml(istr, t);
        m_name = t.get<std::string>("Name");
        m_value = t.get<T>("Value");
        m_description = t.get<std::string>("Description");
    }

    // construct it from a ptree
    Option(const boost::property_tree::ptree t)
    {
        m_name = t.get<std::string>("Name");
        m_value = t.get<T>("Value");
        m_description = t.get<std::string>("Description");
    }

    // getters
    std::string const& getName() const { return m_name; }
    std::string const& getDescription() const { return m_description; }
    T const& getValue() const { return m_value; }
    
    // return a ptree representation
    boost::property_tree::ptree getPTree() const
    {
        boost::property_tree::ptree t;
        t.put("Name", getName());
        t.put("Description", getDescription());
        t.put("Value", getValue());
        return t;
    }
    
    // return an xml representation
    void write_xml(std::ostream& ostr) const
    {
        const boost::property_tree::ptree tree = getPTree();
        boost::property_tree::xml_parser::write_xml(ostr, tree);
        return;
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
//       <name>myname</name>
//       <description>my descr</description>
//       <value>17</value>
//     </option>
//     <option>
//       <name>myname2</name>
//       <description>my descr2</description>
//       <value>13</value>
//     </option>
// although of course that's not valid XML, since it has no single root element.
class PDAL_DLL Options
{
public:
    // defult ctor, empy options list
    Options() {}

    Options(boost::property_tree::ptree t);

    // read options from an xml stream
    Options(std::istream& istr);

    // convenience ctor, which adds 1 option
    template<class T>
    Options(const std::string& name, T value, const std::string& description="")
    {
        Option<T> opt(name, value, description);
        add(opt);
    }

    // add an option
    template<class T> void add(Option<T> const& option)
    {
       boost::property_tree::ptree fields = option.getPTree();
       m_tree.add_child("Option", fields);
    }

    // add an option (shortcut version, bypass need for an Option object)
    template<class T> void add(const std::string& name, T value, const std::string& description="")
    {
        Option<T> opt(name, value, description);
        add(opt);
    }

    // get an option, by name
    // throws pdal::option_not_found if the option name is not valid
    template<typename T> Option<T> getOption(std::string const& name) const
    {
        boost::property_tree::ptree::const_iterator iter = m_tree.begin();
        while (iter != m_tree.end())
        {
            std::string oname = (*iter).first;
            if (oname == "Option")
            {
                boost::property_tree::ptree optionTree = (*iter).second;
                Option<T> option(optionTree);
                if (option.getName() == name)
                    return option;
            }
            ++iter;
        }
        throw option_not_found(name);
    }

    // returns true iff the option name is valid
    template<typename T> bool hasOption(std::string const& name) const
    {
        try
        {
            Option<T> option = getOption<T>(name);
        }
        catch (option_not_found&)
        {
            return false;
        }
        return true;
    }

    // get the ptree for the whole option block
    const boost::property_tree::ptree& getPTree() const;
   
    // the empty options list
    static const Options empty;

private:
    // get the ptree for an option
    // throws pdal::option_not_found if the option name is not valid
    const boost::property_tree::ptree getOptionPTree(const std::string& name) const;

    boost::property_tree::ptree m_tree;
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const OptionsOld&);


} // namespace pdal

#endif
