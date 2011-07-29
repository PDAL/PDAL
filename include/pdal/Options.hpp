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
#include <boost/property_tree/xml_parser.hpp>


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
//     <Value>17</Value>
//     <Description>my descr</Description>
// although of course that's not valid XML, since it has no single root element.

template <typename T>
class PDAL_DLL Option
{
public:
    // construct it manually
    Option(std::string const& name, T value, std::string const& description="")
        : m_name(name)
        , m_value(value)
        , m_description(description)
    {}

    // construct it from an xml stream
    Option(std::istream& istr)
    {
        boost::property_tree::ptree t;
        boost::property_tree::xml_parser::read_xml(istr, t);
        m_name = t.get<std::string>("Name");
        m_value = t.get<T>("Value");
        m_description = t.get<std::string>("Description", "");
    }

    // construct it from a ptree
    Option(const boost::property_tree::ptree t)
    {
        m_name = t.get<std::string>("Name");
        m_value = t.get<T>("Value");
        m_description = t.get<std::string>("Description", "");
    }

    // getters
    std::string const& getName() const { return m_name; }
    T const& getValue() const { return m_value; }
    std::string const& getDescription() const { return m_description; }
    
    // return a ptree representation
    boost::property_tree::ptree getPTree() const
    {
        boost::property_tree::ptree t;
        t.put("Name", getName());
        t.put("Value", getValue());
        if (getDescription() != "")
        {
            t.put("Description", getDescription());
        }
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
    T m_value;
    std::string m_description; // optional field
};




// An Options object is just a tree of Option objects.
//
// Dumped as XML, an Options object with two Option objects looks like this:
//     <?xml...>
//     <Option>
//       <Name>myname</name>
//       <Value>17</value>
//       <Description>my descr</description>
//     </Option>
//     <Option>
//       <Name>myname2</name>
//       <Value>13</value>
//       <Description>my descr2</description>
//     </Option>
// although of course that's not valid XML, since it has no single root element.
class PDAL_DLL Options
{
public:
    // defult ctor, empy options list
    Options() {}

    // what's a better way to do this?
    Options(const Option<std::string>&);
    Options(const Option<std::string>&, const Option<std::string>&);
    Options(const Option<std::string>&, const Option<std::string>&, const Option<std::string>&);
    Options(const Option<std::string>&, const Option<std::string>&, const Option<std::string>&, const Option<std::string>&);

    // initialize from a property tree
    Options(boost::property_tree::ptree t);

    // read options from an xml stream
    Options(std::istream& istr);

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
            if (iter->first != "Option")
                throw pdal_error("malformed Options ptree");

            const boost::property_tree::ptree& optionTree = iter->second;
            if (optionTree.get_child("Name").get_value<std::string>() == name)
            {
                Option<T> option(optionTree);
                return option;
            }
            ++iter;
        }
        throw option_not_found(name);
    }

    // get value of an option, or throw option_not_found if option not present
    template<typename T> T getValueOrThrow(std::string const& name) const
    {
        Option<T> opt = getOption<T>(name);  // might throw
        return opt.getValue();
    }

    // get value of an option, or use given default if option not present
    template<typename T> T getValueOrDefault(std::string const& name, T defaultValue) const
    {
        T result;

        try
        {
            Option<T> opt = getOption<T>(name);  // might throw
            result = opt.getValue();
        }
        catch (option_not_found)
        {
           result = defaultValue;
        }
        
        return result;
    }

    // returns true iff the option name is valid
    template<typename T> bool hasOption(std::string const& name) const
    {
        bool ok = false;

        try
        {
            Option<T> option = getOption<T>(name);
            ok = true;
        }
        catch (option_not_found&)
        {
            ok = false;
        }
        // any other exception will bubble up

        return ok;
    }

    // get the ptree for the whole option block
    const boost::property_tree::ptree& getPTree() const;
   
    // the empty options list
    // BUG: this should be a member variable, not a function, but doing so causes vs2010 to fail to link
    static const Options& none();

private:
    // get the ptree for an option
    // throws pdal::option_not_found if the option name is not valid
    const boost::property_tree::ptree getOptionPTree(const std::string& name) const;

    boost::property_tree::ptree m_tree;
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const OptionsOld&);


} // namespace pdal

#endif
