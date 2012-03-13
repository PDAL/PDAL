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

#include <pdal/pdal_internal.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include <map>
#include <vector>

namespace pdal
{

class Options;
class Option;

typedef std::multimap<std::string, Option> map_t;
typedef boost::shared_ptr<Options> OptionsPtr;

inline const boost::property_tree::ptree &empty_ptree()
{
    static boost::property_tree::ptree pt;
    return pt;
}

// An Option is just a record with three fields: name, value, and description.
//
// The value is stored as a string, and we rely on boost::lexical_cast to do the
// serialization magic for us.  This means that an object you insert as an option
// value must satisfay the requirements for being lexically-castable:
//    - copy ctor
//    - operator=
//    - operator>>
//    - operator<<
//
// Also, you should avoid using int8/uint8 types, as these tend to get confused
// with chars.
//
// Dumped as XML, it looks like this:
//     <?xml...>
//     <Name>myname</Name>
//     <Value>17</Value>
//     <Description>my descr</Description>
// although of course that's not valid XML, since it has no single root element.

class PDAL_DLL Option
{
public:
    Option()
        : m_name("")
        , m_value("")
        , m_description("")
    {
        return;
    }

    template <typename T>
    Option(std::string const& name, const T& value, std::string const& description="")
        : m_name(name)
        , m_value("")
        , m_description(description)
    {
        setValue<T>(value);
        return;
    }

    // copy ctor
    Option(const Option& rhs)
        : m_name(rhs.m_name)
        , m_value(rhs.m_value)
        , m_description(rhs.m_description)
        , m_options(rhs.m_options)
    {
        return;
    }

    // build an Option from a ptree
    Option(const boost::property_tree::ptree& tree);

    // assignment operator
    Option& operator=(const Option& rhs)
    {
        if (&rhs != this)
        {
            m_name = rhs.m_name;
            m_value = rhs.m_value;
            m_description = rhs.m_description;
            m_options = rhs.m_options;
        }
        return *this;
    }

    bool equals(const Option& rhs) const
    {
        if (m_name == rhs.getName() && 
            m_value == rhs.getValue<std::string>() && 
            m_description == rhs.getDescription() && m_options == rhs.m_options)
        {
            return true;
        }
        return false;
    }

    bool operator==(const Option& rhs) const
    {
        return equals(rhs);
    }

    bool operator!=(const Option& rhs) const
    {
        return (!equals(rhs));
    }

    void setName(const std::string& name) { m_name = name; }
    const std::string& getName() const { return m_name; }

    void setDescription(const std::string& description) { m_description = description; }
    const std::string& getDescription() const { return m_description; }
    
    // get the value of an option
    template<typename T> T getValue() const
    { 
        return boost::lexical_cast<T>(m_value);
    }

    // set the value of the option
    template<typename T> void setValue(const T& value)
    { 
        m_value = boost::lexical_cast<std::string>(value);
    }
    
    boost::optional<Options const&> getOptions() const;
    void setOptions(Options const& op);
    
#if defined(PDAL_COMPILER_MSVC)
    // explicit specialization:
    //   boost::lexical_cast only understands "0" and "1" for bools,
    //   so we handle those situations explicitly
    template<> bool getValue() const 
    { 
        if (m_value=="true") return true;
        if (m_value=="false") return false;
        return boost::lexical_cast<bool>(m_value);
    }


    // explicit specialization:
    //   if we want to get out a (const ref) string, we don't need lexical_cast
    template<> const std::string& getValue() const 
    { 
        return m_value;
    }


    // explicit specialization:
    //   if insert a bool, we don't want it to be "0" or "1" (which is
    //   what lexical_cast would do)
    template<> void setValue(const bool& value)
    { 
        m_value = value ? "true" : "false";
    }


    // explicit specialization:
    //   if we want to insert a string, we don't need lexical_cast
    template<> void setValue(const std::string& value)
    { 
        m_value = value;
    }
#endif

    // return a ptree representation
    boost::property_tree::ptree toPTree() const;

private:
    std::string m_name;
    std::string m_value;
    std::string m_description; // optional field
    OptionsPtr m_options; // any other Option instances this field may contain
};


#if !defined(PDAL_COMPILER_VC10)
// explicit specialization:
//   boost::lexical_cast only understands "0" and "1" for bools,
//   so we handle those situations explicitly
template<> bool Option::getValue() const;

// explicit specialization:
//   if we want to get out a (const ref) string, we don't need lexical_cast
template<> const std::string& Option::getValue() const;

// explicit specialization:
//   if insert a bool, we don't want it to be "0" or "1" (which is
//   what lexical_cast would do)
template<> void Option::setValue(const bool& value);

// explicit specialization:
//   if we want to insert a string, we don't need lexical_cast
template<> void Option::setValue(const std::string& value);
#endif

// An Options object is just a map of names to Option objects.
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

    // copy ctor
    Options(const Options&);

    // assignment operator
    Options& operator=(const Options& rhs)
    {
        if (&rhs != this)
        {
            m_options = rhs.m_options;
        }
        return *this;
    }

    // assignment operator
    Options& operator+=(const Options& rhs)
    {

        if (&rhs != this)
        {
            map_t::const_iterator i;
            for (i = rhs.m_options.begin(); i != rhs.m_options.end(); ++i)
            {
                m_options.insert(std::pair<std::string, Option>(i->first, i->second));
            }

        }
        return *this;
    }

    Options const operator+(const Options& rhs)
    {
        return Options(*this) += rhs;
    }

    bool equals(const Options& rhs) const
    {
        if (m_options== rhs.m_options)
        {
            return true;
        }
        return false;
    }

    bool operator==(const Options& rhs) const
    {
        return equals(rhs);
    }

    bool operator!=(const Options& rhs) const
    {
        return (!equals(rhs));
    }

    Options(const Option&);

    Options(const boost::property_tree::ptree& tree);

    // add an option
    void add(const Option& option);

    // if option name not present, just returns
    void remove(const std::string& name);

    // add an option (shortcut version, bypass need for an Option object)
    template<typename T> void add(const std::string& name, T value, const std::string& description="")
    {
        Option opt(name, value, description);
        add(opt);
    }

    // get an option, by name
    // throws pdal::option_not_found if the option name is not valid
    const Option& getOption(const std::string & name) const;
    Option& getOptionByRef(const std::string& name);

    // get value of an option, or throw option_not_found if option not present
    template<typename T> T getValueOrThrow(std::string const& name) const
    {
        const Option& opt = getOption(name);  // might throw
        return opt.getValue<T>();
    }

    // get value of an option, or use given default if option not present
    template<typename T> T getValueOrDefault(std::string const& name, T defaultValue) const
    {
        T result;

        try
        {
            const Option& opt = getOption(name);  // might throw
            result = opt.getValue<T>();
        }
        catch (option_not_found)
        {
           result = defaultValue;
        }
        
        return result;
    }

    // returns true iff the option name is valid
    bool hasOption(std::string const& name) const;

    // returns a ptree for the whole option block
    boost::property_tree::ptree toPTree() const;
   
    // the empty options list
    // BUG: this should be a member variable, not a function, but doing so causes vs2010 to fail to link
    static const Options& none();

    void dump() const;

    std::vector<Option> getOptions(std::string const& name) const;
private:
    map_t m_options;
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Options&);


} // namespace pdal

#endif
