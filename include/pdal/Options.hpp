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

#pragma once

#include <pdal/pdal_internal.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/util/Utils.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <vector>

namespace pdal
{

class Options;
class Option;

namespace options
{
typedef std::multimap<std::string, Option> map_t;
}

/*!
    \verbatim embed:rst
     An Option is just a record with three fields: name, value, and description.
     The value is stored as a string, and we rely on boost::lexical_cast to do the
     serialization magic for us.  This means that an object you insert as an option
     value must satisfay the requirements for being lexically-castable:
        - copy ctor
        - operator=
        - operator>>
        - operator<<

     Dumped as XML, it looks like this although of course that's not valid
     XML, since it has no single root element.

     ::

         <?xml...>
         <Name>myname</Name>
         <Value>17</Value>
         <Description>my descr</Description>

     .. warning::

            you should avoid using int8/uint8 types, as these tend to get confused
            with chars.
    \endverbatim
*/

class PDAL_DLL Option
{
public:

    class not_found : public pdal_error
    {
    public:
        not_found() : pdal_error("Option not found.")
        {}
        not_found(const std::string& msg) : pdal_error(msg)
        {}
    };

/// @name Constructors

    Option()
    {}

    /// Primary constructor
    template <typename T>
    Option(std::string const& name, const T& value,
            std::string const& description = "") :
        m_name(name), m_description(description)
    {
        try
        {
            setValue<T>(value);
        }
        catch (boost::bad_lexical_cast)
        {
        }
    }

    /// Construct from an existing boost::property_tree
    Option(const boost::property_tree::ptree& tree);

    /// Equality operator
    bool operator==(const Option& rhs) const
    {
        return (m_name == rhs.getName() &&
            m_value == rhs.getValue<std::string>() &&
            m_description == rhs.getDescription());
    }

    /// Inequality operator
    bool operator!=(const Option& rhs) const
    {
        return (! operator == (rhs));
    }

/// @name Attribute access

    /// @return the name for the Option instance
    std::string const& getName() const
    {
        return m_name;
    }

    static std::string::size_type
    parse(const std::string& name, std::string::size_type p)
    {
        std::string::size_type count = 0;

        if (std::islower(name[p++]))
        {
            count++;

            auto isname = [](char c)
                { return (std::islower(c) || std::isdigit(c) || c == '_'); };
            count += Utils::extract(name, p, isname);
        }
        return count;
    }

    // Make sure that the option name consists of lowercase characters or
    // underscores.
    static bool nameValid(const std::string& name, bool reportError);

    /// Overwrites the description given in the constructor
    /// @param description new value to use for the description of the Option
    inline void setDescription(const std::string& description)
    {
        m_description = description;
    }

    /// @return the description of the Option
    inline std::string const& getDescription() const
    {
        return m_description;
    }

    /// @return the value of the Option as casted by boost::lexical_cast
    template<typename T>
    T getValue() const
    {
        T t;
        getValue(t);
        return t;
    }

    /// sets the value of the Option to T after boost::lexical_cast'ing
    /// it to a std::string
    template<typename T> void setValue(const T& value)
    {
        m_value = boost::lexical_cast<std::string>(value);
    }

/// @name pdal::Options access

    /*! \return a boost::optional-wrapped const& of the Options set for this Option.
    \verbatim embed:rst

    .. note::

            An Option may have an Options map of of
            infinite depth (Option->Options->Option->Options...)
    \endverbatim
    */

    bool empty() const;

#if defined(PDAL_COMPILER_MSVC)
    /// explicit specialization to insert a bool as "true" and "false" rather
    /// than "0" or "1" (which is what lexical_cast would do)
    template<> void setValue(const bool& value)
    {
        m_value = value ? "true" : "false";
    }

    /// explicit specialization to insert a string so no lexical_cast happens
    template<> void setValue(const std::string& value)
    {
        m_value = value;
    }
#endif

    void toMetadata(MetadataNode& parent) const;

/// @name Private attributes
private:
    std::string m_name;
    std::string m_value;
    std::string m_description; // optional field

    template <typename T>
    void getValue(T& t) const
    {
        t = boost::lexical_cast<T>(m_value);
    }

    void getValue(StringList& values) const
    {
        values = Utils::split2(m_value, ',');
        for (std::string& s : values)
            Utils::trim(s);
    }

    void getValue(bool& value) const
    {
        if (m_value == "true")
            value = true;
        else if (m_value == "false")
            value = false;
        else
            value = boost::lexical_cast<bool>(m_value);
    }

    /// Avoid lexical cast.
    void getValue(std::string& value) const
        { value = m_value; }

    void getValue(char& value)
        { value = (char)std::stoi(m_value); }

    void getValue(unsigned char& value) const
        { value = (unsigned char)std::stoi(m_value); }

    void getValue(signed char& value) const
        { value = (signed char)std::stoi(m_value); }
};


/// @name Specializations
#if !defined(PDAL_COMPILER_VC10)

/// explicit specialization to insert a bool as "true" and "false" rather
/// than "0" or "1" (which is what lexical_cast would do)
template<> void Option::setValue(const bool& value);

/// explicit specialization to insert a string so no lexical_cast happens
template<> void Option::setValue(const std::string& value);
#endif

class PDAL_DLL Options
{
public:
    Options()  {}

    // copy ctor
    Options(const Options&);

    // assignment operator
    Options& operator+=(const Options& rhs)
    {
        if (&rhs != this)
        {
            for (auto i = rhs.m_options.begin(); i != rhs.m_options.end(); ++i)
            {
                m_options.insert(std::pair<std::string, Option>(
                    i->first, i->second));
            }
        }
        return *this;
    }

    bool empty() const
    {
        return (m_options.size() == 0);
    }

    size_t size() const
    {
        return m_options.size();
    }

    Options const operator+(const Options& rhs)
    {
        return Options(*this) += rhs;
    }

    bool equals(const Options& rhs) const
    {
        return m_options == rhs.m_options;
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
    void remove(const Option& option);

    void toMetadata(MetadataNode& parent) const
    {
        for (auto o : getOptions())
            o.toMetadata(parent);
    }

    // add an option (shortcut version, bypass need for an Option object)
    template<typename T> void add(const std::string& name, T value,
        const std::string& description="")
    {
        Option opt(name, value, description);
        add(opt);
    }

    // get an option, by name
    // throws not_found if the option name is not valid
    const Option& getOption(const std::string & name) const;
    Option& getOptionByRef(const std::string& name);

    template<typename T>
    std::vector<T> getValues(const std::string& name) const
    {
        std::vector<T> vals;

        auto ops = getOptions(name);
        for (Option& op : ops)
            vals.push_back(op.getValue<T>());
        return vals;
    }

    StringList getValues(const std::string& name) const
    {
        StringList s;

        auto ops = getOptions(name);
        for (Option& op : ops)
        {
            StringList t = op.getValue<StringList>();
            s.insert(s.end(), t.begin(), t.end());
        }
        return s;
    }

    // get value of an option, or throw not_found if option not present
    template<typename T> T getValueOrThrow(std::string const& name) const
    {
        const Option& opt = getOption(name);  // might throw
        return opt.getValue<T>();
    }

    // get value of an option, or use given default if option not present
    template<typename T>
    T getValueOrDefault(std::string const& name, const T& defaultValue) const
    {
        T result;

        try
        {
            const Option& opt = getOption(name);  // might throw
            result = opt.getValue<T>();
        }
        catch (Option::not_found)
        {
            result = defaultValue;
        }
        return result;
    }

    // get value of an option or use a value-initialized default
    template<typename T>
    T getValueOrDefault(std::string const& name) const
    {
        T out;

#if defined(PDAL_COMPILER_MSVC)
        T *t = new T();
        out = getValueOrDefault(name, *t);
        delete t;
#else
        if (std::is_fundamental<T>::value)
        {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
            T t{};
            out = getValueOrDefault(name, t);
#pragma GCC diagnostic pop
        }
        else
        {
            T t;
            out = getValueOrDefault(name, t);
        }
#endif
        return out;
    }

    // returns true iff the option name is valid
    bool hasOption(std::string const& name) const;

    void dump() const;

    std::vector<Option> getOptions(std::string const& name="") const;


private:
    options::map_t m_options;
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Options&);

} // namespace pdal
