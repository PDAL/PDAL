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
typedef std::shared_ptr<Options> OptionsPtr;
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

    /// Empty constructor
    Option() : m_options(0)
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

/// @name Equality
    /// Equality
    bool equals(const Option& rhs) const
    {
        return (m_name == rhs.getName() &&
            m_value == rhs.getValue<std::string>() &&
            m_description == rhs.getDescription() &&
            m_options == rhs.m_options);
    }

    /// Equality operator
    bool operator==(const Option& rhs) const
    {
        return equals(rhs);
    }

    /// Inequality operator
    bool operator!=(const Option& rhs) const
    {
        return (!equals(rhs));
    }

/// @name Attribute access

    /// Overwrites the name given in the constructor
    /// @param name new value to use for the name of the Option
    inline void setName(const std::string& name)
    {
        m_name = name;
    }

    /// @return the name for the Option instance
    inline std::string const& getName() const
    {
        return m_name;
    }

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

    boost::optional<Options const&> getOptions() const;

    /// sets the Options set for this Option instance
    /// @param op Options set to use
    void setOptions(Options const& op);

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
    options::OptionsPtr m_options; // Sub-options.

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

/*!
    \verbatim embed:rst
     An Options object is just a map of names to Option objects.

     Dumped as XML, an Options object with two Option objects looks like this:

     ::

        <?xml...>
        <Option>
          <Name>myname</name>
          <Value>17</value>
          <Description>my descr</description>
        </Option>
        <Option>
          <Name>myname2</name>
          <Value>13</value>
          <Description>my descr2</description>
        </Option>
    \endverbatim
*/
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
    void remove(const std::string& name);

    MetadataNode toMetadata() const
    {
        MetadataNode cur("options");
        std::vector<Option> optList = getOptions();
        for (auto oi = optList.begin(); oi != optList.end(); ++oi)
        {
            Option& opt = *oi;
            opt.toMetadata(cur);
        }
        return cur;
    }

    void toMetadata(MetadataNode& parent) const
    {
        MetadataNode cur = parent.add("options");
        std::vector<Option> optList = getOptions();
        for (auto oi = optList.begin(); oi != optList.end(); ++oi)
        {
            Option& opt = *oi;
            opt.toMetadata(cur);
        }
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
    T getValueOrDefault(std::string const& name, T defaultValue) const
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

    // the empty options list
    // BUG: this should be a member variable, not a function, but doing so
    // causes vs2010 to fail to link
    static const Options& none();

    void dump() const;

    std::vector<Option> getOptions(std::string const& name="") const;


    template<typename T>
    boost::optional<T> getMetadataOption(std::string const& name) const
    {
        // <Reader type="writers.las">
        //     <Option name="metadata">
        //         <Options>
        //             <Option name="dataformatid">
        //             3
        //             </Option>
        //             <Option name="filesourceid">
        //             forward
        //             </Option>
        //             <Option name="year">
        //             forward
        //             </Option>
        //             <Option name="day_of_year">
        //             forward
        //             </Option>
        //             <Option name="vlr">
        //             forward
        //             <Options>
        //                 <Option name="user_id">
        //                 hobu
        //                 </Option>
        //                 <Option name="record">
        //                 1234
        //                 </Option>
        //             </Options>
        //             </Option>
        //         </Options>
        //     </Option>
        // </Reader>

        Option const* doMetadata(0);
        try
        {
            doMetadata = &getOption("metadata");
        }
        catch (Option::not_found)
        {
            return boost::optional<T>();
        }

        boost::optional<Options const&> meta = doMetadata->getOptions();
        if (meta)
        {
            try
            {
                meta->getOption(name);
            }
            catch (Option::not_found)
            {
                return boost::optional<T>();
            }

            return boost::optional<T>(meta->getOption(name).getValue<T>());
        }
        return boost::optional<T>();
    }

private:
    options::map_t m_options;
};



PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Options&);


} // namespace pdal

