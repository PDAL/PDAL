/******************************************************************************
* Copyright (c) 2016, Hobu Inc., hobu@hobu.co
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
*     * Neither the name of Hobu, Inc. Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
****************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <vector>

#include <pdal/util/Utils.hpp>

namespace pdal
{

class arg_error
{
public:
    arg_error(const std::string& error) : m_error(error)
    {}

    std::string what() const
        { return m_error; }

    std::string m_error;
};

// Specifically, an error in the argument's value.
class arg_val_error : public arg_error
{
public:
    arg_val_error(const std::string& error) : arg_error(error)
    {}
};

namespace
{

class ArgValList
{
    struct ArgVal
    {
        std::string m_val;
        bool m_consumed;

        ArgVal(const std::string& s) :
            m_val(s), m_consumed(false)
        {}
    };

public:
    ArgValList(const std::vector<std::string>& slist) : m_unconsumedStart(0)
    {
        for (const std::string& s : slist)
            add(s);
    }

    void add(const std::string& s)
    {
        if (s.empty())
            return;

        // Turn a short arg list into a set of short args: -afv -> -a -f -v
        // so that each argval represents a single arg.
        if (s.size() > 1 && s[0] == '-' && s[1] != '-')
            for (size_t i = 1; i < s.size(); i++)
                m_vals.push_back({std::string("-") + s[i]});
        else
            m_vals.push_back({s});
    }

    void consume(size_t i)
    {
        m_vals[i].m_consumed = true;
        if (i == m_unconsumedStart)
            while (i < m_vals.size() - 1 && consumed(++i))
                m_unconsumedStart++;
    }

    std::vector<std::string> unconsumedArgs() const
    {
        std::vector<std::string> remainingVals;

        for (size_t i = firstUnconsumed(); i < size(); ++i)
            if (!consumed(i))
                remainingVals.push_back(m_vals[i].m_val);
        return remainingVals;
    }

    size_t size() const
        { return m_vals.size(); }
    const std::string& operator[](size_t i) const
        { return m_vals[i].m_val; }
    bool consumed(size_t i) const
        { return m_vals[i].m_consumed; }
    size_t firstUnconsumed() const
        { return m_unconsumedStart; }
private:
    std::vector<ArgVal> m_vals;
    size_t m_unconsumedStart;
};

} // unnamed namespace


/**
   Description of an argument that can be parsed by \class ProgramArgs.

   Stores information about each argument including the required "longname",
   an optional single-character shortname, a description, and an indicator
   of the positional-type of the argument.
*/
class Arg
{
public:
/**
  Positional type.  Either None, Optional or Required.
*/
enum class PosType
{
    None,       ///< Not positional
    Required,   ///< Required positional
    Optional    ///< Optional positional
};

protected:
    /**
      Constructor.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
    */
    Arg(const std::string& longname, const std::string& shortname,
        const std::string& description) : m_longname(longname),
        m_shortname(shortname), m_description(description), m_set(false),
        m_hidden(false), m_positional(PosType::None)
    {}

public:
    virtual ~Arg()
    {}

    /**
      Indicate that the argument shouldn't be shown in help text.

      \param hidden  Whether the argument should be hidden or not
        [default: true].
      \return  A reference to this \class Arg, to allow the function
        call to be chained.
    */
    Arg& setHidden(bool hidden = true)
    {
        m_hidden = true;
        return *this;
    }
    /**
      Indicate that the argument is positional.

      Positional arguments may be specified on the command line without
      any argument name.  Such arguments are required to be specified
      either with the argument name as a normal option or positionally.
      Missing positional arguments will raise an exception when the
      command line is parsed
    */
    virtual Arg& setPositional()
    {
        m_positional = PosType::Required;
        return *this;
    }
    /**
      Indicate that the argument is positional and optional.

      Positional arguments may be specified on the command line without
      any argument name.  Optional positional arguments must be added to
      \class ProgramArgs after any non-optional arguments.  If optional
      positional arguments are not found, no exception is raised when
      the command line is parsed.
    */
    virtual Arg& setOptionalPositional()
    {
        m_positional = PosType::Optional;
        return *this;
    }
    /**
      Provide error text for the argument to override the default.

      \param error  Error text.
    */
    virtual Arg& setErrorText(const std::string& error)
    {
        m_error = error;
        return *this;
    }
    /**
      Return whether the argument was set during command-line parsing.
    */
    bool set() const
        { return m_set; }
    /**
      Return whether a default value was provided for the argument.

      \return  Whether a default was provided.
    */
    virtual bool defaultProvided() const
        { return false; }
    /**
      Return a string representation of an Arg's default value, or an
      empty string if none exists.

      \return  Default value as a string.
    */
    virtual std::string defaultVal() const
        { return std::string(); }

public:
    /**
      Return whether an option needs a value to be valid.  Generally true
      for all options not bound to boolean values.
      \note  Not intended to be called from user code.
    */
    virtual bool needsValue() const
        { return true; }

    /**
      Set a an argument's value from a string.

      Throws an arg_error exception if \a s can't be converted to
      the argument's type.
      \note  Not intended to be called from user code.

      \param s  Value to set.
    */
    virtual void setValue(const std::string& s) = 0;

    /**
      Reset the argument's state.

      Set the internal state of the argument and it's referenced variable
      as if no command-line parsing had occurred.
      \note  For testing.  Not intended to be called from user code.
    */
    virtual void reset() = 0;

    /**
      Set the argument's value from the list of command-line args.
      \note  Not intended to be called from user code.

      \param vals  The list of command-line argument values.
    */
    virtual void assignPositional(ArgValList& vals)
    {}

    /**
      Returns the positional type of the argument.
      \note  Not intended to be called from user code.
    */
    PosType positional() const
        { return m_positional; }

    /**
      Returns whether the argument is hidden or not.
      \note  Not intended to be called from user code.
    */
    bool hidden() const
        { return m_hidden; }

    /**
      Returns the description of the argument.
      \note  Not intended to be called from user code.
      \return  Argument description.
    */
    std::string description() const
        { return m_description; }

    /**
      Returns the longname of the argument.
      \note  Not intended to be called from user code.
      \return  Argument long name.
    */
    std::string longname() const
        { return m_longname; }

    /**
      Returns text indicating the longname and shortname of the option
      suitable for displaying in help information.
      \note  Not intended to be called from user code.
    */
    std::string nameDescrip() const
    {
        std::string s("--");
        s += m_longname;
        if (m_shortname.size())
            s += ", -" + m_shortname;
        return s;
    }
    /**
      Returns text indicating the name of the option suitable for displaying
      in "usage" text.
      \note  Not intended to be called from user code.
    */
    std::string commandLine() const
    {
        std::string s;
        if (m_positional == PosType::Required)
            s =  m_longname;
        else if (m_positional == PosType::Optional)
            s += '[' + m_longname + ']';
        return s;
    }

protected:
    std::string m_longname;
    std::string m_shortname;
    std::string m_description;
    std::string m_rawVal;
    bool m_set;
    bool m_hidden;
    PosType m_positional;
    std::string m_error;
};

/**
  Description of an argument.  Boolean arguments and vector (list-based)
  arguments are handled separately.
*/
template <typename T>
class TArg : public Arg
{
public:
    /**
      Constructor that takes a default argument.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  Variable to which the value of the argument should
        be bound.
      \param def  Default value to be assigned to the bound variable.
    */
    TArg(const std::string& longname, const std::string& shortname,
        const std::string& description, T& variable, T def) :
        Arg(longname, shortname, description), m_var(variable),
        m_defaultVal(def), m_defaultProvided(true)
    { m_var = m_defaultVal; }

    /**
      Constructor.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  Variable to which the value of the argument should
        be bound.
    */
    TArg(const std::string& longname, const std::string& shortname,
        const std::string& description, T& variable) :
        Arg(longname, shortname, description), m_var(variable),
        m_defaultVal(T()), m_defaultProvided(false)
    { m_var = m_defaultVal; }

    /**
      Set a an argument's value from a string.

      Throws an arg_error exception if \a s can't be converted to
      the argument's type.  Values must be provided for with the
      option name.
      \note  Not intended to be called from user code.

      \param s  Value to set.
    */
    virtual void setValue(const std::string& s)
    {
        if (m_set)
        {
            throw arg_val_error("Attempted to set value twice for argument '" +
                m_longname + "'.");
        }
        if (s.empty())
        {
            throw arg_val_error("Argument '" + m_longname +
                "' needs a value and none was provided.");
        }

        m_rawVal = s;
        auto status = Utils::fromString(s, m_var);
        if (!status)
        {
            std::string error(m_error);

            if (error.empty())
            {
                if (status.what().size())
                    error = "Invalid value for argument '" + m_longname +
                        "': " + status.what();
                else
                    error = "Invalid value '" + s + "' for argument '" +
                        m_longname + "'.";
            }
            throw arg_val_error(error);
        }
        m_set = true;
    }

    /**
      Reset the argument's state.

      Set the interval state of the argument and it's referenced variable
      as if no command-line parsing had occurred.
      \note  For testing.  Not intended to be called from user code.
    */
    virtual void reset()
    {
        m_var = m_defaultVal;
        m_set = false;
        m_hidden = false;
    }

    /**
      Set the argument's value from the command-line args.

      If no value is provided for a required positional option, an arg_error
      exception is thrown.
      \note  Not intended to be called from user code.

      \param vals  The list of command-line args.
    */
    virtual void assignPositional(ArgValList& vals)
    {
        if (m_positional == PosType::None || m_set)
            return;
        for (size_t i = vals.firstUnconsumed(); i < vals.size(); ++i)
        {
            const std::string& val = vals[i];
            if ((val.size() && val[0] == '-') || vals.consumed(i))
                continue;
            setValue(val);
            vals.consume(i);
            return;
        }
        if (m_positional == PosType::Required)
            throw arg_error("Missing value for positional argument '" +
                m_longname + "'.");
    }

    /**
      Return whether a default value was provided for the argument.

      \return  Whether a default was provided.
    */
    virtual bool defaultProvided() const
        { return m_defaultProvided; }

    /**
      Return a string representation of an Arg's default value.

      \return  Default value as a string.
    */
    virtual std::string defaultVal() const
        { return Utils::toString(m_defaultVal); }

private:
    T& m_var;
    T m_defaultVal;
    bool m_defaultProvided;
};

/**
  Description of a boolean argument.  Boolean arguments don't take values.
  Setting a boolean argument inverts its default value.  Boolean arguments
  are normally 'false' by default.
*/
template <>
class TArg<bool> : public Arg
{
public:
    /**
      Constructor for boolean arguments with default value.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  bool variable to which the value of the argument should
        be bound.
      \param def  Default value to be assigned to the bound variable.
    */
    TArg(const std::string& longname, const std::string& shortname,
        const std::string& description, bool& variable, bool def) :
        Arg(longname, shortname, description), m_val(variable),
        m_defaultVal(def), m_defaultProvided(true)
    { m_val = m_defaultVal; }

    /**
      Constructor for boolean arguments without default value.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  bool variable to which the value of the argument should
        be bound.
    */
    TArg(const std::string& longname, const std::string& shortname,
        const std::string& description, bool& variable) :
        Arg(longname, shortname, description), m_val(variable),
        m_defaultVal(false), m_defaultProvided(false)
    { m_val = m_defaultVal; }

    /**
      Return whether an option needs a value to be valid.

      \return false  Boolean values don't need a value.
      \note  Not intended to be called from user code.
    */
    virtual bool needsValue() const
        { return false; }

    /**
      Set a an argument's value from a string.

      \note  The argumet is either 'true' or 'false'.  True means that we're
        setting the option, which sets the negative of the default value.
        False sets the option to the default value (essentially a no-op).
      \note  Not intended to be called from user code.

      \param s  Value to set [ignored].
    */
    virtual void setValue(const std::string& s)
    {
        if (s.size() && s[0] == '-')
        {
            throw arg_val_error("Argument '" + m_longname +
                "' needs a value and none was provided.");
        }
        if (s == "invert")
            m_val = !m_defaultVal;
        else if (s == "true")
            m_val = true;
        else
            m_val = false;
        m_set = true;
    }

    /**
      Reset the argument's state.

      Set the internal state of the argument and it's referenced variable
      as if no command-line parsing had occurred.
      \note  For testing.  Not intended to be called from user code.
    */
    virtual void reset()
    {
        m_val = m_defaultVal;
        m_set = false;
        m_hidden = false;
    }

    /**
      Indicate that the argument is positional.

      Throws an exception to indicate that boolean arguments can't
      positional.
    */
    virtual Arg& setPositional()
    {
        throw arg_error("Boolean argument '" + m_longname +
            "' can't be positional.");
        return *this;
    }

    /**
      Indicate that the argument is positional and optional.

      Throws an exception to indicate that boolean arguments can't
      positional.
    */
    virtual Arg& setOptionalPositional()
    {
        throw arg_error("Boolean argument '" + m_longname +
            "' can't be positional.");
        return *this;
    }
    /**
      Return whether a default value was provided for the argument.

      \return  Whether a default was provided.
    */
    virtual bool defaultProvided() const
        { return m_defaultProvided; }
    /**
      Return a string representation of an Arg's default value.

      \return  Default value as a string.
    */
    virtual std::string defaultVal() const
        { return Utils::toString(m_defaultVal); }

private:
    bool& m_val;
    bool m_defaultVal;
    bool m_defaultProvided;
};

/**
  Description of a list-based (vector) argument.  List-based arguments can
  be specified multiple times, taking multiple values.  List-based
  arguments are necessarily bound to variables that are vectors.
  \note  Doesn't properly support list-based boolean values.
*/
class BaseVArg : public Arg
{
public:
    /**
      Constructor.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
    */
    BaseVArg(const std::string& longname, const std::string& shortname,
        const std::string& description) : Arg(longname, shortname, description),
        m_defaultProvided(false)
    {}

    /**
      Set the argument's value from the command-line args.

      List-based arguments consume ALL positional arguments until
      one is found that can't be converted to the type of the bound variable.
      \note  Not intended to be called from user code.

      \param vals  The list of command-line args.
    */
    virtual void assignPositional(ArgValList& vals)
    {
        if (m_positional == PosType::None || m_set)
            return;

        int cnt = 0;
        for (size_t i = vals.firstUnconsumed(); i < vals.size(); ++i)
        {
            const std::string& val = vals[i];
            if ((val.size() && val[0] == '-') || vals.consumed(i))
                continue;
            try
            {
                setValue(val);
                vals.consume(i);
                cnt++;
            }
            catch (arg_error&)
            {
                break;
            }
        }
        if (cnt == 0 && m_positional == PosType::Required)
        {
            throw arg_error("Missing value for positional argument '" +
                m_longname + "'.");
        }
    }

    /**
      Return whether a default value was provided for the argument.

      \return  Whether a default was provided.
    */
    virtual bool defaultProvided() const
        { return m_defaultProvided; }

protected:
    bool m_defaultProvided;
};

/**
  Description of a generic list-based (vector) argument.
  \note  Doesn't properly support list-based boolean values.
*/
template <typename T>
class VArg : public BaseVArg
{
public:
    /**
      Constructor for arguments with default value.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  Variable to which the argument value(s) should be bound.
      \param def  Default value.
    */
    VArg(const std::string& longname, const std::string& shortname,
        const std::string& description, std::vector<T>& variable,
        std::vector<T> def) :
        BaseVArg(longname, shortname, description), m_var(variable),
        m_defaultVal(def)
    {
        m_var = def;
        m_defaultProvided = true;
    }

    /**
      Constructor for arguments without default value.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  Variable to which the argument value(s) should be bound.
    */
    VArg(const std::string& longname, const std::string& shortname,
        const std::string& description, std::vector<T>& variable) :
        BaseVArg(longname, shortname, description), m_var(variable)
    {
        // Clearing the vector resets to "default" value.
        m_var.clear();
    }

    /**
      Set a an argument's value from a string.

      Throws an arg_error exception if \a s can't be converted to
      the argument's type.
      \note  Not intended to be called from user code.

      \param s  Value to set.
    */
    virtual void setValue(const std::string& s)
    {
        T var;

        m_rawVal = s;
        auto status = Utils::fromString(s, var);
        if (!status)
        {
            std::string error(m_error);

            if (error.empty())
            {
                if (status.what().size())
                    error = "Invalid value for argument '" + m_longname +
                        "': " + status.what();
                else
                    error = "Invalid value '" + s + "' for argument '" +
                        m_longname + "'.";
            }
            throw arg_val_error(error);
        }
        if (!m_set)
            m_var.clear();
        m_var.push_back(std::move(var));
        m_set = true;
    }

    /**
      Reset the argument's state.

      Set the internal state of the argument and it's referenced variable
      as if no command-line parsing had occurred.
      \note  For testing.  Not intended to be called from user code.
    */
    virtual void reset()
    {
        m_var = m_defaultVal;
        m_set = false;
        m_hidden = false;
    }

    /**
      Return a string representation of an Arg's default value, or an
      empty string if none exists.

      \return  Default value as a string.
    */
    virtual std::string defaultVal() const
    {
        std::string s;

        for (size_t i = 0; i < m_defaultVal.size(); ++i)
        {
            if (i > 0)
                s += ", ";
            s += Utils::toString(m_defaultVal[i]);
        }
        return s;
    }

private:
    std::vector<T>& m_var;
    std::vector<T> m_defaultVal;
};

/**
  Description of an argument tied to a string vector.
*/
template <>
class VArg<std::string> : public BaseVArg
{
public:
    /**
      Constructor for arguments wit default value.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  Variable to which the argument value(s) should be bound.
      \param def  Default value.
    */
    VArg(const std::string& longname, const std::string& shortname,
        const std::string& description, std::vector<std::string>& variable,
        std::vector<std::string> def) :
        BaseVArg(longname, shortname, description), m_var(variable),
        m_defaultVal(def)
    {
        m_var = def;
        m_defaultProvided = true;
    }

    /**
      Constructor for arguments without default value.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  Variable to which the argument value(s) should be bound.
    */
    VArg(const std::string& longname, const std::string& shortname,
        const std::string& description, std::vector<std::string>& variable) :
        BaseVArg(longname, shortname, description), m_var(variable)
    {}

    /**
      Set a an argument's value from a string.

      Throws an arg_error exception if \a s can't be converted to
      the argument's type.
      \note  Not intended to be called from user code.

      \param s  Value to set.
    */
    virtual void setValue(const std::string& s)
    {
        std::vector<std::string> slist = Utils::split2(s, ',');
        for (auto& ts : slist)
            Utils::trim(ts);

        if (slist.empty())
            throw arg_val_error("Missing value for argument '" + m_longname +
                "'.");
        m_rawVal = s;
        if (!m_set)
            m_var.clear();
        m_var.reserve(m_var.size() + slist.size());
        m_var.insert(m_var.end(), slist.begin(), slist.end());
        m_set = true;
    }

    /**
      Reset the argument's state.

      Set the internal state of the argument and it's referenced variable
      as if no command-line parsing had occurred.
      \note  For testing.  Not intended to be called from user code.
    */
    virtual void reset()
    {
        m_var = m_defaultVal;
        m_set = false;
        m_hidden = false;
    }

    /**
      Return a string representation of an Arg's default value, or an
      empty string if none exists.

      \return  Default value as a string.
    */
    virtual std::string defaultVal() const
    {
        std::string s;

        for (size_t i = 0; i < m_defaultVal.size(); ++i)
        {
            if (i > 0)
                s += ", ";
            s += m_defaultVal[i];
        }
        return s;
    }

private:
    std::vector<std::string>& m_var;
    std::vector<std::string> m_defaultVal;
};

/**
  Parses command lines, provides validation and stores found values in
  bound variables.  Add arguments with \ref add.  When all arguments
  have been added, use \ref parse to validate command line and assign
  values to variables bound with \ref add.
*/
class ProgramArgs
{

public:
    /**
      Add a string argument to the list of arguments.

      \param name  Name of argument.  Argument names are specified as
        "longname[,shortname]", where shortname is an optional one-character
        abbreviation.
      \param description  Description of the argument.
      \param var  Reference to variable to bind to argument.
      \param def  Default value of argument.
      \return  Reference to the new argument.
    */
    Arg& add(const std::string& name, const std::string description,
        std::string& var, std::string def)
    {
        return add<std::string>(name, description, var, def);
    }

    /**
      Add a list-based (vector) string argument

      \param name  Name of argument.  Argument names are specified as
        "longname[,shortname]", where shortname is an optional one-character
        abbreviation.
      \param description  Description of the argument.
      \param var  Reference to variable to bind to argument.
      \return  Reference to the new argument.
    */
    Arg& add(const std::string& name, const std::string& description,
        std::vector<std::string>& var)
    {
        return add<std::string>(name, description, var);
    }

    /**
      Return whether the argument (as specified by it's longname) had
      its value set during parsing.
    */
    bool set(const std::string& name) const
    {
        Arg *arg = findLongArg(name);
        if (arg)
            return arg->set();
        return false;
    }

    /**
      Add a list-based (vector) argument.

      \param name  Name of argument.  Argument names are specified as
        "longname[,shortname]", where shortname is an optional one-character
        abbreviation.
      \param description  Description of the argument.
      \param var  Reference to variable to bind to argument.
      \return  Reference to the new argument.
    */
    template<typename T>
    Arg& add(const std::string& name, const std::string& description,
        std::vector<T>& var)
    {
        std::string longname, shortname;
        splitName(name, longname, shortname);

        Arg *arg = new VArg<T>(longname, shortname, description, var);
        addLongArg(longname, arg);
        addShortArg(shortname, arg);
        m_args.push_back(std::unique_ptr<Arg>(arg));
        return *arg;
    }

    /**
      Add a list-based (vector) argument with a default.

      \param name  Name of argument.  Argument names are specified as
        "longname[,shortname]", where shortname is an optional one-character
        abbreviation.
      \param description  Description of the argument.
      \param var  Reference to variable to bind to argument.
      \return  Reference to the new argument.
    */
    template<typename T>
    Arg& add(const std::string& name, const std::string& description,
        std::vector<T>& var, std::vector<T> def)
    {
        std::string longname, shortname;
        splitName(name, longname, shortname);

        Arg *arg = new VArg<T>(longname, shortname, description, var, def);
        addLongArg(longname, arg);
        addShortArg(shortname, arg);
        m_args.push_back(std::unique_ptr<Arg>(arg));
        return *arg;
    }

    /**
      Add an argument to the list of arguments with a default.

      \param name  Name of argument.  Argument names are specified as
        "longname[,shortname]", where shortname is an optional one-character
        abbreviation.
      \param description  Description of the argument.
      \param var  Reference to variable to bind to argument.
      \param def  Default value of argument.
      \return  Reference to the new argument.
    */
    template<typename T>
    Arg& add(const std::string& name, const std::string description, T& var,
        T def)
    {
        std::string longname, shortname;
        splitName(name, longname, shortname);

        Arg *arg = new TArg<T>(longname, shortname, description, var, def);
        addLongArg(longname, arg);
        addShortArg(shortname, arg);
        m_args.push_back(std::unique_ptr<Arg>(arg));
        return *arg;
    }

    /**
      Add an argument to the list of arguments.

      \param name  Name of argument.  Argument names are specified as
        "longname[,shortname]", where shortname is an optional one-character
        abbreviation.
      \param description  Description of the argument.
      \param var  Reference to variable to bind to argument.
      \return  Reference to the new argument.
    */
    template<typename T>
    Arg& add(const std::string& name, const std::string description, T& var)
    {
        std::string longname, shortname;
        splitName(name, longname, shortname);

        Arg *arg = new TArg<T>(longname, shortname, description, var);
        addLongArg(longname, arg);
        addShortArg(shortname, arg);
        m_args.push_back(std::unique_ptr<Arg>(arg));
        return *arg;
    }

    /**
      Parse a command line as specified by its argument vector.  No validation
      occurs and only argument value exceptions are raised,
      but assignments are made to bound variables where possible.

      \param s  List of strings that constitute the argument list.
    */
    void parseSimple(std::vector<std::string>& s)
    {
        ArgValList vals(s);

        for (size_t i = 0; i < vals.size();)
        {
            const std::string& arg = vals[i];
            // This may be the value, or it may not.  We're passing it along
            // just in case.  If there is no value, pass along "" to make
            // clear that there is none.
            std::string value((i != vals.size() - 1) ? vals[i + 1] : "");
            try
            {
                int matched = parseArg(arg, value);
                if (!matched)
                    i++;
                else
                    while (matched--)
                        vals.consume(i++);
            }
            catch (arg_val_error&)
            {
                throw;
            }
            catch (arg_error&)
            {
                i++;
            }
        }

        // Go through args and assign those unset to items from the command
        // line not already consumed.
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *arg = ai->get();
            try
            {
                arg->assignPositional(vals);
            }
            catch (arg_val_error&)
            {
                throw;
            }
            catch (arg_error&)
            {}
        }
        s = vals.unconsumedArgs();
    }

    /**
      Parse a command line as specified by its argument list.  Parsing
      validates the argument vector and assigns values to variables bound
      to added arguments.

      \param s  List of strings that constitute the argument list.
    */
    void parse(const std::vector<std::string>& s)
    {
        validate();
        ArgValList vals(s);
        for (size_t i = 0; i < vals.size();)
        {
            const std::string& arg = vals[i];
            // This may be the value, or it may not.  We're passing it along
            // just in case.  If there is no value, pass along "" to make
            // clear that there is none.
            std::string value((i != vals.size() - 1) ? vals[i + 1] : "");
            size_t matched = parseArg(arg, value);
            if (!matched)
                i++;
            else
                while (matched--)
                    vals.consume(i++);
        }

        // Go through args and assign those unset to items from the command
        // line not already consumed.
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *arg = ai->get();
            arg->assignPositional(vals);
        }
        //NOTE - Perhaps we should error here if there are still
        //  unconsumed arguments.
    }

    /**
      Add a synonym for an argument.

      \param name  Longname of existing arugment.
      \param synonym  Synonym for argument.
    */
    void addSynonym(const std::string& name, const std::string& synonym)
    {
        Arg *arg = findLongArg(name);
        if (!arg)
            throw arg_error("Can't set synonym for argument '" + name + "'. "
                "Argument doesn't exist.");
        if (synonym.empty())
            throw arg_error("Invalid (empty) synonym for argument '" +
                name + "'.");
        addLongArg(synonym, arg);
    }

    /**
      Reset the state of all arguments and bound variables as if no parsing
      had occurred.
    */
    void reset()
    {
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
            (*ai)->reset();
    }

    /**
      Return a string suitable for use in a "usage" line for display to
      users as help.
    */
    std::string commandLine() const
    {
        std::string s;

        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *a = ai->get();

            if (a->hidden())
                continue;
            std::string o = a->commandLine();
            if (o.size())
                s += o + " ";
        }
        if (s.size())
            s = s.substr(0, s.size() - 1);
        return s;
    }

    /**
      Write a formatted description of arguments to an output stream.

      Write a list of the names and descriptions of arguments suitable for
      display as help information.

      \param out  Stream to which output should be written.
      \param indent  Number of characters to indent all text.
      \param totalWidth  Total width to assume for formatting output.
        Typically this is the width of a terminal window.
    */
    void dump(std::ostream& out, size_t indent, size_t totalWidth) const
    {
        size_t namelen = 0;
        std::vector<std::pair<std::string, std::string>> info;

        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *a = ai->get();
            if (a->hidden())
                continue;

            std::string nameDescrip = a->nameDescrip();

            info.push_back(std::make_pair(nameDescrip, a->description()));
            namelen = (std::max)(namelen, nameDescrip.size());
        }
        size_t secondIndent = indent + 4;
        int postNameSpacing = 2;
        size_t leadlen = namelen + indent + postNameSpacing;
        size_t firstlen = totalWidth - leadlen - 1;
        size_t secondLen = totalWidth - secondIndent - 1;

        bool skipfirst = (firstlen < 10);
        if (skipfirst)
            firstlen = secondLen;

        for (auto i : info)
        {
            std::vector<std::string> descrip =
                Utils::wordWrap(i.second, secondLen, firstlen);

            std::string name = i.first;
            out << std::string(indent, ' ');
            if (skipfirst)
                out << name << std::endl;
            else
            {
                name.resize(namelen, ' ');
                out << name << std::string(postNameSpacing, ' ') <<
                    descrip[0] << std::endl;
            }
            for (size_t ii = 1; ii < descrip.size(); ++ii)
                out << std::string(secondIndent, ' ') <<
                    descrip[ii] << std::endl;
        }
    }

    /**
      Write a verbose description of arguments to an output stream.  Each
      argument is on its own line.  The argument's description follows
      on subsequent lines.

      \param out  Stream to which output should be written.
      \param nameIndent  Number of characters to indent argument lines.
      \param descripIndent  Number of characters to indent description lines.
      \param totalWidth  Total line width.

    */
    void dump2(std::ostream& out, size_t nameIndent, size_t descripIndent,
        size_t totalWidth) const
    {
        size_t width = totalWidth - descripIndent;
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *a = ai->get();
            out << std::string(nameIndent, ' ') << a->longname();
            if (a->defaultProvided())
                out << " [" << a->defaultVal() << "]";
            out << std::endl;
            std::vector<std::string> descrip =
                Utils::wordWrap(a->description(), width);
            if (descrip.empty())
                descrip.push_back("<no description available>");
            for (std::string& s : descrip)
                out << std::string(descripIndent, ' ') << s << std::endl;
            out << std::endl;
        }
    }

    /**
      Write a JSON array of arguments to an output stream.

      \param out  Stream to which output should be written.

    */
    void dump3(std::ostream& out) const
    {
        out << "[";
        bool bFirst(true);
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *a = ai->get();

            if (!bFirst)
                out << ",";

            out << "{\"name\":\"" << a->longname() << "\"";

            if (a->defaultProvided())
                out << ",\"default\":\"" << a->defaultVal() << "\"";

            out << ",\"description\":\"" << a->description() << "\"}";

            if (bFirst) bFirst = false;


        }
        out << "]";
    }

private:
    /*
      Split an argument name into longname and shortname.

      \param name  Name of argument specified as "longname[,shortname]".
      \param[out] longname  Parsed longname.
      \param[out] shortname  Parsed shortname.
    */
    void splitName(const std::string& name, std::string& longname,
        std::string& shortname)
    {
        // Arg names must be specified as "longname[,shortname]" where
        // shortname is a single character.
        std::vector<std::string> s = Utils::split(name, ',');
        if (s.size() > 2)
            throw arg_error("Invalid program argument specification");
        if (s.size() == 2 && s[1].size() != 1)
            throw arg_error("Short argument not specified as single character");
        if (s.empty())
            throw arg_error("No program argument provided.");
        if (s.size() == 1)
            s.push_back("");
        longname = s[0];
        shortname = s[1];
    }

    /*
      Add an argument to the list of arguments based on its longname.

      \param name  Argument longname.
      \param arg   Pointer to argument.
    */
    void addLongArg(const std::string& name, Arg *arg)
    {
        if (name.empty())
            return;
        if (findLongArg(name))
            throw arg_error("Argument --" + name + " already exists.");
        m_longargs[name] = arg;
    }

    /*
      Add an argument to the list of arguments based on its shortname.

      \param name  Argument shortname.
      \param arg   Pointer to argument.
    */
    void addShortArg(const std::string& name, Arg *arg)
    {
        if (name.empty())
            return;
        if (findShortArg(name[0]))
            throw arg_error("Argument -" + name + " already exists.");
        m_shortargs[name] = arg;
    }

    /*
      Find an argument given its longname.

      \param s  Longname of argument.
      \return  Pointer to matching argument, or NULL if none was found.
    */
    Arg *findLongArg(const std::string& s) const
    {
        auto si = m_longargs.find(s);
        if (si != m_longargs.end())
            return si->second;
        return NULL;
    }

    /*
      Find an argument given its shortname.

      \param c  Shortnamn of argument.
      \return  Pointer to matching argument, or NULL if none was found.
    */
    Arg *findShortArg(char c) const
    {
        std::string s(1, c);
        auto si = m_shortargs.find(s);
        if (si != m_shortargs.end())
            return si->second;
        return NULL;
    }

    /*
      Parse a string-specified argument name and value into its argument.

      \param arg  Name of argument specified on command line.
      \param value  Potential value assigned to argument.
      \return  Number of strings consumed (1 for positional arguments or
        arguments that don't take values or 2 otherwise).
    */
    int parseArg(const std::string& arg, const std::string& value)
    {
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-')
            return parseLongArg(arg, value);
        else if (arg.size() && arg[0] == '-')
            return parseShortArg(arg, value);
        return 0;
    }

    /*
      Parse an argument specified as a long argument (prefixed with "--")
      Long arguments with values can be specified as
      "--name=value" or "--name value".

      \param name  Name of argument specified on command line.
      \param value  Potential value assigned to argument.
      \return  Number of strings consumed (1 for positional arguments or
        arguments that don't take values or 2 otherwise).
    */
    int parseLongArg(const std::string& inName, const std::string& inValue)
    {
        bool attachedValue = false;

        if (inName.size() == 2)
            throw arg_error("No argument found following '--'.");

        std::string name = inName.substr(2);
        std::string value = inValue;

        std::size_t pos = name.find_first_of("=");
        if (pos != std::string::npos)
        {
            if (pos < name.size() + 1)
            {
                value = name.substr(pos + 1);
                name = name.substr(0, pos);
                attachedValue = true;
            }
        }
        else if (value.size() && value[0] == '-')
        {
            // If a value starts with a '-' and isn't attached to a name,
            // we assume it's an option and not a value.
            value.clear();
        }

        Arg *arg = findLongArg(name);
        if (!arg)
            throw arg_error("Unexpected argument '" + name + "'.");

        if (!arg->needsValue())
        {
            if (attachedValue)
            {
                if (value != "true" && value != "false")
                {
                    throw arg_error("Value '" + value +
                        "' provided for argument '" + name +
                        "' when 'true' or 'false' is expected.");
                }
            }
            else
                value = "invert";
            arg->setValue(value);
            return 1;
        }

        arg->setValue(value);
        return (attachedValue ? 1 : 2);
    }

    /*
      Parse an argument specified as a short argument (prefixed with "-")
      Short arguments with values are specified as "-name value".

      \param name  Name of argument specified on command line.
      \param value  Potential value assigned to argument.
      \return  Number of strings consumed (1 for positional arguments or
        arguments that don't take values or 2 otherwise).
    */
    int parseShortArg(const std::string& name, const std::string& value)
    {
        if (name.size() == 1)
            throw arg_error("No argument found following '-'.");
        assert(name.size() == 2);

        Arg *arg = findShortArg(name[1]);
        if (!arg)
            throw arg_error("Unexpected argument '-" + std::string(1, name[1]) +
                "'.");

        int cnt;
        if (arg->needsValue())
        {
            // If the value starts with a '-', assume it's an option
            // rather than a value.
            if (value.empty() || value[0] == '-')
            {
                throw arg_error("Short option '" + name + "' expects value "
                    "but none directly follows.");
            }
            else
            {
                cnt = 2;
                arg->setValue(value);
            }
        }
        else
        {
            arg->setValue("true");
            cnt = 1;
        }
        return cnt;
    }

    /*
      Make sure we don't have any required positional args after
      non-required positional args.
    */
    void validate()
    {
        bool opt = false;
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *arg = ai->get();
            if (arg->positional() == Arg::PosType::Optional)
                opt = true;
            if (opt && (arg->positional() == Arg::PosType::Required))
                throw arg_error("Found required positional argument '" +
                    arg->longname() + "' after optional positional argument.");
        }
    }

    std::vector<std::unique_ptr<Arg>> m_args;  /// Storage for arguments
    std::map<std::string, Arg *> m_shortargs;  /// Map from shortname to args
    std::map<std::string, Arg *> m_longargs;  /// Map from longname to args
};

} // namespace pdal

