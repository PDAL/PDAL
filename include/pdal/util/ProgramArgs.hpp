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

#include <deque>
#include <map>

#include <pdal/util/Utils.hpp>

namespace pdal
{

class arg_error
{
public:
    arg_error(const std::string& error) : m_error(error)
    {}

    std::string m_error;
};


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
      Return whether the argument was set during command-line parsing.
    */
    bool set() const
        { return m_set; }

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
      Set the argument's value from the positional list.
      \note  Not intended to be called from user code.

      \param posList  The list of positional strings specified on the command
        line.
      \return  The number of positional strings consumed by this argument.
    */
    virtual size_t assignPositional(const std::deque<std::string> posList)
        { return 0; }

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
    */
    std::string description() const
        { return m_description; }

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
      Constructor.

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
        m_defaultVal(def)
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
        if (s.size() && s[0] == '-')
        {
            std::stringstream oss;
            oss << "Argument '" << m_longname << "' needs a value and none "
                "was provided.";
            throw arg_error(oss.str());
        }
        m_rawVal = s;
        if (!Utils::fromString(s, m_var))
        {
            std::ostringstream oss;
            oss << "Invalid value for argument '" << m_longname << "'.";
            throw arg_error(oss.str());
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
      Set the argument's value from the positional list.

      If no value is provided for a required positional option, an arg_error
      exception is thrown.
      \note  Not intended to be called from user code.

      \param posList  The list of positional strings specified on the command
        line.
      \return  The number of positional strings consumed by this argument
        (always 0 or 1).
    */
    virtual size_t assignPositional(const std::deque<std::string> posList)
    {
        if (m_positional == PosType::None || m_set)
            return 0;

        if (posList.empty())
        {
            if (m_positional == PosType::Required)
            {
                std::ostringstream oss;

                oss << "Missing value for positional argument '" <<
                    m_longname << "'.";
                throw arg_error(oss.str());
            }
            else
                return 0;
        }
        setValue(posList.front());
        return 1;
    }

private:
    T& m_var;
    T m_defaultVal;
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
      Constructor for boolean arguments.

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
        m_defaultVal(def)
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

      \note  The string argument is ingored.  The value that is set is 'true'
        if the variable's default value is 'false', and vice-versa.
      \note  Not intended to be called from user code.

      \param s  Value to set [ignored].
    */
    virtual void setValue(const std::string& s)
    {
        if (s.size() && s[0] == '-')
        {
            std::stringstream oss;
            oss << "Argument '" << m_longname << "' needs a value and none "
                "was provided.";
            throw arg_error(oss.str());
        }
        m_val = !m_defaultVal;
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
        std::ostringstream oss;
        oss << "Boolean argument '" << m_longname << "' can't be positional.";
        throw arg_error(oss.str());
        return *this;
    }

    /**
      Indicate that the argument is positional and optional.

      Throws an exception to indicate that boolean arguments can't
      positional.
    */
    virtual Arg& setOptionalPositional()
    {
        std::ostringstream oss;
        oss << "Boolean argument '" << m_longname << "' can't be positional.";
        throw arg_error(oss.str());
        return *this;
    }

private:
    bool& m_val;
    bool m_defaultVal;
};

/**
  Description of a list-based (vector) argument.  List-based arguments can
  be specified multiple times, taking multiple values.  List-based
  arguments are necessarily bound to variables that are vectors.
  \note  Doesn't properly support list-based boolean values.
*/
template <typename T>
class VArg : public Arg
{
public:
    /**
      Constructor.

      \param longname  Name of argument specified on command line with "--"
        prefix.
      \param shortname  Optional name of argument specified on command
        line with "-" prefix.
      \param description  Argument description.
      \param variable  Variable to which the argument value(s) should be bound.
    */
    VArg(const std::string& longname, const std::string& shortname,
        const std::string& description, std::vector<T>& variable) :
        Arg(longname, shortname, description), m_var(variable)
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
        if (s.size() && s[0] == '-')
        {
            std::stringstream oss;
            oss << "Argument '" << m_longname << "' needs a value and none "
                "was provided.";
            throw arg_error(oss.str());
        }
        m_rawVal = s;
        T var;
        if (!Utils::fromString(s, var))
        {
            std::ostringstream oss;
            oss << "Invalid value for argument '" << m_longname << "'.";
            throw arg_error(oss.str());
        }
        m_var.push_back(var);
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
        m_var.clear();
        m_set = false;
        m_hidden = false;
    }

    /**
      Set the argument's value from the positional list.

      List-based arguments consume ALL positional arguments until
      one is found that can't be converted to the type of the bound variable.
      \note  Not intended to be called from user code.

      \param posList  The list of positional strings specified on the command
        line.
      \return  The number of positional strings consumed by this argument.
    */
    virtual size_t assignPositional(const std::deque<std::string> posList)
    {
        if (m_positional == PosType::None || m_set)
            return 0;

        size_t cnt;
        for (cnt = 0; cnt < posList.size(); ++cnt)
            try
            {
                setValue(posList[cnt]);
            }
            catch (arg_error&)
            {
                break;
            }
        if (cnt == 0 && m_positional == Arg::PosType::Required)
        {
            std::ostringstream oss;

            oss << "Missing value for positional argument '" <<
                m_longname << "'.";
            throw arg_error(oss.str());
        }
        return cnt;
    }

private:
    std::vector<T>& m_var;
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
      Add an argument to the list of arguments.

      \param name  Name of argument.  Argument names are specified as
        "longname[,shortname]", where shortname is an optional one-character
        abbreviation.
      \param description  Description of the argument.
      \param var  Reference to variable to bind to argument.
      \param def  Default value of argument.  If not specified, a
        default-constructed value is used.
      \return  Reference to the new argument.
    */
    template<typename T>
    Arg& add(const std::string& name, const std::string description, T& var,
        T def = T())
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
      Parse a command line as specified by its argument list.  Parsing
      validates the argument vector and assigns values to variables bound
      to added arguments.

      \param argc  Number of entries in the argument list.
      \param argv  List of strings that constitute the argument list.
    */
    void parse(int argc, char *argv[])
    {
        std::vector<std::string> s;
        for (size_t i = 0; i < (size_t)argc; ++i)
            s.push_back(argv[i]);
        parse(s);
    }

    /**
      Parse a command line as specified by its argument vector.  No validation
      occurs and no exceptions are raised, but assignments are made
      to bound variables where possible.

      \param argc  Number of entries in the argument list.
      \param argv  List of strings that constitute the argument list.
    */
    void parseSimple(int argc, char *argv[])
    {
        std::vector<std::string> s;
        for (size_t i = 0; i < (size_t)argc; ++i)
            s.push_back(argv[i]);
        parseSimple(s);
    }

    /**
      Parse a command line as specified by its argument vector.  No validation
      occurs and no exceptions are raised, but assignments are made
      to bound variables where possible.

      \param s  List of strings that constitute the argument list.
    */
    void parseSimple(std::vector<std::string>& s)
    {
        m_positional.clear();
        for (size_t i = 0; i < s.size();)
        {
            std::string& arg = s[i];
            // This may be the value, or it may not.  We're passing it along
            // just in case.  If there is no value, pass along "-" to make
            // clear that there is none.
            std::string value((i != s.size() - 1) ? s[i + 1] : "-");
            try
            {
                i += parseArg(arg, value);
            }
            catch (arg_error& )
            {
                i++;
            }
        }
    }

    /**
      Parse a command line as specified by its argument list.  Parsing
      validates the argument vector and assigns values to variables bound
      to added arguments.

      \param s  List of strings that constitute the argument list.
    */
    void parse(std::vector<std::string>& s)
    {
        m_positional.clear();
        validate();

        for (size_t i = 0; i < s.size();)
        {
            std::string& arg = s[i];
            // This may be the value, or it may not.  We're passing it along
            // just in case.  If there is no value, pass along "-" to make
            // clear that there is none.
            std::string value((i != s.size() - 1) ? s[i + 1] : "-");
            i += parseArg(arg, value);
        }

        // Go through things looking for matches.
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *arg = ai->get();
            size_t cnt = arg->assignPositional(m_positional);
            while (cnt--)
                m_positional.pop_front();
        }
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
    void dump(std::ostream& out, size_t indent, size_t totalWidth)
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
            namelen = std::max(namelen, nameDescrip.size());
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
            StringList descrip = Utils::wordWrap(i.second, secondLen, firstlen);

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
            for (size_t i = 1; i < descrip.size(); ++i)
                out << std::string(secondIndent, ' ') <<
                    descrip[i] << std::endl;
        }
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
        {
            std::ostringstream oss;

            oss << "Argument --" << name << " already exists.";
            throw arg_error(oss.str());
        }
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
        {
            std::ostringstream oss;

            oss << "Argument -" << name << " already exists.";
            throw arg_error(oss.str());
        }
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
    int parseArg(std::string& arg, std::string value)
    {
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-')
            return parseLongArg(arg, value);
        else if (arg.size() && arg[0] == '-')
            return parseShortArg(arg, value);
        m_positional.push_back(arg);
        return 1;
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
    int parseLongArg(std::string name, std::string value)
    {
        bool attachedValue = false;

        if (name.size() == 2)
            throw arg_error("No argument found following '--'.");

        name = name.substr(2);

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
        Arg *arg = findLongArg(name);
        if (!arg)
        {
            std::ostringstream oss;
            oss << "Unexpected argument '" << name << "'.";
            throw arg_error(oss.str());
        }

        if (!arg->needsValue())
        {
            if (attachedValue)
            {
                std::ostringstream oss;
                oss << "Value '" << value << "' provided for argument '" <<
                    name << "' when none is expected.";
                throw arg_error(oss.str());
            }
            arg->setValue("true");
            return 1;
        }

        arg->setValue(value);
        return (attachedValue ? 1 : 2);
    }

    /*
      Parse an argument specified as a long argument (prefixed with "-")
      Long arguments with values are specified as "-name value".

      \param name  Name of argument specified on command line.
      \param value  Potential value assigned to argument.
      \return  Number of strings consumed (1 for positional arguments or
        arguments that don't take values or 2 otherwise).
    */
    int parseShortArg(std::string& name, std::string value)
    {
        int cnt = 1;

        if (name.size() == 1)
            throw arg_error("No argument found following '-'.");

        Arg *arg = findShortArg(name[1]);
        if (!arg)
        {
            std::ostringstream oss;
            oss << "Unexpected argument '-" << name[1] << "'.";
            throw arg_error(oss.str());
        }

        if (arg->needsValue())
        {
            if (name.size() == 2)
            {
                arg->setValue(value);
                cnt = 2;
            }
            else
            {
                std::ostringstream oss;

                oss << "Short option '" << name[1] << "' expects value "
                    "but appears in option group '" << name << "'.";
                throw arg_error(oss.str());
            }
        }
        else
            arg->setValue("true");
        if (name.size() > 2)
        {
            name = std::string(1, '-') + name.substr(2);
            cnt = 0;
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
                throw arg_error("Found required positional argument after "
                    "optional positional argument.");
        }
    }

    std::vector<std::unique_ptr<Arg>> m_args;  /// Storage for arguments
    std::map<std::string, Arg *> m_shortargs;  /// Map from shortname to args
    std::map<std::string, Arg *> m_longargs;  /// Map from longname to args

    /*
      Contains remaining arguments after positional argument have been
      processed.
    */
    std::deque<std::string> m_positional;
};

} // namespace pdal

