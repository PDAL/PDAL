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


namespace
{
namespace PosType
{

enum Enum
{
    None,
    Required,
    Optional
};

} // namespace PosType
} // unnamed namespace

class Arg
{
protected:
    Arg(const std::string& longname, const std::string& shortname,
        const std::string& description) : m_longname(longname),
        m_shortname(shortname), m_description(description), m_set(false),
        m_hidden(false), m_positional(PosType::None)
    {}

public:
    Arg& setHidden(bool hidden = true)
    {
        m_hidden = true;
        return *this;
    }
    virtual Arg& setPositional()
    {
        m_positional = PosType::Required;
        return *this;
    }
    virtual Arg& setOptionalPositional()
    {
        m_positional = PosType::Optional;
        return *this;
    }
    bool set() const
        { return m_set; }

public:
    virtual bool needsValue() const
        { return true; }
    virtual void setValue(const std::string& s) = 0;
    virtual void reset() = 0;
    virtual size_t assignPositional(const std::deque<std::string> posList)
        { return 0; }
    PosType::Enum positional() const
        { return m_positional; }
    std::string description() const
        { return m_description; }
    std::string nameDescrip() const
    {
        std::string s("--");
        s += m_longname;
        if (m_shortname.size())
            s += ", -" + m_shortname;
        return s;
    }
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
    PosType::Enum m_positional;
};

template <typename T>
class TArg : public Arg
{
public:
    TArg(const std::string& longname, const std::string& shortname,
        const std::string& description, T& variable, T def) :
        Arg(longname, shortname, description), m_var(variable),
        m_defaultVal(def)
    { m_var = m_defaultVal; }

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

    virtual void reset()
    {
        m_var = m_defaultVal;
        m_set = false;
        m_hidden = false;
    }

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

template <>
class TArg<bool> : public Arg
{
public:
    TArg(const std::string& longname, const std::string& shortname,
        const std::string& description, bool& variable, bool def) :
        Arg(longname, shortname, description), m_val(variable),
        m_defaultVal(def)
    { m_val = m_defaultVal; }

    virtual bool needsValue() const
        { return false; }
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
    virtual void reset()
    {
        m_val = m_defaultVal;
        m_set = false;
        m_hidden = false;
    }
    virtual Arg& setPositional()
    {
        std::ostringstream oss;
        oss << "Boolean argument '" << m_longname << "' can't be positional.";
        throw arg_error(oss.str());
        return *this;
    }
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

template <typename T>
class VArg : public Arg
{
public:
    VArg(const std::string& longname, const std::string& shortname,
        const std::string& description, std::vector<T>& variable) :
        Arg(longname, shortname, description), m_var(variable)
    {}

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

    virtual void reset()
    {
        m_var.clear();
        m_set = false;
        m_hidden = false;
    }

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
        if (cnt == 0 && m_positional == PosType::Required)
        {
            std::ostringstream oss;

            oss << "Missing value for positional argument '" <<
                m_longname << "'.";
            throw arg_error(oss.str());
        }
        return cnt;
    }

private:
    std::vector<T>& m_var; };

class ProgramArgs
{
public:
    Arg& add(const std::string& name, const std::string description,
        std::string& var, std::string def)
    {
        return add<std::string>(name, description, var, def);
    }

    Arg& add(const std::string& name, const std::string& description,
        std::vector<std::string>& var)
    {
        return add<std::string>(name, description, var);
    }

    bool set(const std::string& name) const
    {
        Arg *arg = findLongArg(name);
        if (arg)
            return arg->set();
        return false;
    }

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

    void parse(int argc, char *argv[])
    {
        std::vector<std::string> s;
        for (size_t i = 0; i < (size_t)argc; ++i)
            s.push_back(argv[i]);
        parse(s);
    }

    void parseSimple(int argc, char *argv[])
    {
        std::vector<std::string> s;
        for (size_t i = 0; i < (size_t)argc; ++i)
            s.push_back(argv[i]);
        parseSimple(s);
    }


    // This sets variables without throwing exceptions if problems are found.
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
            catch (arg_error& e)
            {
                i++;
            }
        }
    }


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

    void reset()
    {
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
            (*ai)->reset();
    }

    std::string commandLine() const
    {
        std::string s;

        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            std::string o = (*ai)->commandLine();
            if (o.size())
                s += o + " ";
        }
        if (s.size())
            s = s.substr(0, s.size() - 1);
        return s;
    }

    void dump(std::ostream& out, size_t indent, size_t totalWidth)
    {
        size_t namelen = 0;
        std::vector<std::pair<std::string, std::string>> info;

        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *a = ai->get();
            std::string nameDescrip = a->nameDescrip();

            info.push_back(std::make_pair(nameDescrip, a->description()));
            namelen = std::max(namelen, nameDescrip.size());
        }
        int secondIndent = indent + 4;
        int postNameSpacing = 2;
        int leadlen = namelen + indent + postNameSpacing;
        int firstlen = (int)totalWidth - leadlen - 1;
        int secondLen = totalWidth - secondIndent - 1;
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

    Arg *findLongArg(const std::string& s) const
    {
        auto si = m_longargs.find(s);
        if (si != m_longargs.end())
            return si->second;
        return NULL;
    }

    Arg *findShortArg(char c) const
    {
        std::string s(1, c);
        auto si = m_shortargs.find(s);
        if (si != m_shortargs.end())
            return si->second;
        return NULL;
    }

    int parseArg(std::string& arg, std::string value)
    {
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-')
            return parseLongArg(arg, value);
        else if (arg.size() && arg[0] == '-')
            return parseShortArg(arg, value);
        m_positional.push_back(arg);
        return 1;
    }

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

    void validate()
    {
        // Make sure we don't have any required positional args after
        // non-required positional args.
        bool opt = false;
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
        {
            Arg *arg = ai->get();
            if (arg->positional() == PosType::Optional)
                opt = true;
            if (opt && (arg->positional() == PosType::Required))
                throw arg_error("Found required positional argument after "
                    "optional positional argument.");
        }
    }

    std::vector<std::unique_ptr<Arg>> m_args;
    std::map<std::string, Arg *> m_shortargs;
    std::map<std::string, Arg *> m_longargs;

    // Contains remaining arguments after positional argument have been
    // processed.
    std::deque<std::string> m_positional;
};

} // namespace pdal

