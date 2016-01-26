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

class BaseArg
{
protected:
    BaseArg(const std::string& longname, const std::string& shortname,
        const std::string& description) : m_longname(longname),
        m_shortname(shortname), m_description(description), m_set(false)
    {}

public:
    virtual bool needsValue() const
        { return true; }
    virtual void setValue(const std::string& s) = 0;
    virtual void reset() = 0;

protected:
    std::string m_longname;
    std::string m_shortname;
    std::string m_description;
    std::string m_rawVal;
    bool m_set;
};

template <typename T>
class Arg : public BaseArg
{
public:
    Arg(const std::string& longname, const std::string& shortname,
        const std::string& description, T& variable, T def) :
        BaseArg(longname, shortname, description), m_var(variable),
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
    }

private:
    T& m_var;
    T m_defaultVal;
};

template <>
class Arg<bool> : public BaseArg
{
public:
    Arg(const std::string& longname, const std::string& shortname,
        const std::string& description, bool& variable, bool def) :
        BaseArg(longname, shortname, description), m_val(variable),
        m_defaultVal(def)
    {}

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
    }

private:
    bool& m_val;
    bool m_defaultVal;
};

class ProgramArgs
{
public:
    void add(const std::string& name, const std::string description,
        std::string& var, std::string def)
    {
        add<std::string>(name, description, var, def);
    }

    template<typename T>
    void add(const std::string& name, const std::string description, T& var,
        T def = T())
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

        BaseArg *arg = new Arg<T>(s[0], s[1], description, var, def);
        if (s[0].size())
            m_longargs[s[0]] = arg;
        if (s[1].size())
            m_shortargs[s[1]] = arg;
        m_args.push_back(std::unique_ptr<BaseArg>(arg));
    }

    void parse(int argc, char *argv[])
    {
        std::vector<std::string> s;
        for (size_t i = 0; i < (size_t)argc; ++i)
            s.push_back(argv[i]);
        parse(s);
    }

    void parse(std::vector<std::string>& s)
    {
        for (size_t i = 0; i < s.size();)
        {
            std::string& arg = s[i];
            // This may be the value, or it may not.  We're passing it along
            // just in case.  If there is no value, pass along "-" to make
            // clear that there is none.
            std::string value((i != s.size() - 1) ? s[i + 1] : "-");
            i += parseArg(arg, value);
        }
    }

    void reset()
    {
        for (auto ai = m_args.begin(); ai != m_args.end(); ++ai)
            (*ai)->reset();
    }

private:
    BaseArg *findLongArg(const std::string& s)
    {
        auto si = m_longargs.find(s);
        if (si != m_longargs.end())
            return si->second;
        return NULL;
    }

    BaseArg *findShortArg(char c)
    {
        std::string s(1, c);
        auto si = m_shortargs.find(s);
        if (si != m_shortargs.end())
            return si->second;
        return NULL;
    }

    int parseArg(std::string& arg, std::string value)
    {
        if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-')
            return parseLongArg(arg, value);
        else if (arg.size() > 1 && arg[0] == '-')
            return parseShortArg(arg, value);
        m_unrecognized.push_back(arg);
        return 1;
    }

    int parseLongArg(std::string name, std::string value)
    {
        bool attachedValue = false;

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
        BaseArg *arg = findLongArg(name);
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

        BaseArg *arg = findShortArg(name[1]);
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

    std::vector<std::unique_ptr<BaseArg>> m_args;
    std::map<std::string, BaseArg *> m_shortargs;
    std::map<std::string, BaseArg *> m_longargs;
    std::vector<std::string> m_unrecognized;
};

} // namespace pdal

