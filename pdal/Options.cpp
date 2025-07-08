/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (hobu@hobu.co)
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

#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/util/FileUtils.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <sstream>
#include <iostream>

namespace pdal
{

std::string Option::toArg() const
{
    return std::string(2, '-') + getName() + '=' + getValue();
}


void Option::toMetadata(MetadataNode& parent) const
{
    std::string value = getValue();
    parent.addWithType(getName(), value, Metadata::inferType(value));
}


bool Option::nameValid(const std::string& name, bool reportError)
{
    bool valid = (parse(name, 0) == name.size());
    if (!valid && reportError)
    {
        std::ostringstream oss;
        oss << "Invalid option name '" << name << "'.  Options must "
            "consist of only lowercase letters, numbers and '_'.";
        Utils::printError(oss.str());
    }
    return valid;
}


//---------------------------------------------------------------------------


void Options::add(const Option& option)
{
    assert(Option::nameValid(option.getName(), true));
    m_options.insert({ option.getName(), option });
}


void Options::add(const Options& o)
{
    m_options.insert(o.m_options.begin(), o.m_options.end());
}


void Options::addConditional(const Option& option)
{
    assert(Option::nameValid(option.getName(), true));
    if (m_options.find(option.getName()) == m_options.end())
        m_options.insert({ option.getName(), option });
}


void Options::addConditional(const Options& other)
{
    for (auto oi = other.m_options.begin(); oi != other.m_options.end();)
    {
        const std::string& name = oi->first;
        if (m_options.find(name) == m_options.end())
        {
            do
            {
                m_options.insert(*oi++);
            } while (oi != other.m_options.end() && name == oi->first);
        }
        else
            oi++;
    }
}


void Options::remove(const Option& option)
{
    m_options.erase(option.getName());
}


std::vector<Option> Options::getOptions(std::string const& name) const
{
    std::vector<Option> output;

    // If we have an empty name, return them all
    if (name.empty())
    {
        for (auto it = m_options.begin(); it != m_options.end(); ++it)
        {
            output.push_back(it->second);
        }
    }
    else
    {
        auto ret = m_options.equal_range(name);
        for (auto it = ret.first; it != ret.second; ++it)
        {
            output.push_back(it->second);
        }
    }
    return output;
}


/**
  Convert options to a string list appropriate for parsing with ProgramArgs.

  \return  List of options as argument strings.
*/
StringList Options::toCommandLine() const
{
    StringList s;

    for (const auto& op : m_options)
    {
        const Option& o = op.second;
        s.push_back(o.toArg());
    }
    return s;
}

Options Options::fromFile(const std::string& filename, bool throwOnOpenError)
{
    if (!FileUtils::fileExists(filename))
    {
        if (throwOnOpenError)
            throw pdal_error("Can't read options file '" + filename + "'.");
        else
            return Options();
    }

    std::string s = FileUtils::readFileIntoString(filename);

    size_t cnt = Utils::extractSpaces(s, 0);
    if (s[cnt] == '{')
        return fromJsonFile(filename, s);
    else if (s[cnt] == '-')
        return fromCmdlineFile(filename, s);
    else
        throw pdal_error("Option file '" + filename + "' not valid JSON or "
            "command-line format.");
}


Options Options::fromJsonFile(const std::string& filename, const std::string& s)
{
    Options options;

    NL::json node;
    try
    {
        std::ifstream in(filename);
        in >> node;
    }
    catch (NL::json::parse_error& err)
    {
        throw pdal_error("Unable to parse options file '" + filename +
            "' as JSON: \n" + err.what());
    }

    for (auto& element : node.items())
    {
        const std::string& name = element.key();
        const NL::json& n = element.value();

        if (n.is_string())
            options.add(name, n.get<std::string>());
        else if (n.is_number_unsigned())
            options.add(name, n.get<uint64_t>());
        else if (n.is_number_integer())
            options.add(name, n.get<int64_t>());
        else if (n.is_number_float())
            options.add(name, n.get<double>());
        else if (n.is_boolean())
            options.add(name, n.get<bool>());
        else if (n.is_null())
            options.add(name, "");
        else if (n.is_array() || n.is_object())
            options.add(name, n.get<std::string>());
        else
            throw pdal_error("Value of stage option '" +
                name + "' in options file '" + filename +
                "' cannot be converted.");
    }
    return options;
}


Options Options::fromCmdlineFile(const std::string& filename,
    const std::string& s)
{
    Options options;
    StringList args = Utils::simpleWordexp(s);

    for (size_t i = 0; i < args.size(); ++i)
    {
        std::string option = args[i];
        std::string value;
        if (i + 1 < args.size())
            value = args[i + 1];

        if (option.size() < 3)
            throw pdal_error("Invalid option '" + option + "' in option "
                "file '" + filename + "'.");
        if (option[0] != '-' || option[1] != '-')
            throw pdal_error("Option '" + option + "' missing leading \"--\" "
                "in option file '" + filename + "'.");

        std::string::size_type pos = 2;
        std::string::size_type count = Option::parse(option, pos);
        std::string optionName = option.substr(2, count);
        pos += count;
        if (option[pos++] == '=')
            value = option.substr(pos);
        else
            i++;
        if (value.empty())
            throw pdal_error("No value found for option '" + option + "' in "
                "option file '" + filename + "'.");
        Option o(optionName, value);
        options.add(o);
    }
    return options;
}

std::ostream& operator << (std::ostream& out, const Option& op)
{
    out << op.m_name << ":" << op.m_value;
    return out;
}

std::ostream& operator << (std::ostream& out, const Options& ops)
{
    for (auto& op : ops.m_options)
        out << op.second << "\n";
    return out;
}

} // namespace pdal
