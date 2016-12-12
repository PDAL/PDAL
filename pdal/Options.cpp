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
    parent.add(getName(), getValue());
}

//---------------------------------------------------------------------------

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


void Options::add(const Option& option)
{
    assert(Option::nameValid(option.getName(), true));
    m_options.insert({ option.getName(), option });
}


void Options::addConditional(const Option& option)
{
    assert(Option::nameValid(option.getName(), true));
    if (m_options.find(option.getName()) == m_options.end())
        m_options.insert({ option.getName(), option });
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

} // namespace pdal
