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

Option::Option(const boost::property_tree::ptree& tree)
{
    using namespace boost::property_tree;

    m_name = tree.get<std::string>("Name");
    m_value = tree.get<std::string>("Value");
    m_description =
    tree.count("Description") ? tree.get<std::string>("Description") : "";
}

#if !defined(PDAL_COMPILER_MSVC)

// explicit specialization:
//   if insert a bool, we don't want it to be "0" or "1" (which is
//   what lexical_cast would do)
template<> void Option::setValue(const bool& value)
{
    m_value = value ? "true" : "false";
}

// explicit specialization:
//   if we want to insert a string, we don't need lexical_cast
template<> void Option::setValue(const std::string& value)
{
    m_value = value;
}
#endif


void Option::toMetadata(MetadataNode& parent) const
{
    parent.add(getName(), getValue<std::string>());
}

//---------------------------------------------------------------------------


Options::Options(const Options& rhs)
    : m_options(rhs.m_options)
{}


Options::Options(const Option& opt)
{
    add(opt);
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


Options::Options(const boost::property_tree::ptree& tree)
{
    for (auto iter = tree.begin(); iter != tree.end(); ++iter)
    {
        assert(iter->first == "Option");
        Option opt(iter->second);
        add(opt);
    }
}


void Options::add(const Option& option)
{
    assert(Option::nameValid(option.getName(), true));
    m_options.insert(std::pair<std::string, Option>(option.getName(), option));
}


void Options::remove(const Option& option)
{
    m_options.erase(option.getName());
}


Option& Options::getOptionByRef(const std::string& name)
{
    auto iter = m_options.find(name);
    if (iter == m_options.end())
    {
        std::ostringstream oss;
        oss << "Options::getOptionByRef: Required option '" << name <<
            "' was not found on this stage";
        throw Option::not_found(oss.str());
    }
    return iter->second;
}


const Option& Options::getOption(const std::string& name) const
{
    assert(Option::nameValid(name, true));
    auto iter = m_options.find(name);
    if (iter == m_options.end())
    {
        std::ostringstream oss;
        oss << "Options::getOption: Required option '" << name <<
            "' was not found on this stage";
        throw Option::not_found(oss.str());
    }
    return iter->second;
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


bool Options::hasOption(std::string const& name) const
{
    try
    {
        (void)getOption(name);
        return true;
    }
    catch (Option::not_found)
    {}
    return false;
}


void Options::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& ostr, const Options& options)
{
    const boost::property_tree::ptree tree = pdal::Utils::toPTree(options);

    boost::property_tree::write_json(ostr, tree);

    return ostr;
}


} // namespace pdal
