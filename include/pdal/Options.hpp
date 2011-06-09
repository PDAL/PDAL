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

#include <pdal/pdal.hpp>
#include <pdal/Bounds.hpp>
#include <boost/property_tree/ptree.hpp>


namespace pdal
{

template <typename T>
class LIBPC_DLL Option
{

    
private:
    T m_value;
    std::string m_description;
    std::string m_name;

public:

    Option(std::string const& name, T value, std::string const& description) 
    : m_value(value), m_description(description), m_name(name) {}
    
    std::string const& getName() const { return m_name; }
    std::string const& getDescription() const { return m_description; }
    T const& getValue() const { return m_value; }
    
    boost::property_tree::ptree& getTree() const 
    {
        boost::property_tree::ptree t;
        t.put("description", getDescription());
        t.put("value", getValue());
        boost::property_tree::ptree output;
        output.put_child(getName(), t);
        return output;
    }

};


class LIBPC_DLL Options
{

private:
    boost::property_tree::ptree m_tree;

public:

    Options();
    Options(boost::property_tree::ptree const& tree) { m_tree = tree; }
    template<class T> void add(Option<T> const& option) { m_tree.put_child(option.getTree()); }
    template<class T> Option<T> const& get(std::string const& name) { return m_tree.get<T>(name); }
    
    boost::property_tree::ptree& GetPTree() { return m_tree; }
    boost::property_tree::ptree const& GetPTree() const { return m_tree; }
};


LIBPC_DLL std::ostream& operator<<(std::ostream& ostr, const Options&);

} // namespace pdal

#endif
