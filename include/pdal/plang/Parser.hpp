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

#ifndef PARSER_H
#define PARSER_H

#include <pdal/pdal_internal.hpp>

#include <boost/cstdint.hpp>
#include <boost/variant.hpp>

#include <vector>
#include <string>


namespace pdal { namespace plang {


class Program;

typedef boost::variant<boost::uint8_t, boost::uint16_t, boost::uint32_t, boost::uint64_t,
                       boost::int8_t, boost::int16_t, boost::int32_t, boost::int64_t,
                       float, double, bool> variant_t;


class PDAL_DLL Parser
{
public:
    Parser(const std::string&);
    ~Parser();

    bool parse();

    // only valid after parse() or evaluate() returned false
    const std::vector<std::string>&  getErrors() const;

    template<typename T>
    void setVariable(const std::string& name, T value)
    {
        variant_t v = value;
        setVariableV(name, v);
    }
    
    void setVariableV(const std::string& name, variant_t value);

    template<typename T>
    T getVariable(const std::string& name)
    {
        variant_t value = getVariableV(name);
        T v = boost::get<T>(value);
        return v;
    }

    variant_t getVariableV(const std::string& name) const;

    bool evaluate();

private:
    const std::string m_text;
    bool m_parsed;
    Program* m_program;
    std::vector<std::string> m_errors;

    Parser& operator=(Parser const& rhs); // nope
};


} } // namespaces


#endif
