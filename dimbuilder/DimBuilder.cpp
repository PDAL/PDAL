/******************************************************************************
* Copyright (c) 2016, hobu Inc.  (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
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

#include <iostream>

#include "DimBuilder.hpp"

#include <pdal/util/ProgramArgs.hpp>

int main(int argc, char *argv[])
{
    pdal::DimBuilder d;

    argc--;
    argv++;
    try
    {
        if (d.parseArgs(argc, argv))
            d.execute();
    }
    catch (pdal::dimbuilder_error& e)
    {
        std::cerr << e.m_error << "\n";
        return -1;
    }
    return 0;
}


namespace pdal
{

namespace
{

std::string getTypename(Dimension::Type type)
{
    switch (type)
    {
    case Dimension::Type::None:
        return "None";
    case Dimension::Type::Unsigned8:
        return "Unsigned8";
    case Dimension::Type::Signed8:
        return "Signed8";
    case Dimension::Type::Unsigned16:
        return "Unsigned16";
    case Dimension::Type::Signed16:
        return "Signed16";
    case Dimension::Type::Unsigned32:
        return "Unsigned32";
    case Dimension::Type::Signed32:
        return "Signed32";
    case Dimension::Type::Unsigned64:
        return "Unsigned64";
    case Dimension::Type::Signed64:
        return "Signed64";
    case Dimension::Type::Float:
        return "Float";
    case Dimension::Type::Double:
        return "Double";
    }
    return "None";
}

} // unnamed namespace

bool DimBuilder::parseArgs(int argc, char *argv[])
{
    ProgramArgs args;

    args.add("input,i", "Filename of JSON specification of "
        "dimensions", m_input).setPositional();
    args.add("output,o", "Filename of output of C++ header representation of "
        "provided JSON.", m_output).setPositional();
    try
    {
        std::vector<std::string> s;
        for (int i = 0; i < argc; ++i)
            s.push_back(argv[i]);
        args.parse(s);
    }
    catch (arg_error& err)
    {
        throw dimbuilder_error(err.what());
    }
    return true;
}


bool DimBuilder::execute()
{
    std::ifstream in(m_input);

    if (!in)
        throw dimbuilder_error("Can't open input file.");

    NL::json root;
    try
    {
        in >> root;
    }
    catch (NL::json::parse_error& err)
    {
        throw dimbuilder_error(err.what());
    }

    NL::json dims;
    auto it = root.find("dimensions");
    if (it != root.end())
        dims = *it;
    if (root.size() != 1 || !dims.is_array())
    {
        std::ostringstream oss;

        oss << "Root node must contain a single 'dimensions' array.";
        throw dimbuilder_error(oss.str());
    }
    for (auto& dim : dims)
    {
        if (!dim.is_object())
        {
            std::ostringstream oss;

            oss << "Found a dimension that is not an object: " <<
                dim.get<std::string>();
            throw dimbuilder_error(oss.str());
        }
        extractDim(dim);
    }

    std::ofstream out(m_output);
    if (!out)
    {
        std::ostringstream oss;

        oss << "Unable to open output file '" << m_output << "'.";
        throw dimbuilder_error(oss.str());
    }
    writeOutput(out);
    return true;
}


void DimBuilder::extractDim(NL::json& dim)
{
    DimSpec d;

    // Get dimension name.
    auto it = dim.find("name");
    if (it == dim.end())
        throw dimbuilder_error("Dimension missing name.");
    NL::json name = *it;
    if (!name.is_string())
        throw dimbuilder_error("Dimension name must be a string.");
    d.m_name = name.get<std::string>();
    validateDimension(d.m_name);
    dim.erase(it);

    // Get dimension description.
    it = dim.find("description");
    if (it == dim.end())
    {
        std::ostringstream oss;

        oss << "Dimension '" << d.m_name << "' must have a description.";
        throw dimbuilder_error(oss.str());
    }
    NL::json description = *it;
    if (!description.is_string())
    {
        std::ostringstream oss;

        oss << "Description of dimension '" << d.m_name << "' must be a "
            "string.";
        throw dimbuilder_error(oss.str());
    }
    d.m_description = description.get<std::string>();
    dim.erase(it);

    // Get dimension type
    it = dim.find("type");
    if (it == dim.end())
    {
        std::ostringstream oss;

        oss << "Dimension '" << d.m_name << "' must have a type.";
        throw dimbuilder_error(oss.str());
    }
    NL::json dimType = *it;
    if (!dimType.is_string())
    {
        std::ostringstream oss;

        oss << "Type of dimension '" << d.m_name << "' must be a "
            "string.";
        throw dimbuilder_error(oss.str());
    }
    d.m_type = Dimension::type(dimType.get<std::string>());
    if (d.m_type == Dimension::Type::None)
    {
        std::ostringstream oss;

        oss << "Invalid type '" << dimType.get<std::string>() <<
            "' specified for " "dimension '" << d.m_name << "'.";
        throw dimbuilder_error(oss.str());
    }
    dim.erase(it);

    NL::json altNames;
    it = dim.find("alt_names");
    if (it != dim.end())
    {
        altNames = *it;
        dim.erase(it);
    }

    if (!altNames.is_null())
    {
        bool typeError = false;

        if (altNames.is_string())
        {
            try
            {
                std::string s = altNames.get<std::string>();
                d.m_altNames = Utils::split2(s, ',');
            }
            catch (NL::json::parse_error&)
            {
                typeError = true;
            }
        }
        else if (altNames.is_array())
        {
            for (auto it = altNames.begin(); it != altNames.end(); ++it)
                if (it->is_string())
                    d.m_altNames.push_back(it->get<std::string>());
                else
                    typeError = true;
        }
        else
            typeError = true;

        
        if (typeError)
        {
            std::ostringstream oss;

            oss << "Alternate names for dimension '" << d.m_name << "' must "
                "be a string.";
            throw dimbuilder_error(oss.str());
        }
        for (auto& s : d.m_altNames)
        {
            Utils::trim(s);
            validateDimension(s);
        }
    }
    if (dim.size() > 0)
    {
        std::ostringstream oss;

        oss << "Unexpected member '" << dim.begin().key() << "' when "
            "reading dimension '" << d.m_name << "'.";
        throw dimbuilder_error(oss.str());
    }
    m_dims.push_back(d);
}

void DimBuilder::validateDimension(const std::string& dimName)
{
    if (Dimension::extractName(dimName, 0) != dimName.size())
    {
        std::ostringstream oss;

        oss << "Invalid dimension name '" << dimName << "'.  Dimension "
            "names must start with a letter and be followed by letters, "
            "digits or underscores.";
        throw dimbuilder_error(oss.str());
    }
    for (DimSpec& d : m_dims)
    {
        if (d.m_name == dimName)
        {
            std::ostringstream oss;

            oss << "Duplicate dimension name '" << dimName << "' found. "
                "Dimension names must be unique.";
            throw dimbuilder_error(oss.str());
        }
    }
}

void DimBuilder::writeOutput(std::ostream& out)
{
    writeHeader(out);
    out << "\n";
    writeIds(out);
    out << "\n";
    writeDescriptions(out);
    out << "\n";
    writeNameToId(out);
    out << "\n";
    writeIdToName(out);
    out << "\n";
    writeTypes(out);
    out << "\n";
    writeFooter(out);
}


void DimBuilder::writeHeader(std::ostream& out)
{
    out << "// This file was programatically generated from '" << m_input <<
        ".\n";
    out << "// Do not edit directly.\n";
    out << "\n";
    out << "#pragma once\n";
    out << "\n";
    out << "#include <string>\n";
    out << "#include <vector>\n";
    out << "\n";
    out << "#include <pdal/DimUtil.hpp>\n";
    out << "#include <pdal/pdal_types.hpp>\n";
    out << "#include <pdal/util/Utils.hpp>\n";
    out << "\n";
    out << "namespace pdal\n";
    out << "{\n";
    out << "namespace Dimension\n";
    out << "{\n";
}


void DimBuilder::writeFooter(std::ostream& out)
{
    out << "} // namespace Dimension\n";
    out << "} // namespace pdal\n";
    out << "\n";
}


void DimBuilder::writeIds(std::ostream& out)
{
    out << "enum class Id\n";
    out << "{\n";
    out << "    Unknown,\n";
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        DimSpec& d = *di;
        out << "    " << d.m_name;
        if (di + 1 != m_dims.end())
            out << ",";
        out << "\n";
    }
    out << "};\n";
    out << "typedef std::vector<Id> IdList;\n";
    out << "\n";
}


void DimBuilder::writeDescriptions(std::ostream& out)
{
    out << "/// Get a description of a predefined dimension.\n";
    out << "/// \\param[in] id  Dimension ID.\n";
    out << "/// \\return  Dimension description.\n";
    out << "inline std::string description(Id id)\n";
    out << "{\n";
    out << "    switch (id)\n";
    out << "    {\n";
    for (auto& d : m_dims)
    {
        std::vector<std::string> pieces = Utils::wordWrap2(d.m_description,
            63, 60);
        out << "    case Id::" << d.m_name << ":\n";
        out << "        return \"" << pieces[0] << "\"";
        if (pieces.size() == 1)
            out << ";";
        out << "\n";
        auto pi = pieces.begin();
        pi++;
        for (; pi != pieces.end(); ++pi)
        {
            std::string piece = *pi;
            out << "            \"" << piece << "\"";
            if (pi + 1 == pieces.end())
                out << ";";
            out << "\n";
        }
    }
    out << "    case Id::Unknown:\n";
    out << "        return \"\";\n";
    out << "    }\n";
    out << "    return \"\";\n";
    out << "}\n";
}


void DimBuilder::writeNameToId(std::ostream& out)
{
    out << "/// Get a predefined dimension ID given a dimension name. "
        "Multiple names\n";
    out << "/// may map to the same dimension for convenience.  Names "
        "are case-insensitive.\n";
    out << "/// \\param[in] s  Name of dimension.\n";
    out << "/// \\return  Dimension ID associated with the name.  "
        "Id::Unknown is returned\n";
    out << "///    if the name doesn't map to a predefined dimension.\n";
    out << "inline Id id(std::string s)\n";
    out << "{\n";
    out << "    s = Utils::toupper(s);\n";
    out << "\n";
    for (auto& d : m_dims)
    {
        std::vector<std::string> names;
        names.push_back(d.m_name);
        names.insert(names.end(), d.m_altNames.begin(), d.m_altNames.end());
        for (std::string& s : names)
        {
            out << "    if (s == \"" << Utils::toupper(s) << "\")\n";
            out << "        return Id::" << d.m_name << ";\n";
        }
    }
    out << "    return Id::Unknown;\n";
    out << "}\n";
}


void DimBuilder::writeIdToName(std::ostream& out)
{
    out << "/// Get the name of a predefined dimension.\n";
    out << "/// \\param[in] id  Dimension ID\n";
    out << "/// \\return  Dimension name.\n";
    out << "inline std::string name(Id id)\n";
    out << "{\n";
    out << "    switch (id)\n";
    out << "    {\n";
    for (auto& d : m_dims)
    {
        out << "    case Id::" << d.m_name << ":\n";
        out << "        return \"" << d.m_name << "\";\n";
    }
    out << "    case Id::Unknown:\n";
    out << "        return \"\";\n";
    out << "    }\n";
    out << "    return \"\";\n";
    out << "}\n";
}

void DimBuilder::writeTypes(std::ostream& out)
{
    out << "/// Get the default storage type of a predefined dimension.\n";
    out << "/// \\param id  ID of the predefined dimension.\n";
    out << "/// \\return  The dimension's default storage type.  An "
        "exception is thrown if\n";
    out << "///   the id doesn't represent a predefined dimension.\n";
    out << "inline Type defaultType(Id id)\n";
    out << "{\n";
    out << "    switch (id)\n";
    out << "    {\n";
    for (auto& d : m_dims)
    {
        out << "    case Id::" << d.m_name << ":\n";
        out << "        return Type::" << getTypename(d.m_type) << ";\n";
    }
    out << "    case Id::Unknown:\n";
    out << "        throw pdal_error(\"No type found for undefined "
        "dimension.\");\n";
    out << "    }\n";
    out << "    throw pdal_error(\"No type found for undefined "
        "dimension.\");\n";
    out << "}\n";
}

} // namespace pdal

