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

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <pdal/DimUtil.hpp>

namespace pdal
{

struct dimbuilder_error
{
    dimbuilder_error(const std::string& s) : m_error(s)
    {}

    std::string m_error;
};

struct DimSpec
{
    std::string m_name;
    std::string m_description;
    Dimension::Type m_type;
    std::vector<std::string> m_altNames;
};

class DimBuilder
{
public:
    DimBuilder()
    {}

    bool parseArgs(int argc, char *argv[]);
    bool execute();

private:
    std::string m_input;
    std::string m_output;
    std::vector<DimSpec> m_dims;

    void extractDim(NL::json& dim);
    void writeOutput(std::ostream& out);
    void writeHeader(std::ostream& out);
    void writeFooter(std::ostream& out);
    void writeIds(std::ostream& out);
    void writeDescriptions(std::ostream& out);
    void writeNameToId(std::ostream& out);
    void writeIdToName(std::ostream& out);
    void writeTypes(std::ostream& out);
    void validateDimension(const std::string& dimName);
};

} // namespace pdal
