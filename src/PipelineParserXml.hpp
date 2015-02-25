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

#include "PipelineParser.hpp"

#include <map>

#ifndef PDAL_PLATFORM_WIN32
#include <wordexp.h>
#endif


namespace pdal
{


using namespace boost::property_tree;


class StageParserContext
{
// ------------------------------------------------------------------------

// this class helps keep tracks of what child nodes we've seen, so we
// can keep all the error checking in one place
public:

    enum Cardinality { None, One, Many };

    StageParserContext();

    void setCardinality(Cardinality cardinality);
    void addType();
    int getNumTypes();
    void addStage();
    void addUnknown(const std::string& name);
    void validate();

private:

    int m_numTypes;
    Cardinality m_cardinality; // num child stages allowed
    int m_numStages;

};


class PipelineParserXml : public PipelineParser
{
public:

    PipelineParserXml(PipelineManager& manager, const Options& baseOptions);

    virtual bool parse(const std::string& filename);
    virtual bool parse(std::istream& input);

private:

    typedef std::map<std::string, std::string> map_t;

    Option parseElement_Option(const ptree& tree);
    Stage* parseElement_anystage(const std::string& name, const ptree& subtree);
    Reader* parseElement_Reader(const ptree& tree);
    Filter* parseElement_Filter(const ptree& tree);
    void parse_attributes(map_t& attrs, const ptree& tree);
    void collect_attributes(map_t& attrs, const ptree& tree);
    Writer* parseElement_Writer(const ptree& tree);
    bool parseElement_Pipeline(const ptree& tree);

};


} // namespace pdal
