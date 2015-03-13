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

#pragma once

#include <pdal/pdal_internal.hpp>
#include <pdal/StageFactory.hpp>

#include <vector>
#include <string>


namespace pdal
{

class Options;
class PipelineManager;

class PDAL_DLL PipelineReader
{
private:
    class StageParserContext;

public:
    PipelineReader(PipelineManager&, bool debug=false,
        uint32_t verbose = 0);

    // Use this to fill in a pipeline manager with an XML file that
    // contains a <Writer> as the last pipeline stage.
    //
    // returns true iff the xml file is a writer pipeline (otherwise it is
    // assumed to be a reader pipeline)
    bool readPipeline(const std::string& filename);
    bool readPipeline(std::istream& input);

private:
    typedef std::map<std::string, std::string> map_t;

    bool parseElement_Pipeline(const boost::property_tree::ptree&);
    Stage *parseElement_anystage(const std::string& name,
        const boost::property_tree::ptree& subtree);
    Stage *parseElement_Reader(const boost::property_tree::ptree& tree);
    Stage *parseElement_Filter(const boost::property_tree::ptree& tree);
    Stage *parseElement_Writer(const boost::property_tree::ptree& tree);
    Option parseElement_Option(const boost::property_tree::ptree& tree);
    void collect_attributes(map_t& attrs,
        const boost::property_tree::ptree& tree);
    void parse_attributes(map_t& attrs,
        const boost::property_tree::ptree& tree);

private:
    PipelineManager& m_manager;
    bool m_isDebug;
    uint32_t m_verboseLevel;
    Options m_baseOptions;
    std::string m_inputXmlFile;

    PipelineReader& operator=(const PipelineReader&); // not implemented
    PipelineReader(const PipelineReader&); // not implemented
};

} // namespace pdal

