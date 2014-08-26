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

#include <pdal/Dimension.hpp>
#include <pdal/Log.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/SpatialReference.hpp>

#include <boost/property_tree/ptree.hpp>

namespace pdal
{

class Iterator;
class StageSequentialIterator;
class StageRandomIterator;
class StageBlockIterator;
class StageRunner;
class StageTester;
//
// supported options:
//   <bool>debug
//   <uint32>verbose
//

class PDAL_DLL Stage
{
    friend class StageTester;
    friend class StageRunner;
public:
    Stage();
    Stage(const Options& options);
    virtual ~Stage()
        {}

    void setInput(const std::vector<Stage *>& inputs)
        { m_inputs = inputs; }
    void setInput(Stage *input)
        { m_inputs.push_back(input); }

    void prepare(PointContext ctx);
    PointBufferSet execute(PointContext ctx);

    void setSpatialReference(SpatialReference const&);
    const SpatialReference& getSpatialReference() const;
    const Options& getOptions() const
        { return m_options; }
    void setOptions(Options options)
        { m_options = options; }
    virtual boost::property_tree::ptree serializePipeline() const = 0;
    virtual LogPtr log() const
        { return m_log; }
    bool isDebug() const
        { return m_debug; }
    bool isVerbose() const
        { return (m_verbose != 0 ); }
    boost::uint32_t getVerboseLevel() const
        { return m_verbose; }
    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;
    const std::vector<Stage *>& getInputs() const
        { return m_inputs; }
    std::vector<Stage*> findStage(std::string name);
    static Options getDefaultOptions()
        { return Options(); }
    static Dimension::IdList getDefaultDimensions()
        { return Dimension::IdList(); }
    static std::string s_getInfoLink()
        { return std::string(); }
    virtual boost::property_tree::ptree toPTree(PointContext ctx) const 
        { return boost::property_tree::ptree(); }

#define SET_STAGE_NAME(name, description)  \
    static std::string s_getName() { return name; }  \
    std::string getName() const { return name; }  \
    static std::string s_getDescription() { return description; }  \
    std::string getDescription() const { return description; }

#define SET_STAGE_LINK(infolink) \
    static std::string s_getInfoLink() { return infolink; }  \
    std::string getInfoLink() const { return infolink; }

#define SET_STAGE_ENABLED(YES_OR_NO) \
    static bool s_isEnabled() { return YES_OR_NO; } \
    bool isEnabled() const { return YES_OR_NO; }

    virtual StageSequentialIterator*
    createSequentialIterator(PointBuffer&) const
        { return NULL; }
    virtual StageSequentialIterator*
    createSequentialIterator() const
        { std::cerr << "Created crap sequential iterator!\n"; return NULL; }
    virtual StageRandomIterator* createRandomIterator(PointBuffer&) const
        { return NULL; }
    inline MetadataNode getMetadata() const 
        { return m_metadata; }

protected:
    Options m_options;
    MetadataNode m_metadata;

    void setSpatialReference(MetadataNode& m, SpatialReference const&);

private:
    bool m_debug;
    boost::uint32_t m_verbose;
    std::vector<Stage *> m_inputs;
    LogPtr m_log;
    SpatialReference m_spatialReference;

    Stage& operator=(const Stage&); // not implemented
    Stage(const Stage&); // not implemented
    void Construct();
    void l_processOptions(const Options& options);
    virtual void processOptions(const Options& /*options*/)
        {}
    virtual void writerProcessOptions(const Options& /*options*/)
        {}
    void l_initialize(PointContext ctx);
    void l_done(PointContext ctx);
    virtual void initialize()
        {}
    virtual void addDimensions(PointContext ctx)
        { (void)ctx; }
    virtual void ready(PointContext ctx)
        { (void)ctx; }
    virtual void done(PointContext ctx)
        { (void)ctx; }
    virtual PointBufferSet run(PointBufferPtr buffer)
    {
        (void)buffer;
        std::cerr << "Can't run stage = " << getName() << "!\n";
        return PointBufferSet();
    }
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Stage&);

} // namespace pdal

