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
#include <pdal/QuickInfo.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/UserCallback.hpp>

#include <boost/property_tree/ptree.hpp>

namespace pdal
{

class Iterator;
class StageSequentialIterator;
class StageRandomIterator;
class StageBlockIterator;
class StageRunner;
class StageTester;

class PDAL_DLL Stage : public std::enable_shared_from_this<Stage>
{
    friend class StageTester;
    friend class StageRunner;
public:
    Stage();
    virtual ~Stage()
        {}

    void setInput(const std::vector<std::shared_ptr<Stage> >& inputs)
        { m_inputs = inputs; }
    void setInput(std::shared_ptr<Stage> input)
        { m_inputs.push_back(input); }

    QuickInfo preview()
    {
        l_processOptions(m_options);
        processOptions(m_options);
        return inspect();
    }
    void prepare(PointContextRef ctx);
    PointBufferSet execute(PointContextRef ctx);

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
        {
            return m_options.getValueOrDefault<bool>("debug", false);
        }
    bool isVerbose() const
        { return (getVerboseLevel() != 0 ); }
    uint32_t getVerboseLevel() const
        {
            return m_options.getValueOrDefault<uint32_t>("verbose", 0);
        }
    virtual std::string getName() const = 0;
    const std::vector<std::shared_ptr<Stage> >& getInputs() const
        { return m_inputs; }
    std::vector<std::shared_ptr<Stage> > findStage(std::string name);
    virtual Options getDefaultOptions()
        { return Options(); }
    static Dimension::IdList getDefaultDimensions()
        { return Dimension::IdList(); }
    static std::string s_getInfoLink()
        { return std::string(); }
    static std::string s_getPluginVersion()
        { return std::string(); }
    virtual boost::property_tree::ptree toPTree(PointContextRef ctx) const
        { return boost::property_tree::ptree(); }

    virtual StageSequentialIterator* createSequentialIterator() const
        { return NULL; }
    inline MetadataNode getMetadata() const
        { return m_metadata; }

    /// Sets the UserCallback to manage progress/cancel operations
    void setUserCallback(UserCallback* userCallback)
        { m_callback.reset(userCallback); }

protected:
    std::unique_ptr<UserCallback> m_callback;
    Options m_options;
    MetadataNode m_metadata;

    void setSpatialReference(MetadataNode& m, SpatialReference const&);

private:
    bool m_debug;
    uint32_t m_verbose;
    std::vector<std::shared_ptr<Stage> > m_inputs;
    LogPtr m_log;
    SpatialReference m_spatialReference;

    Stage& operator=(const Stage&); // not implemented
    Stage(const Stage&); // not implemented
    void Construct();
    void l_processOptions(const Options& options);
    virtual void processOptions(const Options& /*options*/)
        {}
    virtual void readerProcessOptions(const Options& /*options*/)
        {}
    virtual void writerProcessOptions(const Options& /*options*/)
        {}
    void l_initialize(PointContextRef ctx);
    void l_done(PointContextRef ctx);
    virtual QuickInfo inspect()
        { return QuickInfo(); }
    virtual void initialize()
        {}
    virtual void addDimensions(PointContextRef ctx)
        { (void)ctx; }
    virtual void prepared(PointContextRef ctx)
        { (void)ctx; }
    virtual void ready(PointContextRef ctx)
        { (void)ctx; }
    virtual void done(PointContextRef ctx)
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

