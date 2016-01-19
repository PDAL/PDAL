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
#include <pdal/plugin.hpp>

#include <pdal/Dimension.hpp>
#include <pdal/Log.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/Options.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/PointView.hpp>
#include <pdal/QuickInfo.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/UserCallback.hpp>

#include <boost/property_tree/ptree.hpp>

#include <gtest/gtest.h>

namespace pdal
{

class StageRunner;
class StageWrapper;

class PDAL_DLL Stage
{
    FRIEND_TEST(OptionsTest, conditional);
    friend class StageWrapper;
    friend class StageRunner;
public:
    Stage();
    virtual ~Stage()
        {}

    void setInput(Stage& input)
        { m_inputs.push_back(&input); }

    void setProgressFd(int fd)
        { m_progressFd = fd; }

    QuickInfo preview()
    {
        l_processOptions(m_options);
        processOptions(m_options);
        return inspect();
    }
    void prepare(PointTableRef table);
    PointViewSet execute(PointTableRef table);
    void execute(StreamPointTable& table);

    void setSpatialReference(SpatialReference const&);
    const SpatialReference& getSpatialReference() const;
    void setOptions(Options options)
        { m_options = options; }
    void addConditionalOptions(const Options& opts);
    void addOptions(const Options& opts)
    {
        for (const auto& o : opts.getOptions())
            m_options.add(o);
    }
    void removeOptions(const Options& opts)
    {
        for (const auto& o : opts.getOptions())
            m_options.remove(o);
    }
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
    virtual std::string tagName() const
        { return getName(); }
    const std::vector<Stage*>& getInputs() const
        { return m_inputs; }
    std::vector<Stage *> findStage(std::string name);
    virtual Options getDefaultOptions()
        { return Options(); }
    static Dimension::IdList getDefaultDimensions()
        { return Dimension::IdList(); }
    static std::string s_getPluginVersion()
        { return std::string(); }
    inline MetadataNode getMetadata() const
        { return m_metadata; }
    void serialize(MetadataNode root, PipelineWriter::TagMap& tags) const;

    /// Sets the UserCallback to manage progress/cancel operations
    void setUserCallback(UserCallback* userCallback)
        { m_callback.reset(userCallback); }

protected:
    std::unique_ptr<UserCallback> m_callback;
    Options m_options;
    MetadataNode m_metadata;
    int m_progressFd;

    void setSpatialReference(MetadataNode& m, SpatialReference const&);

private:
    bool m_debug;
    uint32_t m_verbose;
    std::vector<Stage *> m_inputs;
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
    void l_initialize(PointTableRef table);
    virtual QuickInfo inspect()
        { return QuickInfo(); }
    virtual void initialize(PointTableRef /*table*/)
        { initialize(); }
    virtual void initialize()
        {}
    virtual void addDimensions(PointLayoutPtr /*layout*/)
        {}
    virtual void prepared(PointTableRef /*table*/)
        {}
    virtual void ready(PointTableRef /*table*/)
        {}
    virtual void done(PointTableRef /*table*/)
        {}
    virtual bool processOne(PointRef& /*point*/)
    {
        std::ostringstream oss;
        oss << "Point streaming not supported for stage " << getName() << ".";
        throw pdal_error(oss.str());
    }
    virtual PointViewSet run(PointViewPtr /*view*/)
    {
        std::cerr << "Can't run stage = " << getName() << "!\n";
        return PointViewSet();
    }
    void execute(StreamPointTable& table, std::list<Stage *>& stages);
    const Options& getOptions() const
        { return m_options; }
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Stage&);

} // namespace pdal
