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

#include <list>

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

#include <boost/property_tree/ptree.hpp>

namespace pdal
{

class StageRunner;
class StageWrapper;

/**
  A stage performs the actual processing in PDAL.  Stages may read data,
  modify or filter read data, create metadata or write processed data.
*/
class PDAL_DLL Stage
{
    FRIEND_TEST(OptionsTest, conditional);
    friend class StageWrapper;
    friend class StageRunner;
public:
    Stage();
    virtual ~Stage()
        {}

    /**
      Add a stage to the input list of this stage.

      \param input  Stage to use as input.
    */
    void setInput(Stage& input)
        { m_inputs.push_back(&input); }

    /**
      Set a file descriptor to which progress information should be written.

      \param fd  Progress file descriptor.
    */
    void setProgressFd(int fd)
        { m_progressFd = fd; }

    /**
      Retrieve some basic point information without reading all data when
      possible.  Usually implemented only by Readers.
    */
    QuickInfo preview()
    {
        l_processOptions(m_options);
        processOptions(m_options);
        return inspect();
    }

    /**
      Prepare a stage for execution.  This function needs to be called on the
      terminal stage of a pipeline (linked set of stages) before \ref execute
      can be called.  Prepare recurses through all input stages.

      \param table  PointTable being used for stage pipeline.
    */
    void prepare(PointTableRef table);

    /**
      Execute a prepared pipeline (linked set of stages).

      This performs the action associated with the stage by executing the
      \ref run function of each stage in depth first order.  Each stage is run
      to completion (all points are processed) before the next stages is run.o

      \param table  Point table being used for stage pipeline.  This must be
        the same \ref table used in the \ref prepare function.
    */
    PointViewSet execute(PointTableRef table);

    /**
      Execute a prepared pipeline (linked set of stages) in streaming mode.

      This performs the action associated with the stage by executing the
      \ref processOne function of each stage in depth first order.  Points
      are processed up to the capacity of the provided StreamPointTable.
      Not all stages support streaming mode and an exception will be thrown
      when attempting to \ref execute an unsupported stage.

      Streaming points can reduce memory consumption, but may limit access
      to algorithms that need to operate on full point sets.

      \param table  Streming point table used for stage pipeline.  This must be
        the same \ref table used in the \ref prepare function.

    */
    void execute(StreamPointTable& table);

    /**
      Set the spatial reference of a stage.

      Set the spatial reference that will override that being carried by the
      PointView being processed.  This is usually used when reprojecting data
      to a new spatial reference.  The stage spatial reference will be carried
      by PointViews processes by this stage to subsequent stages.

      \param srs  Spatial reference to set.
    */
    void setSpatialReference(SpatialReference const& srs);

    /**
      Get the spatial reference of the stage.

      Get the spatial reference that will override that being carried by the
      PointView being processed.  This is usually used when reprojecting data
      to a new spatial reference.  The stage spatial reference will be carried
      by PointViews processes by this stage to subsequent stages.

      \return  The stage's spatial reference.
    */
    const SpatialReference& getSpatialReference() const;

    /**
      Set a stage's options.

      Set the options on a stage, clearing all previously set options.

      \param options  Options to set.
    */
    void setOptions(Options options)
        { m_options = options; }

    /**
      Add options if an option with the same name doesn't already exist on
      the stage.

      \param opts  Options to add.
    */
    void addConditionalOptions(const Options& opts);

    /**
      Add options to the existing option set.

      \param opts  Options to add.
    */
    void addOptions(const Options& opts)
    {
        for (const auto& o : opts.getOptions())
            m_options.add(o);
    }

    /**
      Remove options from a stage's option set.

      \param opts  Options to remove.
    */
    void removeOptions(const Options& opts)
    {
        for (const auto& o : opts.getOptions())
            m_options.remove(o);
    }

    /**
      Return the stage's log pointer.

      \return  Log pointer.
    */
    virtual LogPtr log() const
        { return m_log; }

    /**
      Determine whether the stage is in debug mode or not.

      \return  The stage's debug state.
    */
    bool isDebug() const
        { return m_options.getValueOrDefault<bool>("debug", false); }

    /**
      Return the name of a stage.

      \return  The stage's name.
    */
    virtual std::string getName() const = 0;

    /**
      Return the tag name of a stage.

      The tag name is used when writing a JSON pipeline.  It is generally
      the same as the stage name, but a number is appended to maintain
      uniqueness when stages appear more than once in a pipeline.
      the same as

      \return  The tag's name.
    */
    virtual std::string tagName() const
        { return getName(); }

    /**
      Return a list of the stage's inputs.

      \return  A vector pointers to input stages.
    **/
    const std::vector<Stage*>& getInputs() const
        { return m_inputs; }

    /**
      Return the stage's accepted options.

      \return  The options that a stage handles.
    */
    virtual Options getDefaultOptions()
        { return Options(); }

    /**
      Get the stage's metadata node.

      \return  Stage's metadata.
    */
    MetadataNode getMetadata() const
        { return m_metadata; }

    /**
      Serialize a stage by inserting apporpritate data into the provided
      MetadataNode.  Used to dump a pipeline specification in a portable
      format.

      \param root  Node to which a stages meatdata should be added.
      \param tags  Pipeline writer's current list of stage tags.
    */
    void serialize(MetadataNode root, PipelineWriter::TagMap& tags) const;

protected:
    Options m_options;          ///> Stage's options.
    MetadataNode m_metadata;    ///> Stage's metadata.
    int m_progressFd;           ///> Descriptor for progress info.

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
