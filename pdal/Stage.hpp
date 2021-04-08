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

#include <pdal/Dimension.hpp>
#include <pdal/DimType.hpp>
#include <pdal/Log.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/Options.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PluginHelper.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/PointView.hpp>
#include <pdal/QuickInfo.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

class ProgramArgs;
class StageRunner;
class StageWrapper;
class Streamable;

namespace expr
{
    class ConditionalExpression;
}

/**
  A stage performs the actual processing in PDAL.  Stages may read data,
  modify or filter read data, create metadata or write processed data.

  Stages are linked with setInput() into a pipeline.  The pipeline is
  run with by calling in sequence \ref prepare() and \ref execute() on the
  stage at the end of the pipeline.  PipelineManager can also be used to
  create and run a pipeline.
*/
class PDAL_DLL Stage
{
    FRIEND_TEST(OptionsTest, conditional);
    friend class StageWrapper;
    friend class StageRunner;
    friend class Streamable;
    friend class Reader;
    friend class Filter;
    friend class Writer;

public:
    enum class WhereMergeMode
    {
        Auto,
        True,
        False
    };

    Stage();
    virtual ~Stage();
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
    QuickInfo preview();

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

    virtual void execute(StreamPointTable& table)
    {
        throw pdal_error("Attempting to use stream mode with a non-streamable "
            "stage.");
    }

    /**
      Determine if a pipeline with this stage as a sink is streamable.

      \return Whether the pipeline is streamable.
    */
    virtual bool pipelineStreamable() const
    { return false; }

    /**
      Return a pointer to a pipeline's first non-streamable stage,
      if one exists.

      \return  nullptr if the stage is streamable, a pointer to this stage
        otherwise.
    */
    virtual const Stage *findNonstreamable() const
    { return this; }

    /**
      Set the spatial reference of a stage.

      Set the spatial reference that will override that being carried by the
      PointView being processed.  This is usually used when reprojecting data
      to a new spatial reference.  The stage spatial reference will be carried
      by PointViews processes by this stage to subsequent stages.

      If called by a Reader whose spatial reference has been set with option
      'spatialreference' or 'override_srs', then this function will have no
      effect.

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
      Add a stage's options to a ProgramArgs set.

      \param args  ProgramArgs to add to.
    */
    void addAllArgs(ProgramArgs& args);

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
      Set the stage's log.

      \param log  Log pointer.
    */
    void setLog(const LogPtr& log)
        { m_log = log; }

    /**
      Return the stage's log pointer.

      \return  Log pointer.
    */
    virtual LogPtr log() const
        { return m_log; }

    /**
      Push the stage's leader into the log.
    */
    void startLogging() const;

    /**
        Pop the stage's leader from the log.
    */
    void stopLogging() const;

    /**
      Determine whether the stage is in debug mode or not.

      \return  The stage's debug state.
    */
    bool isDebug() const
        { return m_log && m_log->getLevel() > LogLevel::Debug; }

    /**
      Return the name of a stage.

      \return  The stage's name.
    */
    virtual std::string getName() const = 0;

    /**
        Set a specific tag name.
    */
    void setTag(const std::string& tag)
        { m_tag = tag; }

    /**
      Return the tag name of a stage.

      \return  The tag name.
    */
    virtual std::string tag() const
        { return m_tag; }

    /**
      Return a list of the stage's inputs.

      \return  A vector pointers to input stages.
    **/
    std::vector<Stage*>& getInputs()
        { return m_inputs; }

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

      \param root  Node to which a stages metadata should be added.
      \param tags  Pipeline writer's current list of stage tags.
    */
    void serialize(MetadataNode root, PipelineWriter::TagMap& tags) const;

    /**
      Parse a stage name from a string.  Return the name and update the
      position in the input string to the end of the stage name.

      \param o     Input string to parse.
      \param pos   Parsing start/end position.
      \return  Whether the parsed name is a valid stage name.
    */
    static bool parseName(std::string o, std::string::size_type& pos);

    /**
      Parse a tag name from a string.  Return the name and update the
      position in the input string to the end of the tag name.

      \param o    Input string to parse.
      \param pos  Parsing start/end position.
      \param tag  Parsed tag name.
      \return  Whether the parsed name is a valid tag name.
    */
    static bool parseTagName(std::string o, std::string::size_type& pos);

protected:
    Options m_options;          ///< Stage's options.
    MetadataNode m_metadata;    ///< Stage's metadata.
    int m_progressFd;           ///< Descriptor for progress info.

    virtual void setSpatialReference(MetadataNode& m, SpatialReference const&);
    void throwError(const std::string& s) const;
    /**
      Return the point count of all point views at the start of execution.
      Only valid during execute().

      \return  Total number of points in all point views being executed.
    */
    point_count_t pointCount() const
        { return m_pointCount; }
    /**
      Return the count of faces in all primary meshes for all point views.
      Only valid during execute().

      \return  Total number of faces in all point views being executed.
    */
    point_count_t faceCount() const
        { return m_faceCount; }

private:
    uint32_t m_verbose;
    std::string m_logname;
    std::vector<Stage *> m_inputs;
    LogPtr m_log;
    std::string m_logLeader;
    SpatialReference m_spatialReference;
    std::unique_ptr<ProgramArgs> m_args;
    std::string m_tag;
    // This is never used after it is set.  It just provides a place to
    // bind the user_data argument that is essentially a comment in pipeline
    // files.
    std::string m_userDataJSON;
    point_count_t m_pointCount;
    point_count_t m_faceCount;
    // This is never used, but we want something to bind to the argument
    // we stick in ProgramArgs so that it shows up in help and an options list.
    std::string m_optionFile;

    Stage& operator=(const Stage&) = delete;
    Stage(const Stage&) = delete;

    virtual const expr::ConditionalExpression *whereExpr() const = 0;
    virtual WhereMergeMode mergeMode() const = 0;
    void setupLog();
    void handleOptions();
    void countElements(const PointViewSet& views);

    virtual void l_addArgs(ProgramArgs& args);
    virtual void l_initialize(PointTableRef table);
    virtual void l_prepared(PointTableRef table);

    /**
      Potentially split a point view into keeps and skips based on a where clause.

      \param view  Point view to split.
      \param keep  PointView to hold the kept points.
      \param skip  PointView to hold the skipped points.
    */
    void splitView(const PointViewPtr& view, PointViewPtr& keep, PointViewPtr& skip);

    /**
      Get basic metadata (avoids reading points).  Implement in subclass.

      \return  QuickInfo data.
    */
    virtual QuickInfo inspect()
        { return QuickInfo(); }

    /**
      Add arguments(options) handled by this stage.  Implement in subclass.

      \param args  ProgramArgs object to which arguments should be added.
    */
    virtual void addArgs(ProgramArgs& /*args*/)
    {}

    /**
      Process options.  Implement in subclass.

      \param options  Options to process.
    */
    virtual void processOptions(const Options& /*options*/)
        {}

    /**
      Initialize stage after options have been processed.  Implement in
      subclass.  If you don't require the \ref table argument, you
      can implement the version of this function that takes no arguments
      instead of this function.

      \param table  PointTable associated with pipeline.
    */
    virtual void initialize(PointTableRef /*table*/)
        { initialize(); }

    /**
      Initialize stage after options have been processed.  Implement in
      subclass.
    */
    virtual void initialize()
        {}

    /**
      Add dimensions to a layout.

      \param layout  Point layout.
    */
    virtual void addDimensions(PointLayoutPtr /*layout*/)
        {}

    /**
      Execute a single stage.

      \param table  PointTable
      \param pvSet  Input PointViewSet
      \return  Output PointViewSet
    */
    PointViewSet execute(PointTableRef table, PointViewSet& pvSet);

    /**
      Functions called after dimensions have been added.  Implement in
      subclass.

      \param table  PointTable associated with pipeline.
    */
    virtual void prepared(PointTableRef /*table*/)
        {}

    /**
      First part of the execute step.  Called after all stages have been
      prepared.  Implement in subclass.

      \param table  PointTable associated with the pipeline.
    */
    virtual void ready(PointTableRef /*table*/)
        {}

    /**
      Pass all the point views at once to the stage for cases where we need
      such information.  You normally shouldn't need to implement this call.

      \param pvSet  PointViewSet being processed for the stage.
    */
    virtual void prerun(const PointViewSet& /*pvSet*/)
        {}

    /**
      Process all points in a view.  Implement in subclass.

      \param view  PointView to process.
    */
    virtual PointViewSet run(PointViewPtr /*view*/)
    {
        std::cerr << "Can't run stage = " << getName() << "!\n";
        return PointViewSet();
    }

    /**
      Called after all point views have been processed.  Implement in subclass.

      \param table  PointTable associated with pipeline.
    */
    virtual void done(PointTableRef /*table*/)
        {}

    /*
      Test hook.
    */
    const Options& getOptions() const
        { return m_options; }

    friend PDAL_DLL std::istream& operator>>(std::istream& in, WhereMergeMode& mode);
    friend PDAL_DLL std::ostream& operator<<(std::ostream& out, const WhereMergeMode& mode);
};

} // namespace pdal
