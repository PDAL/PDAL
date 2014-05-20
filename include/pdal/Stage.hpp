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

#ifndef INCLUDED_STAGE_HPP
#define INCLUDED_STAGE_HPP

#include <pdal/pdal_internal.hpp>

#include <pdal/Bounds.hpp>
#include <pdal/Log.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/SpatialReference.hpp>

namespace pdal
{

class Iterator;
class StageSequentialIterator;
class StageRandomIterator;
class StageBlockIterator;
class StageRunner;
//
// supported options:
//   <uint32>id
//   <bool>debug
//   <uint32>verbose
//

class PDAL_DLL Stage
{
    friend class StageRunner;
public:
    Stage();
    Stage(const Options& options);
    virtual ~Stage()
        {}

    void setInput(const std::vector<Stage *>& inputs);
    void setInput(Stage *input);

    void prepare(PointContext ctx);
    PointBufferSet execute(PointContext ctx);
    bool isInitialized() const
        { return m_initialized; }

    inline Schema const& getSchema() const
        { return *(m_context.getSchema()); }
    
    virtual boost::uint64_t getNumPoints() const;
    const Bounds<double>& getBounds() const;
    const SpatialReference& getSpatialReference() const;
    const Options& getOptions() const
        { return m_options; }
    virtual boost::property_tree::ptree serializePipeline() const = 0;
    virtual LogPtr log() const
        { return m_log; }
    bool isDebug() const
        { return m_debug; }
    bool isVerbose() const
        { return (bool)m_verbose; }
    boost::uint32_t getVerboseLevel() const
        { return m_verbose; }
    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;
    boost::uint32_t getId() const
        { return m_id; }
    const std::vector<Stage *>& getInputs() const
        { return m_inputs; }
    const std::vector<Stage *>& getOutputs() const
        { return m_outputs; }
    Stage& getPrevStage() const;
    std::vector<Stage *> getPrevStages() const;
    virtual Metadata getMetadata() const
        { return m_metadata; }
    virtual Metadata collectMetadata() const;
    static Options getDefaultOptions()
        { return Options(); }
    static std::vector<Dimension> getDefaultDimensions()
        { return std::vector<Dimension>(); }
    static std::string s_getInfoLink()
        { return std::string(); }

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

protected:
    PointContext m_context;
    Schema m_schema;
    Options m_options;
    Bounds<double> m_bounds;

    StageOperationType getDimensionOperationType() const
        { return m_dimensionsType; }
    void setSchema(Schema const&);
    void setNumPoints(boost::uint64_t);
    void setBounds(Bounds<double> const&);
    void setSpatialReference(SpatialReference const&);
    Metadata& getMetadataRef()
        { return m_metadata; }

    // convenience function, for doing a "copy ctor" on all the core props
    // (used by the Filter stage, for example)
    void setCoreProperties(const Stage&);

    static std::vector<Stage *> makeVector()
        { return std::vector<Stage *>(); }
    static std::vector<Stage *> makeVector(Stage& src);
    static std::vector<Stage *> makeVector(const std::vector<Stage *>& src);

private:
    bool m_initialized;
    bool m_debug;
    boost::uint32_t m_verbose;
    boost::uint32_t m_id;
    std::vector<Stage *> m_inputs;
    std::vector<Stage *> m_outputs;
    StageOperationType m_dimensionsType;
    LogPtr m_log;
    Metadata m_metadata;
    mutable boost::uint64_t m_numPoints;
    SpatialReference m_spatialReference;

    Stage& operator=(const Stage&); // not implemented
    Stage(const Stage&); // not implemented
    void Construct();
    void l_processOptions(const Options& options);
    virtual void processOptions(const Options& options)
        {}
    void l_initialize();
    virtual void initialize()
        {}
    virtual void buildSchema(Schema *schema)
        { (void)schema; }
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

#endif
