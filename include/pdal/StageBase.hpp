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

#ifndef INCLUDED_STAGEBASE_HPP
#define INCLUDED_STAGEBASE_HPP

#include <boost/shared_ptr.hpp>

#include <pdal/pdal_internal.hpp>
#include <pdal/Log.hpp>

#include <string>
#include <vector>
#include <iosfwd>

namespace pdal
{


class Stage;
class Dimension;

//
// supported options:
//   <uint32>id
//   <bool>debug
//   <uint32>verbose
//

// both Stages and Writers have a few common properties, so
class PDAL_DLL StageBase
{
    friend class Stage;
public:

    /// Constructor.
    ///
    /// @param  inputs  The input stages.
    /// @param  options Options for controlling the pipeline stage.
    StageBase(const std::vector<StageBase*>& inputs, const Options& options);

    /// Destructor.
    virtual ~StageBase();

    /// Initializes this object.
    ///
    /// This function is for derived stages to perform "static" validation, e.g. bad Option arguments.
    /// It will recursively call initialize() on all previous stages.
    /// Users must call this after the last stage in the pipeline has been consturcted.
    /// It is illegal to call this twice for a stage.
    /// Derived stages should feel free to provide their own implementations.  Remember to call initialize() on
    ///   the parent class before your own class-specific code.
    /// This function will throw when errors are found.
    virtual void initialize();

    /// Query if this object is initialized.
    ///
    /// @return true if initialized, false if not.
    bool isInitialized() const;

    /// Gets the Options set for the stage.
    ///
    /// @return The options.
    const Options& getOptions() const;


    /// Returns the serialized pipeline.
    ///
    /// This is used to generate pipeline xml files.  It will
    /// recursively visit all child stages to populate the tree.
    ///
    /// @return the ptree for the stage
    virtual boost::property_tree::ptree serializePipeline() const = 0;

    /// Put data to the log
    /// @param input a string to put into the Stage's log
    // virtual void log(std::ostringstream& input, boost::uint32_t nVerbosity = 1) const;

    virtual LogPtr log(void) const
    {
        return m_log;
    }


    /// Put data to the log
    /// @param input a string to put into the Stage's log
    // virtual void log(std::string const& input, boost::uint32_t nVerbosity = 1) const;

    /// Query if this object is debug.
    ///
    /// This is set by the "debug" option, which is a boolean.
    ///
    /// This is intended to be used for adding debug code to stages, e.g. more than just the
    /// extra logging that "verbose" implies.
    ///
    /// @return true if debug, false if not.
    bool isDebug() const;


    /// Query if this object is verbose.
    ///
    /// This is set by the "verbose" option
    ///    0 - no verbosity at all
    ///    >0 - meaning is left to the implementors of the individual stages
    ///
    /// "Verbose" is intended to only add logging/tracing/output functionality; to add or enable
    /// extra validation checks and such (code which is potentially side-effecting) you want to use
    /// the "debug" option.
    ///
    /// @return true if verbosity>0, false if not.

    bool isVerbose() const;

    /// Gets the verbose level.
    ///
    /// @return The verbose level.
    boost::uint32_t getVerboseLevel() const;


    /// Gets the default options.
    ///
    /// Everyone must implement this.  If you want to access the list of
    /// options "statically", you are free to construct the stage with no
    /// arguments and cal getDefaultOptions() on it -- there is no need
    /// to call initialize(), so it should be a fast/safe operation.
    ///
    /// @return The default options.
    static Options getDefaultOptions()
    {
        return Options();
    }
    
    /// Gets the default dimensions that a give stage produces
    static std::vector<Dimension> getDefaultDimensions()
    {
        return std::vector<Dimension>();
    }

    /// Gets the name.
    ///
    /// Use a dotted, XPath-style name for your
    /// stage.  For example, 'drivers.las.reader' or 'filters.crop'.  This
    /// XPath-style name will also correspond to an entry in the pdal::Options
    /// tree for the given stage.
    ///
    /// @return The name.
    virtual std::string getName() const = 0;

    /// Gets the description.
    ///
    /// @return The description.
    virtual std::string getDescription() const = 0;

    /// For getName() and getDescription(), each stage provides a static and
    /// a dynamic version of the function.  Each (concrete) stage should call
    /// the following macro to create the functions for you.

#define SET_STAGE_NAME(name, description)  \
    static std::string s_getName() { return name; }  \
    std::string getName() const { return name; }  \
    static std::string s_getDescription() { return description; }  \
    std::string getDescription() const { return description; }

    /// Converts this object to a ptree.
    ///
    /// @return This object as a boost::property_tree::ptree.
    virtual boost::property_tree::ptree toPTree() const;

    /// Dumps this object.
    virtual void dump() const;

    /// Gets the stage's id.
    ///
    /// @return The id.
    boost::uint32_t getId() const
    {
        return m_id;
    }

    /// Gets the list of input stages
    ///
    /// @return vector of input stages (may be empty)
    const std::vector<StageBase*>& getInputs() const;

    /// Gets the list of output stages
    ///
    /// @return vector of output stages (may be empty)
    const std::vector<StageBase*>& getOutputs() const;

    /// Gets the previous stage.
    ///
    /// convenience function: returns the first input stage, as a Stage&
    /// (don't call this unless you know it is safe to do so, e.g Readers
    /// should not use this since they have no input stages)
    ///
    /// @return The previous stage.
    Stage& getPrevStage() const;

    /// Gets the previous stages.
    ///
    /// convenience function: returns the input stages, as a Stage* vector
    /// (don't call this unless you know it is safe to do so, e.g Readers
    /// should not use this since they have no input stages, and Filters
    /// probably won't want to use this either since they only have one
    /// prev stage)
    ///
    /// @return null if it fails, else the previous stages.
    std::vector<Stage*> getPrevStages() const;

    virtual Metadata getMetadata() const;
    virtual Metadata collectMetadata() const;

protected:

    /// Gets the options.
    ///
    /// @return The options.
    Options& getOptions();

    inline StageOperationType getDimensionOperationType() const
    {
        return m_dimensionsType;
    }
    /// Makes an "empty" vector of StageBase pointers.
    ///
    /// This is used by the ctors of the derived classes, so they can call
    /// the ctor of StageBase.  This function goes along with the other
    /// makeVector() functions.
    ///
    /// @return an empty vector
    static std::vector<StageBase*> makeVector();

    /// Makes a vector of one StageBase pointer from a single Stage reference
    ///
    /// This is used by the ctors of the derived classes, so they can call
    /// the ctor of StageBase.  C++ doesn't support polymorphic arrays or
    /// inlined initialization of vectors, so we do it this way.
    ///
    /// @param src the Stage reference
    ///
    /// @return a vector of one StageBase pointer
    static std::vector<StageBase*> makeVector(Stage& src);

    /// Makes a vector of StageBase pointers from a vector of Stages
    ///
    /// This is used by the ctors of the derived classes, so they can call
    /// the ctor of StageBase.  C++ doesn't support polymorphic arrays or
    /// inlined initialization of vectors, so we do it this way.
    ///
    /// @param src the vector of Stages
    ///
    /// @return a vector of StageBase pointers
    static std::vector<StageBase*> makeVector(const std::vector<Stage*>& src);

    /// @return a modifiable reference to the metadata for the stage
    Metadata& getMetadataRef()
    {
        return m_metadata;
    }

private:
    bool m_initialized;
    Options m_options;
    bool m_debug;
    boost::uint32_t m_verbose;
    const boost::uint32_t m_id;

    std::vector<StageBase*> m_inputs;
    std::vector<StageBase*> m_outputs;
    StageOperationType m_dimensionsType;
    LogPtr m_log;
    Metadata m_metadata;

    StageBase& operator=(const StageBase& rhs); // not implemented
    StageBase(const StageBase&); // not implemented
};

/// Output operator for serialization
///
/// @param ostr    The output stream to write to
/// @param src     The StageBase to be serialized out
///
/// @return The output stream

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const StageBase& src);

} // namespace pdal

#endif
