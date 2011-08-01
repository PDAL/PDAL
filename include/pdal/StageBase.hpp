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

#include <pdal/pdal.hpp>
#include <pdal/Options.hpp>

#include <string>


namespace pdal
{


// both Stages and Writers have a few common properties, so 
class PDAL_DLL StageBase
{
public:
    StageBase(const Options& options);
    virtual ~StageBase();

    // This function is for derived stages to perform "static" validation, e.g. bad Option arguments.
    // It will recursively call initialize() on all previous stages.
    // Users must call this after the last stage in the pipeline has been consturcted.  
    // It is not illegal to call this twice for a stage, but that doesn't mean you should.
    // Derived stages should feel free to provide their own implementations.  Remeber to call initialize() on
    //   the parent class before your own class-specific code.
    // This function will throw when errors are found.
    virtual void initialize();
    bool isInitialized() const;

    const Options& getOptions() const;

    // This is used to generate pipeline xml files.  It will
    // recursively visit all child stages to populate the tree.
    virtual boost::property_tree::ptree generatePTree() const = 0;

    // This is set by the "debug" option, which is a boolean.
    // 
    // This is intended to be used for adding debug code to stages, e.g. more than just the
    // extra logging that "verbose" implies.
    bool isDebug() const;
    
    // This is set by the "verbose" option, which is in range [0..255].
    //    0 - no verbosity at all
    //    >0 - meaning is left to the implementors of the individual stages
    //
    // "Verbose" is intended to only add logging/tracing/output functionality; to add or enable
    // extra validation checks and such (code which is potentially side-effecting) you want to
    // use the "debug" option.
    bool isVerbose() const; // true iff verbosity>0 
    boost::uint8_t getVerboseLevel() const; 

    // For Name, Description, and DefaultOptions:
    //   each concrete class should provide a static function s_getX() which returns a static object
    //   each concrete class should provide a virtual getX() which returns s_getX()
    // This is automated via the GenerateStatics() macro below.

    // Use a dotted, XPath-style name for your 
    // stage.  For example, 'drivers.las.reader' or 'filters.crop'.  This 
    // XPath-style name will also correspond to an entry in the pdal::Options
    // tree for the given stage.

    virtual const Options& getDefaultOptions() const = 0; // { return s_getDefaultOptions(); }
    virtual const std::string& getName() const = 0; // { return s_getName(); }
    virtual const std::string& getDescription() const = 0; // { return s_getDescription(); }
    //static const Options& s_getDefaultOptions();
    //static const std::string& s_getName();
    //static const std::string& s_getDescription();
    
#define DECLARE_STATICS  \
    public: \
    static const Options& s_getDefaultOptions(); \
    virtual const Options& getDefaultOptions() const;  \
    static const std::string& s_getName();  \
    virtual const std::string& getName() const;  \
    static const std::string& s_getDescription();  \
    virtual const std::string& getDescription() const;  \
    private:

#define IMPLEMENT_STATICS(T, name, description)  \
    const Options& T::getDefaultOptions() const { return s_getDefaultOptions(); }  \
    const std::string& T::s_getName() { static std::string s(name); return s; }  \
    const std::string& T::getName() const { return s_getName(); }  \
    const std::string& T::s_getDescription() { static std::string s(description); return s; }  \
    const std::string& T::getDescription() const { return s_getDescription(); }

protected:
    Options& getOptions();

private:
    bool m_initialized;
    Options m_options;
    bool m_debug;
    boost::uint8_t m_verbose;

    StageBase& operator=(const StageBase&); // not implemented
    StageBase(const StageBase&); // not implemented
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const StageBase&);

} // namespace pdal

#endif
