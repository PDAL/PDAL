/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#ifndef INCLUDED_STAGEINFO_HPP
#define INCLUDED_STAGEINFO_HPP

#include <pdal/pdal_internal.hpp>
#include <pdal/Options.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/Dimension.hpp>

#include <string>
#include <vector>
#include <iosfwd>

namespace pdal
{

class StageBase;
class Stage;
class Dimension;

class PDAL_DLL StageInfo
{
    friend class StageBase;
public:

    /// Constructor.
    ///
    StageInfo(std::string const& stageName, std::string const& stageDescription="");

    StageInfo& operator=(const StageInfo& rhs); 
    StageInfo(const StageInfo&); 

    /// Destructor.
    virtual ~StageInfo() {};

    /// Gets the available Options for the Stage.
    ///
    /// @return The options.


    /// Gets the dimensions that a give stage produces
    inline std::vector<Dimension> const& getProvidedDimensions() const
    {
        return m_dimensions;
    }
    
    inline void addProvidedDimension(Dimension const& dim)
    {
        m_dimensions.push_back(dim);
    }
    
    inline std::vector<Option> const& getProvidedOptions() const
    {
        return m_options;
    }
    
    inline void addProvidedOption(Option const& opt)
    {
        m_options.push_back(opt);
    }
    
    inline std::string const& getName() const
    {
        return m_name;
    }
    
    inline std::string const& getDescription() const
    {
        return m_description;
    }

private:
    std::string m_name;
    std::string m_description;
    std::vector<Dimension> m_dimensions;
    std::vector<Option> m_options;


};

/// Output operator for serialization
///
/// @param ostr    The output stream to write to
/// @param src     The StageInfo to be serialized out
///
/// @return The output stream

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const StageInfo& src);

} // namespace pdal

#endif
