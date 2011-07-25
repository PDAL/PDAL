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

    const Options& getOptions() const;

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
    const Options& T::s_getDefaultOptions() { return s_defaultOptions; } \
    const Options& T::getDefaultOptions() const { return s_getDefaultOptions(); }  \
    const std::string& T::s_getName() { static std::string s(name); return s; }  \
    const std::string& T::getName() const { return s_getName(); }  \
    const std::string& T::s_getDescription() { static std::string s(description); return s; }  \
    const std::string& T::getDescription() const { return s_getDescription(); }

protected:
    Options& getOptions();

private:
    Options m_options;

    StageBase& operator=(const StageBase&); // not implemented
    StageBase(const StageBase&); // not implemented
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const StageBase&);

} // namespace pdal

#endif
