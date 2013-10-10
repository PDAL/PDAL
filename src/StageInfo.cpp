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

#include <pdal/StageInfo.hpp>


namespace pdal
{


std::ostream& operator<<(std::ostream& ostr, const StageInfo& info)
{
    // boost::property_tree::ptree tree = stage.toPTree();
    //
    // boost::property_tree::write_json(ostr, tree);

    return ostr;
}

StageInfo::StageInfo(std::string const& name, std::string const& description)
    : m_name(name), m_description(description) {}

/// copy constructor
StageInfo::StageInfo(StageInfo const& other)
    : m_name(other.m_name)
    , m_description(other.m_description)
    , m_dimensions(other.m_dimensions)
    , m_options(other.m_options)
    , m_link(other.m_link)
{
    return;
}

/// assignment operator
StageInfo& StageInfo::operator=(StageInfo const& rhs)
{
    if (&rhs != this)
    {
        m_name = rhs.m_name;
        m_description = rhs.m_description;
        m_dimensions = rhs.m_dimensions;
        m_options = rhs.m_options;
        m_link = rhs.m_link;
    }

    return *this;
}


} // namespace pdal
