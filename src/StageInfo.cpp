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

#include <iomanip>

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
    : m_name(name), m_description(description), m_isEnabled(false) {}

/// copy constructor
StageInfo::StageInfo(StageInfo const& other)
    : m_name(other.m_name)
    , m_description(other.m_description)
    , m_dimensions(other.m_dimensions)
    , m_options(other.m_options)
    , m_link(other.m_link)
    , m_isEnabled(other.m_isEnabled)
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

std::string StageInfo::optionsToRST() const
{
    std::ostringstream strm;

    std::vector<Option> options = getProvidedOptions();

    strm << toRST() << std::endl;

    if (!options.size())
    {
        strm << "No options documented" << std::endl << std::endl;
        return strm.str();
    } 
 
    std::string tablehead("================================ =============== =========================================");
    std::string headings ("Name                              Default          Description");
    
    strm << std::endl;
    strm << tablehead << std::endl;
    strm << headings << std::endl;
    strm << tablehead << std::endl;
    
    boost::uint32_t default_column(15);
    boost::uint32_t name_column(32);
    boost::uint32_t description_column(40);
    for (std::vector<Option>::const_iterator it = options.begin();
        it != options.end();
        ++it)
    {
        pdal::Option const& opt = *it;
        std::string default_value(opt.getValue<std::string>() );
        default_value = boost::algorithm::erase_all_copy(default_value, "\n");
        if (default_value.size() > default_column -1 )
        {
            default_value = default_value.substr(0, default_column-3);
            default_value = default_value + "...";
        }
        
        std::vector<std::string> lines;
        std::string description(opt.getDescription());
        description = boost::algorithm::erase_all_copy(description, "\n");
        
        Utils::wordWrap(description, lines, description_column-1);
        if (lines.size() == 1)
        {
            
            strm   << std::setw(name_column) << opt.getName() << " " 
                   << std::setw(default_column) << default_value << " " 
                   << std::left << std::setw(description_column) << description << std::endl;
        } else
            strm   << std::setw(name_column) << opt.getName() << " " 
                   << std::setw(default_column) << default_value << " " 
                   << lines[0] << std::endl;
        
        std::stringstream blank;
        size_t blanks(49);
        for (size_t i = 0; i < blanks; ++i)
            blank << " ";
        for (size_t i = 1; i < lines.size(); ++i)
        {
            strm << blank.str() <<lines[i] << std::endl;
        }

    }

    strm << tablehead << std::endl;
    strm << std::endl;
    return strm.str();    
}


} // namespace pdal
