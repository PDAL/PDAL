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

#include <pdal/pdal_internal.hpp>

#include "ProgrammableFilter.hpp"
#include "../plang/Plang.hpp"
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.programmable",
    "Manipulate data using inline Python",
    "http://pdal.io/stages/filters.programmable.html" );

CREATE_SHARED_PLUGIN(1, 0, ProgrammableFilter, Filter, s_info)

std::string ProgrammableFilter::getName() const { return s_info.name; }

Options ProgrammableFilter::getDefaultOptions()
{
    Options options;
    options.add("script", "");
    options.add("module", "");
    options.add("function", "");
    return options;
}


void ProgrammableFilter::processOptions(const Options& options)
{
    m_source = options.getValueOrDefault<std::string>("source", "");
    if (m_source.empty())
        m_source = FileUtils::readFileIntoString(
            options.getValueOrThrow<std::string>("script"));
    m_module = options.getValueOrThrow<std::string>("module");
    m_function = options.getValueOrThrow<std::string>("function");

    auto addDims = options.getOptions("add_dimension");
    for (auto it = addDims.cbegin(); it != addDims.cend(); ++it)
    {
        m_addDimensions.push_back(it->getValue<std::string>());
    }
}


void ProgrammableFilter::addDimensions(PointLayoutPtr layout)
{
    for (auto it = m_addDimensions.cbegin(); it != m_addDimensions.cend(); ++it)
    {
        Dimension::Id::Enum id = layout->registerOrAssignDim(*it,
                                    pdal::Dimension::Type::Double);
    }
}


void ProgrammableFilter::ready(PointTableRef table)
{
    plang::Environment::get()->set_stdout(log()->getLogStream());
    m_script = new plang::Script(m_source, m_module, m_function);
    m_pythonMethod = new plang::BufferedInvocation(*m_script);
    m_pythonMethod->compile();
    m_totalMetadata = table.metadata();
}


void ProgrammableFilter::filter(PointView& view)
{
    log()->get(LogLevel::Debug5) << "Python script " << *m_script <<
        " processing " << view.size() << " points." << std::endl;
    m_pythonMethod->resetArguments();
    m_pythonMethod->begin(view, m_totalMetadata);
    m_pythonMethod->execute();
    m_pythonMethod->end(view, getMetadata());
}


void ProgrammableFilter::done(PointTableRef table)
{
    plang::Environment::get()->reset_stdout();
    delete m_pythonMethod;
    delete m_script;
}

} // namespace pdal
