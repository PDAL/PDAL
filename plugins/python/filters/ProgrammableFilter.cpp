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
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.programmable",
    "Manipulate data using inline Python",
    "http://pdal.io/stages/filters.programmable.html" );

CREATE_SHARED_PLUGIN(1, 0, ProgrammableFilter, Filter, s_info)

std::string ProgrammableFilter::getName() const { return s_info.name; }

void ProgrammableFilter::addArgs(ProgramArgs& args)
{
    args.add("source", "Python script to run", m_source);
    args.add("script", "File containing script to run", m_scriptFile);
    args.add("module", "Python module containing the function to run",
        m_module);
    args.add("function", "Function to call", m_function);
    args.add("add_dimension", "Dimensions to add", m_addDimensions);
}


void ProgrammableFilter::addDimensions(PointLayoutPtr layout)
{
    for (const std::string& s : m_addDimensions)
        layout->registerOrAssignDim(s, pdal::Dimension::Type::Double);
}


void ProgrammableFilter::ready(PointTableRef table)
{
    if (m_source.empty())
        m_source = FileUtils::readFileIntoString(m_scriptFile);
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
