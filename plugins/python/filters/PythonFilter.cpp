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

#include "PythonFilter.hpp"

#include <nlohmann/json.hpp>

#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "filters.python",
    "Manipulate data using inline Python",
    "http://pdal.io/stages/filters.python.html"
};

CREATE_SHARED_STAGE(PythonFilter, s_info)

struct PythonFilter::Args
{
    std::string m_module;
    std::string m_function;
    std::string m_source;
    std::string m_scriptFile;
    StringList m_addDimensions;
    NL::json m_pdalargs;
};

PythonFilter::PythonFilter() :
    m_script(nullptr), m_pythonMethod(nullptr), m_args(new Args)
{}


PythonFilter::~PythonFilter()
{}


std::string PythonFilter::getName() const
{
    return s_info.name;
}


void PythonFilter::addArgs(ProgramArgs& args)
{
    args.add("module", "Python module containing the function to run",
        m_args->m_module).setPositional();
    args.add("function", "Function to call",
        m_args->m_function).setPositional();
    args.add("source", "Python script to run", m_args->m_source);
    args.add("script", "File containing script to run", m_args->m_scriptFile);
    args.add("add_dimension", "Dimensions to add", m_args->m_addDimensions);
    args.add("pdalargs", "Dictionary to add to module globals when "
        "calling function", m_args->m_pdalargs);
}


void PythonFilter::addDimensions(PointLayoutPtr layout)
{
    for (const std::string& s : m_args->m_addDimensions)
        layout->registerOrAssignDim(s, pdal::Dimension::Type::Double);
}


void PythonFilter::ready(PointTableRef table)
{
    if (m_args->m_source.empty())
        m_args->m_source = FileUtils::readFileIntoString(m_args->m_scriptFile);
    std::ostream *out = log()->getLogStream();
    plang::EnvironmentPtr env = plang::Environment::get();
    env->set_stdout(out);
    m_script = new plang::Script(m_args->m_source, m_args->m_module,
        m_args->m_function);

	m_pythonMethod = new plang::Invocation(*m_script);
    m_pythonMethod->compile();
    m_totalMetadata = table.metadata();
}


PointViewSet PythonFilter::run(PointViewPtr view)
{
    log()->get(LogLevel::Debug5) << "filters.python " << *m_script <<
        " processing " << view->size() << " points." << std::endl;
    m_pythonMethod->resetArguments();
    m_pythonMethod->begin(*view, m_totalMetadata);

    if (!m_args->m_pdalargs.empty())
    {
        std::ostringstream args;
        args << m_args->m_pdalargs;
        m_pythonMethod->setKWargs(args.str());
    }
    m_pythonMethod->execute();

    PointViewSet viewSet;

    if (m_pythonMethod->hasOutputVariable("Mask"))
    {
        PointViewPtr outview = view->makeNew();

        size_t arrSize(0);
        void *pydata = m_pythonMethod->extractResult("Mask",
            Dimension::Type::Unsigned8, arrSize);
        char *ok = (char *)pydata;
        for (PointId idx = 0; idx < arrSize; ++idx)
            if (*ok++)
                outview->appendPoint(*view, idx);

        viewSet.insert(outview);
    }
    else
    {
        m_pythonMethod->end(*view, getMetadata());
        viewSet.insert(view);
    }
    return viewSet;
}


void PythonFilter::done(PointTableRef table)
{
    static_cast<plang::Environment*>(plang::Environment::get())->reset_stdout();
    delete m_pythonMethod;
    delete m_script;
}

} // namespace pdal
