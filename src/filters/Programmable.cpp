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
#ifdef PDAL_HAVE_PYTHON

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/filters/Programmable.hpp>

#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace filters
{

Options Programmable::getDefaultOptions()
{
    Options options;
    options.add("script", "");
    options.add("module", "");
    options.add("function", "");
    return options;
}


void Programmable::processOptions(const Options& options)
{
    m_source = options.getValueOrDefault<std::string>("source", "");
    if (m_source.empty())
        m_source = FileUtils::readFileIntoString(
            options.getValueOrThrow<std::string>("filename"));
    m_module = options.getValueOrThrow<std::string>("module");
    m_function = options.getValueOrThrow<std::string>("function");
}


void Programmable::ready(PointContext ctx)
{
    m_script = new plang::Script(m_source, m_module, m_function);
    m_pythonMethod = new plang::BufferedInvocation(*m_script);
    m_pythonMethod->compile();
    GlobalEnvironment::get().getPythonEnvironment().set_stdout(
        log()->getLogStream());
}


void Programmable::filter(PointBuffer& buf)
{
    log()->get(LogLevel::Debug5) << "Python script " << *m_script <<
        " processing " << buf.size() << " points." << std::endl;
    m_pythonMethod->resetArguments();
    m_pythonMethod->begin(buf);
    m_pythonMethod->execute();
    m_pythonMethod->end(buf);
}


void Programmable::done(PointContext ctx)
{
    GlobalEnvironment::get().getPythonEnvironment().reset_stdout();
    delete m_pythonMethod;
    delete m_script;
}

} // namespace filters
} // namespace pdal

#endif
