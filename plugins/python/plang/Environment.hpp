/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#pragma once

#include <Python.h>
#undef toupper
#undef tolower
#undef isspace

#include <pdal/pdal_internal.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/Dimension.hpp>

#include "Redirector.hpp"
#include "Script.hpp"

namespace pdal
{
namespace plang
{

PDAL_DLL PyObject *fromMetadata(MetadataNode m);
PDAL_DLL void addMetadata(PyObject *list, MetadataNode m);

PDAL_DLL std::string getTraceback();

class Environment;
typedef Environment *EnvironmentPtr;

class PDAL_DLL Environment
{
public:
    Environment();
    ~Environment();

    // these just forward into the Redirector class
    void set_stdout(std::ostream* ostr);
    void reset_stdout();

    void execute(Script& script) {};

    static EnvironmentPtr get();

    static int getPythonDataType(Dimension::Type t);
    static pdal::Dimension::Type getPDALDataType(int t);

private:
    Redirector m_redirector;
};

} // namespace plang
} // namespace pdal

