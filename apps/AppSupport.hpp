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

#ifndef INCLUDED_APPSUPPORT_HPP
#define INCLUDED_APPSUPPORT_HPP

#include <string>

#include <pdal/Options.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Writer.hpp>

#include "Application.hpp"


// this is a static class with some helper functions the cmd line apps need
class AppSupport
{
public:
    // makes a reader/stage, from just the filename and some options
    static pdal::Stage* AppSupport::makeReader(const std::string& inputFile, const Application& app);

    // makes a writer, from just the filename and some options (and the input stage)
    static pdal::Writer* AppSupport::makeWriter(const std::string& outputFile, pdal::Stage& stage, const Application& app);

private:
    // infer the driver to use based on filename extension
    // returns "" if no driver found
    // 
    // this may also add on an option to pass to the driver, such as the filename
    static std::string inferReaderDriver(const std::string& filename, pdal::Options& options);

    // infer the driver to use based on filename extension
    // returns "" if no driver found
    // 
    // this may also add on an option to pass to the driver, such as the filename
    // (or something inferred from the extension, such as .laz means we need to use compress=true)
    static std::string inferWriterDriver(const std::string& filename, pdal::Options& options);

    AppSupport& operator=(const AppSupport&); // not implemented
    AppSupport(const AppSupport&); // not implemented
};

#endif
