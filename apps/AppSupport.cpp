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

#include "AppSupport.hpp"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/StageFactory.hpp>


std::string AppSupport::inferReaderDriver(const std::string& filename)
{
    std::string ext = boost::filesystem::extension(filename);
    if (ext == "") return "";
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return "";

    boost::to_lower(ext);

    // maybe this should live in StageFactory?
    std::map<std::string, std::string> drivers;
    drivers["las"] = "drivers.las.reader";
    drivers["laz"] = "drivers.las.reader";
    drivers["bin"] = "drivers.terrasolid.reader";
    drivers["qi"] = "drivers.qfit.reader";
    drivers["xml"] = "drivers.pipeline.reader";

    std::string driver = drivers[ext];
    return driver; // will be "" if not found
}


std::string AppSupport::inferWriterDriver(const std::string& filename)
{
    std::string ext = boost::filesystem::extension(filename);
    if (ext == "") return "";
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return "";

    boost::to_lower(ext);

    // maybe this should live in StageFactory?
    std::map<std::string, std::string> drivers;
    drivers["las"] = "drivers.las.writer";
    drivers["laz"] = "drivers.las.writer";

    std::string driver = drivers[ext];
    return driver; // will be "" if not found
}


pdal::Stage* AppSupport::createReader(const std::string& driver, const std::string& filename, const pdal::Options& extraOptions)
{
    pdal::Stage* reader = NULL;

    pdal::Options opts(extraOptions);
    opts.add<std::string>("filename", filename);

    pdal::StageFactory factory;
    reader = factory.createReader(driver, opts);

    return reader;
}


pdal::Writer* AppSupport::createWriter(const std::string& driver, const std::string& filename, pdal::Stage& stage, const pdal::Options& extraOptions)
{
    pdal::Writer* writer = NULL;

    pdal::Options opts(extraOptions);
    opts.add<std::string>("filename", filename);

    pdal::StageFactory factory;
    writer = factory.createWriter(driver, stage, opts);

    return writer;
}


pdal::Stage* AppSupport::makeReader(const std::string& inputFile, const Application& app)
{
    if (!pdal::FileUtils::fileExists(inputFile))
    {
        throw app_runtime_error("file not found: " + inputFile);
    }

    std::string driver = AppSupport::inferReaderDriver(inputFile);
    if (driver == "")
    {
        throw app_runtime_error("Cannot determine file type of " + inputFile);
    }

    if (app.hasOption("liblas") && driver == "drivers.las.reader")
    {
        driver = "drivers.liblas.reader";
    }

    pdal::Options options;
    options.add<bool>("debug", app.isDebug());
    options.add<boost::uint8_t>("verbose", app.getVerboseLevel());

    pdal::Stage* stage = AppSupport::createReader(driver, inputFile, options);

    return stage;
}


pdal::Writer* AppSupport::makeWriter(const std::string& outputFile, pdal::Stage& stage, const Application& app)
{
    std::string driver = AppSupport::inferWriterDriver(outputFile);
    if (driver == "")
    {
        throw app_runtime_error("Cannot determine file type of " + outputFile);
    }

    if (app.hasOption("liblas") && driver == "drivers.las.writer")
    {
        driver = "drivers.liblas.writer";
    }

    pdal::Options options;
    options.add<bool>("debug", app.isDebug());
    options.add<boost::uint8_t>("verbose", app.getVerboseLevel());

    pdal::Writer* writer = AppSupport::createWriter(driver, outputFile, stage, options);

    return writer;
}
