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


std::string AppSupport::inferReaderDriver(const std::string& filename, pdal::Options& options)
{
    std::string ext = boost::filesystem::extension(filename);
    if (ext == "") return "";
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return "";

    boost::to_lower(ext);

    pdal::Option& fn = options.getOptionByRef("filename");
    fn.setValue<std::string>(filename);

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


std::string AppSupport::inferWriterDriver(const std::string& filename, pdal::Options& options)
{
    std::string ext = boost::filesystem::extension(filename);
    if (ext == "") return "";
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return "";

    boost::to_lower(ext);

    if (ext == "laz")
    {
        options.add("compression", true);
    }

    options.add<std::string>("filename", filename);

    // maybe this should live in StageFactory?
    std::map<std::string, std::string> drivers;
    drivers["las"] = "drivers.las.writer";
    drivers["laz"] = "drivers.las.writer";

    std::string driver = drivers[ext];
    return driver; // will be "" if not found
}


pdal::Stage* AppSupport::makeReader(pdal::Options& options)
{
    const std::string inputFile = options.getValueOrThrow<std::string>("filename");

    if (!pdal::FileUtils::fileExists(inputFile))
    {
        throw app_runtime_error("file not found: " + inputFile);
    }

    std::string driver = AppSupport::inferReaderDriver(inputFile, options);
    if (driver == "")
    {
        throw app_runtime_error("Cannot determine file type of " + inputFile);
    }

    if (options.getValueOrDefault<bool>("liblas", false) && driver == "drivers.las.reader")
    {
        driver = "drivers.liblas.reader";
    }

    pdal::StageFactory factory;
    pdal::Stage* stage = factory.createReader(driver, options);
    if (!stage)
    {
        throw app_runtime_error("reader creation failed");
    }

    return stage;
}


pdal::Writer* AppSupport::makeWriter(pdal::Options& options, pdal::Stage& stage)
{
    const std::string outputFile = options.getValueOrThrow<std::string>("filename");

    std::string driver = AppSupport::inferWriterDriver(outputFile, options);
    if (driver == "")
    {
        throw app_runtime_error("Cannot determine file type of " + outputFile);
    }

    if (options.getValueOrDefault<bool>("liblas", false) && driver == "drivers.las.writer")
    {
        driver = "drivers.liblas.writer";
    }
        
    pdal::StageFactory factory;
    pdal::Writer* writer = factory.createWriter(driver, stage, options);
    if (!writer)
    {
        throw app_runtime_error("writer creation failed");
    }

    return writer;
}


PercentageCallback::PercentageCallback()
    : m_lastMajorPerc(-10.0)
    , m_lastMinorPerc(-2.0)
    , m_done(false)
{
    return;
}


void PercentageCallback::callback()
{
    if (m_done) return;

    double currPerc = getPercentComplete();
    
    if (pdal::Utils::compare_distance<double>(currPerc, 100.0))
    {
        std::cout << "100\n";
        m_done = true;
    }
    else if (currPerc >= m_lastMajorPerc + 10.0)
    {
        std::cout << (int)currPerc;
        m_lastMajorPerc = currPerc;
        m_lastMinorPerc = currPerc;
    }
    else if (currPerc >= m_lastMinorPerc + 2.0)
    {
        std::cout << '.';
        m_lastMinorPerc = currPerc;
    }

    return;
}


HeartbeatCallback::HeartbeatCallback()
{
    return;
}


void HeartbeatCallback::callback()
{
    std::cout << '.';

    return;
}
