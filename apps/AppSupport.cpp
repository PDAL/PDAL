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
#include <pdal/Utils.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/StageFactory.hpp>

std::string AppSupport::inferReaderDriver(const std::string& filename)
{
    const std::string ext = boost::filesystem::extension(filename);

    // maybe this should live in StageFactory?
    std::map<std::string, std::string> drivers;
    drivers[".las"] = "drivers.las.reader";
    drivers[".laz"] = "drivers.las.reader";
    drivers[".bin"] = "drivers.terrasolid.reader";
    drivers[".qi"] = "drivers.qfit.reader";
    drivers[".xml"] = "drivers.pipeline.reader";

    std::string driver = drivers[ext];
    if (driver == "")
    {
        return "";
    }

    return driver;
}


pdal::Stage* AppSupport::createReader(const std::string& driver, const std::string& filename, const pdal::Options& extraOptions)
{
    pdal::Stage* reader = NULL;

    pdal::Options opts(extraOptions);
    opts.add<std::string>("filename", filename);

    if (driver == "drivers.pipeline.reader")
    {
        pdal::PipelineManager* pipeManager(new pdal::PipelineManager); // BUG: memleak
        pdal::PipelineReader pipeReader(*pipeManager);
        pipeReader.readReaderPipeline(filename);
        reader = pipeManager->getStage();
    }
    else
    {
        pdal::StageFactory factory;
        reader = factory.createReader(driver, opts);
    }

    return reader;
}
