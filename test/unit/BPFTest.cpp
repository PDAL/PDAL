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
#include <boost/test/unit_test.hpp>


#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/Utils.hpp>
#include <pdal/FileUtils.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(BPFReaderTest)

BOOST_AUTO_TEST_CASE(BPFTest_test)
{
    
    
    std::string p = Support::binpath("/../../bpf/test/pipeline_bpf.xml");
    
    // BPF driver is a (closed source) plugin. If the pipeline file for its 
    // example data isn't alongside the PDAL source tree, we skip the test.
    if (!pdal::FileUtils::fileExists(p))
        return;
    // BPF driver is a (closed source) plugin. If we don't have PDAL_DRIVER_PATH 
    // set, we aren't going to bother trying to run the test.
    std::string drivers;
    std::string driver_path("PDAL_DRIVER_PATH");
    drivers = pdal::Utils::getenv(driver_path);
    if (drivers.size() == 0)
        return;

    std::cout << "path: " << p << std::endl;
    
    pdal::Option option("filename", p);
    std::cout << option << std::endl;
    std::cout << option.getValue<std::string>() << std::endl;
    pdal::PipelineManager manager;
    
    
    
    pdal::PipelineReader reader(manager, false, 0);
    reader.readPipeline(option.getValue<std::string>());
    
    
    return;
}


BOOST_AUTO_TEST_SUITE_END()
