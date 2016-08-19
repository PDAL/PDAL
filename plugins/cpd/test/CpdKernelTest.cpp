/******************************************************************************
* Copyright (c) 2015, Pete Gadomski <pete.gadomski@gmail.com>
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

#include <pdal/pdal_test_main.hpp>
#include "kernel/Cpd.hpp"

#include <pdal/Filter.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/Reader.hpp>
#include "LasReader.hpp"
#include "Support.hpp"

namespace pdal
{

namespace
{

class CpdKernelTest : public ::testing::Test
{
public:

    CpdKernelTest()
        : m_x(Support::datapath("las/simple.las"))
        , m_y(Support::datapath("las/simple_transformed.las"))
        , m_outfile(Support::datapath("las/simple_cpd.las"))
    {}

protected:

    virtual void SetUp()
    {
        PipelineManager mrManager;

        Options readerOptions;
        readerOptions.add("filename", m_x);
        Stage& reader = mrManager.addReader("readers.las");
        reader.setOptions(readerOptions);

        Options transformationOptions;
        transformationOptions.add("matrix",
            "1 0 0 1\n0 1 0 2\n0 0 1 3\n0 0 0 1");
        Stage& filter = mrManager.addFilter("filters.transformation");
        filter.setInput(reader);
        filter.setOptions(transformationOptions);

        Options writerOptions;
        writerOptions.add("filename", m_y);
        Stage& writer = mrManager.addWriter("writers.las");
        writer.setInput(filter);
        writer.setOptions(writerOptions);

        point_count_t np = mrManager.execute();
    }

    virtual void TearDown()
    {
        FileUtils::deleteFile(m_y);
        FileUtils::deleteFile(m_outfile);
    }

    std::string m_x;
    std::string m_y;
    std::string m_outfile;

};

} // namespace


TEST_F(CpdKernelTest, Execution)
{
    KernelFactory f;
    void *stage = PluginManager::createObject("kernels.cpd");
    std::unique_ptr<Kernel> cpdKernel(static_cast<Kernel*>(stage));

    int argc = 4;
    LogPtr log(new Log("pdal cpd", &std::clog));
    StringList argv { "rigid", m_x, m_y, m_outfile };
    int retval = cpdKernel->run(argv, log);
    EXPECT_EQ(0, retval);
}

} // namespace pdal
