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

#include "Support.hpp"

#include <pdal/FileUtils.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineWriter.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PipelineWriterTest)



BOOST_AUTO_TEST_CASE(PipelineWriterTest_test1)
{
    {
        PipelineManager manager;
        PipelineReader reader(manager);
        PipelineWriter writer(manager);

        reader.readPipeline(Support::datapath("pipeline/pipeline_write.xml"));

        writer.writePipeline(Support::temppath("test.xml"));
    }

    // we can't compare top the reference fiel directly, since the filename
    // paths are now absolute and not relative -- instead, we'll round trip once
    // more and compare that way
    {
        PipelineManager manager;
        PipelineReader reader(manager);
        PipelineWriter writer(manager);

        reader.readPipeline(Support::temppath("test.xml"));

        writer.writePipeline(Support::temppath("test2.xml"));
    }

    bool filesSame = Support::compare_text_files(Support::temppath("test.xml"), Support::temppath("test2.xml"));
    BOOST_CHECK(filesSame);
    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath("test.xml"));
        FileUtils::deleteFile(Support::temppath("test2.xml"));
    }

    FileUtils::deleteFile(Support::datapath("pipeline/out.las"));
}


BOOST_AUTO_TEST_CASE(PipelineWriterTest_attr_test)
{
    std::stringstream s;

    {
        boost::property_tree::ptree tree;

        boost::property_tree::ptree subtree;
        subtree.put("a", "aaa");
        subtree.put("b", "bbb");

        tree.put("child1", "one");
        tree.put_child("x.<xmlattr>", subtree);
        tree.put("child2", "two");

        boost::property_tree::write_xml(s, tree);
    }

    {
        boost::property_tree::ptree tree;
        boost::property_tree::read_xml(s, tree);
        BOOST_CHECK_EQUAL(tree.get<std::string>("child1"), "one");
        BOOST_CHECK_EQUAL(tree.get<std::string>("x.<xmlattr>.a"), "aaa");
        BOOST_CHECK_EQUAL(tree.get<std::string>("x.<xmlattr>.b"), "bbb");
        BOOST_CHECK_EQUAL(tree.get<std::string>("child2"), "two");
    }
}


BOOST_AUTO_TEST_CASE(PipelineWriterTest_multioptions)
{

    {
        PipelineManager manager;
        PipelineReader reader(manager);
        PipelineWriter writer(manager);

        reader.readPipeline(Support::datapath("pipeline/pipeline_multioptions.xml"));
        writer.writePipeline(Support::temppath("test-multi.xml"));
    }

    {
        PipelineManager manager;
        PipelineReader reader(manager);

        reader.readPipeline(Support::temppath("test-multi.xml"));
        manager.getStage()->prepare();

        Stage const& stage = manager.getStage()->getPrevStage();
        Options opt = stage.getOptions();

        Option fname = opt.getOption("filename");

        boost::optional<Options const&> more = fname.getOptions();

        Option meaning_of_life = more->getOption("somemore");

        BOOST_CHECK_EQUAL(meaning_of_life.getValue<int>(), 42);
    }

    FileUtils::deleteFile(Support::temppath("test-multi.xml"));
}

BOOST_AUTO_TEST_SUITE_END()
