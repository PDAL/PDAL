/******************************************************************************
* Copyright (c) 2014, Hobu Inc., hobu.inc@gmail.com
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

#include <pdal/PipelineManager.hpp>

#include "Support.hpp"


TEST(MergeTest, test4)
{
    using namespace pdal;

    PipelineManager mgr;
    mgr.readPipeline(Support::configuredpath("filters/merge.json"));
    mgr.execute();

    PointViewSet viewSet = mgr.views();

    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(2130u, view->size());
}

TEST(MergeTest, test5)
{
    using namespace pdal;

    PipelineManager mgr;
    mgr.readPipeline(Support::configuredpath("filters/merge2.json"));
    mgr.execute();

    PointViewSet viewSet = mgr.views();

    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(2130u, view->size());
}

TEST(MergeTest, test6)
{
    using namespace pdal;

    LogPtr log(Log::makeLog("pdal merge", &std::clog));
    log->setLevel((LogLevel)5);
    PipelineManager mgr;
    mgr.setLog(log);
    mgr.readPipeline(Support::configuredpath("filters/merge3.json"));

    std::ostringstream oss;
    std::ostream& o = std::clog;
    auto ctx = Utils::redirect(o, oss);

    mgr.execute();
    std::string s = oss.str();
    EXPECT_TRUE(s.find("inconsistent spatial references") != s.npos);
    Utils::restore(o, ctx);

    PointViewSet viewSet = mgr.views();

    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(2130u, view->size());
}
