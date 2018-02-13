/******************************************************************************
* Copyright (c) 2016, Mateusz Loskot (mateusz@loskot.net).
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/PluginManager.hpp>
#include <pdal/pdal_config.hpp>
#include <pdal/Filter.hpp>
#include <pdal/util/Algorithm.hpp>

#include "Support.hpp"

namespace pdal
{

struct DummyPlugin : Filter
{
    // Plugin management
    static int32_t dummyExitFunc()
    {
        return 0;
    }

    static PF_ExitFunc initPlugin() // PF_InitFunc
    {
        PluginInfo pi { "filters.dummytest", "A dummy plugin registered at run-time", "http://somewhere" };
        pdal::PluginManager<Stage>::registerPlugin<DummyPlugin>(pi);
    }

    // Stage
    std::string getName() const final { return "filters.dummytest"; }
    // Filter
    void filter(PointView& /*view*/) final {}
};

TEST(PluginManagerTest, NoPluginsNoNames)
{
    auto ns = PluginManager<Stage>::names();
    EXPECT_TRUE(ns.empty());
    ns = PluginManager<Kernel>::names();
    EXPECT_TRUE(ns.empty());
}

TEST(PluginManagerTest, InitPlugin)
{
    EXPECT_TRUE(PluginManager<Stage>::initializePlugin(DummyPlugin::initPlugin));
}

TEST(PluginManagerTest, MissingPlugin)
{
    std::unique_ptr<Stage> p(PluginManager<Stage>::createObject("filters.nonexistentplugin"));
    EXPECT_EQ(p.get(), nullptr);
}

TEST(PluginManagerTest, CreateObject)
{
    std::unique_ptr<Stage> p(PluginManager<Stage>::createObject("filters.dummytest"));
    EXPECT_NE(p.get(), nullptr);
}

TEST(PluginManagerTest, SearchPaths)
{
    std::string curPath;
    int set = Utils::getenv("PDAL_DRIVER_PATH", curPath);
    Utils::unsetenv("PDAL_DRIVER_PATH");

    StringList paths = PluginManager<Stage>::test_pluginSearchPaths();
    EXPECT_TRUE(Utils::contains(paths, "./lib"));
    EXPECT_TRUE(Utils::contains(paths, "../lib"));
    EXPECT_TRUE(Utils::contains(paths, "../bin"));
    EXPECT_TRUE(Utils::contains(paths, Config::pluginInstallPath()));

    Utils::setenv("PDAL_DRIVER_PATH", "/foo/bar://baz");
    paths = PluginManager<Stage>::test_pluginSearchPaths();
    EXPECT_EQ(paths.size(), 2U);
    EXPECT_TRUE(Utils::contains(paths, "/foo/bar"));
    EXPECT_TRUE(Utils::contains(paths, "//baz"));
    Utils::setenv("PDAL_DRIVER_PATH", "/this/is/a/path");
    paths = PluginManager<Stage>::test_pluginSearchPaths();
    EXPECT_EQ(paths.size(), 1U);
    EXPECT_TRUE(Utils::contains(paths, "/this/is/a/path"));

    if (set == 0)
        Utils::setenv("PDAL_DRIVER_PATH", curPath);
}

} // namespace pdal

