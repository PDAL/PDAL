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

#include <pdal/PluginDirectory.hpp>
#include <pdal/PluginManager.hpp>
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
        PluginInfo pi {
            "filters.dummytest",
            "A dummy plugin registered at run-time",
            "http://somewhere"
        };
        pdal::PluginManager<Stage>::registerPlugin<DummyPlugin>(pi);
    }

    // Stage
    std::string getName() const final { return "filters.dummytest"; }
    // Filter
    void filter(PointView& /*view*/) final {}
};

TEST(PluginManagerTest, MissingPlugin)
{
    std::unique_ptr<Stage> p(
        PluginManager<Stage>::createObject("filters.nonexistentplugin"));
    EXPECT_EQ(p.get(), nullptr);
}

TEST(PluginManagerTest, CreateObject)
{
    DummyPlugin::initPlugin();
    std::unique_ptr<Stage> p(
        PluginManager<Stage>::createObject("filters.dummytest"));
    EXPECT_NE(p.get(), nullptr);
}


TEST(PluginManagerTest, validnames)
{
#if defined(__APPLE__) && defined(__MACH__)
    static const std::string dlext(".dylib");
#elif defined _WIN32
    static const std::string dlext(".dll");
#else
    static const std::string dlext(".so");
#endif

    StringList type1 { "reader", "writer" };
    StringList type2 { "foo" };

    // Invalid names
    EXPECT_EQ(PluginDirectory::test_validPlugin("I'm a plugin", { "foo" }), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_ac.dylib" + dlext, type1), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_a_b_c.dylib" + dlext, type1), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "reader_a" + dlext, type1), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_rea_a" + dlext, type1), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_a" + dlext, type2), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_" , type1), "");

    // Almost valid names
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_writer" + dlext, type1), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_writer_" + dlext, type1), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_1a_b" + dlext, type1), "");
    EXPECT_EQ(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_a+_b" + dlext, type1), "");

    // Valid names.
    EXPECT_NE(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_aP" + dlext, type1), "");
    EXPECT_NE(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_Pa" + dlext, type1), "");
    EXPECT_NE(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_a" + dlext, type1), "");
    EXPECT_NE(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_projecta_vendor1_IntensitySeparationWithTreeWeights" + dlext, type1), "");
    EXPECT_NE(PluginDirectory::test_validPlugin(
        "libpdal_plugin_foo_foo" + dlext, type2), "");
    EXPECT_NE(PluginDirectory::test_validPlugin(
        "libpdal_plugin_reader_a_b" + dlext, type1), "");

}

} // namespace pdal

