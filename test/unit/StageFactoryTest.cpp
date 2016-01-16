/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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
#include <pdal/StageFactory.hpp>
#include <pdal/util/Algorithm.hpp>

#include "Support.hpp"

namespace pdal
{

TEST(StageManagerTest, Load)
{
    StageFactory f(false);

    StringList ns = PluginManager::names(PF_PluginType_Filter |
        PF_PluginType_Reader | PF_PluginType_Writer);
    ASSERT_TRUE(Utils::contains(ns, "filters.crop"));
    ASSERT_TRUE(Utils::contains(ns, "readers.las"));
    ASSERT_TRUE(Utils::contains(ns, "writers.bpf"));
}

TEST(StageManagerTest, Load2)
{
    StageFactory f(false);

    StringList ns = PluginManager::names(PF_PluginType_Filter);
    ASSERT_TRUE(Utils::contains(ns, "filters.crop"));
    ASSERT_FALSE(Utils::contains(ns, "readers.las"));
    ASSERT_FALSE(Utils::contains(ns, "writers.bpf"));
}

TEST(StageManagerTest, Load3)
{
    StageFactory f(false);

    StringList ns = PluginManager::names(PF_PluginType_Reader);
    ASSERT_FALSE(Utils::contains(ns, "filters.crop"));
    ASSERT_TRUE(Utils::contains(ns, "readers.las"));
    ASSERT_FALSE(Utils::contains(ns, "writers.bpf"));
}

TEST(StageManagerTest, Load4)
{
    StageFactory f(false);

    StringList ns = PluginManager::names(PF_PluginType_Writer);
    ASSERT_FALSE(Utils::contains(ns, "filters.crop"));
    ASSERT_FALSE(Utils::contains(ns, "readers.las"));
    ASSERT_TRUE(Utils::contains(ns, "writers.bpf"));
}
    
} // namespace pdal

