/******************************************************************************
* Copyright (c) 2026, Hobu, Inc. (howard@hobu.co)
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

#include <io/private/stac/Utils.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

namespace pdal
{

using namespace stac::StacUtils;

TEST(StacReaderTest, relative_path_resolution)
{
    const std::string vpc =
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/index.vpc";

    // A relative href on a remote source resolves against the source URL, not
    // against the current working directory.
    EXPECT_EQ(handleRelativePath(vpc, "./LHD_FXX_0890_6263.copc.laz"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/"
            "LHD_FXX_0890_6263.copc.laz");
    EXPECT_EQ(handleRelativePath(vpc, "./../laz/x.laz"),
        "https://data.geopf.fr/chunk/download/laz/x.laz");
    EXPECT_EQ(handleRelativePath(vpc, "sub/x.laz"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/sub/x.laz");
    EXPECT_EQ(handleRelativePath(vpc, "/other/x.laz"),
        "https://data.geopf.fr/other/x.laz");
    EXPECT_EQ(handleRelativePath(vpc, "sub/../x.laz"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/x.laz");

    // '..' can't walk above the authority.
    EXPECT_EQ(handleRelativePath(vpc, "../../../../../x.laz"),
        "https://data.geopf.fr/x.laz");

    // A base with no path of its own still resolves.
    EXPECT_EQ(handleRelativePath("https://data.geopf.fr", "./x.laz"),
        "https://data.geopf.fr/x.laz");

    // A relative reference doesn't inherit the base's query string.
    EXPECT_EQ(handleRelativePath(vpc + "?sig=abc", "./x.laz"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/x.laz");

    // A query carried by the reference itself survives, untouched by
    // dot-segment removal even when it contains slash-dot patterns.
    EXPECT_EQ(handleRelativePath(vpc, "./x.laz?sig=abc"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/x.laz?sig=abc");
    EXPECT_EQ(handleRelativePath(vpc, "x.laz?d=a/../b"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/x.laz?d=a/../b");
    EXPECT_EQ(handleRelativePath(vpc, "x.laz#frag"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/x.laz#frag");

    // "://" inside a reference's query is not a scheme -- the reference
    // is still relative.
    EXPECT_EQ(handleRelativePath(vpc, "x.laz?u=https://cdn/y"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/"
            "x.laz?u=https://cdn/y");

    // A base with a query but no path.
    EXPECT_EQ(handleRelativePath("https://data.geopf.fr?sig=1", "x.laz"),
        "https://data.geopf.fr/x.laz");

    // A query-only reference names the base resource itself, not its
    // directory; a fragment-only reference keeps the base's query too.
    EXPECT_EQ(handleRelativePath(vpc, "?page=2"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/index.vpc?page=2");
    EXPECT_EQ(handleRelativePath(vpc + "?sig=1", "#frag"),
        "https://data.geopf.fr/chunk/download/NUALHD_1-0/"
            "index.vpc?sig=1#frag");

    // Absolute hrefs pass through untouched.
    EXPECT_EQ(handleRelativePath(vpc, "s3://bucket/x.laz"),
        "s3://bucket/x.laz");
    EXPECT_EQ(handleRelativePath(vpc, "https://other/x.laz"),
        "https://other/x.laz");

    // A rooted reference replaces the base path entirely; a URL embedded
    // in its query -- a signed URL, say -- doesn't make it absolute.
    EXPECT_EQ(handleRelativePath(vpc, "/data/x.laz?u=https://other/y"),
        "https://data.geopf.fr/data/x.laz?u=https://other/y");
#ifdef _WIN32
    // A drive-letter path isn't a reference we can resolve against a URL.
    EXPECT_EQ(handleRelativePath(vpc, "C:/data/x.laz"), "C:/data/x.laz");
#endif

    // Local sources keep resolving on the filesystem.
    const std::string local = Support::datapath("stac/wrench.vpc");
    EXPECT_TRUE(FileUtils::fileExists(
        handleRelativePath(local, "./../laz/autzen_trim.laz")));
}

} // namespace pdal
