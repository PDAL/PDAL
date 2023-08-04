/******************************************************************************
 * Copyright (c) 2023 TileDB, Inc.
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

#include "../io/TileDBUtils.hpp"
#include <pdal/pdal_test_main.hpp>
#include <tiledb/tiledb>

namespace pdal
{

TEST(DomainBounds, parse_empty)
{
    std::stringstream ss("()");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_TRUE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 0);
}

TEST(DomainBounds, parse_bbox1d)
{
    std::stringstream ss("([-1.0, 1.0])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 1);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
}

TEST(DomainBounds, parse_bbox2d)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 2);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
    EXPECT_FLOAT_EQ(bounds.minY(), -2.0);
    EXPECT_FLOAT_EQ(bounds.maxY(), 2.0);
}

TEST(DomainBounds, parse_bbox3d)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 3);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
    EXPECT_FLOAT_EQ(bounds.minY(), -2.0);
    EXPECT_FLOAT_EQ(bounds.maxY(), 2.0);
    EXPECT_FLOAT_EQ(bounds.minZ(), -3.0);
    EXPECT_FLOAT_EQ(bounds.maxZ(), 3.0);
}

TEST(DomainBounds, parse_bbox4d)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0], [12, 16])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 4);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
    EXPECT_FLOAT_EQ(bounds.minY(), -2.0);
    EXPECT_FLOAT_EQ(bounds.maxY(), 2.0);
    EXPECT_FLOAT_EQ(bounds.minZ(), -3.0);
    EXPECT_FLOAT_EQ(bounds.maxZ(), 3.0);
    EXPECT_FLOAT_EQ(bounds.minGpsTime(), 12.0);
    EXPECT_FLOAT_EQ(bounds.maxGpsTime(), 16.0);
}

TEST(DomainBounds, parse_bbox5d_with_error)
{
    std::stringstream ss(
        "([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0], [12, 16], [1.0, 2.0])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_missing_comma_xy_with_error)
{
    std::stringstream ss("([-1.0, 1.0] [-2.0, 2.0], [-3.0, 3.0], [12, 16])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_missing_comma_yz_with_error)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0] [-3.0, 3.0], [12, 16])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_missing_comma_zt_with_error)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0] [12, 16])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_pair_no_comma_with_error)
{
    std::istringstream ss("[-1.0 1.0]");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

TEST(DomainBounds, parse_pair_one_value_with_error)
{
    std::istringstream ss("[-1.0]");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

TEST(DomainBounds, parse_pair_missing_front_bracket_with_error)
{
    std::istringstream ss("-1.0, 1.0]");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

TEST(DomainBounds, parse_pair_missing_back_bracket_with_error)
{
    std::istringstream ss("[-1.0, 1.0)");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

TEST(FilterFactory, filter_type_from_string)
{
    std::vector<std::pair<std::string, tiledb_filter_type_t>> test_pairs{
        {"", TILEDB_FILTER_NONE},
        {"NONE", TILEDB_FILTER_NONE},
        {"gzip", TILEDB_FILTER_GZIP},
        {"GZIP", TILEDB_FILTER_GZIP},
        {"zstd", TILEDB_FILTER_ZSTD},
        {"ZSTD", TILEDB_FILTER_ZSTD},
        {"lz4", TILEDB_FILTER_LZ4},
        {"LZ4", TILEDB_FILTER_LZ4},
        {"rle", TILEDB_FILTER_RLE},
        {"RLE", TILEDB_FILTER_RLE},
        {"bzip2", TILEDB_FILTER_BZIP2},
        {"BZIP2", TILEDB_FILTER_BZIP2},
        {"double-delta", TILEDB_FILTER_DOUBLE_DELTA},
        {"DOUBLE_DELTA", TILEDB_FILTER_DOUBLE_DELTA},
        {"bit-width-reduction", TILEDB_FILTER_BIT_WIDTH_REDUCTION},
        {"BIT_WIDTH_REDUCTION", TILEDB_FILTER_BIT_WIDTH_REDUCTION},
        {"bit-shuffle", TILEDB_FILTER_BITSHUFFLE},
        {"BITSHUFFLE", TILEDB_FILTER_BITSHUFFLE},
        {"byte-shuffle", TILEDB_FILTER_BYTESHUFFLE},
        {"BYTESHUFFLE", TILEDB_FILTER_BYTESHUFFLE},
        {"positive-delta", TILEDB_FILTER_POSITIVE_DELTA},
        {"POSITIVE_DELTA", TILEDB_FILTER_POSITIVE_DELTA},
        {"DICTIONARY_ENCODING", TILEDB_FILTER_DICTIONARY},
        {"float-scale", TILEDB_FILTER_SCALE_FLOAT},
        {"scale-float", TILEDB_FILTER_SCALE_FLOAT},
        {"SCALE_FLOAT", TILEDB_FILTER_SCALE_FLOAT},
        {"XOR", TILEDB_FILTER_XOR},
        {"WEBP", TILEDB_FILTER_WEBP},
    };
    for (const auto& params : test_pairs)
    {
        auto result = FilterFactory::filterTypeFromString(params.first);
        EXPECT_EQ(result, params.second);
    }
}

TEST(FilterFactory, dim_with_unset_default_filter)
{
    NL::json jsonOptions({});
    tiledb::Context ctx{};
    FilterFactory factory{jsonOptions,        "none", {{0.01, 0.01, 0.01}},
                          {{1.0f, 1.0, 1.0}}, "",     0};
    auto filterList = factory.filterList(ctx, "Curvature");
    auto nfilters = filterList.nfilters();
    EXPECT_EQ(nfilters, 0);
}

TEST(FilterFactory, dim_with_set_default_filter)
{
    NL::json jsonOptions({});
    tiledb::Context ctx{};
    FilterFactory factory{jsonOptions,       "none", {{0.01, 0.01, 0.01}},
                          {{1.0, 1.0, 1.0}}, "zstd", 9};
    auto filterList = factory.filterList(ctx, "Curvature");
    auto nfilters = filterList.nfilters();
    EXPECT_EQ(nfilters, 1);
    if (nfilters >= 1)
    {
        auto filter = filterList.filter(0);
        EXPECT_EQ(filter.filter_type(), TILEDB_FILTER_ZSTD);
        int32_t compressionLevel;
        filter.get_option<int32_t>(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 9);
    }
}

TEST(FilterFactory, user_set_scale_float)
{
    NL::json jsonOptions({});
    jsonOptions["Z"] = {{{"compression", "scale-float"},
                         {"scale_float_bytewidth", 2},
                         {"scale_float_factor", 2.0},
                         {"scale_float_offset", 100.0}}};
    FilterFactory factory{jsonOptions,       "balanced", {{0.01, 0.01, 0.01}},
                          {{1.0, 1.0, 1.0}}, "zstd",     7};

    tiledb::Context ctx{};
    auto filterList = factory.filterList(ctx, "Z");
    auto nfilters = filterList.nfilters();
    EXPECT_EQ(nfilters, 1);

    if (nfilters >= 1)
    {
        auto filter = filterList.filter(0);
        EXPECT_EQ(filter.filter_type(), TILEDB_FILTER_SCALE_FLOAT);
        uint64_t bytewidth{0};
        double factor{1.0};
        double offset{0.0};
        filter.get_option<uint64_t>(TILEDB_SCALE_FLOAT_BYTEWIDTH, &bytewidth);
        filter.get_option<double>(TILEDB_SCALE_FLOAT_FACTOR, &factor);
        filter.get_option<double>(TILEDB_SCALE_FLOAT_OFFSET, &offset);
        EXPECT_EQ(bytewidth, 2);
        EXPECT_FLOAT_EQ(factor, 2.0);
        EXPECT_FLOAT_EQ(offset, 100.0);
    }
}

TEST(FilterFactory, user_set_delta)
{
    NL::json jsonOptions({});
    jsonOptions["GpsTime"] = {
        {{"compression", "delta"}, {"reinterpret_datatype", "UINT64"}}};
    FilterFactory factory{jsonOptions,       "balanced", {{0.01, 0.01, 0.01}},
                          {{1.0, 1.0, 1.0}}, "zstd",     7};

    tiledb::Context ctx{};
#if TILEDB_VERSION_MAJOR == 2 && TILEDB_VERSION_MINOR < 16
    EXPECT_THROW(factory.filterList(ctx, "GpsTime"), tiledb::TileDBError);
#else
    auto filterList = factory.filterList(ctx, "GpsTime");
    auto nfilters = filterList.nfilters();
    EXPECT_EQ(nfilters, 1);

    if (nfilters >= 1)
    {
        auto filter = filterList.filter(0);
        EXPECT_EQ(filter.filter_type(), TILEDB_FILTER_DELTA);
        tiledb_datatype_t output_type{};
        filter.get_option(TILEDB_COMPRESSION_REINTERPRET_DATATYPE,
                          &output_type);
        EXPECT_EQ(output_type, TILEDB_UINT64);
    }
#endif
}

TEST(FilterFactory, user_set_two_filter_list)
{
    NL::json jsonOptions({});
    jsonOptions["X"] = {{{"compression", "bit-shuffle"}},
                        {{"compression", "gzip"}, {"compression_level", 9}}};
    FilterFactory factory{jsonOptions,       "balanced", {{0.01, 0.01, 0.01}},
                          {{1.0, 1.0, 1.0}}, "zstd",     7};

    tiledb::Context ctx{};
    auto filterList = factory.filterList(ctx, "X");
    auto nfilters = filterList.nfilters();
    EXPECT_EQ(nfilters, 2);
    if (nfilters >= 1)
    {
        auto filter0 = filterList.filter(0);
        EXPECT_EQ(filter0.filter_type(), TILEDB_FILTER_BITSHUFFLE);
    }
    if (nfilters >= 2)
    {
        auto filter1 = filterList.filter(1);
        EXPECT_EQ(filter1.filter_type(), TILEDB_FILTER_GZIP);
        int32_t compressionLevel;
        filter1.get_option<int32_t>(TILEDB_COMPRESSION_LEVEL,
                                    &compressionLevel);
        EXPECT_EQ(compressionLevel, 9);
    }
}

} // namespace pdal
