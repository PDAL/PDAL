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

#include "TileDBUtils.hpp"

namespace pdal
{

void DomainBounds::parsePair(std::istringstream& ss, const std::string& dimName)
{
    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, '['))
        throw error("No opening '[' for '" + dimName + "' range.");

    double low{0.0};
    ss >> low;
    if (!ss.good())
        throw error("No valid minimum value for '" + dimName + "' range.");

    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, ','))
        throw error("No ',' separating minimum/maximum values on '" + dimName +
                    "'.");

    double high{0.0};
    ss >> high;
    if (!ss.good())
        throw error("No valid maximum value for '" + dimName + "' range.");

    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, ']'))
        throw error("No closing ']' for '" + dimName + "' range.");

    m_data.push_back({low, high});
}

void DomainBounds::parse(std::string input)
{
    m_data.clear();

    static thread_local Utils::IStringStreamClassicLocale ss;
    ss.str(input);

    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, '('))
        throw error("No opening '('.");
    Utils::eatwhitespace(ss);
    if (ss.peek() == ')')
        return;
    std::vector<std::string> names{"X", "Y", "Z", "GpsTime"};
    for (uint32_t index{0}; index < 4; ++index)
    {
        parsePair(ss, names[0]);
        Utils::eatwhitespace(ss);
        auto nextChar = ss.get();
        if (nextChar == ')')
            return;
        if (index == 3)
            throw error("Can only add 4 ranges to the bounding box.");
        if (nextChar != ',')
            throw error("No comma separating '" + names[index] + "' and '" +
                        names[index + 1] + "' dimensions.");
    }
    throw error("No closing ')' for the bounding box.");
}

std::istream& operator>>(std::istream& in, DomainBounds& bounds)
{
    std::string boundsLine;
    std::getline(in, boundsLine);
    bounds.parse(boundsLine);
    return in;
}

std::ostream& operator<<(std::ostream& out, const DomainBounds& bounds)
{
    if (bounds.empty())
    {
        out << "()";
        return out;
    }

    Utils::StringStreamClassicLocale ss;
    ss.precision(16);
    ss << "(";
    auto ndim = bounds.ndim();
    if (ndim >= 1)
        ss << "[" << bounds.minX() << ", " << bounds.maxX() << "]";
    if (ndim >= 2)
        ss << ", [" << bounds.minY() << ", " << bounds.maxY() << "]";
    if (ndim >= 3)
        ss << ", [" << bounds.minZ() << ", " << bounds.maxZ() << "]";
    if (ndim >= 4)
        ss << ", [" << bounds.minGpsTime() << ", " << bounds.maxGpsTime()
           << "]";
    ss << ")";
    out << ss.str();
    return out;
}

FilterFactory::FilterFactory(const NL::json& userProvidedFilters,
                             const std::string& defaultCompressor,
                             int32_t defaultCompressionLevel)
{
    m_user_filters.update(userProvidedFilters);
    if (!defaultCompressor.empty())
    {
        m_default_filter_type =
            FilterFactory::filterTypeFromString(defaultCompressor);
        m_default_compression_level = defaultCompressionLevel;
    }
}

tiledb::Filter FilterFactory::filter(const tiledb::Context& ctx,
                                     const NL::json& options)
{
    if (options.empty())
        return tiledb::Filter(ctx, TILEDB_FILTER_NONE);
    std::string filter_type_str = options["compression"];
    auto filter_type = FilterFactory::filterTypeFromString(filter_type_str);

    tiledb::Filter filter{ctx, filter_type};

    for (auto& opt : options.items())
    {
        const auto& key = opt.key();
        if (key == "compression")
            continue;
        else if (key == "compression_level")
            filter.set_option(TILEDB_COMPRESSION_LEVEL,
                              options[key].get<int32_t>());
        else if (key == "bit_width_max_window")
            filter.set_option(TILEDB_BIT_WIDTH_MAX_WINDOW,
                              options[key].get<int32_t>());
        else if (key == "positive_delta_max_window")
            filter.set_option(TILEDB_POSITIVE_DELTA_MAX_WINDOW,
                              options[key].get<int32_t>());
        else if (key == "bit_width_max_window")
            filter.set_option(TILEDB_BIT_WIDTH_MAX_WINDOW,
                              options[key].get<uint32_t>());
        else if (key == "positive_delta_max_window")
            filter.set_option(TILEDB_POSITIVE_DELTA_MAX_WINDOW,
                              options[key].get<uint32_t>());
        else if (key == "scale_float_bytewidth")
            filter.set_option(TILEDB_SCALE_FLOAT_BYTEWIDTH,
                              options[key].get<uint64_t>());
        else if (key == "scale_float_factor")
            filter.set_option(TILEDB_SCALE_FLOAT_FACTOR,
                              options[key].get<double>());
        else if (key == "scale_float_offset")
            filter.set_option(TILEDB_SCALE_FLOAT_OFFSET,
                              options[key].get<double>());
        else
            throw tiledb::TileDBError("Unable to set filter option '" + key +
                                      "'. Not a valid TileDB filter option.");
    }
    return filter;
}

tiledb_filter_type_t
FilterFactory::filterTypeFromString(const std::string& filter_str)
{
    if (filter_str.empty())
        return TILEDB_FILTER_NONE;
    else if (filter_str == "gzip")
        return TILEDB_FILTER_GZIP;
    else if (filter_str == "zstd")
        return TILEDB_FILTER_ZSTD;
    else if (filter_str == "lz4")
        return TILEDB_FILTER_LZ4;
    else if (filter_str == "rle")
        return TILEDB_FILTER_RLE;
    else if (filter_str == "bzip2")
        return TILEDB_FILTER_BZIP2;
    else if (filter_str == "double-delta")
        return TILEDB_FILTER_DOUBLE_DELTA;
    else if (filter_str == "bit-width-reduction")
        return TILEDB_FILTER_BIT_WIDTH_REDUCTION;
    else if (filter_str == "bit-shuffle")
        return TILEDB_FILTER_BITSHUFFLE;
    else if (filter_str == "byte-shuffle")
        return TILEDB_FILTER_BYTESHUFFLE;
    else if (filter_str == "positive-delta")
        return TILEDB_FILTER_POSITIVE_DELTA;
    else
    {
        // Capitalize the filter name and convert '-' to '_'.
        std::string filter_name = filter_str;
        std::transform(
            filter_name.cbegin(), filter_name.cend(), filter_name.begin(),
            [](unsigned char c)
            { return (c == '-') ? static_cast<int>('_') : std::toupper(c); });
        // Get filter type from TileDB C-API.
        tiledb_filter_type_t filter_type = TILEDB_FILTER_NONE;
        auto rc =
            tiledb_filter_type_from_str(filter_name.c_str(), &filter_type);
        if (rc != TILEDB_OK)
            throw tiledb::TileDBError(
                "Unable to parse compression type '" + filter_str +
                "' using TileDB compression string '" + filter_name + "'.");
        return filter_type;
    }
}

tiledb::FilterList FilterFactory::filterList(const tiledb::Context& ctx,
                                             const std::string& dimName) const
{
    if (m_user_filters.count(dimName) > 0)
    {
        NL::json options = m_user_filters[dimName];
        tiledb::FilterList filterList{ctx};
        if (options.is_array())
        {
            for (auto& element : options.items())
            {
                auto value = element.value();
                filterList.add_filter(FilterFactory::filter(ctx, value));
            }
        }
        else
            filterList.add_filter(FilterFactory::filter(ctx, options));
        return filterList;
    }
    return defaultProfileFilterList(ctx, dimName);
}

tiledb::FilterList
FilterFactory::defaultFilterList(const tiledb::Context& ctx) const
{
    tiledb::FilterList filterList{ctx};
    if (!m_default_filter_type.has_value())
        return filterList;
    tiledb::Filter filter(ctx, m_default_filter_type.value());
    if (m_default_compression_level.has_value())
        filter.set_option(TILEDB_COMPRESSION_LEVEL,
                          m_default_compression_level.value());

    filterList.add_filter(filter);
    return filterList;
}

tiledb::FilterList
FilterFactory::defaultProfileFilterList(const tiledb::Context& ctx,
                                        const std::string& dimName) const
{
    tiledb::FilterList filterList{ctx};
    if (dimName == "X" || dimName == "Y" || dimName == "Z" ||
        dimName == "Red" || dimName == "Green" || dimName == "Blue" ||
        dimName == "GpsTime" || dimName == "ReturnNumber" ||
        dimName == "NumberOfReturns")
    {
        tiledb::Filter filter{ctx, TILEDB_FILTER_ZSTD};
        filter.set_option(TILEDB_COMPRESSION_LEVEL, 7);
        filterList.add_filter(filter);
        return filterList;
    }
    if (dimName == "Intensity" || dimName == "ScanDirectionFlag" ||
        dimName == "ScanAngleRank" || dimName == "EdgeOfFlightLine")
    {
        tiledb::Filter filter{ctx, TILEDB_FILTER_BZIP2};
        filter.set_option(TILEDB_COMPRESSION_LEVEL, 5);
        filterList.add_filter(filter);
        return filterList;
    }
    if (dimName == "UserData" || dimName == "Classification")
    {
        tiledb::Filter filter{ctx, TILEDB_FILTER_GZIP};
        filter.set_option(TILEDB_COMPRESSION_LEVEL, 9);
        filterList.add_filter(filter);
        return filterList;
    }
    if (dimName == "PointSourceId")
    {
        filterList.add_filter(tiledb::Filter(ctx, TILEDB_FILTER_BZIP2));
        return filterList;
    }
    return defaultFilterList(ctx);
}

} // namespace pdal
