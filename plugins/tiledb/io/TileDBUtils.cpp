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
                             const std::string& filterProfile,
                             const std::array<double, 3>& scaleFactor,
                             const std::array<double, 3>& addOffset,
                             const std::string& defaultCompressor,
                             int32_t defaultCompressionLevel)
    : m_scale_factor{scaleFactor}, m_add_offset{addOffset}
{
    m_user_filters.update(userProvidedFilters);
    if (filterProfile == "balanced")
        m_filter_profile = FilterFactory::Profile::balanced;
    else if (filterProfile == "aggressive")
        m_filter_profile = FilterFactory::Profile::aggressive;
    else if (filterProfile == "none")
        m_filter_profile = FilterFactory::Profile::none;
    else
        throw tiledb::TileDBError("Filter profile '" + filterProfile +
                                  "' is not a valid filter profile");

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
        else if (key == "compression_level" || key == "COMPRESSION_LEVEL")
            filter.set_option(TILEDB_COMPRESSION_LEVEL,
                              options[key].get<int32_t>());
        else if (key == "bit_width_max_window" || key == "BIT_WIDTH_MAX_WINDOW")
            filter.set_option(TILEDB_BIT_WIDTH_MAX_WINDOW,
                              options[key].get<uint32_t>());
        else if (key == "positive_delta_max_window" ||
                 key == "POSITIVE_DELTA_MAX_WINDOW")
            filter.set_option(TILEDB_POSITIVE_DELTA_MAX_WINDOW,
                              options[key].get<uint32_t>());
        else if (key == "scale_float_bytewidth" ||
                 key == "SCALE_FLOAT_BYTEWIDTH")
            filter.set_option(TILEDB_SCALE_FLOAT_BYTEWIDTH,
                              options[key].get<uint64_t>());
        else if (key == "scale_float_factor" || key == "SCALE_FLOAT_FACTOR")
            filter.set_option(TILEDB_SCALE_FLOAT_FACTOR,
                              options[key].get<double>());
        else if (key == "scale_float_offset" || key == "SCALE_FLOAT_OFFSET")
            filter.set_option(TILEDB_SCALE_FLOAT_OFFSET,
                              options[key].get<double>());
        else if (key == "webp_quality" || key == "WEBP_QUALITY")
            filter.set_option(TILEDB_WEBP_QUALITY, options[key].get<float>());
        else if (key == "webp_input_format" || key == "WEBP_INPUT_FORMAT")
        {
            auto input_format_str = options[key].get<std::string>();
            uint8_t input_format = 0;
            if (input_format_str == "none")
                input_format = 0;
            else if (input_format_str == "rgb")
                input_format = 1;
            else if (input_format_str == "bgr")
                input_format = 2;
            else if (input_format_str == "rgba")
                input_format = 3;
            else if (input_format_str == "bgra")
                input_format = 4;
            else
                throw tiledb::TileDBError("Unrecognized webp input format '" +
                                          input_format_str + "'.");
            filter.set_option<uint8_t>(TILEDB_WEBP_INPUT_FORMAT, input_format);
        }
        else if (key == "webp_lossless" || key == "WEBP_LOSSLESS")
        {
            bool lossless = options[key].get<bool>();
            filter.set_option<uint8_t>(TILEDB_WEBP_LOSSLESS,
                                       static_cast<uint8_t>(lossless));
        }
        else if (key == "reinterpret_datatype" ||
                 key == "COMPRESSION_REINTERPRET_DATATYPE")
        {
            auto datatype_str = options[key].get<std::string>();
            tiledb_datatype_t output_datatype{};
            auto rc = tiledb_datatype_from_str(datatype_str.c_str(),
                                               &output_datatype);
            if (rc != TILEDB_OK)
                throw tiledb::TileDBError(
                    "Unable to get TileDB datatype from datatype string '" +
                    datatype_str + "'.");
            uint8_t datatype_int = static_cast<uint8_t>(output_datatype);
            filter.set_option<uint8_t>(TILEDB_COMPRESSION_REINTERPRET_DATATYPE,
                                       datatype_int);
        }
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
    else if (filter_str == "float-scale")
        return TILEDB_FILTER_SCALE_FLOAT;
    else if (filter_str == "none")
        return TILEDB_FILTER_NONE;
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
    switch (m_filter_profile)
    {
    case FilterFactory::Profile::none:
        return defaultFilterList(ctx);
    case FilterFactory::Profile::balanced:
        return defaultProfileFilterList(ctx, dimName, true);
    case FilterFactory::Profile::aggressive:
        return defaultProfileFilterList(ctx, dimName, false);
    default:
        throw std::logic_error("Invalid filter profile");
    }
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

tiledb::FilterList FilterFactory::defaultProfileFilterList(
    const tiledb::Context& ctx, const std::string& dimName, bool balanced) const
{
    tiledb::FilterList filterList{ctx};
    if (dimName == "X" || dimName == "Y" || dimName == "Z")
    {
        // Float scale filter
        uint32_t index{dimName == "X"
                           ? (uint32_t)0
                           : (dimName == "Y" ? (uint32_t)1 : (uint32_t)2)};
        uint64_t bytewidth = 4;
        tiledb::Filter floatScale{ctx, TILEDB_FILTER_SCALE_FLOAT};
        floatScale.set_option(TILEDB_SCALE_FLOAT_BYTEWIDTH, bytewidth);
        floatScale.set_option(TILEDB_SCALE_FLOAT_FACTOR, m_scale_factor[index]);
        floatScale.set_option(TILEDB_SCALE_FLOAT_OFFSET, m_add_offset[index]);
        filterList.add_filter(floatScale);

        // Delta filter
        tiledb::Filter delta{ctx, TILEDB_FILTER_DELTA};
        uint8_t deltaDatatype = static_cast<uint8_t>(TILEDB_INT32);
        delta.set_option<uint8_t>(TILEDB_COMPRESSION_REINTERPRET_DATATYPE,
                                  deltaDatatype);
        filterList.add_filter(delta);

        if (balanced)
        {
            // Bit shuffle filter
            tiledb::Filter bit_shuffle{ctx, TILEDB_FILTER_BITSHUFFLE};
            filterList.add_filter(bit_shuffle);

            // Zstd filter
            tiledb::Filter zstd{ctx, TILEDB_FILTER_ZSTD};
            zstd.set_option(TILEDB_COMPRESSION_LEVEL, 7);
            filterList.add_filter(zstd);
        }
        else
        {
            // Bit width reduction
            tiledb::Filter bit_width_reduction{
                ctx, TILEDB_FILTER_BIT_WIDTH_REDUCTION};
            filterList.add_filter(bit_width_reduction);

            // Bzip filter
            tiledb::Filter bzip2{ctx, TILEDB_FILTER_BZIP2};
            bzip2.set_option(TILEDB_COMPRESSION_LEVEL, 9);
            filterList.add_filter(bzip2);
        }
    }
    else if (dimName == "GpsTime")
    {
        // Delta filter
        tiledb::Filter delta{ctx, TILEDB_FILTER_DELTA};
        uint8_t deltaDatatype = static_cast<uint8_t>(TILEDB_INT64);
        delta.set_option<uint8_t>(TILEDB_COMPRESSION_REINTERPRET_DATATYPE,
                                  deltaDatatype);
        filterList.add_filter(delta);

        // Bit width reduction
        tiledb::Filter bit_width_reduction{ctx,
                                           TILEDB_FILTER_BIT_WIDTH_REDUCTION};
        filterList.add_filter(bit_width_reduction);

        if (balanced)
        {
            // Zstd filter
            tiledb::Filter zstd{ctx, TILEDB_FILTER_ZSTD};
            zstd.set_option(TILEDB_COMPRESSION_LEVEL, 7);
            filterList.add_filter(zstd);
        }
        else
        {
            // Bzip filter
            tiledb::Filter bzip2{ctx, TILEDB_FILTER_BZIP2};
            bzip2.set_option(TILEDB_COMPRESSION_LEVEL, 9);
            filterList.add_filter(bzip2);
        }
    }
    else if (dimName == "Red" || dimName == "Green" || dimName == "Blue")
    {
        // Delta filter
        tiledb::Filter delta{ctx, TILEDB_FILTER_DELTA};
        filterList.add_filter(delta);

        // Bit width reduction
        tiledb::Filter bit_width_reduction{ctx,
                                           TILEDB_FILTER_BIT_WIDTH_REDUCTION};
        filterList.add_filter(bit_width_reduction);

        if (balanced)
        {
            // Zstd filter
            tiledb::Filter zstd{ctx, TILEDB_FILTER_ZSTD};
            zstd.set_option(TILEDB_COMPRESSION_LEVEL, 7);
            filterList.add_filter(zstd);
        }
        else
        {

            // Bzip filter
            tiledb::Filter bzip2{ctx, TILEDB_FILTER_BZIP2};
            bzip2.set_option(TILEDB_COMPRESSION_LEVEL, 9);
            filterList.add_filter(bzip2);
        }
    }
    else if (dimName == "Intensity")
    {
        // Delta filter
        tiledb::Filter delta{ctx, TILEDB_FILTER_DELTA};
        filterList.add_filter(delta);

        if (balanced)
        {
            // Zstd filter
            tiledb::Filter zstd{ctx, TILEDB_FILTER_ZSTD};
            zstd.set_option(TILEDB_COMPRESSION_LEVEL, 5);
            filterList.add_filter(zstd);
        }
        else
        {
            // Bit width reduction
            tiledb::Filter bit_width_reduction{
                ctx, TILEDB_FILTER_BIT_WIDTH_REDUCTION};
            filterList.add_filter(bit_width_reduction);

            // Bzip2 filter
            tiledb::Filter bzip2{ctx, TILEDB_FILTER_BZIP2};
            bzip2.set_option(TILEDB_COMPRESSION_LEVEL, 5);
            filterList.add_filter(bzip2);
        }
    }
    else if (dimName == "Classification" || dimName == "ReturnNumber" ||
             dimName == "NumberOfReturns" || dimName == "ScanDirectionFlag" ||
             dimName == "ScanAngleRank" || dimName == "EdgeOfFlightLine" ||
             dimName == "PointSourceId" || dimName == "UserData" ||
             dimName == "BitFields")
    {
        if (balanced)
        {
            // Zstd filter
            tiledb::Filter zstd{ctx, TILEDB_FILTER_ZSTD};
            zstd.set_option(TILEDB_COMPRESSION_LEVEL, 5);
            filterList.add_filter(zstd);
        }
        else
        {
            // Bzip2 filter
            tiledb::Filter bzip2{ctx, TILEDB_FILTER_BZIP2};
            bzip2.set_option(TILEDB_COMPRESSION_LEVEL, 9);
            filterList.add_filter(bzip2);
        }
    }
    else
    {
        return defaultFilterList(ctx);
    }
    return filterList;
}

BitFieldsBuffer::BitFieldsBuffer(const std::string& name, PointLayoutPtr layout)
    : m_name{name}
{
    for (uint32_t index = 0; index < 6; ++index)
    {
        const auto dimName = BitFieldsBuffer::bitFieldName(index);
        m_ids[index] =
            layout->registerOrAssignDim(dimName, Dimension::Type::Signed8);
    }
}

BitFieldsBuffer::BitFieldsBuffer(
    const std::string& name,
    const std::array<std::optional<Dimension::Id>, 6> bitFieldIds)
    : m_name{name}, m_ids{bitFieldIds}
{
}

std::optional<uint32_t>
BitFieldsBuffer::bitFieldIndex(const std::string& dimName)
{
    if (dimName == "ReturnNumber")
        return 0;
    if (dimName == "NumberOfReturns")
        return 1;
    if (dimName == "ClassFlags")
        return 2;
    if (dimName == "ScanChannel")
        return 3;
    if (dimName == "ScanDirectionFlag")
        return 4;
    if (dimName == "EdgeOfFlightLine")
        return 5;
    return std::nullopt;
}

std::string BitFieldsBuffer::bitFieldName(const uint32_t index)
{
    switch (index)
    {
    case 0:
        return "ReturnNumber";
    case 1:
        return "NumberOfReturns";
    case 2:
        return "ClassFlags";
    case 3:
        return "ScanChannel";
    case 4:
        return "ScanDirectionFlag";
    case 5:
        return "EdgeOfFlightLine";
    default:
        throw tiledb::TileDBError("Bit field index " + std::to_string(index) +
                                  " is out of bounds.");
    }
}

void BitFieldsBuffer::copyDataToBuffer(PointRef& point, size_t index)
{
    // Get bit field data. Set to zero if bit field does not exist in table.
    std::array<uint8_t, 6> unpacked{};
    if (m_ids[0].has_value())
        unpacked[0] = point.getFieldAs<uint8_t>(m_ids[0].value()) & 0x0F;
    if (m_ids[1].has_value())
        unpacked[1] = (point.getFieldAs<uint8_t>(m_ids[1].value()) & 0x0F) << 4;
    if (m_ids[2].has_value())
        unpacked[2] = point.getFieldAs<uint8_t>(m_ids[2].value()) & 0x0F;
    if (m_ids[3].has_value())
        unpacked[3] = (point.getFieldAs<uint8_t>(m_ids[3].value()) & 0x03) << 4;
    if (m_ids[4].has_value())
        unpacked[4] = (point.getFieldAs<uint8_t>(m_ids[4].value()) & 0x01) << 6;
    if (m_ids[5].has_value())
        unpacked[5] = (point.getFieldAs<uint8_t>(m_ids[5].value()) & 0x01) << 7;

    // Set the 2 bytes of data in the buffer.
    auto value1 = static_cast<uint16_t>(unpacked[0] | unpacked[1]) << 8;
    auto value2 = static_cast<uint16_t>(unpacked[2] | unpacked[3] |
                                        unpacked[4] | unpacked[5]);
    m_data[index] = value1 | value2;
}

void BitFieldsBuffer::copyDataToPoint(PointRef& point, size_t index)
{
    // Read the 2 uint8_t values that store the packed bits.
    const uint16_t full_value = m_data[index];
    const auto value1 = static_cast<uint8_t>(0xFF & (full_value >> 8));
    const auto value2 = static_cast<uint8_t>(0xFF & full_value);

    // Unpack the bits.
    std::array<uint8_t, 6> unpacked{};
    unpacked[0] = value1 & 0x0F;
    unpacked[1] = (value1 >> 4) & 0x0F;
    unpacked[2] = value2 & 0x0F;
    unpacked[3] = (value2 >> 4) & 0x03;
    unpacked[4] = (value2 >> 6) & 0x01;
    unpacked[5] = (value2 >> 7) & 0x01;

    // Set the dimension data on the PDAL point.
    for (uint32_t index = 0; index < 6; ++index)
        point.setField(m_ids[index].value(), unpacked[index]);
}

const std::string& BitFieldsBuffer::name() const
{
    return m_name;
}

tiledb::Attribute
BitFieldsBuffer::createAttribute(const tiledb::Context& ctx) const
{
    auto attr = tiledb::Attribute::create<uint16_t>(ctx, m_name);
    return attr;
}

void BitFieldsBuffer::resizeBuffer(size_t nelements)
{
    m_data.resize(nelements);
}

void BitFieldsBuffer::setQueryBuffer(tiledb::Query& query)
{
    query.set_data_buffer(m_name, m_data);
}

} // namespace pdal
