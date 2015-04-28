/******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
 * Copyright (c) 2008, Phil Vachon
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#pragma once

#include <array>
#include <vector>

#include <boost/uuid/uuid.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pdal/util/Bounds.hpp>
#include <pdal/pdal_config.hpp>
#include <pdal/gitsha.h>

#include "VariableLengthRecord.hpp"

namespace pdal
{
class OLeStream;
class ILeStream;

typedef uint8_t PointFormat;
std::string GetDefaultSoftwareId();
class SummaryData;

class PDAL_DLL LasHeader
{
public:
    static const size_t LEGACY_RETURN_COUNT = 5;
    static const size_t RETURN_COUNT = 15;
    static const std::string FILE_SIGNATURE;
    inline std::string getSystemIdentifier() const { return "PDAL"; }

    LasHeader();

    /// Get ASPRS LAS file signature.
    /// \return 4-characters long string - \b "LASF".
    std::string fileSignature() const
        { return m_fileSig; }

    /// Set ASPRS LAS file signature.
    /// The only value allowed as file signature is \b "LASF",
    /// defined as FileSignature constant.
    /// \exception std::invalid_argument - if invalid signature given.
    /// \param v - string contains file signature, at least 4-bytes long
    /// with "LASF" as first four bytes.
    void SetFileSignature(std::string const& v);

    /// Get file source identifier.
    /// \exception No throw
    uint16_t fileSourceId() const
        { return m_sourceId; }

    /// Set file source identifier.
    /// \param v - should be set to a value between 1 and 65535.
    /// \exception No throw
    void setFileSourceId(uint16_t v)
        { m_sourceId = v; }

    uint16_t globalEncoding() const
        { return m_globalEncoding; }
    void setGlobalEncoding(uint16_t globalEncoding)
        { m_globalEncoding = globalEncoding; }

    /// Get project identifier.
    /// \return Global Unique Identifier as an instance of liblas::guid class.
    boost::uuids::uuid projectId() const
        { return m_projectGuid; }

    /// Set project identifier.
    void setProjectId(boost::uuids::uuid const& v)
        { m_projectGuid = v; }

    /// Get the LAS major version.
    /// \return  LAS major version
    uint8_t versionMajor() const
        { return (uint8_t)1; }

    /// Get minor component of version of LAS format.
    /// \return Valid values are 0, 1, 2, 3.
    uint8_t versionMinor() const
        { return m_versionMinor; }

    /// Set minor component of version of LAS format.
    /// \exception std::out_of_range - invalid value given.
    /// \param v - value between eVersionMinorMin and eVersionMinorMax.
    void setVersionMinor(uint8_t v)
    {
        assert(v <= 4);
        m_versionMinor = v;
    }

    /// Determine if the header is for a LAS file version of at least
    ///   a certain level.
    /// \param major - Major version.
    /// \param minor - Minor version.
    /// \return  Whether the version meets the criteria.
    bool versionAtLeast(uint8_t major, uint8_t minor) const
        { return (1 >= major && m_versionMinor >= minor); }

    /// Determine if the header is for a particular LAS file version.
    /// \param major - Major version.
    /// \param minor - Minor version.
    /// \return  Whether the version meets the criteria.
    bool versionEquals(uint8_t major, uint8_t minor) const
        { return (major == 1 && minor == m_versionMinor); }

    /// Get system identifier.
    /// Default value is \b "libLAS" specified as the SystemIdentifier constant.
    /// \param pad - if true the returned string is padded right with spaces and
    /// its length is 32 bytes, if false (default) no padding occurs and
    /// length of the returned string is <= 32 bytes.
    /// \return value of system identifier field.
    std::string systemId() const
        { return m_systemId; }

    /// Set system identifier.
    /// \param v - system identifiers string.
    void setSystemId(std::string const& v)
        { m_systemId = v; }

    /// Get software identifier.
    /// Default value is \b "libLAS 1.0", specified as the SoftwareIdentifier
    /// constant.
    std::string softwareId() const
        { return m_softwareId; }

    /// Set software identifier.
    /// \param v - software identifiers string.
    void setSoftwareId(std::string const& v)
        { m_softwareId = v; }

    /// Get day of year of file creation date.
    uint16_t creationDOY() const
        { return m_createDOY; }

    /// Set day of year of file creation date.
    /// \exception std::out_of_range - given value is higher than number 366.
    void setCreationDOY(uint16_t v)
        { m_createDOY = v; }

    /// Set year of file creation date.
    uint16_t creationYear() const
        { return m_createYear; }

    /// Get year of file creation date.
    /// \exception std::out_of_range - given value is higher than number 9999.
    void setCreationYear(uint16_t v)
        { m_createYear = v; }

    /// Get number of bytes of generic verion of public header block storage.
    /// Standard version of the public header block is 227 bytes long.
    uint16_t vlrOffset() const
        { return m_vlrOffset; }

    void setVlrOffset(uint16_t offset)
        { m_vlrOffset = offset; }

    /// Get number of bytes from the beginning to the first point record.
    uint32_t pointOffset() const
        { return m_pointOffset; }

    /// Set number of bytes from the beginning to the first point record.
    /// \param  offset - Offset to start of point data.
    void setPointOffset(uint32_t offset)
          { m_pointOffset = offset; }

    /// Set the point format.
    /// \param format  Point format
    void setPointFormat(uint8_t format)
        { m_pointFormat = format; }

    /// Get identifier of point data (record) format.
    uint8_t pointFormat() const
        { return m_pointFormat; }
    bool pointFormatSupported() const
    {
        if (versionAtLeast(1, 4))
            return m_pointFormat <= 10 && !hasWave();
        else
            return m_pointFormat <= 5 && !hasWave();
    }

    /// The length in bytes of each point.  All points in the file are
    /// considered to be fixed in size, and the PointFormatName is used
    /// to determine the fixed portion of the dimensions in the point.
    uint16_t pointLen() const
        { return m_pointLen; }
	void setPointLen(uint16_t v)
        { m_pointLen = v; }
    uint16_t basePointLen()
        { return basePointLen(m_pointFormat); }
    uint16_t basePointLen(uint8_t format);

    /// Set the number of points.
    /// \param pointCount  Number of points in the file.
    void setPointCount(uint64_t pointCount)
        { m_pointCount = pointCount; }
    /// Get total number of point records stored in the LAS file.
    uint64_t pointCount() const
        { return m_pointCount; }
    //
    /// Set values point count by return number.
    /// \param index - Return number.
    /// \param v - Point count for return number.
    void setPointCountByReturn(std::size_t index, uint64_t v)
        { m_pointCountByReturn[index] = v; }

    /// Get the point count by return number.
    /// \param index - Return number.
    /// \return - Point count.
    uint64_t pointCountByReturn(std::size_t index)
        { return m_pointCountByReturn[index]; }

    size_t maxReturnCount() const
        { return (versionAtLeast(1, 4) ? RETURN_COUNT : LEGACY_RETURN_COUNT); }

    /// Get scale factor for X coordinate.
    double scaleX() const
        { return m_scales[0]; }

    /// Get scale factor for Y coordinate.
    double scaleY() const
        { return m_scales[1]; }

    /// Get scale factor for Z coordinate.
    double scaleZ() const
        { return m_scales[2]; }

    /// Set values of scale factor for X, Y and Z coordinates.
    void setScale(double x, double y, double z);

    /// Get X coordinate offset.
    double offsetX() const
        { return m_offsets[0]; }

    /// Get Y coordinate offset.
    double offsetY() const
        { return m_offsets[1]; }

    /// Get Z coordinate offset.
    double offsetZ() const
        { return m_offsets[2]; }

    /// Set values of X, Y and Z coordinates offset.
    void setOffset(double x, double y, double z);

    /// Get minimum value of extent of X coordinate.
    double maxX() const
        { return m_bounds.maxx; }

    /// Get maximum value of extent of X coordinate.
    double minX() const
        { return m_bounds.minx; }

    /// Get minimum value of extent of Y coordinate.
    double maxY() const
        { return m_bounds.maxy; }

    /// Get maximum value of extent of Y coordinate.
    double minY() const
        { return m_bounds.miny; }

    /// Get minimum value of extent of Z coordinate.
    double maxZ() const
        { return m_bounds.maxz; }

    /// Get maximum value of extent of Z coordinate.
    double minZ() const
       { return m_bounds.minz; }

    const BOX3D& getBounds() const
        { return m_bounds; }
    void setBounds(const BOX3D& bounds)
        { m_bounds = bounds; }

    bool hasTime() const
    {
        PointFormat f = pointFormat();
        return f == 1 || f >= 3;
    }

    bool hasColor() const
    {
        PointFormat f = pointFormat();
        return f == 2 || f == 3 || f == 5 || f == 7 || f == 8 || f == 10;
    }

    bool hasWave() const
    {
        PointFormat f = pointFormat();
        return f == 4 || f == 5 || f == 9 || f == 10;
    }

    bool hasInfrared() const
    {
        PointFormat f = pointFormat();
        return f == 8;
    }

    bool has14Format() const
    {
        PointFormat f = pointFormat();
        return f > 5;
    }

    /// Returns true iff the file is compressed (laszip),
    /// as determined by the high bit in the point type
    bool compressed() const
        { return m_isCompressed; }

    /// Sets whether or not the points are compressed.
    void setCompressed(bool b)
        { m_isCompressed = b; }

    void setVlrCount(uint32_t vlrCount)
        { m_vlrCount = vlrCount; }
    uint32_t vlrCount() const
        { return m_vlrCount; }
    void setEVlrOffset(uint64_t offset)
        { m_eVlrOffset = offset; }
    uint64_t eVlrOffset() const
        { return m_eVlrOffset; }
    void setEVlrCount(uint32_t count)
        { m_eVlrCount = count; }
    uint32_t eVlrCount() const
        { return m_eVlrCount; }

    std::string const& compressionInfo() const
        { return m_compressionInfo; }
    void setCompressionInfo(std::string const& info)
        { m_compressionInfo = info; }

    void setSummary(const SummaryData& summary);
    bool valid() const;

    friend ILeStream& operator>>(ILeStream&, LasHeader& h);
    friend OLeStream& operator<<(OLeStream&, const LasHeader& h);
    friend std::ostream& operator<<(std::ostream& ostr, const LasHeader& h);

private:
    std::string m_fileSig;
    uint16_t m_sourceId;
    uint16_t m_globalEncoding;
    boost::uuids::uuid m_projectGuid;
    uint8_t m_versionMinor;
    std::string m_systemId;
    std::string m_softwareId;
    uint16_t m_createDOY;
    uint16_t m_createYear;
    uint16_t m_vlrOffset;  // Same as header size.
    uint32_t m_pointOffset;
    uint32_t m_vlrCount;
    uint8_t m_pointFormat;
    uint16_t m_pointLen;
    uint64_t m_pointCount;
    std::array<uint64_t, RETURN_COUNT> m_pointCountByReturn;
    std::array<double, 3> m_scales;
    std::array<double, 3> m_offsets;
    bool m_isCompressed;
    uint64_t m_eVlrOffset;
    uint32_t m_eVlrCount;
    BOX3D m_bounds;
    std::string m_compressionInfo;

    static void get(ILeStream& in, boost::uuids::uuid& uuid);
    static void put(OLeStream& in, boost::uuids::uuid uuid);
};

} // namespace pdal
