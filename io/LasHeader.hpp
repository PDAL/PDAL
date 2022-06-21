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

#include <memory>
#include <vector>

#include <pdal/Dimension.hpp>
#include <pdal/Log.hpp>
#include <pdal/util/Utils.hpp>

#include "LasVLR.hpp"

namespace pdal
{
class BOX3D;
class OLeStream;
class ILeStream;
class LasHeader;
class LasReader;
class LasSummaryData;
class Scaling;
class SpatialReference;

namespace las
{
    struct Header;
    class Srs;
    struct Vlr;
    using VlrList = std::vector<Vlr>;
}

[[deprecated]] PDAL_DLL ILeStream& operator>>(ILeStream&, LasHeader& h);
[[deprecated]] OLeStream& operator<<(OLeStream&, const LasHeader& h);
[[deprecated]] std::ostream& operator<<(std::ostream& ostr, const LasHeader& h);

class PDAL_DLL LasHeader
{
public:
    static const size_t LEGACY_RETURN_COUNT {5};
    static const size_t RETURN_COUNT {15};
    static const std::string FILE_SIGNATURE;

    std::string getSystemIdentifier() const;

    /// Get ASPRS LAS file signature.
    /// \return 4-characters long string - \b "LASF".
    std::string fileSignature() const;

    /// Get file source identifier.
    uint16_t fileSourceId() const;

    /// Set file source identifier.
    /// \param v - should be set to a value between 1 and 65535.
    [[deprecated]] void setFileSourceId(uint16_t v);

    uint16_t globalEncoding() const;
    [[deprecated]] void setGlobalEncoding(uint16_t globalEncoding);

    /// Get project identifier.
    /// \return UUID
    Uuid projectId() const;

    /// Set project identifier.
    [[deprecated]] void setProjectId(const Uuid& v);

    /// Get the LAS major version.
    /// \return  LAS major version
    uint8_t versionMajor() const;

    std::string versionString() const;

    /// Get minor component of version of LAS format.
    /// \return Valid values are 0, 1, 2, 3.
    uint8_t versionMinor() const;

    /// Set minor component of version of LAS format.
    /// \param v - value between eVersionMinorMin and eVersionMinorMax.
    [[deprecated]] void setVersionMinor(uint8_t v);

    /// Determine if the header is for a LAS file version of at least
    ///   a certain level.
    /// \param major - Major version.
    /// \param minor - Minor version.
    /// \return  Whether the version meets the criteria.
    bool versionAtLeast(uint8_t major, uint8_t minor) const;

    /// Determine if the header is for a particular LAS file version.
    /// \param major - Major version.
    /// \param minor - Minor version.
    /// \return  Whether the version meets the criteria.
    bool versionEquals(uint8_t major, uint8_t minor) const;

    /// Get system identifier.
    /// Default value is \b "libLAS" specified as the SystemIdentifier constant.
    /// \param pad - if true the returned string is padded right with spaces and
    /// its length is 32 bytes, if false (default) no padding occurs and
    /// length of the returned string is <= 32 bytes.
    /// \return value of system identifier field.
    std::string systemId() const;

    /// Set system identifier.
    /// \param v - system identifiers string.
    [[deprecated]] void setSystemId(std::string const& v);

    /// Get software identifier.
    /// Default value is \b "libLAS 1.0", specified as the SoftwareIdentifier
    /// constant.
    std::string softwareId() const;

    /// Set software identifier.
    /// \param v - software identifiers string.
    [[deprecated]] void setSoftwareId(std::string const& v);

    /// Get day of year of file creation date.
    uint16_t creationDOY() const;

    /// Set day of year of file creation date.
    [[deprecated]] void setCreationDOY(uint16_t v);

    /// Set year of file creation date.
    uint16_t creationYear() const;

    /// Get year of file creation date.
    [[deprecated]] void setCreationYear(uint16_t v);

    /// Get number of bytes of generic verion of public header block storage.
    /// Standard version of the public header block is 227 bytes long.
    uint16_t vlrOffset() const;

    [[deprecated]] void setVlrOffset(uint16_t offset);

    /// Get number of bytes from the beginning to the first point record.
    uint32_t pointOffset() const;

    /// Set number of bytes from the beginning to the first point record.
    /// \param  offset - Offset to start of point data.
    [[deprecated]] void setPointOffset(uint32_t offset);

    /// Get identifier of point data (record) format.
    uint8_t pointFormat() const;

    /// Set the point format.
    /// \param format  Point format
    [[deprecated]] void setPointFormat(uint8_t format);

    Utils::StatusWithReason pointFormatSupported() const;

    /// The length in bytes of each point.  All points in the file are
    /// considered to be fixed in size, and the point format is used
    /// to determine the fixed portion of the dimensions in the point.
    uint16_t pointLen() const;
	[[deprecated]] void setPointLen(uint16_t v);

    uint16_t basePointLen();
    uint16_t basePointLen(uint8_t format);

    /// Get total number of point records stored in the LAS file.
    uint64_t pointCount() const;
    /// Set the number of points.
    /// \param pointCount  Number of points in the file.
    [[deprecated]] void setPointCount(uint64_t pointCount);

    /// Get the point count by return number.
    /// \param index - Return number.
    /// \return - Point count.
    uint64_t pointCountByReturn(std::size_t index) const;

    /// Set values point count by return number.
    /// \param index - Return number.
    /// \param v - Point count for return number.
    [[deprecated]] void setPointCountByReturn(std::size_t index, uint64_t v);

    size_t maxReturnCount() const;

    /// Get scale factor for X coordinate.
    double scaleX() const;

    /// Get scale factor for Y coordinate.
    double scaleY() const;

    /// Get scale factor for Z coordinate.
    double scaleZ() const;

    /// Set values of scale/offset factor for X, Y and Z coordinates.
    [[deprecated]] void setScaling(const Scaling& scaling);

    /// Get X coordinate offset.
    double offsetX() const;

    /// Get Y coordinate offset.
    double offsetY() const;

    /// Get Z coordinate offset.
    double offsetZ() const;

    /// Set values of X, Y and Z coordinates offset.
    [[deprecated]] void setOffset(double x, double y, double z);

    /// Get minimum value of extent of X coordinate.
    double maxX() const;

    /// Get maximum value of extent of X coordinate.
    double minX() const;

    /// Get minimum value of extent of Y coordinate.
    double maxY() const;

    /// Get maximum value of extent of Y coordinate.
    double minY() const;

    /// Get minimum value of extent of Z coordinate.
    double maxZ() const;

    /// Get maximum value of extent of Z coordinate.
    double minZ() const;

    const BOX3D& getBounds() const;
    [[deprecated]] void setBounds(const BOX3D& bounds);
    bool hasTime() const;
    bool hasColor() const;
    bool hasWave() const;
    bool hasInfrared() const;
    bool has14PointFormat() const;
    bool useWkt() const;

    /// Returns true iff the file is compressed (laszip),
    /// as determined by the high bit in the point type
    bool compressed() const;

    /// Sets whether or not the points are compressed.
    [[deprecated]] void setCompressed(bool b);

    uint32_t vlrCount() const;
    [[deprecated]] void setVlrCount(uint32_t vlrCount);
    uint64_t eVlrOffset() const;
    [[deprecated]] void setEVlrOffset(uint64_t offset);
    uint32_t eVlrCount() const;
    void setEVlrCount(uint32_t count);
    [[deprecated]] std::string const& compressionInfo() const;
    [[deprecated]] void setCompressionInfo(std::string const& info);
    SpatialReference srs() const;
    std::string geotiffPrint();
    [[deprecated]] void setSummary(const LasSummaryData& summary);
    [[deprecated]] bool valid() const;
    Dimension::IdList usedDims() const;
    const LasVLR *findVlr(const std::string& userId, uint16_t recordId) const;
    [[deprecated]] void removeVLR(const std::string& userId, uint16_t recordId);
    [[deprecated]] void removeVLR(const std::string& userId);
    [[deprecated]] void initialize(LogPtr log, uintmax_t fileSize, bool nosrs);
    [[deprecated]] const VlrList& vlrs() const;

    friend ILeStream& operator>>(ILeStream&, LasHeader& h);
    friend OLeStream& operator<<(OLeStream&, const LasHeader& h);
    friend std::ostream& operator<<(std::ostream& ostr, const LasHeader& h);

    LasHeader(las::Header& h, las::Srs& srs, las::VlrList& vlrs);
    LasHeader(const LasHeader& src);
    LasHeader(LasHeader&& src);
    LasHeader& operator=(const LasHeader& src);
    LasHeader& operator=(LasHeader&& src);
    ~LasHeader();

private:
    struct Private;
    std::unique_ptr<Private> d;

    static void get(ILeStream& in, Uuid& uuid);
    static void put(OLeStream& in, Uuid uuid);
};

} // namespace pdal
