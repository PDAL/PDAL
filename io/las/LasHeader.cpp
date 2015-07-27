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

#include "LasHeader.hpp"

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <pdal/util/Utils.hpp>
#include <pdal/pdal_config.hpp>

#include "SummaryData.hpp"

namespace pdal
{

const std::string LasHeader::FILE_SIGNATURE("LASF");
#ifndef _WIN32
const size_t LasHeader::LEGACY_RETURN_COUNT;
const size_t LasHeader::RETURN_COUNT;
#endif

std::string GetDefaultSoftwareId()
{
    std::string ver(PDAL_VERSION_STRING);
    std::stringstream oss;
    std::ostringstream revs;
    revs << GetSHA1();


    oss << "PDAL " << ver << " (" << revs.str().substr(0, 6) <<")";
    return oss.str();
}

LasHeader::LasHeader() : m_fileSig(FILE_SIGNATURE), m_sourceId(0),
    m_globalEncoding(0), m_versionMinor(2), m_systemId(getSystemIdentifier()),
    m_createDOY(0), m_createYear(0), m_vlrOffset(0), m_pointOffset(0),
    m_vlrCount(0), m_pointFormat(0), m_pointLen(0), m_pointCount(0),
    m_isCompressed(false), m_eVlrOffset(0), m_eVlrCount(0)
{
    std::time_t now;
    std::time(&now);
    std::tm* ptm = std::gmtime(&now);
    if (ptm)
    {
        m_createDOY = static_cast<uint16_t>(ptm->tm_yday);
        m_createYear = static_cast<uint16_t>(ptm->tm_year + 1900);
    }

    m_pointLen = basePointLen(m_pointFormat);
    m_pointCountByReturn.fill(0);
    m_scales.fill(1.0);
    m_offsets.fill(0.0);
}


void LasHeader::setSummary(const SummaryData& summary)
{
    m_pointCount = summary.getTotalNumPoints();
    for (size_t num = 0; num < RETURN_COUNT; ++num)
        m_pointCountByReturn[num] = summary.getReturnCount(num);
    m_bounds = summary.getBounds();
}


void LasHeader::setScale(double x, double y, double z)
{
    if (Utils::compare_distance(0.0, x))
        throw std::invalid_argument("X scale of 0.0 is invalid!");

    if (Utils::compare_distance(0.0, y))
        throw std::invalid_argument("Y scale of 0.0 is invalid!");

    if (Utils::compare_distance(0.0, z))
        throw std::invalid_argument("Z scale of 0.0 is invalid!");

    m_scales[0] = x;
    m_scales[1] = y;
    m_scales[2] = z;
}


void LasHeader::setOffset(double x, double y, double z)
{
    m_offsets[0] = x;
    m_offsets[1] = y;
    m_offsets[2] = z;
}


uint16_t LasHeader::basePointLen(uint8_t type)
{
    switch (type)
    {
    case 0:
        return 20;
    case 1:
        return 28;
    case 2:
        return 26;
    case 3:
        return 34;
    case 6:
        return 30;
    case 7:
        return 36;
    case 8:
        return 38;
    }
    return 0;
}


bool LasHeader::valid() const
{
    if (m_fileSig != FILE_SIGNATURE)
        return false;
    if (m_versionMinor > 10)
        return false;
    if (m_createDOY > 366)
        return false;
    if (m_createYear < 1970 || m_createYear > 2100)
       return false;
    return true;
}


void LasHeader::get(ILeStream& in, boost::uuids::uuid& uuid)
{
    union
    {
        char buf[16];
        struct
        {
            uint32_t uidPart1;
            uint16_t uidPart2;
            uint16_t uidPart3;
            char uidPart4[8];
        };
    } u;

    in >> u.uidPart1 >> u.uidPart2 >> u.uidPart3;
    in.get(u.uidPart4, sizeof(u.uidPart4));
    memcpy(uuid.data, u.buf, sizeof(u.buf));
}


void LasHeader::put(OLeStream& out, boost::uuids::uuid uuid)
{
    union
    {
        char buf[16];
        struct
        {
            uint32_t uidPart1;
            uint16_t uidPart2;
            uint16_t uidPart3;
            char uidPart4[8];
        };
    } u;

    memcpy(u.buf, uuid.data, sizeof(u.buf));
    out << u.uidPart1 << u.uidPart2 << u.uidPart3;
    out.put(u.uidPart4, sizeof(u.uidPart4));
}


ILeStream& operator>>(ILeStream& in, LasHeader& h)
{
    uint8_t versionMajor;
    uint32_t legacyPointCount;
    uint32_t legacyReturnCount;

    in.get(h.m_fileSig, 4);
    if (!Utils::iequals(h.m_fileSig, "LASF"))
    {
        throw pdal::pdal_error("File signature is not 'LASF', is this an LAS/LAZ file?");
    }
    in >> h.m_sourceId >> h.m_globalEncoding;
    LasHeader::get(in, h.m_projectGuid);
    in >> versionMajor >> h.m_versionMinor;
    in.get(h.m_systemId, 32);

    in.get(h.m_softwareId, 32);
    in >> h.m_createDOY >> h.m_createYear >> h.m_vlrOffset >>
        h.m_pointOffset >> h.m_vlrCount >> h.m_pointFormat >>
        h.m_pointLen >> legacyPointCount;
    h.m_pointCount = legacyPointCount;

    // Although it isn't part of the LAS spec, the two high bits have been used
    // to indicate compression, though only the high bit is currently used.
    if (h.m_pointFormat & 0x80)
        h.setCompressed(true);
    h.m_pointFormat &= ~0xC0;

    for (size_t i = 0; i < LasHeader::LEGACY_RETURN_COUNT; ++i)
    {
        in >> legacyReturnCount;
        h.m_pointCountByReturn[i] = legacyReturnCount;
    }

    in >> h.m_scales[0] >> h.m_scales[1] >> h.m_scales[2];
    in >> h.m_offsets[0] >> h.m_offsets[1] >> h.m_offsets[2];

    double maxX, minX;
    double maxY, minY;
    double maxZ, minZ;
    in >> maxX >> minX >> maxY >> minY >> maxZ >> minZ;
    h.m_bounds = BOX3D(minX, minY, minZ, maxX, maxY, maxZ);

    if (h.versionAtLeast(1, 3))
    {
        uint64_t waveformOffset;
        in >> waveformOffset;
    }
    if (h.versionAtLeast(1, 4))
    {
        in >> h.m_eVlrOffset >> h.m_eVlrCount >> h.m_pointCount;
        for (size_t i = 0; i < LasHeader::RETURN_COUNT; ++i)
            in >> h.m_pointCountByReturn[i];
    }

    return in;
}


OLeStream& operator<<(OLeStream& out, const LasHeader& h)
{
    uint32_t legacyPointCount = 0;
    if (h.m_pointCount <= (std::numeric_limits<uint32_t>::max)())
        legacyPointCount = h.m_pointCount;

    out.put(h.m_fileSig, 4);
    if (h.versionEquals(1, 0))
        out << (uint32_t)0;
    else if (h.versionEquals(1, 1))
        out << h.m_sourceId << (uint16_t)0;
    else
        out << h.m_sourceId << h.m_globalEncoding;
    LasHeader::put(out, h.m_projectGuid);
    out << (uint8_t)1 << h.m_versionMinor;
    out.put(h.m_systemId, 32);
    out.put(h.m_softwareId, 32);

    uint8_t pointFormat = h.m_pointFormat;
    if (h.compressed())
        pointFormat |= 0x80;

    out << h.m_createDOY << h.m_createYear << h.m_vlrOffset <<
        h.m_pointOffset << h.m_vlrCount << pointFormat <<
        h.m_pointLen << legacyPointCount;

    for (size_t i = 0; i < LasHeader::LEGACY_RETURN_COUNT; ++i)
    {
        uint32_t legacyReturnCount = std::min(h.m_pointCountByReturn[i],
            (uint64_t)(std::numeric_limits<uint32_t>::max)());
        out << legacyReturnCount;
    }

    out << h.m_scales[0] << h.m_scales[1] << h.m_scales[2];
    out << h.m_offsets[0] << h.m_offsets[1] << h.m_offsets[2];

    out << h.maxX() << h.minX() << h.maxY() << h.minY() << h.maxZ() << h.minZ();

    if (h.versionAtLeast(1, 3))
        out << (uint64_t)0;
    if (h.versionAtLeast(1, 4))
    {
        out << h.m_eVlrOffset << h.m_eVlrCount << h.m_pointCount;
        for (size_t i = 0; i < LasHeader::RETURN_COUNT; ++i)
            out << h.m_pointCountByReturn[i];
    }

    return out;
}


std::ostream& operator<<(std::ostream& out, const LasHeader& h)
{
    out << "File version = " << "1." << (int)h.m_versionMinor << "\n";
    out << "File signature: " << h.m_fileSig << "\n";
    out << "File source ID: " << h.m_sourceId << "\n";
    out << "Global encoding: " << h.m_globalEncoding << "\n";
    out << "Project GUID: " << h.m_projectGuid << "\n";
    out << "System ID: " << h.m_systemId << "\n";
    out << "Software ID: " << h.m_softwareId << "\n";
    out << "Creation DOY: " << h.m_createDOY << "\n";
    out << "Creation Year: " << h.m_createYear << "\n";
    out << "VLR offset (header size): " << h.m_vlrOffset << "\n";
    out << "VLR Count: " << h.m_vlrCount << "\n";
    out << "Point format: " << (int)h.m_pointFormat << "\n";
    out << "Point offset: " << h.m_pointOffset << "\n";
    out << "Point count: " << h.m_pointCount << "\n";
    for (size_t i = 0; i < LasHeader::RETURN_COUNT; ++i)
        out << "Point count by return[" << i << "]: " <<
            h.m_pointCountByReturn[i] << "\n";
    out << "Scales X/Y/Z: " << h.m_scales[0] << "/" <<
        h.m_scales[1] << "/" << h.m_scales[2] << "\n";
    out << "Offsets X/Y/Z: " << h.m_offsets[0] << "/" <<
        h.m_offsets[1] << "/" << h.m_offsets[2] << "\n";
    out << "Max X/Y/Z: " << h.maxX() << "/" <<
        h.maxY() << "/" << h.maxZ() << "\n";
    out << "Min X/Y/Z: " << h.minX() << "/" <<
        h.minY() << "/" << h.minZ() << "\n";
    if (h.versionAtLeast(1, 4))
    {
        out << "Ext. VLR offset: " << h.m_eVlrOffset << "\n";
        out << "Ext. VLR count: " << h.m_eVlrCount << "\n";
    }
    out << "Compressed: " << (h.m_isCompressed ? "true" : "false") << "\n";
    return out;
}

} // namespace pdal
