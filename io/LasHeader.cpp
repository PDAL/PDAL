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

#include <pdal/pdal_config.hpp>
#include <pdal/Scaling.hpp>
#include <pdal/util/Utils.hpp>

#include "LasSummaryData.hpp"

#include "GeotiffSupport.hpp"

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


void LasHeader::setSummary(const LasSummaryData& summary)
{
    m_pointCount = summary.getTotalNumPoints();
    for (size_t num = 0; num < RETURN_COUNT; ++num)
        m_pointCountByReturn[num] = (int)summary.getReturnCount(num);
    m_bounds = summary.getBounds();
}


void LasHeader::setScaling(const Scaling& scaling)
{
    const double& xs = scaling.m_xXform.m_scale.m_val;
    const double& ys = scaling.m_yXform.m_scale.m_val;
    const double& zs = scaling.m_zXform.m_scale.m_val;
    if (xs == 0)
        throw std::invalid_argument("X scale of 0.0 is invalid!");

    if (ys == 0)
        throw std::invalid_argument("Y scale of 0.0 is invalid!");

    if (zs == 0)
        throw std::invalid_argument("Z scale of 0.0 is invalid!");

    m_scales[0] = xs;
    m_scales[1] = ys;
    m_scales[2] = zs;

    m_offsets[0] = scaling.m_xXform.m_offset.m_val;
    m_offsets[1] = scaling.m_yXform.m_offset.m_val;
    m_offsets[2] = scaling.m_zXform.m_offset.m_val;
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


void LasHeader::get(ILeStream& in, Uuid& uuid)
{
    char buf[uuid.size];

    in.get(buf, uuid.size);
    uuid.unpack(buf);
}


void LasHeader::put(OLeStream& out, Uuid uuid)
{
    char buf[uuid.size];

    uuid.pack(buf);
    out.put(buf, uuid.size);
}


Dimension::IdList LasHeader::usedDims() const
{
    using namespace Dimension;

    Dimension::Id dims[] = { Id::ReturnNumber, Id::NumberOfReturns,
        Id::X, Id::Y, Id::Z, Id::Intensity, Id::ScanChannel,
        Id::ScanDirectionFlag, Id::EdgeOfFlightLine, Id::Classification,
        Id::UserData, Id::ScanAngleRank, Id::PointSourceId };

    Dimension::IdList ids(std::begin(dims), std::end(dims));

    if (hasTime())
        ids.push_back(Id::GpsTime);
    if (hasColor())
    {
        ids.push_back(Id::Red);
        ids.push_back(Id::Green);
        ids.push_back(Id::Blue);
    }
    if (hasInfrared())
        ids.push_back(Id::Infrared);

    return ids;
}

void LasHeader::setSrs()
{
    bool useWkt = false;

    if (incompatibleSrs())
    {
        m_log->get(LogLevel::Error) << "Invalid SRS specification.  "
            "GeoTiff not allowed with point formats 6 - 10." << std::endl;
    }
    else if (findVlr(TRANSFORM_USER_ID, WKT_RECORD_ID) &&
        findVlr(TRANSFORM_USER_ID, GEOTIFF_DIRECTORY_RECORD_ID))
    {
        m_log->get(LogLevel::Debug) << "File contains both "
            "WKT and GeoTiff VLRs which is disallowed." << std::endl;
    }
    else
        useWkt = (m_versionMinor >= 4);

    try
    {
        if (useWkt)
            setSrsFromWkt();
        else
            setSrsFromGeotiff();
    }
    catch (...)
    {
        m_log->get(LogLevel::Error) << "Could not create an SRS" << std::endl;
    }
}


LasVLR *LasHeader::findVlr(const std::string& userId,
    uint16_t recordId)
{
    for (auto vi = m_vlrs.begin(); vi != m_vlrs.end(); ++vi)
    {
        LasVLR& vlr = *vi;
        if (vlr.matches(userId, recordId))
            return &vlr;
    }
    return NULL;
}


void LasHeader::setSrsFromWkt()
{
    LasVLR *vlr = findVlr(TRANSFORM_USER_ID, WKT_RECORD_ID);
    if (!vlr)
        vlr = findVlr(LIBLAS_USER_ID, WKT_RECORD_ID);
    if (!vlr || vlr->dataLen() == 0)
        return;

    // There is supposed to be a NULL byte at the end of the data,
    // but sometimes there isn't because some people don't follow the
    // rules.  If there is a NULL byte, don't stick it in the
    // wkt string.
    size_t len = vlr->dataLen();
    const char *c = vlr->data() + len - 1;
    if (*c == 0)
        len--;
    m_srs.set(std::string(vlr->data(), len));
}


void LasHeader::setSrsFromGeotiff()
{
// These are defined in geo_simpletags.h
// We're not including that file because it includes
// geotiff.h, which includes a ton of other stuff
// that might conflict with the messy libgeotiff/GDAL
// symbol mess

#define STT_SHORT   1
#define STT_DOUBLE  2
#define STT_ASCII   3

    GeotiffSupport geotiff;
    geotiff.resetTags();

    LasVLR *vlr;

    vlr = findVlr(TRANSFORM_USER_ID, GEOTIFF_DIRECTORY_RECORD_ID);
    // We must have a directory entry.
    if (!vlr)
        return;
    if (!geotiff.setShortKeys(vlr->recordId(), (void *)vlr->data(),
        (int)vlr->dataLen()))
    {
        std::ostringstream oss;

        oss << "Invalid GeoTIFF directory record.  Can't "
            "interpret spatial reference.";
        throw pdal_error(oss.str());
    }

    vlr = findVlr(TRANSFORM_USER_ID, GEOTIFF_DOUBLES_RECORD_ID);
    if (vlr && !vlr->isEmpty())
        geotiff.setDoubleKeys(vlr->recordId(), (void *)vlr->data(),
            (int)vlr->dataLen());
    vlr = findVlr(TRANSFORM_USER_ID, GEOTIFF_ASCII_RECORD_ID);
    if (vlr && !vlr->isEmpty())
        geotiff.setAsciiKeys(vlr->recordId(), (void *)vlr->data(),
            (int)vlr->dataLen());

    geotiff.setTags();
    std::string wkt(geotiff.getWkt(false, false));
    if (wkt.size())
        m_srs.set(wkt);

    m_log->get(LogLevel::Debug5) << "GeoTIFF keys: " << geotiff.getText() <<
        std::endl;
}


ILeStream& operator>>(ILeStream& in, LasHeader& h)
{
    uint8_t versionMajor;
    uint32_t legacyPointCount;
    uint32_t legacyReturnCount;

    in.get(h.m_fileSig, 4);
    if (!Utils::iequals(h.m_fileSig, "LASF"))
    {
        throw pdal::pdal_error("File signature is not 'LASF', "
            "is this an LAS/LAZ file?");
    }
    in >> h.m_sourceId >> h.m_globalEncoding;
    LasHeader::get(in, h.m_projectUuid);
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

    // Read regular VLRs.
    in.seek(h.m_vlrOffset);
    for (size_t i = 0; i < h.m_vlrCount; ++i)
    {
        LasVLR r;
        in >> r;
        h.m_vlrs.push_back(std::move(r));
    }

    // Read extended VLRs.
    if (h.versionAtLeast(1, 4))
    {
        in.seek(h.m_eVlrOffset);
        for (size_t i = 0; i < h.m_eVlrCount; ++i)
        {
            ExtLasVLR r;
            in >> r;
            h.m_vlrs.push_back(std::move(r));
        }
    }
    h.setSrs();

    return in;
}


OLeStream& operator<<(OLeStream& out, const LasHeader& h)
{
    uint32_t legacyPointCount = 0;
    if (h.m_pointCount <= (std::numeric_limits<uint32_t>::max)())
        legacyPointCount = (uint32_t)h.m_pointCount;

    out.put(h.m_fileSig, 4);
    if (h.versionEquals(1, 0))
        out << (uint32_t)0;
    else if (h.versionEquals(1, 1))
        out << h.m_sourceId << (uint16_t)0;
    else
        out << h.m_sourceId << h.m_globalEncoding;
    LasHeader::put(out, h.m_projectUuid);
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
    out << "Project UUID: " << h.m_projectUuid << "\n";
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
