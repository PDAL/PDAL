/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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

#include "Header.hpp"
#include "Lasdump.hpp"

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace pdal
{
namespace lasdump
{

const std::string Header::FILE_SIGNATURE("LASF");
#ifndef WIN32
const size_t Header::LEGACY_RETURN_COUNT;
const size_t Header::RETURN_COUNT;
#endif

Header::Header() : m_sourceId(0), m_globalEncoding(0), m_versionMinor(0),
    m_createDOY(0), m_createYear(0), m_vlrOffset(0), m_pointOffset(0),
    m_vlrCount(0), m_pointFormat(0), m_pointLen(0), m_pointCount(0),
    m_isCompressed(false), m_eVlrOffset(0), m_eVlrCount(0)
{
    m_pointCountByReturn.fill(0);
    m_scales.fill(1.0);
    m_offsets.fill(0.0);
}


uint16_t Header::basePointLen(uint8_t type)
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


bool Header::valid() const
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


void Header::get(ILeStream& in, boost::uuids::uuid& uuid)
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


ILeStream& operator>>(ILeStream& in, Header& h)
{
    uint8_t versionMajor;
    uint32_t legacyPointCount;
    uint32_t legacyReturnCount;

    in.get(h.m_fileSig, 4);
    if (!h.signatureValid())
        throw Exception("Not a LAS/LAZ file.  Invalid file signature.");
    in >> h.m_sourceId >> h.m_globalEncoding;
    Header::get(in, h.m_projectGuid);
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
    
    for (size_t i = 0; i < Header::LEGACY_RETURN_COUNT; ++i)
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
        uint32_t numPoints = (uint32_t)h.m_pointCount;
        in >> h.m_eVlrOffset >> h.m_eVlrCount >> h.m_pointCount;
        for (size_t i = 0; i < Header::RETURN_COUNT; ++i)
            in >> h.m_pointCountByReturn[i];
        if (numPoints && (numPoints != h.m_pointCount))
        {
            std::stringstream ss;
            ss << "1.4 point count (" << h.m_pointCount << ") "
                "doesn't match legacy point count (" << numPoints << ").";
            throw Exception(ss.str());
        }
    }

    return in;
}


std::ostream& operator<<(std::ostream& out, const Header& h)
{
    out << "File version: " << "1." << (int)h.m_versionMinor << "\n";
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
    for (size_t i = 0; i < Header::RETURN_COUNT; ++i)
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

} // namespace lasdump
} // namespace pdal

