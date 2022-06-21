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
#include <pdal/util/Algorithm.hpp>
#include <io/private/las/Header.hpp>
#include <io/private/las/Srs.hpp>
#include <io/private/las/Utils.hpp>

namespace pdal
{

const std::string LasHeader::FILE_SIGNATURE("LASF");

std::string GetDefaultSoftwareId()
{
    return las::generateSoftwareId();
}

struct LasHeader::Private
{
    Private(las::Header& h, las::Srs& srs, las::VlrList& vlrs) : h(h), srs(srs), vlrs(vlrs)
    {}

    Private(const Private& src) : h(src.h), srs(src.srs), vlrs(src.vlrs),
        interfaceVlrs(src.interfaceVlrs)
    {}

    las::Header& h;
    las::Srs& srs;
    las::VlrList& vlrs;

    VlrList interfaceVlrs;
};

LasHeader::LasHeader(las::Header& h, las::Srs& srs, las::VlrList& vlrs) :
    d(std::make_unique<Private>(h, srs, vlrs))
{}

LasHeader::LasHeader(const LasHeader& src) : d(std::make_unique<Private>(*(src.d)))
{}

LasHeader::LasHeader(LasHeader&& src) : d(std::move(src.d))
{}

LasHeader& LasHeader::operator=(const LasHeader& src)
{
    d.reset(new Private(*src.d));
    return *this;
}

LasHeader& LasHeader::operator=(LasHeader&& src)
{
    d.swap(src.d);
    return *this;
}

LasHeader::~LasHeader()
{}

std::string LasHeader::getSystemIdentifier() const
{
    return d->h.systemId;
}

std::string LasHeader::fileSignature() const
{
    return d->h.magic;
}

uint16_t LasHeader::fileSourceId() const
{
    return d->h.fileSourceId;
}

void LasHeader::setFileSourceId(uint16_t v)
{
    d->h.fileSourceId = v;
}

uint16_t LasHeader::globalEncoding() const
{
    return d->h.globalEncoding;
}

void LasHeader::setGlobalEncoding(uint16_t globalEncoding)
{
    d->h.globalEncoding = globalEncoding;
}

Uuid LasHeader::projectId() const
{
    return d->h.projectGuid;
}

void LasHeader::setProjectId(const Uuid& v)
{
    d->h.projectGuid = v;
}

uint8_t LasHeader::versionMajor() const
{
    return d->h.versionMajor;
}

uint8_t LasHeader::versionMinor() const
{
    return d->h.versionMinor;
}

void LasHeader::setVersionMinor(uint8_t v)
{
    d->h.versionMinor = v;
}

std::string LasHeader::versionString() const
{
    return std::to_string(versionMajor()) + "." + std::to_string(versionMinor());
}

bool LasHeader::versionAtLeast(uint8_t major, uint8_t minor) const
{
    return d->h.versionAtLeast(major, minor);
}

bool LasHeader::versionEquals(uint8_t major, uint8_t minor) const
{
    return (major == 1 && minor == versionMinor());
}

std::string LasHeader::systemId() const
{
    return d->h.systemId;
}

void LasHeader::setSystemId(std::string const& v)
{
    d->h.systemId = v;
}

std::string LasHeader::softwareId() const
{
    return d->h.softwareId;
}

void LasHeader::setSoftwareId(std::string const& v)
{
    d->h.softwareId = v;
}

uint16_t LasHeader::creationDOY() const
{
    return d->h.creationDoy;
}

void LasHeader::setCreationDOY(uint16_t v)
{
    d->h.creationDoy = v;
}

uint16_t LasHeader::creationYear() const
{
    return d->h.creationYear;
}

void LasHeader::setCreationYear(uint16_t v)
{
    d->h.creationYear = v;
}

uint16_t LasHeader::vlrOffset() const
{
    return d->h.vlrOffset;
}

void LasHeader::setVlrOffset(uint16_t offset)
{
    (void)offset;
}

uint32_t LasHeader::pointOffset() const
{
    return d->h.pointOffset;
}

void LasHeader::setPointOffset(uint32_t offset)
{
    (void)offset;
}

uint8_t LasHeader::pointFormat() const
{
    return d->h.pointFormatBits;
}

void LasHeader::setPointFormat(uint8_t format)
{
    d->h.pointFormatBits = format;
}

Utils::StatusWithReason LasHeader::pointFormatSupported() const
{
    if (hasWave())
        return { -1, "PDAL does not support point formats with waveform data (4, 5, 9 and 10)" };
    if (versionAtLeast(1, 4))
    {
        if (pointFormat() > 10)
            return { -1, "LAS version " + versionString() + " only supports point formats 0-10." };
    }
    else
    {
        if (pointFormat() > 5)
            return { -1, "LAS version '" + versionString() + " only supports point formats 0-5." };
    }
    return true;
}

uint16_t LasHeader::pointLen() const
{
    return d->h.pointSize;
}

void LasHeader::setPointLen(uint16_t length)
{
    (void)length;
}

uint16_t LasHeader::basePointLen()
{
    return (uint16_t)d->h.baseCount();
}

uint16_t LasHeader::basePointLen(uint8_t format)
{
    return (uint16_t)(las::baseCount(format));
}

uint64_t LasHeader::pointCount() const
{
    return d->h.pointCount();
}

void LasHeader::setPointCount(uint64_t pointCount)
{
    (void)pointCount;
}

uint64_t LasHeader::pointCountByReturn(std::size_t index) const
{
    return versionMinor() == 4 ? d->h.ePointsByReturn[index] : d->h.legacyPointsByReturn[index];
}

void LasHeader::setPointCountByReturn(std::size_t index, uint64_t count)
{
    (void)index;
    (void)count;
}

size_t LasHeader::maxReturnCount() const
{
    return (versionAtLeast(1, 4) ? RETURN_COUNT : LEGACY_RETURN_COUNT);
}

double LasHeader::scaleX() const
{
    return d->h.scale.x;
}

double LasHeader::scaleY() const
{
    return d->h.scale.y;
}

double LasHeader::scaleZ() const
{
    return d->h.scale.z;
}

void LasHeader::setScaling(const Scaling& scaling)
{
    (void)scaling;
}

double LasHeader::offsetX() const
{
    return d->h.offset.x;
}

double LasHeader::offsetY() const
{
    return d->h.offset.y;
}

double LasHeader::offsetZ() const
{
    return d->h.offset.z;
}

void LasHeader::setOffset(double x, double y, double z)
{
    (void)x;
    (void)y;
    (void)z;
}

double LasHeader::maxX() const
{
    return d->h.bounds.maxx;
}

double LasHeader::minX() const
{
    return d->h.bounds.minx;
}

double LasHeader::maxY() const
{
    return d->h.bounds.maxy;
}

double LasHeader::minY() const
{
    return d->h.bounds.miny;
}

double LasHeader::maxZ() const
{
    return d->h.bounds.maxz;
}

double LasHeader::minZ() const
{
    return d->h.bounds.minz;
}

const BOX3D& LasHeader::getBounds() const
{
    return d->h.bounds;
}

void LasHeader::setBounds(const BOX3D& bounds)
{
    (void)bounds;
}

bool LasHeader::hasTime() const
{
    return d->h.hasTime();
}

bool LasHeader::hasColor() const
{
    return d->h.hasColor();
}

bool LasHeader::hasWave() const
{
    return d->h.hasWave();
}

bool LasHeader::hasInfrared() const
{
    return d->h.hasInfrared();
}

bool LasHeader::has14PointFormat() const
{
    return d->h.has14PointFormat();
}

bool LasHeader::useWkt() const
{
    return d->h.useWkt();
}

bool LasHeader::compressed() const
{
    return d->h.dataCompressed();
}

void LasHeader::setCompressed(bool b)
{
    (void)b;
}

uint32_t LasHeader::vlrCount() const
{
    return d->h.vlrCount;
}

void LasHeader::setVlrCount(uint32_t vlrCount)
{
    (void)vlrCount;
}

uint64_t LasHeader::eVlrOffset() const
{
    return d->h.evlrOffset;
}

void LasHeader::setEVlrOffset(uint64_t offset)
{
    (void)offset;
}

uint32_t LasHeader::eVlrCount() const
{
    return d->h.evlrCount;
}

void LasHeader::setEVlrCount(uint32_t count)
{
    (void)count;
}

std::string const& LasHeader:: compressionInfo() const
{
    static std::string dummy;
    return dummy;
}

void LasHeader::setCompressionInfo(std::string const& info)
{
    (void)info;
}

SpatialReference LasHeader::srs() const
{
    return d->srs.get();
}

std::string LasHeader::geotiffPrint()
{
    return d->srs.geotiffString();
}

void LasHeader::setSummary(const LasSummaryData& summary)
{
    (void)summary;
}

bool LasHeader::valid() const
{
    return true;
}

Dimension::IdList LasHeader::usedDims() const
{
    return las::pdrfDims(pointFormat());
}

const LasVLR *LasHeader::findVlr(const std::string& userId, uint16_t recordId) const
{
    for (const LasVLR& v : vlrs())
        if (v.matches(userId, recordId))
            return &v;
    return nullptr;
}

void LasHeader::removeVLR(const std::string& userId, uint16_t recordId)
{
    (void)userId;
    (void)recordId;
}

void LasHeader::removeVLR(const std::string& userId)
{
    (void)userId;
}

void LasHeader::initialize(LogPtr log, uintmax_t fileSize, bool nosrs)
{
    (void)log;
    (void)fileSize;
    (void)nosrs;
}

const VlrList& LasHeader::vlrs() const
{
    d->interfaceVlrs.clear();
    for (las::Vlr& v : d->vlrs)
        d->interfaceVlrs.emplace_back(&v);
    return d->interfaceVlrs;
}

ILeStream& operator>>(ILeStream& in, LasHeader& h)
{
    return in;
}

OLeStream& operator<<(OLeStream& out, const LasHeader& h)
{
    return out;
}

std::ostream& operator<<(std::ostream& ostr, const LasHeader& h)
{
    return ostr;
}

} // namespace pdal
