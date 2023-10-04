
#include <iostream>

#include "Header.hpp"

#include <pdal/util/Extractor.hpp>
#include <pdal/util/Inserter.hpp>

namespace pdal
{
namespace las
{

int baseCount(int format)
{
    // LAZ screws with the high bits of the format, so we mask down to the low four bits.
    switch (format & Header::FormatMask)
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
    default:
        return 0;
    }
}

void Header::fill(const char *buf, size_t bufsize)
{
    LeExtractor s(buf, bufsize);

    s.get(magic, 4);
    s >> fileSourceId >> globalEncoding;

    char guidBuf[Uuid::size()];
    s.get(guidBuf, Uuid::size());
    projectGuid.unpack(guidBuf);

    s >> versionMajor >> versionMinor;
    s.get(systemId, 32);
    s.get(softwareId, 32);
    s >> creationDoy >> creationYear;
    s >> headerSize >> pointOffset >> vlrCount;
    s >> pointFormatBits >> pointSize >> legacyPointCount;

    for (uint32_t& pbr : legacyPointsByReturn)
        s >> pbr;

    s >> scale.x >> scale.y >> scale.z;
    s >> offset.x >> offset.y >> offset.z;
    s >> bounds.maxx >> bounds.minx >> bounds.maxy >> bounds.miny >> bounds.maxz >> bounds.minz;
    if (versionMinor >= 3)
    {
        s >> waveOffset;
        if (versionMinor >= 4)
        {
            s >> evlrOffset >> evlrCount >> ePointCount;
            for (uint64_t& pbr : ePointsByReturn)
                s >> pbr;
        }
    }
}

std::vector<char> Header::data() const
{
    size_t size = Size12;
    if (versionMinor >= 4)
        size = Size14;
    else if (versionMinor == 3)
        size = Size13;

    std::vector<char> buf(size);
    LeInserter s(buf.data(), buf.size());

    s.put(magic, 4);
    s << fileSourceId << globalEncoding;

    char guidBuf[Uuid::size()];
    projectGuid.pack(guidBuf);
    s.put(guidBuf, Uuid::size());

    s << versionMajor << versionMinor;
    s.put(systemId, 32);
    s.put(softwareId, 32);
    s << creationDoy << creationYear;
    s << headerSize << pointOffset << vlrCount;
    s << pointFormatBits << pointSize << legacyPointCount;

    for (const uint32_t& pbr : legacyPointsByReturn)
        s << pbr;

    s << scale.x << scale.y << scale.z;
    s << offset.x << offset.y << offset.z;
    s << bounds.maxx << bounds.minx << bounds.maxy << bounds.miny << bounds.maxz << bounds.minz;
    if (versionMinor >= 3)
    {
        s << waveOffset;
        if (versionMinor >= 4)
        {
            s << evlrOffset << evlrCount << ePointCount;
            for (const uint64_t& pbr : ePointsByReturn)
                s << pbr;
        }
    }
    return buf;
}

StringList Header::validate(uint64_t fileSize, bool nosrs) const
{
    StringList errors;

    if (magic != "LASF")
        errors.push_back("Invalid file signature. Was expecting 'LASF', Check the first four "
            " bytes of the file.");
    if (!nosrs)
        if (has14PointFormat() && !useWkt())
            errors.push_back("Global encoding WKT flag not set for point format 6 - 10.");
    if (!dataCompressed() && (pointOffset > fileSize))
        errors.push_back("Invalid point offset - exceeds file size.");
    if (!dataCompressed() && (pointOffset + pointCount() * pointSize > fileSize))
        errors.push_back("Invalid point count: " + std::to_string(pointCount()) +
            ". Number of points too large for file size.");
    if (vlrOffset > fileSize)
        errors.push_back("Invalid VLR offset - exceeds file size.");
    return errors;
}

void Header::setPointCount(uint64_t pointCount)
{
    ePointCount = pointCount;
    bool condition = (ePointCount <= (std::numeric_limits<uint32_t>::max)()) &&
        !((versionMinor >= 4) && (pointFormat() >= 6) );
    legacyPointCount =
        condition ? (uint32_t)ePointCount : 0;
}

void Header::setPointsByReturn(int returnNum, uint64_t pointCount)
{
    ePointsByReturn[returnNum] = pointCount;
    if (returnNum < LegacyReturnCount)
    {
        bool condition = (pointCount <= (std::numeric_limits<uint32_t>::max)()) &&
            !((versionMinor >= 4) && (pointFormat() >= 6) );

        legacyPointsByReturn[returnNum] = condition ? (uint32_t)pointCount : 0;
    }
}

} // namespace las
} // namespace pdal

