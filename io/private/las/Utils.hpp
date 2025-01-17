/******************************************************************************
 * Copyright (c) 2015, Hobu Inc.
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

#include <functional>
#include <mutex>
#include <string>

#include <pdal/pdal_config.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/DimType.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/Scaling.hpp>

namespace pdal
{

class PointRef;

namespace las
{

struct Header;
class Srs;
class Summary;
struct Vlr;

enum class Compression
{
    True,
    False
};

inline std::istream& operator>>(std::istream& in, Compression& c)
{
    std::string s;

    in >> s;
    s = Utils::toupper(s);
    if (s == "LASZIP"  || s == "TRUE" || s == "LAZPERF")
        c = Compression::True;
    else
        c = Compression::False;
    return in;
}

inline std::ostream& operator<<(std::ostream& out, const Compression& c)
{
    switch (c)
    {
    case Compression::True:
        out << "true";
        break;
    case Compression::False:
        out << "false";
        break;
    }
    return out;
}

inline bool pointFormatSupported(int format)
{
    switch (format)
    {
    case 0:
    case 1:
    case 2:
    case 3:
    case 6:
    case 7:
    case 8:
        return true;
    default:
        return false;
    }
}

// Metadata

inline void addForwardMetadata(MetadataNode& forward, MetadataNode& m,
    const std::string& name, double val, const std::string description,
    size_t precision)
{
    MetadataNode n = m.add(name, val, description, precision);

    // If the entry doesn't already exist, just add it.
    MetadataNode f = forward.findChild(name);
    if (!f.valid())
    {
        forward.add(n);
        return;
    }

    // If the old value and new values aren't the same, set an invalid flag.
    MetadataNode temp = f.addOrUpdate("temp", val, description, precision);
    if (f.value<std::string>() != temp.value<std::string>())
        forward.addOrUpdate(name + "INVALID", "");
}

// Store data in the normal metadata place.  Also store it in the private
// lasforward metadata node.
template <typename T>
void addForwardMetadata(MetadataNode& forward, MetadataNode& m,
    const std::string& name, T val, const std::string description)
{
    MetadataNode n = m.add(name, val, description);

    // If the entry doesn't already exist, just add it.
    MetadataNode f = forward.findChild(name);
    if (!f.valid())
    {
        forward.add(n);
        return;
    }

    // If the old value and new values aren't the same, set an invalid flag.
    MetadataNode temp = f.addOrUpdate("temp", val);
    if (f.value<std::string>() != temp.value<std::string>())
        forward.addOrUpdate(name + "INVALID", "");
}

// Extra Dim

struct ExtraDim
{
    ExtraDim(const std::string name, Dimension::Type type, int byteOffset,
             double scale = 1.0, double offset = 0.0) :
        m_name(name), m_dimType(Dimension::Id::Unknown, type, scale, offset),
        m_size((uint8_t)Dimension::size(type)), m_byteOffset(byteOffset)
    {}
    ExtraDim(const std::string name, uint8_t size, int byteOffset) : m_name(name),
        m_dimType(Dimension::Id::Unknown, Dimension::Type::None), m_size(size),
        m_byteOffset(byteOffset)
    {}

    ExtraDim(const std::string name,  Dimension::Type type, Dimension::Id id, size_t size,
             int byteOffset, double scale = 1.0, double offset = 0.0) :
        m_name(name), m_dimType(id, type, scale, offset),
        m_size((uint8_t)size), m_byteOffset(byteOffset)
    {}
    friend bool operator == (const ExtraDim& ed1, const ExtraDim& ed2);

    std::string m_name;
    DimType m_dimType;
    uint8_t m_size;
    int32_t m_byteOffset;
};
using ExtraDims = std::vector<ExtraDim>;

inline bool operator == (const ExtraDim& ed1, const ExtraDim& ed2)
{
    // This is an incomplete comparison, but it should suffice since we
    // only use it to compare an ExtraDim specified in an option with
    // one created from a VLR entry.
    return (ed1.m_name == ed2.m_name &&
        ed1.m_dimType.m_type == ed2.m_dimType.m_type &&
        ed1.m_size == ed2.m_size);
}

// This is the structure of each record in the extra bytes spec.  Not used
// directly for storage, but here mostly for reference.
#pragma pack(push, 1)
struct ExtraBytesSpec
{
    char m_reserved[2];
    uint8_t m_dataType;
    uint8_t m_options;
    char m_name[32];
    char m_reserved2[4];
    uint64_t m_noData[3]; // 24 = 3*8 bytes
    double m_min[3]; // 24 = 3*8 bytes
    double m_max[3]; // 24 = 3*8 bytes
    double m_scale[3]; // 24 = 3*8 bytes
    double m_offset[3]; // 24 = 3*8 bytes
    char m_description[32];
};
#pragma pack(pop)
const int ExtraBytesSpecSize = sizeof(ExtraBytesSpec);

class ExtraBytesIf
{
public:
    ExtraBytesIf() : m_type(Dimension::Type::None), m_fieldCnt(0), m_size(0)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            m_scale[i] = 1.0;
            m_offset[i] = 0.0;
        }
    }

    ExtraBytesIf(const std::string& name, Dimension::Type type,
            const std::string& description) :
        m_type(type), m_name(name), m_description(description), m_size(0)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            // Setting the scale to 0 looks wrong, but it isn't.  If the
            // scale option flag isn't set, the scale is supposed to be 0.
            // When we write the VLR, we always clear the scale flag.
            m_scale[i] = 0.0;
            m_offset[i] = 0.0;
        }
        m_fieldCnt = (m_type == Dimension::Type::None ? 0 : 1);
    }

    void appendTo(std::vector<char>& ebBytes);
    void readFrom(const char *buf);
    void setType(uint8_t lastype);
    static std::vector<ExtraDim> toExtraDims(const char *buf, size_t bufsize, int byteOffset);

private:
    Dimension::Type m_type;
    unsigned m_fieldCnt; // Must be 0 - 3;
    double m_scale[3];
    double m_offset[3];
    std::string m_name;
    std::string m_description;
    size_t m_size;
};

// Function declarations

void extractHeaderMetadata(const Header& h, MetadataNode& forward, MetadataNode& m);
void extractSrsMetadata(const Srs& srs, MetadataNode& m);
void addVlrMetadata(const Vlr& vlr, std::string name, MetadataNode& forward, MetadataNode& m);
void setSummary(Header& header, const Summary& summary);
std::string generateSoftwareId();
ExtraDims parse(const StringList& dimString, bool allOk);
const Dimension::IdList& pdrfDims(int pdrf);
uint8_t lasType(Dimension::Type type, int fieldCnt);

struct error : public std::runtime_error
{
    error(const std::string& err) : std::runtime_error(err)
    {}
};

// Loader

class LoaderDriver;

class PointLoader
{
    friend class LoaderDriver;
public:
    virtual ~PointLoader() = default;

private:
    virtual void load(PointRef& point, const char *buf, int bufsize) = 0;
    virtual void pack(const PointRef& point, char *buf, int bufsize) = 0;
};
using PointLoaderPtr = std::unique_ptr<PointLoader>;

class PointFilter
{
    friend class LoaderDriver;

public:
    virtual ~PointFilter() = default;

private:
    virtual bool passes(PointRef& point) = 0;
};
using PointFilterPtr = std::unique_ptr<PointFilter>;

class V10BaseLoader : public PointLoader
{
public:
    V10BaseLoader(const Scaling& scaling);

private:
    virtual void load(PointRef& point, const char *buf, int bufsize) override;
    virtual void pack(const PointRef& point, char *buf, int bufsize) override;

    Scaling m_scaling;
};

class V14BaseLoader : public PointLoader
{
public:
    V14BaseLoader(const Scaling& scaling);

private:
    virtual void load(PointRef& point, const char *buf, int bufsize) override;
    virtual void pack(const PointRef& point, char *buf, int bufsize) override;

    Scaling m_scaling;
};

class GpstimeLoader : public PointLoader
{
public:
    GpstimeLoader(int offset) : m_offset(offset)
    {}

private:
    virtual void load(PointRef& point, const char *buf, int bufsize) override;
    virtual void pack(const PointRef& point, char *buf, int bufsize) override;

    int m_offset;
};

class ColorLoader : public PointLoader
{
public:
    ColorLoader(int offset) : m_offset(offset)
    {}

private:
    virtual void load(PointRef& point, const char *buf, int bufsize) override;
    virtual void pack(const PointRef& point, char *buf, int bufsize) override;

    int m_offset;
};

class NirLoader : public PointLoader
{
public:
    NirLoader(int offset) : m_offset(offset)
    {}

private:
    virtual void load(PointRef& point, const char *buf, int bufsize) override;
    virtual void pack(const PointRef& point, char *buf, int bufsize) override;

    int m_offset;
};

class ExtraDimLoader : public PointLoader
{
public:
    ExtraDimLoader(const ExtraDims& extraDims) : m_extraDims(extraDims)
    {}

private:
    virtual void load(PointRef& point, const char *buf, int bufsize) override;
    virtual void pack(const PointRef& point, char *buf, int bufsize) override;

    ExtraDims m_extraDims;
};

class LoaderDriver
{
public:
    LoaderDriver() = default;
    LoaderDriver(int pdrf, const Scaling& scaling, const ExtraDims& dims);

    void init(int pdrf, const Scaling& scaling, const ExtraDims& dims);
    bool load(PointRef& point, const char *buf, int bufsize);
    bool pack(const PointRef& point, char *buf, int bufsize);
private:
    std::vector<PointLoaderPtr> m_loaders;
};

} // namespace las
} // namespace pdal
