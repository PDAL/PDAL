/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include <pdal/PointView.hpp>
#include <pdal/Writer.hpp>

namespace pdal
{

class Triangle;

class PDAL_DLL PlyWriter : public Writer
{
public:
    enum class Format
    {
        Ascii,
        BinaryLe,
        BinaryBe
    };

    std::string getName() const;

    PlyWriter();

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr data);
    virtual void done(PointTableRef table);

    std::string getType(Dimension::Type type) const;
    void writeHeader(PointLayoutPtr layout) const;
    void writeValue(PointRef& point, Dimension::Id dim, Dimension::Type type);
    void writePoint(PointRef& point, PointLayoutPtr layout);
    void writeTriangle(const Triangle& t, size_t offset);

    std::ostream *m_stream;
    std::string m_filename;
    Format m_format;
    bool m_faces;
    StringList m_dimNames;
    DimTypeList m_dims;
    int m_precision;
    bool m_sizedTypes;
    Arg *m_precisionArg;
    std::vector<PointViewPtr> m_views;
};

inline std::istream& operator>>(std::istream& in, PlyWriter::Format& f)
{
    std::string s;
    std::getline(in, s);
    Utils::trim(s);
    Utils::tolower(s);
    if (s == "ascii")
        f = PlyWriter::Format::Ascii;
    else if (s == "little endian" || s == "binary_little_endian")
        f = PlyWriter::Format::BinaryLe;
    else if (s == "big endian" || s == "binary_big_endian")
        f = PlyWriter::Format::BinaryBe;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}


inline std::ostream& operator<<(std::ostream& out, const PlyWriter::Format& f)
{
    switch (f)
    {
    case PlyWriter::Format::Ascii:
        out << "ascii";
        break;
    case PlyWriter::Format::BinaryLe:
        out << "binary_little_endian";
        break;
    case PlyWriter::Format::BinaryBe:
        out << "binary_big_endian";
        break;
    }
    return out;
}

}
