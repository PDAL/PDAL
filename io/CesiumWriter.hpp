/******************************************************************************
* Copyright (c) 2018, Hobu Inc., info@hobu.co
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

#pragma once

#include <pdal/Writer.hpp>

namespace pdal
{
class OLeStream;

typedef std::shared_ptr<std::ostream> FileStreamPtr;

class PDAL_DLL CesiumWriter : public Writer
{
    class Tile;
public:
    CesiumWriter()
    {}
    CesiumWriter(const CesiumWriter&) = delete;
    CesiumWriter& operator=(const CesiumWriter&) = delete;

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table);
    virtual void write(const PointViewPtr v);
    virtual void done(PointTableRef table);

    std::string binFilename(int tileNum) const;
    std::string b3dmFilename(int tileNum) const;
    void writeTilesetFile();
    void writeB3dmFile(const Tile& tile);
    void writeB3dmHeader(OLeStream& out, const Tile& tile);
    void writeGltfHeader(OLeStream& out);
    void writeGltfJsonHeader(OLeStream& out);

    size_t m_tileNum = 0;
    std::string m_filename;
    std::vector<Tile> m_tiles;
};

} // namespace pdal
