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

#include "PlyWriter.hpp"

#include <sstream>

#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{
namespace
{

void createErrorCallback(p_ply ply, const char* message)
{
    std::stringstream ss;
    ss << "Error when creating ply file: " << message;
    throw pdal_error(ss.str());
}


e_ply_type getPlyType(Dimension::Type type)
{
    static std::map<Dimension::Type, e_ply_type> types =
    {
        { Dimension::Type::Unsigned8, PLY_UINT8 },
        { Dimension::Type::Signed8, PLY_INT8 },
        { Dimension::Type::Unsigned16, PLY_UINT16 },
        { Dimension::Type::Signed16, PLY_INT16 },
        { Dimension::Type::Unsigned32, PLY_UINT32 },
        { Dimension::Type::Signed32, PLY_INT32 },
        { Dimension::Type::Float, PLY_FLOAT32 },
        { Dimension::Type::Double, PLY_FLOAT64 }
    };

    return types[type];
}

} // unnamed namespace


static PluginInfo const s_info = PluginInfo(
        "writers.ply",
        "ply writer",
        "http://pdal.io/stages/writers.ply.html"
        );

CREATE_STATIC_PLUGIN(1, 0, PlyWriter, Writer, s_info)

std::string PlyWriter::getName() const { return s_info.name; }


PlyWriter::PlyWriter()
    : m_ply(nullptr)
    , m_pointCollector(nullptr)
    , m_storageMode(PLY_DEFAULT)
{}


void PlyWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("storage_mode", "PLY Storage mode", m_storageModeSpec, "default");
}


void PlyWriter::initialize()
{
    std::string storageMode(m_storageModeSpec);
    storageMode = Utils::tolower(storageMode);

    if (storageMode == "ascii")
    {
        m_storageMode = PLY_ASCII;
    }
    else if (storageMode == "little endian")
    {
        m_storageMode = PLY_LITTLE_ENDIAN;
    }
    else if (storageMode == "big endian")
    {
        m_storageMode = PLY_BIG_ENDIAN;
    }
    else if (storageMode == "default")
    {
        m_storageMode = PLY_DEFAULT;
    }
    else
    {
        std::stringstream ss;
        ss << "Unknown storage mode '" << m_storageModeSpec <<
            "'. Known storage modes are: 'ascii', 'little endian', "
            "'big endian', and 'default'";
        throw pdal_error(ss.str());
    }
}


void PlyWriter::ready(PointTableRef table)
{
    m_ply = ply_create(m_filename.c_str(), m_storageMode, createErrorCallback,
        0, nullptr);
    if (!m_ply)
    {
        std::stringstream ss;
        ss << "Could not open file for writing: " << m_filename;
        throw pdal_error(ss.str());
    }
    m_pointCollector.reset(new PointView(table));
}


void PlyWriter::write(const PointViewPtr data)
{
    m_pointCollector->append(*data);
}


void PlyWriter::done(PointTableRef table)
{
    if (!ply_add_element(m_ply, "vertex", m_pointCollector->size()))
    {
        std::stringstream ss;
        ss << "Could not add vertex element";
        throw pdal_error(ss.str());
    }
    auto dimensions = table.layout()->dims();
    for (auto dim : dimensions) {
        std::string name = Dimension::name(dim);
        e_ply_type plyType = getPlyType(Dimension::defaultType(dim));
        if (!ply_add_scalar_property(m_ply, name.c_str(), plyType))
        {
            std::stringstream ss;
            ss << "Could not add scalar property '" << name << "'";
            throw pdal_error(ss.str());
        }
    }
    if (!ply_add_comment(m_ply, "Generated by PDAL"))
    {
        std::stringstream ss;
        ss << "Could not add comment";
        throw pdal_error(ss.str());
    }
    if (!ply_write_header(m_ply))
    {
        std::stringstream ss;
        ss << "Could not write ply header";
        throw pdal_error(ss.str());
    }

    for (PointId index = 0; index < m_pointCollector->size(); ++index)
    {
        for (auto dim : dimensions)
        {
            double value = m_pointCollector->getFieldAs<double>(dim, index);
            if (!ply_write(m_ply, value))
            {
                std::stringstream ss;
                ss << "Error writing dimension '" << Dimension::name(dim) <<
                    "' of point number " << index;
                throw pdal_error(ss.str());
            }
        }
    }

    if (!ply_close(m_ply))
        throw pdal_error("Error closing ply file");

    getMetadata().addList("filename", m_filename);
}

}
