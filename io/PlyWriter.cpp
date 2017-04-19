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

#include <limits>
#include <sstream>

#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{
namespace
{

void createErrorCallback(p_ply ply, const char* message)
{
    throw PlyWriter::error(message);
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
    args.add("dims", "Dimension names", m_dimNames);
    args.add("faces", "Write faces", m_faces);
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
        throwError("Unknown storage mode '" + m_storageModeSpec +
            "'. Known storage modes are: 'ascii', 'little endian', "
            "'big endian', and 'default'");
    }
}


void PlyWriter::prepared(PointTableRef table)
{
    if (m_dimNames.size())
    {
        for (auto& name : m_dimNames)
        {
            auto id = table.layout()->findDim(name);
            if (id == Dimension::Id::Unknown)
                throwError("Unknown dimension '" + name + "' in provided "
                    "dimension list.");
            m_dims.push_back(id);
        }
    }
    else
    {
        m_dims = table.layout()->dims();
        for (auto dim : m_dims)
            m_dimNames.push_back(Utils::tolower(table.layout()->dimName(dim)));
    }
}


void PlyWriter::ready(PointTableRef table)
{
    try
    {
        m_ply = ply_create(m_filename.c_str(), m_storageMode,
            createErrorCallback, 0, nullptr);
        if (!m_ply)
            throwError("Could not open file '" + m_filename + "for writing.");
    }
    catch(const error& err)
    {
        throwError(err.what());
    }
    m_pointCollector.reset(new PointView(table));
}


void PlyWriter::write(const PointViewPtr data)
{
    if (m_faces && m_pointCollector->size())
        throwError("Can't output faces for pipeline with multiple "
            "point views.");
    m_pointCollector->append(*data);
    if (m_faces)
    {
        m_mesh = data->mesh();
        if (!m_mesh)
            throwError("Can't write mesh faces.  No mesh has been generated.");
    }
    if (data->size() > std::numeric_limits<uint32_t>::max())
        throwError("Can't write PLY file.  Only " +
            std::to_string(std::numeric_limits<uint32_t>::max()) +
            " points supported.");
}


void PlyWriter::done(PointTableRef table)
{
    try
    {
    if (!ply_add_element(m_ply, "vertex", m_pointCollector->size()))
        throwError("Could not add vertex element");

    auto ni = m_dimNames.begin();
    for (auto dim : m_dims) {
        std::string name = *ni++;
        e_ply_type plyType = getPlyType(table.layout()->dimType(dim));
        if (!ply_add_scalar_property(m_ply, name.c_str(), plyType))
            throwError("Could not add scalar property '" + name  + "'");
    }
    if (m_faces)
    {
        ply_add_element(m_ply, "face", m_mesh->size());
        ply_add_list_property(m_ply, "vertex_indices", PLY_UINT8, PLY_UINT32);
    }

    if (!ply_add_comment(m_ply, "Generated by PDAL"))
        throwError("Could not add comment");

    if (!ply_write_header(m_ply))
        throwError("Could not write ply header");

    for (PointId index = 0; index < m_pointCollector->size(); ++index)
    {
        auto ni = m_dimNames.begin();
        auto di = m_dims.begin();
        for (; di != m_dims.end(); ++di, ++ni)
        {
            double value = m_pointCollector->getFieldAs<double>(*di, index);
            if (!ply_write(m_ply, value))
                throwError("Error writing dimension '" + *ni + "' of point "
                    "number " + Utils::toString(index) + ".");
        }
    }
    if (m_faces)
    {
        for (PointId id = 0; id < m_mesh->size(); id++)
        {
            const Triangle& t = (*m_mesh)[id];
            if (!(ply_write(m_ply, 3) &&
                    ply_write(m_ply, t.m_a) &&
                    ply_write(m_ply, t.m_b) &&
                    ply_write(m_ply, t.m_c)))
                throwError("Error writing face number " +
                    Utils::toString(index) + ".");
        }
    }

    if (!ply_close(m_ply))
        throwError("Error closing file");
    }
    catch (const error& err)
    {
        throwError(err.what());
    }

    getMetadata().addList("filename", m_filename);
}

}
