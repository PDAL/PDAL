/******************************************************************************
* Copyright (c) 2018, Hobu Inc. <info@hobu.co>
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

/**
#include "FbxWriter.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
        "writers.fbx",
        "fbx writer",
        "http://pdal.io/stages/writers.fbx.html"
        );

CREATE_STATIC_PLUGIN(1, 0, FbxWriter, Writer, s_info)

std::string FbxWriter::getName() const { return s_info.name; }

void FbxWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("faces", "Write faces", m_faces);
}


void FbxWriter::prepared(PointTableRef table)
{
}


std::string PlyWriter::getType(Dimension::Type type) const
{
   static std::map<Dimension::Type, std::string> types =
    {
        { Dimension::Type::Signed8, "int8" },
        { Dimension::Type::Unsigned8, "uint8" },
        { Dimension::Type::Signed16, "int16" },
        { Dimension::Type::Unsigned16, "uint16" },
        { Dimension::Type::Signed32, "int32" },
        { Dimension::Type::Unsigned32, "uint32" },
        { Dimension::Type::Float, "float32" },
        { Dimension::Type::Double, "float64" }
    };

    try
    {
        return types.at(type);
    }
    catch (std::out_of_range)
    {
        throwError("Can't write dimension of type '" +
                Dimension::interpretationName(type) + "'.");
    }
    return "";
}


void FbxWriter::ready(PointTableRef table)
{
    m_manager = FbxManager::Create();
    FbxIOSettings *ios = FbxIOSettings::Create(m_manager, IOSROOT);
    m_manager->SetIOSettings(ios);

    m_scene = FbxScene::Create(m_manager, "Scene");
}


void FbxWriter::write(const PointViewPtr data)
{
    TriangularMesh *mesh = v->mesh();
    if (!mesh)
        throwError("Can't write FBX file without generated mesh.");

    FbxMesh *fbxMesh = FbxMesh::Create(m_scene, "mesh");

    fbxMesh->InitControlPoints(data->size());
    FbxVector4 *points = fbxMesh->GetControlPoints();
    for (size_t i = 0; i < data->size(); ++i)
    {
        double x = data->getFieldAs<double>(Dimension::Id::X, i);
        double y = data->getFieldAs<double>(Dimension::Id::Y, i);
        double z = data->getFieldAs<double>(Dimension::Id::Z, i);
        points[i] = FbxVector4(x, y, z);
    }

    for (size_t id = 0; id < mesh->size(); ++id)
    {
        const Triangle& t = (*mesh)[id];
        fbxMesh->BeginPolygon();
        fbxMesh->AddPolygon(t.m_a);
        fbxMesh->AddPolygon(t.m_b);
        fbxMesh->AddPolygon(t.m_c);
        fbxMesh->EndPolygon();
    }
    FbxNode *node = FbxNode::Create(m_scene, mesh);
    node->SetNodeAttirbute(mesh);
    m_scene->GetRootNode()->AddChild(node);
}


void PlyWriter::done(PointTableRef table)
{
    FbxExpoerter *exporter = FbxExporter::Create(m_manager, "");
    exporter->Initialize(m_filename.data(), -1, m_manager->GetIOSettings());
    status = exporter->Export(m_scene);

    m_exporter->Destroy();
    m_manager->Destroy();
    m_manager = nullptr;
    m_exporter = nullptr;
    m_scene = nullptr;
}

} // namespace pdal

**/
