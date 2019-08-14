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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wnull-dereference"
#pragma GCC diagnostic ignored "-Wcast-qual"
//ABELL - The issue with the diag below is that the FBX SDK sets packing
//  around a ton of include files which ends up setting packing for some
//  system includes.  Bad.  I've filed an issue with both Apple (which can
//  improve its header file to mitigate) and Autodesk (which can avoid doing
//  something so stupid).
// This went away for some reason, so we can't ignore it.
//#pragma GCC diagnostic ignored "-Wpragma-pack"
#include <fbxsdk.h>
#pragma GCC diagnostic pop

#include "FbxWriter.hpp"

using namespace fbxsdk;

namespace pdal
{

static PluginInfo const s_info
{
    "writers.fbx",
    "fbx writer",
    "https://pdal.io/stages/writers.fbx.html"
};

CREATE_SHARED_STAGE(FbxWriter, s_info)

std::string FbxWriter::getName() const { return s_info.name; }

FbxWriter::FbxWriter()
{}


void FbxWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("ascii", "Write FBX as ASCII", m_ascii, false);
}


void FbxWriter::ready(PointTableRef table)
{
    m_manager = FbxManager::Create();
    FbxIOSettings *ios = FbxIOSettings::Create(m_manager, IOSROOT);
    m_manager->SetIOSettings(ios);
    m_scene = FbxScene::Create(m_manager, "Scene");
}


void FbxWriter::write(const PointViewPtr v)
{
    TriangularMesh *mesh = v->mesh();
    if (!mesh)
        throwError("Can't write FBX file without generated mesh.");

    if (v->size() == 0)
        log()->get(LogLevel::Error) << "Writing FBX file with no vertices." <<
            std::endl;

    FbxMesh *fbxMesh = FbxMesh::Create(m_scene, "mesh");
    fbxMesh->InitControlPoints((int)v->size());
    FbxVector4 *points = fbxMesh->GetControlPoints();
    for (size_t i = 0; i < v->size(); ++i)
    for (size_t i = 0; i < v->size(); ++i)
    {
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        double y = v->getFieldAs<double>(Dimension::Id::Y, i);
        double z = v->getFieldAs<double>(Dimension::Id::Z, i);
        points[i] = FbxVector4(x, y, z);
    }

    for (size_t id = 0; id < mesh->size(); ++id)
    {
        const Triangle& t = (*mesh)[id];
        fbxMesh->BeginPolygon();
        fbxMesh->AddPolygon((int)t.m_a);
        fbxMesh->AddPolygon((int)t.m_b);
        fbxMesh->AddPolygon((int)t.m_c);
        fbxMesh->EndPolygon();
    }
}


void FbxWriter::done(PointTableRef table)
{
    FbxExporter *exporter = FbxExporter::Create(m_manager, "");
    FbxIOSettings *settings = m_manager->GetIOSettings();
    FbxIOPluginRegistry *registry = m_manager->GetIOPluginRegistry();

    // "FBX binary (*.fbx)"
    // "FBX ascii (*.fbx)"
    // "FBX encrypted (*.fbx)"
    // ...
    const char *format = m_ascii ? "FBX ascii (*.fbx)" : "FBX binary (*.fbx)";
    int writer = registry->FindWriterIDByDescription(format);

    // This is just to let you know what writers are supported and their
    // magic number strings.  Uncomment if you need and then comment again.
    /**
    int numWriters = registry->GetWriterFormatCount();
    for (int i = 0; i < numWriters; ++i)
        std::cerr << registry->GetWriterFormatDescription(i) << "\n";
    **/

    exporter->Initialize(m_filename.data(), writer, settings);
    exporter->Export(m_scene);

    exporter->Destroy();
    m_manager->Destroy();
    m_manager = nullptr;
    m_scene = nullptr;
}

} // namespace pdal

