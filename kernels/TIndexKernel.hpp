/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include <pdal/Stage.hpp>
#include <pdal/SubcommandKernel.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{
    class Polygon;

namespace gdal
{
    class SpatialRef;
}

class StageFactory;

class PDAL_DLL TIndexKernel : public SubcommandKernel
{
    struct FileInfo
    {
        std::string m_filename;
        std::string m_srs;
        std::string m_boundary;
        struct tm m_ctime;
        struct tm m_mtime;
    };

    struct FieldIndexes
    {
        int m_filename;
        int m_srs;
        int m_ctime;
        int m_mtime;
    };

public:
    std::string getName() const;
    TIndexKernel();

private:
    virtual void addSubSwitches(ProgramArgs& args,
        const std::string& subcommand);
    virtual void validateSwitches(ProgramArgs& args);
    virtual int execute();
    virtual StringList subcommands() const;

    void createFile();
    void mergeFile();
    bool openDataset(const std::string& filename);
    bool createDataset(const std::string& filename);
    bool openLayer(const std::string& layerName);
    bool createLayer(const std::string& layerName);
    FieldIndexes getFields();
    bool getFileInfo(StageFactory& factory, const std::string& filename,
        FileInfo& info);
    bool createFeature(const FieldIndexes& indexes, FileInfo& info);
    pdal::Polygon prepareGeometry(const FileInfo& fileInfo);
    void createFields();
    bool fastBoundary(Stage& reader, FileInfo& fileInfo);
    bool slowBoundary(PipelineManager& manager, FileInfo& fileInfo);

    bool isFileIndexed( const FieldIndexes& indexes, const FileInfo& fileInfo);

    std::string m_idxFilename;
    std::string m_filespec;
    StringList m_files;
    std::string m_layerName;
    std::string m_driverName;
    std::string m_tileIndexColumnName;
    std::string m_srsColumnName;
    std::string m_wkt;
    BOX2D m_bounds;
    bool m_absPath;

    void *m_dataset;
    void *m_layer;
    std::string m_tgtSrsString;
    std::string m_assignSrsString;
    bool m_fastBoundary;
    bool m_usestdin;
    bool m_overrideASrs;
};

} // namespace pdal
