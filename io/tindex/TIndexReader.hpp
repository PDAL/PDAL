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

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <merge/MergeFilter.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/GDALUtils.hpp>

extern "C" int32_t TIndexReader_ExitFunc();
extern "C" PF_ExitFunc TIndexReader_InitPlugin();

namespace pdal
{
class PDAL_DLL TIndexReader : public pdal::Reader
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
    TIndexReader()
        : Reader()
        , m_dataset(0)
        , m_layer(0)
        , m_tablePtr(new PointTable())
        , m_table(*m_tablePtr)
        {}

    ~TIndexReader();
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();
    static Dimension::IdList getDefaultDimensions();

private:

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void processOptions(const Options& options);
    virtual void ready(PointTableRef table);
    virtual PointViewSet run(PointViewPtr /*view*/);

    virtual void initialize();

    std::vector<FileInfo> m_files;
    std::string m_layerName;
    std::string m_driverName;
    std::string m_tileIndexColumnName;
    std::string m_srsColumnName;
    std::string m_wkt;
    std::string m_tgtSrsString;
    std::string m_filterSRS;
    std::string m_attributeFilter;
    std::string m_dialect;
    BOX2D m_boundary;
    std::string m_sql;

    std::unique_ptr<gdal::SpatialRef> m_out_ref;
    void *m_dataset;
    void *m_layer;

    std::unique_ptr<PointTable> m_tablePtr;
    PointTableRef m_table;
    StageFactory m_factory;
    MergeFilter m_merge;

    std::vector<FileInfo> getFiles();
    FieldIndexes getFields();

};


} // namespace pdal
