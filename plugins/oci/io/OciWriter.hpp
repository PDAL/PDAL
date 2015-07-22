/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/DbWriter.hpp>
#include <pdal/GDALUtils.hpp>

pdal::Writer* createOciWriter();

#include "OciCommon.hpp"

namespace pdal
{

class PDAL_DLL OciWriter : public DbWriter
{
public:
    OciWriter();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    template<typename T>
    T getDefaultedOption(const Options& options,
        const std::string& option_name)
    {
        T default_value =
            getDefaultOptions().getOption(option_name).getValue<T>();
        return options.getValueOrDefault<T>(option_name, default_value);
    }

    virtual void processOptions(const Options& options);
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);
    void writeInit();
    void writeTile(const PointViewPtr view);

    void runCommand(std::ostringstream const& command);
    void wipeBlockTable();
    void createBlockIndex();
    void createBlockTable();
    void createSDOEntry();
    void createPCEntry();
    long getGType();
    std::string createPCElemInfo();
    bool blockTableExists();
    void runFileSQL(std::string const& filename);
    bool isGeographic(int32_t srid);
    std::string loadSQLData(std::string const& filename);
    void setOrdinates(Statement statement, OCIArray* ordinates,
        const BOX3D& extent);
    void setElements(Statement statement, OCIArray* elem_info);
    void updatePCExtent();
    std::string shutOff_SDO_PC_Trigger();
    void turnOn_SDO_PC_Trigger(std::string trigger_name);
    bool isValidWKT(std::string const& wkt);
    void writeDimMajor(PointViewPtr view, std::vector<char>& outbuf);
    void writePointMajor(PointViewPtr view, std::vector<char>& outbuf);

    long m_lastBlockId;
    BOX3D m_bounds; // Bounds of the entire point cloud
    Connection m_connection;
    bool m_createIndex;
    bool m_bDidCreateBlockTable;
    BOX3D m_pcExtent;
    BOX3D m_baseTableBounds;
    int m_pc_id;
    std::string m_blockTableName;
    std::string m_blockTablePartitionColumn;
    int32_t m_blockTablePartitionValue;
    uint32_t m_srid;
    long m_gtype;
    bool m_3d;
    bool m_solid;
    uint32_t m_precision;
    bool m_overwrite;
    bool m_trace;
    bool m_compression;

    std::string m_baseTableName;
    std::string m_cloudColumnName;
    std::string m_baseTableAuxColumns;
    std::string m_baseTableAuxValues;

    std::string m_baseTableBoundaryColumn;
    std::string m_baseTableBoundaryWkt;

    std::string m_triggerName;
    bool m_reenableCloudTrigger;
    bool m_disableCloudTrigger;
    bool m_sdo_pc_is_initialized;
    uint32_t m_chunkCount;
    uint32_t m_capacity;
    bool m_streamChunks;
    Orientation::Enum m_orientation;
    std::string m_connSpec;

    OciWriter& operator=(const OciWriter&); // not implemented
    OciWriter(const OciWriter&); // not implemented

};

} // namespace pdal
