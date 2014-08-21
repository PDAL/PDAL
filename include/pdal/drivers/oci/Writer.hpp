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

#include <pdal/Writer.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/GDALUtils.hpp>

pdal::Writer* createOciWriter(const pdal::Options& options);

#ifdef USE_PDAL_PLUGIN_OCI
PDAL_C_START

PDAL_DLL void PDALRegister_writer_oci(void* factory);

PDAL_C_END
#endif

#include "common.hpp"

namespace pdal
{
namespace drivers
{
namespace oci
{

class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.oci.writer", "OCI Writer")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.oci.writer.html")
#ifdef PDAL_HAVE_ORACLE
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif    
    Writer(const Options&);
    ~Writer();

    static Options getDefaultOptions();

protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite)
    {}
    virtual void writeBufferBegin(PointBuffer const&)
    {}
    
    virtual boost::uint32_t writeBuffer(const PointBuffer&)
    { return 0; }

    virtual void writeEnd(boost::uint64_t actualNumPointsWritten)
    {}

private:
    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented

    template<typename T>
    T getDefaultedOption(const Options& options,
        const std::string& option_name) const
    {
        T default_value =
            getDefaultOptions().getOption(option_name).getValue<T>();
        return options.getValueOrDefault<T>(option_name, default_value);
    }

    virtual void processOptions(const Options& options);
    virtual void initialize();
    virtual void ready(PointContext ctx);
    virtual void write(const PointBuffer& buffer);
    virtual void done(PointContext ctx);
    void writeInit();
    void writeTile(const PointBuffer& buffer);

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
    bool isGeographic(boost::int32_t srid);
    std::string loadSQLData(std::string const& filename);
    void setOrdinates(Statement statement, OCIArray* ordinates,
        pdal::Bounds<double> const& extent);
    void setElements(Statement statement, OCIArray* elem_info);
    void updatePCExtent();
    std::string shutOff_SDO_PC_Trigger();
    void turnOn_SDO_PC_Trigger(std::string trigger_name);
    bool isValidWKT(std::string const& wkt);

    size_t m_pointSize;
    long m_lastBlockId;
    pdal::Bounds<double> m_bounds; // Bounds of the entire point cloud
    Connection m_connection;
    bool m_createIndex;
    bool m_bDidCreateBlockTable;
    Bounds<double> m_pcExtent;
    Bounds<double> m_baseTableBounds;
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
    bool m_pack;
    Dimension::IdList m_dims;
    std::vector<Dimension::Type::Enum> m_types;

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
};

} // namespace oci
} // namespace drivers
} // namespace pdal

