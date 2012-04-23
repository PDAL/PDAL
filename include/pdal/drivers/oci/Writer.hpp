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

#ifndef INCLUDED_PDAL_DRIVER_OCI_WRITER_HPP
#define INCLUDED_PDAL_DRIVER_OCI_WRITER_HPP

#include <pdal/Writer.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/GDALUtils.hpp>



#include "common.hpp"

namespace pdal
{
namespace drivers
{
namespace oci
{

class PDAL_DLL Writer : public pdal::Writer, pdal::drivers::oci::OracleDriver
{
public:
    SET_STAGE_NAME("drivers.oci.writer", "OCI Writer")

    Writer(Stage& prevStage, const Options&);
    ~Writer();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    void run(std::ostringstream const& command);
    inline void setBounds(pdal::Bounds<double> bounds)
    {
        m_bounds = bounds;
    }
    inline pdal::Bounds<double>  getBounds() const
    {
        return m_bounds;
    }

    inline Connection getConnection() const
    {
        return m_connection;
    }

    // for dumping
    virtual boost::property_tree::ptree toPTree() const;

protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
    virtual boost::uint32_t writeBuffer(const PointBuffer&);
    virtual void writeEnd(boost::uint64_t actualNumPointsWritten);

private:


    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented

    void WipeBlockTable();
    void CreateBlockIndex();
    void CreateBlockTable();
    void CreateSDOEntry();
    void CreatePCEntry();
    long GetGType();
    std::string CreatePCElemInfo();
    bool BlockTableExists();
    void RunFileSQL(std::string const& filename);
    bool IsGeographic(boost::int32_t srid);
    std::string LoadSQLData(std::string const& filename);

    bool FillOraclePointBuffer(PointBuffer const& buffer,
                               std::vector<boost::uint8_t>& point_data);
    bool WriteBlock(PointBuffer const& buffer);

    void SetOrdinates(Statement statement,
                      OCIArray* ordinates,
                      pdal::Bounds<double> const& extent);
    void SetElements(Statement statement,
                     OCIArray* elem_info);

    template<typename T> T getDefaultedOption(std::string const& option_name) const
    {
        T default_value = getDefaultOptions().getOption(option_name).getValue<T>();
        return getOptions().getValueOrDefault<T>(option_name, default_value);
    }



    bool is3d() const;
    bool isSolid() const;
    boost::int32_t getPCID() const;
    void UpdatePCExtent();
    std::string ShutOff_SDO_PC_Trigger();
    void TurnOn_SDO_PC_Trigger(std::string trigger_name);
    pdal::Bounds<double> CalculateBounds(PointBuffer const& buffer);
    bool IsValidWKT(std::string const& wkt);

    pdal::Bounds<double> m_bounds; // Bounds of the entire point cloud
    Connection m_connection;
    bool m_doCreateIndex;
    Bounds<double> m_pcExtent;
    int m_pc_id;
    std::string m_block_table_name;
    std::string m_block_table_partition_column;
    boost::int32_t m_block_table_partition_value;
    boost::uint32_t m_srid;
    long m_gtype;
    bool m_is3d;
    bool m_issolid;

    std::string m_base_table_name;
    std::string m_cloud_column_name;
    std::string m_base_table_aux_columns;
    std::string m_base_table_aux_values;

    std::string m_base_table_boundary_column;
    std::string m_base_table_boundary_wkt;
    boost::shared_ptr<pdal::gdal::Debug> m_gdal_debug;
    std::string m_trigger_name;

};

}
}
} // namespace pdal::driver::oci


#endif // INCLUDED_OCIWRITER_HPP
