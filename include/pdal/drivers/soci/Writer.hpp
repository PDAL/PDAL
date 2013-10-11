/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#ifndef INCLUDED_DRIVERS_SOCI_WRITER_HPP
#define INCLUDED_DRIVERS_SOCI_WRITER_HPP


#include <pdal/Writer.hpp>
#include <pdal/drivers/soci/common.hpp>
#include <pdal/third/nanoflann.hpp>


namespace pdal
{
namespace drivers
{
namespace soci
{


class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.soci.writer", "SOCI Writer")

    Writer(Stage& prevStage, const Options&);
    ~Writer();

    virtual void initialize();
    static Options getDefaultOptions();



protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
    virtual void writeBufferBegin(PointBuffer const&);

    virtual boost::uint32_t writeBuffer(const PointBuffer&);
    virtual void writeEnd(boost::uint64_t actualNumPointsWritten);

private:

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented

    void CreateBlockTable(std::string const& name, boost::uint32_t srid);
    void CreateCloudTable(std::string const& name, boost::uint32_t srid);    
    bool CheckTableExists(std::string const& name);
    void DeleteBlockTable(std::string const& cloud_table_name,
                          std::string const& cloud_column_name, 
                          std::string const& block_table_name);
    void DeleteCloudTable(std::string const& cloud_table_name,
                          std::string const& cloud_column_name);                          
    void CreateIndexes(std::string const& table_name, 
                       std::string const& spatial_column_name, 
                       bool is3d,
                       bool isBlockTable=true);
    void CreateSDOEntry(std::string const& block_table, 
                        boost::uint32_t srid, 
                        pdal::Bounds<double> bounds,
                        bool is3d);
    Schema getPackedSchema( Schema const& schema) const;
    bool IsValidGeometryWKT(std::string const& wkt) const;
    std::string loadGeometryWKT(std::string const& filename_or_wkt) const;
    void CreateCloud(Schema const& buffer_schema);    

    void PackPointData( PointBuffer const& buffer,
                        boost::uint8_t** point_data,
                        boost::uint32_t& point_data_len,
                        boost::uint32_t& schema_byte_size);
    bool WriteBlock(PointBuffer const& buffer);                        
    
#ifdef PDAL_HAVE_SOCI
    ::soci::session* m_session;
	::soci::statement* m_block_statement;
#else
    void* m_session;
#endif

    DatabaseType m_type;
    bool m_doCreateIndex;
    pdal::Bounds<double> m_bounds; // Bounds of the entire point cloud    
    bool m_sdo_pc_is_initialized;
	std::ostringstream m_block_insert_query;
	std::ostringstream m_block_bytes;
	std::string m_block_data;
	std::string m_extent;
	std::string m_bbox;
	boost::int32_t m_obj_id;
	boost::int32_t m_block_id;
	boost::uint32_t m_srid;
	boost::int64_t m_num_points;
};

}
}
} // namespaces

#endif
