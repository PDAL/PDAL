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

#ifdef PDAL_HAVE_SOCI
#include <boost-optional.h>
#include <boost-tuple.h>
#include <boost-fusion.h>
#include <boost-gregorian-date.h>
#include <soci/soci.h>
#include <soci/postgresql/soci-postgresql.h>
#endif

pdal::Writer* createSociWriter(pdal::Stage& prevStage, const pdal::Options& options);


namespace pdal
{
namespace drivers
{
namespace soci
{

#ifdef USE_PDAL_PLUGIN_SOCI
PDAL_C_START

PDAL_DLL void PDALRegister_writer_soci(void* factory);

PDAL_C_END
#endif

enum Database_Type
{
    Database_Postgresql,
    Database_Oracle,
    Database_Unknown = 128
};

// class PDAL_DLL Table 
// {
// public:
// 
// 
//     Table(  std::string const& name, 
//             ::soci::session* session);
//     
//     inline std::string const getName() const { return m_name; }
//     
//     virtual void create() = 0;
//     virtual void destroy() = 0;
//     virtual bool exists() = 0;
// 
// private:
//     std::string m_name;
//     ::soci::session* m_session;
// };
// 
// class PDAL_DLL Postgresql : public Table
// {
// public:
//     Postgresql( std::string const& name, 
//                 ::soci::session* session);   
//     
//     virtual void create() {};
//     virtual void destroy() {};
//     virtual bool exists() { return true; }
// };

class soci_driver_error : public pdal_error
{
public:
    soci_driver_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.soci.writer", "Database Writer")

    Writer(Stage& prevStage, const Options&);
    ~Writer();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;



protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
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
    bool IsValidWKT(std::string const& wkt) const;
    std::string loadWKT(std::string const& filename_or_wkt) const;
    
#ifdef PDAL_HAVE_SOCI
    ::soci::session* m_session;
#else
    void* m_session;
#endif

    Database_Type m_type;
    bool m_doCreateIndex;
    pdal::Bounds<double> m_bounds; // Bounds of the entire point cloud    
};

}
}
} // namespaces

#endif
