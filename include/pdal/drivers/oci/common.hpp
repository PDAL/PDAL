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

#ifndef INCLUDED_DRIVER_OCI_COMMON_HPP
#define INCLUDED_DRIVER_OCI_COMMON_HPP

#include <pdal/Options.hpp>

#include "oci_wrapper.h"
#include <pdal/Endian.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/function.hpp>
#include <boost/concept_check.hpp> // ignore_unused_variable_warning
#include <boost/make_shared.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

#include <cpl_port.h>

//
//
// void CPL_STDCALL OCIGDALErrorHandler(CPLErr eErrClass, int err_no, const char *msg);
// void CPL_STDCALL OCIGDALDebugErrorHandler(CPLErr eErrClass, int err_no, const char *msg);


namespace pdal
{
namespace drivers
{
namespace oci
{

typedef boost::shared_ptr<OWConnection> Connection ;
typedef boost::shared_ptr<OWStatement> Statement ;

class connection_failed : public pdal_error
{
public:
    connection_failed(std::string const& msg)
        : pdal_error(msg)
    {}
};

class buffer_too_small : public pdal_error
{
public:
    buffer_too_small(std::string const& msg)
        : pdal_error(msg)
    {}
};


class read_error : public pdal_error
{
public:
    read_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


class OracleDriver
{
public:
    OracleDriver(const Options& options)
        : m_options(options)
    {


    }

    pdal::drivers::oci::Connection connect()
    {
        std::string connection  = m_options.getValueOrThrow<std::string>("connection");

        if (connection.empty())
            throw pdal_error("Oracle connection string empty! Unable to connect");

        std::string::size_type slash_pos = connection.find("/",0);
        std::string username = connection.substr(0,slash_pos);
        std::string::size_type at_pos = connection.find("@",slash_pos);

        std::string password = connection.substr(slash_pos+1, at_pos-slash_pos-1);
        std::string instance = connection.substr(at_pos+1);

        Connection con = boost::make_shared<OWConnection>(username.c_str(),password.c_str(),instance.c_str());

        if (!con->Succeeded())
        {
            throw connection_failed("Oracle connection failed");
        }

        return con;

    }

private:
    Options const& m_options;

    OracleDriver& operator=(const OracleDriver&); // not implemented
    OracleDriver(const OracleDriver&); // not implemented
};





enum QueryType
{
    QUERY_SDO_PC = 0,
    QUERY_SDO_PC_BLK_TYPE,
    QUERY_SDO_BLK_PC_VIEW,
    QUERY_UNKNOWN = 512
};


class Block
{

public:

    Block(Connection connection);
    ~Block() ;

    boost::int32_t           obj_id;
    boost::int32_t           blk_id;
    sdo_geometry*   blk_extent;
    sdo_orgscl_type* blk_domain;

    double           pcblk_min_res;
    double           pcblk_max_res;
    boost::int32_t           num_points;
    boost::int32_t           num_unsorted_points;
    boost::int32_t           pt_sort_dim;
    boost::scoped_ptr<std::vector<uint8_t> > chunk;
    OCILobLocator           *locator;
    Connection              m_connection;
    sdo_pc*                 pc;


};

typedef boost::shared_ptr<Block> BlockPtr;




}
}
} // namespace pdal::driver::oci


#endif
