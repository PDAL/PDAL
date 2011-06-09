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

#include <libpc/libpc.hpp>
#include <libpc/exceptions.hpp>
#include <libpc/Options.hpp>

#include "oci_wrapper.h"
#include <libpc/Endian.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

#include <cpl_port.h>



void CPL_STDCALL OCIGDALErrorHandler(CPLErr eErrClass, int err_no, const char *msg);
void CPL_STDCALL OCIGDALDebugErrorHandler(CPLErr eErrClass, int err_no, const char *msg);


namespace libpc { namespace drivers { namespace oci {

typedef boost::shared_ptr<OWConnection> Connection ;
typedef boost::shared_ptr<OWStatement> Statement ;


extern Options LIBPC_DLL GetDefaultOptions();

class connection_failed : public libpc_error
{
public:
    connection_failed(std::string const& msg)
        : libpc_error(msg)
    {}
};

class buffer_too_small : public libpc_error
{
public:
    buffer_too_small(std::string const& msg)
        : libpc_error(msg)
    {}
};



struct FiveDimensionOCI
{
    double x;
    double y;
    double z;
    double t;
    double c;
    boost::uint32_t blk_id;
    boost::uint32_t pc_id;
};

struct EightDimensionOCI
{
    double x;
    double y;
    double z;
    double time;
    double cls;
    double intensity;
    boost::int8_t returnNumber;
    boost::int8_t numberOfReturns;
    boost::int8_t scanDirFlag;
    boost::int8_t edgeOfFlightLine;
    boost::int8_t scanAngleRank;
    boost::int8_t userData;
    boost::uint16_t pointSourceId;
    boost::uint16_t red;
    boost::uint16_t green;
    boost::uint16_t blue;
    boost::uint16_t alpha;
    boost::uint32_t blk_id;
    boost::uint32_t pc_id;
};


enum QueryType
{
    QUERY_SDO_PC,
    QUERY_SDO_PC_BLK_TYPE,
    QUERY_UNKNOWN
};


class Cloud
{
public:
    Cloud(Connection connection);
    ~Cloud();
    
    std::string base_table;
    std::string base_column;
    boost::int32_t pc_id;
    std::string blk_table;
    sdo_geometry* pc_geometry;
    Connection              connection;
        
};
typedef boost::shared_ptr<Cloud> CloudPtr;

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


private:
    boost::uint32_t m_capacity;
};

typedef boost::shared_ptr<Block> BlockPtr;


LIBPC_DLL Connection Connect(libpc::Options const& options);

std::string to_upper(std::string const& input);




}}} // namespace libpc::driver::oci


#endif
