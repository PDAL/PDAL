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

#ifndef INCLUDED_DRIVER_SOCI_COMMON_HPP
#define INCLUDED_DRIVER_SOCI_COMMON_HPP

#ifdef PDAL_HAVE_SOCI
#include <boost-optional.h>
#include <boost-tuple.h>
#include <boost-fusion.h>
#include <boost-gregorian-date.h>
#include <soci/soci.h>
#include <soci/postgresql/soci-postgresql.h>
#endif

#include <pdal/pdal_error.hpp>
#include <pdal/Options.hpp>

namespace pdal
{
namespace drivers
{
namespace soci
{

    class soci_driver_error : public pdal_error
    {
    public:
        soci_driver_error(std::string const& msg)
            : pdal_error(msg)
        {}
    };

    class connection_failed : public soci_driver_error
    {
    public:
        connection_failed(std::string const& msg)
            : soci_driver_error(msg)
        {}
    };

    class buffer_too_small : public soci_driver_error
    {
    public:
        buffer_too_small(std::string const& msg)
            : soci_driver_error(msg)
        {}
    };


    enum DatabaseType
    {
        DATABASE_POSTGRESQL,
        DATABASE_ORACLE,
        DATABASE_UNKNOWN = 128
    };

    enum QueryType
    {
        QUERY_CLOUD = 0,
        QUERY_PC = 128,
        QUERY_BLOCKS_PLUS_CLOUD_VIEW,
        QUERY_UNKNOWN = 512
    };


inline DatabaseType getDatabaseConnectionType(std::string const& connection_type)
{
    DatabaseType output;

    if (boost::iequals(connection_type, "oracle"))
        output = DATABASE_ORACLE;
    else if (boost::iequals(connection_type, "postgresql"))
        output = DATABASE_POSTGRESQL;
    else
        output = DATABASE_UNKNOWN;
    
    return output;
    
}

inline ::soci::session* connectToDataBase(std::string const& connection, DatabaseType dtype)
{
    ::soci::session* output(0);
    if (dtype == DATABASE_UNKNOWN)
    {
        std::stringstream oss;
        oss << "Database connection type '" << dtype << "' is unknown or not configured";
        throw soci_driver_error(oss.str());
    }
    
    try
    {
        if (dtype == DATABASE_POSTGRESQL)
            output = new ::soci::session(::soci::postgresql, connection);

    } catch (::soci::soci_error const& e)
    {
        std::stringstream oss;
        oss << "Unable to connect to database with error '" << e.what() << "'";
        throw connection_failed(oss.str());
    }
    
    return output;
}

}
}
} // namespace pdal::driver::soci


#endif
