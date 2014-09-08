/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
* Copyright (c) 2013, Paul Ramsey, pramsey@cleverelephant.ca
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

#include <boost/algorithm/string.hpp>

#include "libpq-fe.h"
#include <pdal/pdal_error.hpp>
#include <pdal/Options.hpp>

namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{

namespace CompressionType
{

enum Enum
{
    None = 0,
    Ght = 1,
    Dimensional = 2,
    Unknown = 256
};

} // namespace CompressionType


inline CompressionType::Enum getCompressionType(
    std::string const& compression_type)
{
    if (boost::iequals(compression_type, "dimensional"))
        return CompressionType::Dimensional;
    else if (boost::iequals(compression_type, "ght"))
        return CompressionType::Ght;
    return CompressionType::None;
}

inline PGconn* pg_connect(std::string const& connection)
{
    PGconn* conn;
    if ( ! connection.size() )
    {
        throw pdal_error("unable to connect to database, no connection string was given!");
    }

    /* Validate the connection string and get verbose error (?) */
#ifdef PQconninfoParse
    char *errstr;
    PQconninfoOption *connOptions = PQconninfoParse(connection.c_str(), &errstr);
    if ( ! connOptions )
    {
        throw pdal_error(errstr);      
    }
#endif
   
    /* connect to database */
    conn = PQconnectdb(connection.c_str());
    if ( (!conn) || (PQstatus(conn) != CONNECTION_OK) )
    {
        throw pdal_error("unable to connect to database");        
    }
    
    return conn;
}

inline void pg_execute(PGconn* session, std::string const& sql)
{
    PGresult *result = PQexec(session, sql.c_str());
    if ( (!result) || (PQresultStatus(result) != PGRES_COMMAND_OK) )
    {
        std::string errmsg = std::string(PQerrorMessage(session));
        throw pdal_error(errmsg);
    }
    PQclear(result);
}

inline void pg_begin(PGconn* session)
{
    std::string sql = "BEGIN";
    pg_execute(session, sql);
}

inline void pg_commit(PGconn* session)
{
    std::string sql = "COMMIT";
    pg_execute(session, sql);
}

inline char* pg_query_once(PGconn* session, std::string const& sql)
{
    PGresult *result = PQexec(session, sql.c_str());
    
    if ( (!result) ||
         PQresultStatus(result) != PGRES_TUPLES_OK || 
         PQntuples(result) == 0 )
    {
        PQclear(result);
        return NULL;
    }

    char *str = strdup(PQgetvalue(result, 0, 0));
    PQclear(result);
    return str;
}

inline PGresult* pg_query_result(PGconn* session, std::string const& sql)
{
    std::string errmsg;
    PGresult *result = PQexec(session, sql.c_str());
    if ( ! result )
    {
        errmsg = std::string(PQerrorMessage(session));
        throw pdal_error(errmsg);
    }
        
    if ( PQresultStatus(result) != PGRES_TUPLES_OK )
    {
        errmsg = std::string(PQresultErrorMessage(result));
        PQclear(result);
        throw pdal_error(errmsg);
    }

    return result;
}



} // pgpointcloud
} // drivers
} // pdal

