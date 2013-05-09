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

#ifndef INCLUDED_DRIVER_PGPOINTCLOUD_COMMON_HPP
#define INCLUDED_DRIVER_PGPOINTCLOUD_COMMON_HPP

#ifdef PDAL_HAVE_SOCI
#include <boost-optional.h>
#include <boost-tuple.h>
#include <boost-fusion.h>
#include <boost-gregorian-date.h>
#include <soci/soci.h>
#include <soci/postgresql/soci-postgresql.h>
#include <soci/error.h>
#include <soci/use.h>
#endif

#include <pdal/pdal_error.hpp>
#include <pdal/Options.hpp>

namespace pdal
{
namespace drivers
{
namespace pgpointcloud
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


enum CompressionType
{
    COMPRESSION_NONE = 0,
    COMPRESSION_GHT = 1,
    COMPRESSION_DIMENSIONAL = 2
};


inline CompressionType getCompressionType(std::string const& compression_type)
{
    CompressionType output;

    if (boost::iequals(compression_type, "dimensional"))
        output = COMPRESSION_DIMENSIONAL;
    else if (boost::iequals(compression_type, "ght"))
        output = COMPRESSION_GHT;
    else
        output = COMPRESSION_NONE;
    
    return output;
    
}


}
}
} // namespace pdal::driver::pgpointcloud


#endif
