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
#include <soci/sqlite3/soci-sqlite3.h>
#include <soci/error.h>
#include <soci/use.h>
#endif

#include <pdal/pdal_error.hpp>
#include <pdal/Options.hpp>

namespace pdal
{
namespace drivers
{
namespace sqlite
{

    class sqlite_driver_error : public pdal_error
    {
    public:
        sqlite_driver_error(std::string const& msg)
            : pdal_error(msg)
        {}
    };

    class connection_failed : public sqlite_driver_error
    {
    public:
        connection_failed(std::string const& msg)
            : sqlite_driver_error(msg)
        {}
    };

    class buffer_too_small : public sqlite_driver_error
    {
    public:
        buffer_too_small(std::string const& msg)
            : sqlite_driver_error(msg)
        {}
    };



    enum QueryType
    {
        QUERY_CLOUD = 0,
        QUERY_BLOCKS_PLUS_CLOUD_VIEW,
        QUERY_UNKNOWN = 512
    };





}
}
} // namespace pdal::driver::soci


#endif
