/******************************************************************************
* Copyright (c) 2014, Howard Butler, hobu.inc@gmail.com
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

#ifndef INCLUDED_PDAL_DRIVERS_HPP
#define INCLUDED_PDAL_DRIVERS_HPP

#include <pdal/pdal_config.hpp>

#include <pdal/drivers/faux/Reader.hpp>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>

#include <pdal/drivers/bpf/BpfReader.hpp>

#include <pdal/drivers/sbet/Reader.hpp>

#ifdef PDAL_HAVE_GREYHOUND
#include <pdal/drivers/greyhound/Reader.hpp>
#endif

#ifdef PDAL_HAVE_HDF5
#include <pdal/drivers/icebridge/Reader.hpp>
#endif

#ifdef PDAL_HAVE_ORACLE
#ifndef USE_PDAL_PLUGIN_OCI
#include <pdal/drivers/oci/OciReader.hpp>
#endif
#endif

#ifdef PDAL_HAVE_CARIS
#ifndef USE_PDAL_PLUGIN_CARIS
#include <pdal/drivers/caris/CloudReader.hpp>
#endif
#endif

#ifdef PDAL_HAVE_MRSID
#ifndef USE_PDAL_PLUGIN_MRSID
#include <pdal/drivers/mrsid/Reader.hpp>
#endif
#endif

#include <pdal/drivers/qfit/Reader.hpp>
#include <pdal/drivers/terrasolid/Reader.hpp>
#include <pdal/drivers/text/Writer.hpp>

#ifdef PDAL_HAVE_ORACLE
#ifndef USE_PDAL_PLUGIN_OCI
#include <pdal/drivers/oci/Writer.hpp>
#endif
#endif

#ifdef PDAL_HAVE_NITRO
#ifndef USE_PDAL_PLUGIN_NITF
#include <pdal/drivers/nitf/Writer.hpp>
#endif
#endif

#ifdef PDAL_HAVE_GDAL
#include <pdal/drivers/nitf/Reader.hpp>
#endif

#ifdef PDAL_HAVE_P2G
#include <pdal/drivers/p2g/P2gWriter.hpp>
#endif

#ifdef PDAL_HAVE_HDF5
#include <pdal/drivers/icebridge/Reader.hpp>
#endif

#ifdef PDAL_HAVE_SQLITE
#ifndef USE_PDAL_PLUGIN_SQLITE
#include <pdal/drivers/sqlite/SQLiteReader.hpp>
#include <pdal/drivers/sqlite/SQLiteWriter.hpp>
#endif
#endif

#ifdef PDAL_HAVE_POSTGRESQL
#ifndef USE_PDAL_PLUGIN_PGPOINTCLOUD
#include <pdal/drivers/pgpointcloud/PgReader.hpp>
#include <pdal/drivers/pgpointcloud/Writer.hpp>
#endif
#endif

#endif
