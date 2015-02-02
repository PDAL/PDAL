/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/Reader.hpp>

#ifdef PDAL_HAVE_GEOS
#include <geos_c.h>
#endif

#include "jace/proxy/mil/nga/giat/geowave/store/CloseableIterator.h"
using jace::proxy::mil::nga::giat::geowave::store::CloseableIterator;

namespace pdal
{

class PDAL_DLL GeoWaveReader : public Reader
{
public:
    SET_STAGE_NAME("readers.geowave", "Geowave Reader")
	SET_STAGE_LINK("http://pdal.io/stages/drivers.geowave.reader.html")

private:

    virtual void processOptions(const Options& ops);
    virtual void addDimensions(PointContext ctx);
	virtual void ready(PointContext ctx);
	virtual point_count_t read(PointBuffer& buf, point_count_t count);
	virtual void done(PointContextRef ctx);

	int createJvm();
	int createCloseableIterator();

	std::string m_zookeeperUrl;
	std::string m_instanceName;
	std::string m_username;
	std::string m_password;
	std::string m_tableNamespace;

	Bounds<double> m_bounds;
	std::string m_poly;

	CloseableIterator m_iterator;

#ifdef PDAL_HAVE_GEOS
	GEOSContextHandle_t m_geosEnvironment;
    GEOSGeometry* m_geosGeometry; 
    GEOSPreparedGeometry const* m_geosPreparedGeometry;
#else   
    void* m_geosEnvironment;
    void* m_geosGeometry;
    void* m_geosPreparedGeometry;
    typedef struct GEOSGeometry* GEOSGeometryHS;
#endif

	Bounds <double> computeBounds(GEOSGeometry const *geometry);
};

} // namespace pdal
