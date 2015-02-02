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

#include "GeoWaveReader.hpp"

#include <jace/Jace.h>
using jace::java_cast;
using jace::java_new;

#include "jace/JNIException.h"
using jace::JNIException;

#include "jace/VirtualMachineShutdownError.h"
using jace::VirtualMachineShutdownError;

#include "jace/OptionList.h"
using jace::OptionList;
using jace::Option;
using jace::ClassPath;
using jace::Verbose;
using jace::CustomOption;

#include <jace/StaticVmLoader.h>
using jace::StaticVmLoader;

#ifdef _WIN32
  #include "jace/Win32VmLoader.h"
  using jace::Win32VmLoader;
  const std::string os_pathsep(";");
#else
  #include "jace/UnixVmLoader.h"
  using ::jace::UnixVmLoader;
  const std::string os_pathsep(":");
#endif

#include "jace/proxy/types/JInt.h"
using jace::proxy::types::JInt;

#include "jace/JArray.h"
using jace::JArray;

#include "jace/proxy/types/JBoolean.h"
using jace::proxy::types::JBoolean;
#include "jace/proxy/types/JDouble.h"
using jace::proxy::types::JDouble;

#include "jace/proxy/java/lang/Double.h"
using jace::proxy::java::lang::Double;
#include "jace/proxy/java/lang/String.h"
using jace::proxy::java::lang::String;
#include "jace/proxy/java/util/List.h"
using jace::proxy::java::util::List;

#include "jace/proxy/com/vividsolutions/jts/geom/Polygon.h"
using jace::proxy::com::vividsolutions::jts::geom::Polygon;
#include "jace/proxy/com/vividsolutions/jts/geom/Coordinate.h"
using jace::proxy::com::vividsolutions::jts::geom::Coordinate;
#include "jace/proxy/com/vividsolutions/jts/geom/GeometryFactory.h"
using jace::proxy::com::vividsolutions::jts::geom::GeometryFactory;
#include "jace/proxy/com/vividsolutions/jts/geom/Point.h"
using jace::proxy::com::vividsolutions::jts::geom::Point;

#include "jace/proxy/org/opengis/feature/simple/SimpleFeature.h"
using jace::proxy::org::opengis::feature::simple::SimpleFeature;
#include "jace/proxy/org/opengis/feature/simple/SimpleFeatureType.h"
using jace::proxy::org::opengis::feature::simple::SimpleFeatureType;
#include "jace/proxy/org/opengis/feature/type/AttributeDescriptor.h"
using jace::proxy::org::opengis::feature::type::AttributeDescriptor;

#include "jace/proxy/mil/nga/giat/geowave/index/ByteArrayId.h"
using jace::proxy::mil::nga::giat::geowave::index::ByteArrayId;
#include "jace/proxy/mil/nga/giat/geowave/index/NumericIndexStrategy.h"
using jace::proxy::mil::nga::giat::geowave::index::NumericIndexStrategy;
#include "jace/proxy/mil/nga/giat/geowave/index/NumericIndexStrategyFactory.h"
using jace::proxy::mil::nga::giat::geowave::index::NumericIndexStrategyFactory;
#include "jace/proxy/mil/nga/giat/geowave/index/NumericIndexStrategyFactory_DataType.h"
using jace::proxy::mil::nga::giat::geowave::index::NumericIndexStrategyFactory_DataType;
#include "jace/proxy/mil/nga/giat/geowave/index/NumericIndexStrategyFactory_SpatialFactory.h"
using jace::proxy::mil::nga::giat::geowave::index::NumericIndexStrategyFactory_SpatialFactory;
#include "jace/proxy/mil/nga/giat/geowave/store/dimension/DimensionField.h"
using jace::proxy::mil::nga::giat::geowave::store::dimension::DimensionField;
#include "jace/proxy/mil/nga/giat/geowave/store/dimension/LongitudeField.h"
using jace::proxy::mil::nga::giat::geowave::store::dimension::LongitudeField;
#include "jace/proxy/mil/nga/giat/geowave/store/dimension/LatitudeField.h"
using jace::proxy::mil::nga::giat::geowave::store::dimension::LatitudeField;
#include "jace/proxy/mil/nga/giat/geowave/store/index/Index.h"
using jace::proxy::mil::nga::giat::geowave::store::index::Index;
#include "jace/proxy/mil/nga/giat/geowave/store/index/BasicIndexModel.h"
using jace::proxy::mil::nga::giat::geowave::store::index::BasicIndexModel;
#include "jace/proxy/mil/nga/giat/geowave/store/query/Query.h"
using jace::proxy::mil::nga::giat::geowave::store::query::Query;
#include "jace/proxy/mil/nga/giat/geowave/store/query/SpatialQuery.h"
using jace::proxy::mil::nga::giat::geowave::store::query::SpatialQuery;

#include "jace/proxy/mil/nga/giat/geowave/accumulo/AccumuloOptions.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::AccumuloOptions;
#include "jace/proxy/mil/nga/giat/geowave/accumulo/BasicAccumuloOperations.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::BasicAccumuloOperations;
#include "jace/proxy/mil/nga/giat/geowave/accumulo/AccumuloDataStore.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::AccumuloDataStore;

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

namespace pdal
{

#ifdef PDAL_HAVE_GEOS
namespace geos
{
static void _GEOSErrorHandler(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    char buf[1024];

    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cerr << "GEOS Error: " << buf << std::endl;

    va_end(args);
}

static void _GEOSWarningHandler(const char *fmt, ...)
{
    va_list args;

    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cout << "GEOS warning: " << buf << std::endl;

    va_end(args);
}

} // geos
#endif

Options GeoWaveReader::getDefaultOptions()
{
	Options options;

    Option zookeeperUrl("zookeeperUrl", "", "The comma-delimited URLs for all zookeeper servers, this will be directly used to instantiate a ZookeeperInstance");
    Option instanceName("instanceName", "", "The zookeeper instance name, this will be directly used to instantiate a ZookeeperInstance");
    Option username("username", "", "The username for an account to establish an Accumulo connector");
    Option password("password", "", "The password for the account to establish an Accumulo connector");
	Option tableNamespace("tableNamespace", "", "An optional string that is prefixed to any of the table names");
	Option bounds("bounds", "", "The extent of the bounding rectangle to use to query points, expressed as a string, eg: ([xmin, xmax], [ymin, ymax], [zmin, zmax])");
	Option poly("polygon", "", "WKT POLYGON() string to use to query points");

    options.add(zookeeperUrl);
    options.add(instanceName);
    options.add(username);
    options.add(password);
	options.add(tableNamespace);
	options.add(bounds);
	options.add(poly);

    return options;
}

void GeoWaveReader::processOptions(const Options& ops)
{
	m_zookeeperUrl = ops.getValueOrThrow<std::string>("zookeeperUrl");
	m_instanceName = ops.getValueOrThrow<std::string>("instanceName");
	m_username = ops.getValueOrThrow<std::string>("username");
	m_password = ops.getValueOrThrow<std::string>("password");
	m_tableNamespace = ops.getValueOrDefault<std::string>("tableNamespace", "");

	m_bounds = ops.getValueOrDefault<Bounds<double>>("bounds", Bounds<double>());
	m_poly = ops.getValueOrDefault<std::string>("polygon", "");
}

void GeoWaveReader::addDimensions(PointContext ctx)
{
    using namespace Dimension;

	ctx.registerDim(Id::X);
    ctx.registerDim(Id::Y);
	ctx.registerDim(Id::Z);
}

void GeoWaveReader::ready(PointContext ctx)
{
	int status = createJvm();
	if (status == 0)
		log()->get(LogLevel::Debug) << "JVM Creation Successful" << std::endl;
	else
		log()->get(LogLevel::Error) << "JVM Creation Failed: Error ["  << status << "]" << std::endl;


#ifdef PDAL_HAVE_GEOS
	if (!m_poly.empty())
    {
		m_geosEnvironment = initGEOS_r(pdal::geos::_GEOSWarningHandler,
			pdal::geos::_GEOSErrorHandler);
		m_geosGeometry = GEOSGeomFromWKT_r(m_geosEnvironment, m_poly.c_str());
		if (!m_geosGeometry)
			throw pdal_error("unable to import polygon WKT");

		int gtype = GEOSGeomTypeId_r(m_geosEnvironment, m_geosGeometry);
		if (!(gtype == GEOS_POLYGON || gtype == GEOS_MULTIPOLYGON))
			throw pdal_error("input WKT was not a POLYGON or MULTIPOLYGON");

		char* out_wkt = GEOSGeomToWKT_r(m_geosEnvironment, m_geosGeometry);
		log()->get(LogLevel::Debug2) << "Ingested WKT for drivers.geowave: " <<
			std::string(out_wkt) <<std::endl;
		GEOSFree_r(m_geosEnvironment, out_wkt);

		if (!GEOSisValid_r(m_geosEnvironment, m_geosGeometry))
		{
			char* reason =
				GEOSisValidReason_r(m_geosEnvironment, m_geosGeometry);
			std::ostringstream oss;
			oss << "WKT is invalid: " << std::string(reason) << std::endl;
			GEOSFree_r(m_geosEnvironment, reason);
			throw pdal_error(oss.str());
		}

		m_geosPreparedGeometry =
			GEOSPrepare_r(m_geosEnvironment, m_geosGeometry);
		if (!m_geosPreparedGeometry)
			throw pdal_error("unable to prepare geometry for "
				"index-accellerated intersection");
		m_bounds = computeBounds(m_geosGeometry);
		log()->get(LogLevel::Debug) << "Computed bounds from given WKT: " <<
			m_bounds <<std::endl;
	}
#endif

	status = createCloseableIterator();
	if (status == 0)
		log()->get(LogLevel::Debug) << "Closeable Iterator Creation Successful" << std::endl;
	else
		log()->get(LogLevel::Error) << "Closeable Iterator Creation Failed: Error ["  << status << "]" << std::endl;
}

point_count_t GeoWaveReader::read(PointBuffer& buf, point_count_t count)
{
	using namespace Dimension;

	String location = java_new<String>("location");

	point_count_t numRead = 0;

	while (m_iterator.hasNext() && count-- > 0){
		SimpleFeature simpleFeature = java_cast<SimpleFeature>(m_iterator.next());
		List attribs = simpleFeature.getType().getAttributeDescriptors();

		//Point point = java_cast<Point>(simpleFeature.getAttribute(location));
		//Coordinate coord = point.getCoordinate();
		//std::cout << "Point " << (numRead+1) << ": [" << coord.x() << ", " << coord.y() << "]" << std::endl;

		for (int i = 0; i < attribs.size(); ++i){
			AttributeDescriptor attribDesc = java_cast<AttributeDescriptor>(attribs.get(i));
			String name = attribDesc.getLocalName();
			
			if (!name.equals(location))
			{
				JDouble value = java_cast<Double>(simpleFeature.getAttribute(name).toString()).doubleValue();
				Id::Enum dim = id(name);
				buf.setField(dim, numRead, value);
				//std::cout << "     " << name << ": " << value << std::endl;
			}
		}
		
		++numRead;
	}

    return numRead;
}

void GeoWaveReader::done(PointContext ctx)
{
	m_iterator.close();
}

Dimension::IdList GeoWaveReader::getDefaultDimensions()
{
	Dimension::IdList ids;

	ids.push_back(Dimension::Id::X);
    ids.push_back(Dimension::Id::Y);
	ids.push_back(Dimension::Id::Z);

	return ids;
}

int GeoWaveReader::createJvm()
{
	try
	{
		StaticVmLoader loader(JNI_VERSION_1_2);

		std::string jaceClasspath = TOSTRING(JACE_RUNTIME_JAR);
		std::string geowaveClasspath = TOSTRING(GEOWAVE_RUNTIME_JAR);

		OptionList options;
		//options.push_back(CustomOption("-Xcheck:jni"));
		//options.push_back(Verbose (Verbose::JNI));
		//options.push_back(Verbose (Verbose::CLASS));
		options.push_back(ClassPath(jaceClasspath + os_pathsep + geowaveClasspath));

		jace::createVm(loader, options);
	}
	catch (VirtualMachineShutdownError&)
	{
		log()->get(LogLevel::Error) << "The JVM was terminated in mid-execution. " << std::endl;
		return -1;
	}
	catch (JNIException& jniException)
	{
		log()->get(LogLevel::Error) << "An unexpected JNI error has occured: " << jniException.what() << std::endl;
		return -2;
	}
	catch (std::exception& e)
	{
		log()->get(LogLevel::Error) << "An unexpected C++ error has occurred: " << e.what() << std::endl;
		return -3;
	}

	return 0;
}

int GeoWaveReader::createCloseableIterator()
{
	if (m_bounds.size() != 2)
		return 0;

	BasicAccumuloOperations accumuloOperations;
	AccumuloOptions accumuloOptions;
	AccumuloDataStore accumuloDataStore;

	try
	{
		accumuloOperations = java_new<BasicAccumuloOperations>(
				java_new<String>(m_zookeeperUrl),
				java_new<String>(m_instanceName),
				java_new<String>(m_username),
				java_new<String>(m_password),
				java_new<String>(m_tableNamespace));

		accumuloOptions = java_new<AccumuloOptions>();

		accumuloDataStore = java_new<AccumuloDataStore>(
			accumuloOperations,
			accumuloOptions);
	}	
	catch (JNIException& jniException)
	{
		log()->get(LogLevel::Error) << "An unexpected JNI error has occured: " << jniException.what() << std::endl;
		return -2;
	}
	catch (std::exception& e)
	{
		log()->get(LogLevel::Error) << "An unexpected C++ error has occurred: " << e.what() << std::endl;
		return -3;
	}

	JArray<DimensionField> dimArray(2);
	dimArray[0] = java_new<LongitudeField>();	
	dimArray[1] = java_new<LatitudeField>();

	Index index = java_new<Index>(
		java_new<NumericIndexStrategyFactory_SpatialFactory>().createIndexStrategy(NumericIndexStrategyFactory_DataType::VECTOR()),
		java_new<BasicIndexModel>(
			dimArray));

	GeometryFactory factory = java_new<GeometryFactory>();
	
	JDouble lonMin = m_bounds.getMinimum(0);
	JDouble lonMax = m_bounds.getMaximum(0);
	JDouble latMin = m_bounds.getMinimum(1);
	JDouble latMax = m_bounds.getMaximum(1);

	JArray<Coordinate> coordArray(5);
	coordArray[0] = java_new<Coordinate>(lonMin, latMin);
	coordArray[1] = java_new<Coordinate>(lonMax, latMin);
	coordArray[2] = java_new<Coordinate>(lonMax, latMax);
	coordArray[3] = java_new<Coordinate>(lonMin, latMax);
	coordArray[4] = java_new<Coordinate>(lonMin, latMin);

	Polygon geom = factory.createPolygon(coordArray);
	Query query = java_new<SpatialQuery>(geom);

	m_iterator = accumuloDataStore.query(index, query);

	return 0;
}

Bounds<double> GeoWaveReader::computeBounds(GEOSGeometry const *geometry)
{
    uint32_t numInputDims;
    Bounds<double> output;

#ifdef PDAL_HAVE_GEOS
    bool bFirst(true);

    GEOSGeometry const* ring = GEOSGetExteriorRing_r(m_geosEnvironment,
        geometry);
    GEOSCoordSequence const* coords = GEOSGeom_getCoordSeq_r(m_geosEnvironment,
        ring);

    GEOSCoordSeq_getDimensions_r(m_geosEnvironment, coords, &numInputDims);
    log()->get(LogLevel::Debug) << "Inputted WKT had " << numInputDims <<
        " dimensions" <<std::endl;

    uint32_t count(0);
    GEOSCoordSeq_getSize_r(m_geosEnvironment, coords, &count);
    pdal::Vector<double> p(0.0, 0.0, 0.0);

    double x(0.0);
    double y(0.0);
    double z(0.0);
    for (unsigned i = 0; i < count; ++i)
    {
        GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 0, &x);
        GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 1, &y);
        if (numInputDims > 2)
            GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 2, &z);
        p.set(0, x);
        p.set(1, y);
        if (numInputDims > 2)
            p.set(2, z);
        if (bFirst)
        {
            output = Bounds<double>(p, p);
            bFirst = false;
        }
        output.grow(p);
    }
#else
    boost::ignore_unused_variable_warning(geometry);
#endif
    return output;
}

} // namespace pdal
