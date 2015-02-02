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

#include "GeoWaveWriter.hpp"

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

#include "jace/proxy/types/JDouble.h"
using jace::proxy::types::JDouble;

#include "jace/JArray.h"
using jace::JArray;

#include "jace/proxy/java/lang/Object.h"
using jace::proxy::java::lang::Object;
#include "jace/proxy/java/lang/Double.h"
using jace::proxy::java::lang::Double;
#include "jace/proxy/java/lang/String.h"
using jace::proxy::java::lang::String;
#include "jace/proxy/java/util/Iterator.h"
using jace::proxy::java::util::Iterator;
#include "jace/proxy/java/util/List.h"
using jace::proxy::java::util::List;
#include "jace/proxy/java/util/ArrayList.h"
using jace::proxy::java::util::ArrayList;
#include "jace/proxy/java/util/UUID.h"
using jace::proxy::java::util::UUID;

#include "jace/proxy/com/vividsolutions/jts/geom/Coordinate.h"
using jace::proxy::com::vividsolutions::jts::geom::Coordinate;
#include "jace/proxy/com/vividsolutions/jts/geom/GeometryFactory.h"
using jace::proxy::com::vividsolutions::jts::geom::GeometryFactory;
#include "jace/proxy/com/vividsolutions/jts/geom/Point.h"
using jace::proxy::com::vividsolutions::jts::geom::Point;

#include "jace/proxy/org/geotools/data/DataUtilities.h"
using jace::proxy::org::geotools::data::DataUtilities;
#include "jace/proxy/org/geotools/feature/simple/SimpleFeatureBuilder.h"
using jace::proxy::org::geotools::feature::simple::SimpleFeatureBuilder;
#include "jace/proxy/org/geotools/feature/simple/SimpleFeatureTypeBuilder.h"
using jace::proxy::org::geotools::feature::simple::SimpleFeatureTypeBuilder;
#include "jace/proxy/org/geotools/geometry/jts/JTSFactoryFinder.h"
using jace::proxy::org::geotools::geometry::jts::JTSFactoryFinder;

#include "jace/proxy/org/opengis/feature/simple/SimpleFeature.h"
using jace::proxy::org::opengis::feature::simple::SimpleFeature;
#include "jace/proxy/org/opengis/feature/simple/SimpleFeatureType.h"
using jace::proxy::org::opengis::feature::simple::SimpleFeatureType;

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
#include "jace/proxy/mil/nga/giat/geowave/vector/adapter/FeatureDataAdapter.h"
using jace::proxy::mil::nga::giat::geowave::vector::adapter::FeatureDataAdapter;
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

Options GeoWaveWriter::getDefaultOptions()
{
	Options options;

    Option zookeeperUrl("zookeeperUrl", "", "The comma-delimited URLs for all zookeeper servers, this will be directly used to instantiate a ZookeeperInstance");
    Option instanceName("instanceName", "", "The zookeeper instance name, this will be directly used to instantiate a ZookeeperInstance");
    Option username("username", "", "The username for an account to establish an Accumulo connector");
    Option password("password", "", "The password for the account to establish an Accumulo connector");
	Option tableNamespace("tableNamespace", "", "An optional string that is prefixed to any of the table names");

    options.add(zookeeperUrl);
    options.add(instanceName);
    options.add(username);
    options.add(password);
	options.add(tableNamespace);

    return options;
}

void GeoWaveWriter::processOptions(const Options& ops)
{
	m_zookeeperUrl = ops.getValueOrThrow<std::string>("zookeeperUrl");
	m_instanceName = ops.getValueOrThrow<std::string>("instanceName");
	m_username = ops.getValueOrThrow<std::string>("username");
	m_password = ops.getValueOrThrow<std::string>("password");
	m_tableNamespace = ops.getValueOrDefault<std::string>("tableNamespace", "");
}

void GeoWaveWriter::ready(PointContext ctx)
{
	int status = createJvm();
	if (status == 0)
		log()->get(LogLevel::Debug) << "JVM Creation Successful" << std::endl;
	else
		log()->get(LogLevel::Error) << "JVM Creation Failed: Error ["  << status << "]" << std::endl;

	// get a list of all the dimensions & their types
    Dimension::IdList all = ctx.dims();
    for (auto di = all.begin(); di != all.end(); ++di)
        if (!Algorithm::contains(m_dims, *di))
            m_dims.push_back(*di);
}

void GeoWaveWriter::write(const PointBuffer& data)
{	
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
		return;
	}
	catch (std::exception& e)
	{
		log()->get(LogLevel::Error) << "An unexpected C++ error has occurred: " << e.what() << std::endl;
		return;
	}
	
	using namespace Dimension;

	std::ostringstream os;

	os << "location:Point:srid=4326";
	for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
		os << "," << data.dimName(*di) << ":Double";

	SimpleFeatureType TYPE = DataUtilities::createType(
		java_new<String>("PDAL_Point"),
        java_new<String>(os.str()));

	String location = java_new<String>("location");

	FeatureDataAdapter dataAdapter = java_new<FeatureDataAdapter>(TYPE);

	JArray<DimensionField> dimArray(2);
	dimArray[0] = java_new<LongitudeField>();	
	dimArray[1] = java_new<LatitudeField>();

	Index index = java_new<Index>(
		java_new<NumericIndexStrategyFactory_SpatialFactory>().createIndexStrategy(NumericIndexStrategyFactory_DataType::VECTOR()),
		java_new<BasicIndexModel>(
			dimArray));

	GeometryFactory geometryFactory = JTSFactoryFinder::getGeometryFactory();
	SimpleFeatureBuilder builder = java_new<SimpleFeatureBuilder>(TYPE);

    for (PointId idx = 0; idx < data.size(); ++idx)
    {
		JDouble X = data.getFieldAs<double>(Id::X, idx);
		JDouble Y = data.getFieldAs<double>(Id::Y, idx);

		Point point = geometryFactory.createPoint(
			java_new<Coordinate>(
				X, 
				Y));

		builder.set(location, point);

		for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
			if (data.hasDim(*di))
				builder.set(java_new<String>(data.dimName(*di)), java_new<Double>(data.getFieldAs<double>(*di, idx)));

		SimpleFeature feature = builder.buildFeature(UUID::randomUUID().toString());

		List results = accumuloDataStore.ingest(
			dataAdapter,
			index,
			feature);
	}
}

int GeoWaveWriter::createJvm()
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

} // namespaces
