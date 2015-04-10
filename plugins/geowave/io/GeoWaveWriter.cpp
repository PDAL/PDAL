/******************************************************************************
* Copyright (c) 2015, James W. O'Meara (james.w.omeara@gmail.com)
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

#include <pdal/util/Algorithm.hpp>

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
#include "jace/proxy/org/geotools/feature/DefaultFeatureCollection.h"
using jace::proxy::org::geotools::feature::DefaultFeatureCollection;
#include "jace/proxy/org/geotools/geometry/jts/JTSFactoryFinder.h"
using jace::proxy::org::geotools::geometry::jts::JTSFactoryFinder;

#include "jace/proxy/org/apache/accumulo/core/client/AccumuloException.h"
using jace::proxy::org::apache::accumulo::core::client::AccumuloException;
#include "jace/proxy/org/apache/accumulo/core/client/AccumuloSecurityException.h"
using jace::proxy::org::apache::accumulo::core::client::AccumuloSecurityException;

#include "jace/proxy/org/opengis/feature/simple/SimpleFeature.h"
using jace::proxy::org::opengis::feature::simple::SimpleFeature;
#include "jace/proxy/org/opengis/feature/simple/SimpleFeatureType.h"
using jace::proxy::org::opengis::feature::simple::SimpleFeatureType;
#include "jace/proxy/org/opengis/feature/type/AttributeDescriptor.h"
using jace::proxy::org::opengis::feature::type::AttributeDescriptor;

#include "jace/proxy/mil/nga/giat/geowave/index/ByteArrayId.h"
using jace::proxy::mil::nga::giat::geowave::index::ByteArrayId;
#include "jace/proxy/mil/nga/giat/geowave/vector/adapter/FeatureDataAdapter.h"
using jace::proxy::mil::nga::giat::geowave::vector::adapter::FeatureDataAdapter;
#include "jace/proxy/mil/nga/giat/geowave/vector/adapter/FeatureCollectionDataAdapter.h"
using jace::proxy::mil::nga::giat::geowave::vector::adapter::FeatureCollectionDataAdapter;
#include "jace/proxy/mil/nga/giat/geowave/store/adapter/WritableDataAdapter.h"
using jace::proxy::mil::nga::giat::geowave::store::adapter::WritableDataAdapter;
#include "jace/proxy/mil/nga/giat/geowave/store/index/Index.h"
using jace::proxy::mil::nga::giat::geowave::store::index::Index;
#include "jace/proxy/mil/nga/giat/geowave/store/index/IndexType_JaceIndexType.h"
using jace::proxy::mil::nga::giat::geowave::store::index::IndexType_JaceIndexType;
#include "jace/proxy/mil/nga/giat/geowave/accumulo/BasicAccumuloOperations.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::BasicAccumuloOperations;
#include "jace/proxy/mil/nga/giat/geowave/accumulo/AccumuloDataStore.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::AccumuloDataStore;
#include "jace/proxy/mil/nga/giat/geowave/accumulo/AccumuloIndexWriter.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::AccumuloIndexWriter;

static PluginInfo const s_info = PluginInfo(
    "writers.geowave",
    "Write data using GeoWave.",
    "http://pdal.io/stages/drivers.geowave.writer.html" );

CREATE_SHARED_PLUGIN(1, 0, GeoWaveWriter, Writer, s_info)

std::string pdal::GeoWaveWriter::getName() const { return s_info.name; }


#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

namespace pdal
{

    Options GeoWaveWriter::getDefaultOptions()
    {
        Options options;

        Option zookeeperUrl("zookeeperUrl", "", "The comma-delimited URLs for all zookeeper servers, this will be directly used to instantiate a ZookeeperInstance.");
        Option instanceName("instanceName", "", "The zookeeper instance name, this will be directly used to instantiate a ZookeeperInstance.");
        Option username("username", "", "The username for the account to establish an Accumulo connector.");
        Option password("password", "", "The password for the account to establish an Accumulo connector.");
        Option tableNamespace("tableNamespace", "", "The table name to be used when interacting with GeoWave.");
        Option featureTypeName("featureTypeName", "PDAL_Point", "The feature type name to be used when interacting with GeoWave.");
        Option dataAdapter("dataAdapter", "FeatureDataAdapter", "FeatureCollectionDataAdapter stores multiple points per Accumulo entry.  FeatureDataAdapter stores a single point per Accumulo entry.");
        Option pointsPerEntry("pointsPerEntry", 5000u, "Sets the maximum number of points per Accumulo entry when using FeatureCollectionDataAdapter.");

        options.add(zookeeperUrl);
        options.add(instanceName);
        options.add(username);
        options.add(password);
        options.add(tableNamespace);
        options.add(featureTypeName);
        options.add(dataAdapter);
        options.add(pointsPerEntry);

        return options;
    }

    void GeoWaveWriter::processOptions(const Options& ops)
    {
        m_zookeeperUrl = ops.getValueOrThrow<std::string>("zookeeperUrl");
        m_instanceName = ops.getValueOrThrow<std::string>("instanceName");
        m_username = ops.getValueOrThrow<std::string>("username");
        m_password = ops.getValueOrThrow<std::string>("password");
        m_tableNamespace = ops.getValueOrThrow<std::string>("tableNamespace");
        m_featureTypeName = ops.getValueOrDefault<std::string>("featureTypeName", "PDAL_Point");
        m_useFeatCollDataAdapter = !(ops.getValueOrDefault<std::string>("dataAdapter", "FeatureCollectionDataAdapter").compare("FeatureDataAdapter") == 0);
        m_pointsPerEntry = ops.getValueOrDefault<uint32_t>("pointsPerEntry", 5000u);
    }

    void GeoWaveWriter::initialize()
    {
        if (!jace::isRunning())
        {
            int status = createJvm();
            if (status == 0)
                log()->get(LogLevel::Debug) << "JVM Creation Successful" << std::endl;
            else
                log()->get(LogLevel::Error) << "JVM Creation Failed: Error ["  << status << "]" << std::endl;
        }
    }

    void GeoWaveWriter::ready(PointTableRef table)
    {
        // get a list of all the dimensions & their types
        Dimension::IdList all = table.layout()->dims();
        for (auto di = all.begin(); di != all.end(); ++di)
            if (!contains(m_dims, *di))
                m_dims.push_back(*di);
    }

    void GeoWaveWriter::write(const PointViewPtr view)
    {

        using namespace Dimension;

        std::ostringstream os;

        BasicAccumuloOperations accumuloOperations;
        try 
        {
            accumuloOperations = java_new<BasicAccumuloOperations>(
                java_new<String>(m_zookeeperUrl),
                java_new<String>(m_instanceName),
                java_new<String>(m_username),
                java_new<String>(m_password),
                java_new<String>(m_tableNamespace));
        }
        catch (AccumuloException& e)
        {
            log()->get(LogLevel::Error) << "There was a problem establishing a connector. " << e;
            return;
        }
        catch (AccumuloSecurityException& e)
        {
            log()->get(LogLevel::Error) << "The credentials passed are invalid. " << e;
            return;
        }

        AccumuloDataStore accumuloDataStore = java_new<AccumuloDataStore>(
            accumuloOperations);

        Index index = IndexType_JaceIndexType::createSpatialVectorIndex();

        AccumuloIndexWriter accumuloIndexWriter = java_new<AccumuloIndexWriter>(
            index,
            accumuloOperations,
            accumuloDataStore);

        // treat all types as double
        os << "location:Point:srid=4326";
        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
            os << "," << view->dimName(*di) << ":Double";

        SimpleFeatureType TYPE = DataUtilities::createType(
            java_new<String>(m_featureTypeName),
            java_new<String>(os.str()));

        String location = java_new<String>("location");

        WritableDataAdapter dataAdapter;
        if (m_useFeatCollDataAdapter)
            dataAdapter = java_new<FeatureCollectionDataAdapter>(
            TYPE,
            m_pointsPerEntry);
        else
            dataAdapter = java_new<FeatureDataAdapter>(TYPE);


        GeometryFactory geometryFactory = JTSFactoryFinder::getGeometryFactory();
        SimpleFeatureBuilder builder = java_new<SimpleFeatureBuilder>(TYPE);

        DefaultFeatureCollection featureCollection = java_new<DefaultFeatureCollection>(
            UUID::randomUUID().toString(),
            TYPE);

        for (PointId idx = 0; idx < view->size(); ++idx)
        {
            JDouble X = view->getFieldAs<double>(Id::X, idx);
            JDouble Y = view->getFieldAs<double>(Id::Y, idx);

            Point point = geometryFactory.createPoint(
                java_new<Coordinate>(
                X,
                Y));

            builder.set(location, point);

            for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
                if (view->hasDim(*di))
                    builder.set(java_new<String>(view->dimName(*di)), java_new<Double>(view->getFieldAs<double>(*di, idx)));

            SimpleFeature feature = builder.buildFeature(UUID::randomUUID().toString());

            if (m_useFeatCollDataAdapter)
                featureCollection.add(feature);
            else
                accumuloIndexWriter.write(
                dataAdapter,
                feature);
        }

        if (m_useFeatCollDataAdapter)
            accumuloIndexWriter.write(
            dataAdapter,
            featureCollection);

        accumuloIndexWriter.close();
    }

    int GeoWaveWriter::createJvm()
    {
        try
        {
            StaticVmLoader loader(JNI_VERSION_1_2);

            std::string jaceClasspath = TOSTRING(JACE_RUNTIME_JAR);
            std::string geowaveClasspath = TOSTRING(GEOWAVE_RUNTIME_JAR);

            OptionList options;
            //options.push_back(CustomOption("-Xdebug"));
            //options.push_back(CustomOption("-Xrunjdwp:server=y,transport=dt_socket,address=4000,suspend=y"));
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
