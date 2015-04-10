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

#include "jace/JArray.h"
using jace::JArray;

#include "jace/proxy/types/JBoolean.h"
using jace::proxy::types::JBoolean;
#include "jace/proxy/types/JDouble.h"
using jace::proxy::types::JDouble;
#include "jace/proxy/types/JInt.h"
using jace::proxy::types::JInt;

#include "jace/proxy/java/lang/Double.h"
using jace::proxy::java::lang::Double;
#include "jace/proxy/java/lang/String.h"
using jace::proxy::java::lang::String;
#include "jace/proxy/java/util/List.h"
using jace::proxy::java::util::List;

#include "jace/proxy/org/geotools/data/simple/SimpleFeatureCollection.h"
using jace::proxy::org::geotools::data::simple::SimpleFeatureCollection;
#include "jace/proxy/org/geotools/data/simple/SimpleFeatureIterator.h"
using jace::proxy::org::geotools::data::simple::SimpleFeatureIterator;

#include "jace/proxy/com/vividsolutions/jts/geom/Polygon.h"
using jace::proxy::com::vividsolutions::jts::geom::Polygon;
#include "jace/proxy/com/vividsolutions/jts/geom/Coordinate.h"
using jace::proxy::com::vividsolutions::jts::geom::Coordinate;
#include "jace/proxy/com/vividsolutions/jts/geom/GeometryFactory.h"
using jace::proxy::com::vividsolutions::jts::geom::GeometryFactory;
#include "jace/proxy/com/vividsolutions/jts/geom/Point.h"
using jace::proxy::com::vividsolutions::jts::geom::Point;

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
#include "jace/proxy/mil/nga/giat/geowave/store/index/Index.h"
using jace::proxy::mil::nga::giat::geowave::store::index::Index;
#include "jace/proxy/mil/nga/giat/geowave/store/index/IndexType_JaceIndexType.h"
using jace::proxy::mil::nga::giat::geowave::store::index::IndexType_JaceIndexType;
#include "jace/proxy/mil/nga/giat/geowave/store/query/Query.h"
using jace::proxy::mil::nga::giat::geowave::store::query::Query;
#include "jace/proxy/mil/nga/giat/geowave/store/query/SpatialQuery.h"
using jace::proxy::mil::nga::giat::geowave::store::query::SpatialQuery;

#include "jace/proxy/mil/nga/giat/geowave/accumulo/BasicAccumuloOperations.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::BasicAccumuloOperations;
#include "jace/proxy/mil/nga/giat/geowave/accumulo/AccumuloDataStore.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::AccumuloDataStore;
#include "jace/proxy/mil/nga/giat/geowave/accumulo/metadata/AccumuloAdapterStore.h"
using jace::proxy::mil::nga::giat::geowave::accumulo::metadata::AccumuloAdapterStore;

static PluginInfo const s_info = PluginInfo(
    "readers.geowave",
    "\"GeoWave\"  reader support. ",
    "http://pdal.io/stages/drivers.geowave.reader.html" );

CREATE_SHARED_PLUGIN(1, 0, GeoWaveReader, Reader, s_info)

std::string pdal::GeoWaveReader::getName() const { return s_info.name; }


#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

namespace pdal
{

    Options GeoWaveReader::getDefaultOptions()
    {
        Options options;

        Option zookeeperUrl("zookeeperUrl", "", "The comma-delimited URLs for all zookeeper servers, this will be directly used to instantiate a ZookeeperInstance");
        Option instanceName("instanceName", "", "The zookeeper instance name, this will be directly used to instantiate a ZookeeperInstance");
        Option username("username", "", "The username for the account to establish an Accumulo connector");
        Option password("password", "", "The password for the account to establish an Accumulo connector");
        Option tableNamespace("tableNamespace", "", "The table name to be used when interacting with GeoWave");
        Option featureTypeName("featureTypeName", "", "The feature type name to be used when interacting with GeoWave");
        Option dataAdapter("dataAdapter", "FeatureDataAdapter", "FeatureCollectionDataAdapter stores multiple points per Accumulo entry.  FeatureDataAdapter stores a single point per Accumulo entry.");
        Option pointsPerEntry("pointsPerEntry", 5000u, "Sets the maximum number of points per Accumulo entry when using FeatureCollectionDataAdapter.");
        Option bounds("bounds", "", "The extent of the bounding rectangle to use to query points, expressed as a string, eg: ([xmin, xmax], [ymin, ymax], [zmin, zmax])");

        options.add(zookeeperUrl);
        options.add(instanceName);
        options.add(username);
        options.add(password);
        options.add(tableNamespace);
        options.add(featureTypeName);
        options.add(dataAdapter);
        options.add(pointsPerEntry);
        options.add(bounds);

        return options;
    }

    void GeoWaveReader::initialize()
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

    void GeoWaveReader::processOptions(const Options& ops)
    {
        m_zookeeperUrl = ops.getValueOrThrow<std::string>("zookeeperUrl");
        m_instanceName = ops.getValueOrThrow<std::string>("instanceName");
        m_username = ops.getValueOrThrow<std::string>("username");
        m_password = ops.getValueOrThrow<std::string>("password");
        m_tableNamespace = ops.getValueOrThrow<std::string>("tableNamespace");
        m_featureTypeName =  ops.getValueOrDefault<std::string>("featureTypeName", "PDAL_Point");
        m_useFeatCollDataAdapter = !(ops.getValueOrDefault<std::string>("dataAdapter", "FeatureCollectionDataAdapter").compare("FeatureDataAdapter") == 0);
        m_pointsPerEntry = ops.getValueOrDefault<uint32_t>("pointsPerEntry", 5000u);
        m_bounds = ops.getValueOrDefault<BOX3D>("bounds", BOX3D());
    }

    void GeoWaveReader::addDimensions(PointLayoutPtr layout)
    {
        layout->registerDims(getDefaultDimensions());

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
        
        AccumuloAdapterStore accumuloAdapterStore = java_new<AccumuloAdapterStore>(accumuloOperations);

        List attribs;
        if (m_useFeatCollDataAdapter)
            attribs = java_cast<FeatureCollectionDataAdapter>(accumuloAdapterStore.getAdapter(java_new<ByteArrayId>(std::to_string(m_pointsPerEntry) + m_featureTypeName))).getType().getAttributeDescriptors();
        else
            attribs = java_cast<FeatureDataAdapter>(accumuloAdapterStore.getAdapter(java_new<ByteArrayId>(m_featureTypeName))).getType().getAttributeDescriptors();

        for (int i = 0; i < attribs.size(); ++i){
            std::string name = java_cast<AttributeDescriptor>(attribs.get(i)).getLocalName();
            if (name.compare("location") != 0 && name.compare("X") != 0 && name.compare("Y") != 0)
                layout->registerDim(Dimension::id(name));
        }
    }

    Dimension::IdList GeoWaveReader::getDefaultDimensions()
    {
        Dimension::IdList ids;
        ids.push_back(Dimension::Id::X);
        ids.push_back(Dimension::Id::Y);
        return ids;
    }

    void GeoWaveReader::ready(PointTableRef table)
    {
        if (m_bounds.empty())
            return;

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

        GeometryFactory factory = java_new<GeometryFactory>();

        JDouble lonMin = m_bounds.minx;
        JDouble lonMax = m_bounds.maxx;
        JDouble latMin = m_bounds.miny;
        JDouble latMax = m_bounds.maxy;

        JArray<Coordinate> coordArray(5);
        coordArray[0] = java_new<Coordinate>(lonMin, latMin);
        coordArray[1] = java_new<Coordinate>(lonMax, latMin);
        coordArray[2] = java_new<Coordinate>(lonMax, latMax);
        coordArray[3] = java_new<Coordinate>(lonMin, latMax);
        coordArray[4] = java_new<Coordinate>(lonMin, latMin);

        Polygon geom = factory.createPolygon(coordArray);
        Query query = java_new<SpatialQuery>(geom);

        JInt count = m_count;
        m_iterator = accumuloDataStore.query(index, query, count);
    }

    point_count_t GeoWaveReader::read(PointViewPtr view, point_count_t count)
    {
        using namespace Dimension;

        String location = java_new<String>("location");
        point_count_t numRead = 0;

        if (m_useFeatCollDataAdapter)
        {
            while (m_iterator.hasNext() && count > 0)
            {
                SimpleFeatureCollection featureCollection = java_cast<SimpleFeatureCollection>(m_iterator.next());
                SimpleFeatureIterator featItr = featureCollection.features();

                while (featItr.hasNext() && count-- > 0)
                {
                    SimpleFeature simpleFeature = java_cast<SimpleFeature>(featItr.next());
                    List attribs = simpleFeature.getType().getAttributeDescriptors();

                    for (int i = 0; i < attribs.size(); ++i){
                        String name = java_cast<AttributeDescriptor>(attribs.get(i)).getLocalName();

                        if (!name.equals(location))
                            view->setField(id(name), numRead, java_cast<Double>(simpleFeature.getAttribute(name)).doubleValue());
                    }

                    if (m_cb)
                        m_cb(*view, numRead);

                    ++numRead;
                }
                featItr.close();
            }
        }
        else
        {
            while (m_iterator.hasNext() && count-- > 0){
                SimpleFeature simpleFeature = java_cast<SimpleFeature>(m_iterator.next());
                List attribs = simpleFeature.getType().getAttributeDescriptors();

                for (int i = 0; i < attribs.size(); ++i){
                    String name = java_cast<AttributeDescriptor>(attribs.get(i)).getLocalName();

                    if (!name.equals(location))
                        view->setField(id(name), numRead, java_cast<Double>(simpleFeature.getAttribute(name)).doubleValue());
                }

                if (m_cb)
                    m_cb(*view, numRead);

                ++numRead;
            }
        }

        return numRead;
    }

    void GeoWaveReader::done(PointTableRef table)
    {
        m_iterator.close();
    }

    int GeoWaveReader::createJvm()
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
} // namespace pdal
