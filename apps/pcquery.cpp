/******************************************************************************
* Copyright (c) 2012, Howard Butler (hobu.inc@gmail.com)
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


#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <pdal/Stage.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/Index.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "AppSupport.hpp"
#include "Application.hpp"
#include <cstdarg>

#include <boost/tokenizer.hpp>
#define SEPARATORS ",| "
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

#ifdef PDAL_HAVE_GEOS
#include <geos_c.h>


namespace pcquery
{
    static void _GEOSErrorHandler(const char *fmt, ...)
    {
        va_list args;

        va_start(args, fmt);
        char buf[1024];  

        vsnprintf( buf, sizeof( buf), fmt, args);
        std::cerr << "GEOS Error: " << buf << std::endl;

        va_end(args);
    }

    static void _GEOSWarningHandler(const char *fmt, ...)
    {
        va_list args;

        char buf[1024];  
        vsnprintf( buf, sizeof( buf), fmt, args);
        std::cout << "GEOS warning: " << buf << std::endl;

        va_end(args);
    }

} // geos

#endif

using namespace pdal;


class PcQuery : public Application
{
public:
    PcQuery(int argc, char* argv[]);
    int execute(); // overrride

private:
    void addSwitches(); // overrride
    void validateSwitches(); // overrride

    void readPoints(
                                StageSequentialIterator* iter,
                                PointBuffer& data);
                                
    std::string m_inputFile;
    std::string m_wkt;
    std::string m_point;
    

#ifdef PDAL_HAVE_GEOS
	GEOSContextHandle_t m_geosEnvironment;
#endif
	
    pdal::Options m_options;
};


PcQuery::PcQuery(int argc, char* argv[])
    : Application(argc, argv, "pcquery")
    , m_inputFile("")
    , m_point("")
{
    return;
}


void PcQuery::validateSwitches()
{
    
    if (m_wkt.size())
    {
#ifdef PDAL_HAVE_GEOS
        // read the WKT using GEOS
        m_geosEnvironment = initGEOS_r(pcquery::_GEOSWarningHandler, pcquery::_GEOSErrorHandler);
        GEOSWKTReader* reader = GEOSWKTReader_create_r(m_geosEnvironment);
        GEOSGeometry* geom = GEOSWKTReader_read_r(m_geosEnvironment, reader, m_wkt.c_str());
        if (!geom)
            throw app_runtime_error("unable to ingest given WKT string");
#endif
            
    }
    return;
}


void PcQuery::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");
    

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
        ("point", po::value< std::string>(&m_point), "A 2d or 3d point to use for querying")
        ("wkt", po::value<std::string>(&m_wkt)->default_value(""), "WKT object to use for querying")
        ;

    addSwitchSet(file_options);

    po::options_description* processing_options = new po::options_description("processing options");
    
    processing_options->add_options()

        ;
    
    addSwitchSet(processing_options);

    addPositionalSwitch("input", 1);

    return;
}



int PcQuery::execute()
{


    Options readerOptions;
    {
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    Stage* reader = AppSupport::makeReader(readerOptions);

    

    pdal::Options options = m_options + readerOptions;
    options.add<boost::uint32_t>("dimensions", 2);
    
    pdal::filters::Index* filter = new pdal::filters::Index(*reader, options);

    filter->initialize();

    boost::uint32_t chunkSize(pdal::Writer::s_defaultChunkSize);
    if (filter->getNumPoints() > 0 && m_chunkSize == 0)
    {
        chunkSize = filter->getNumPoints();
    } 
    else if (m_chunkSize > 0)
    {
        chunkSize = m_chunkSize;
    }    
    PointBuffer data(filter->getSchema(), chunkSize);
    StageSequentialIterator* iter = filter->createSequentialIterator(data);

    readPoints(iter, data);

    pdal::filters::iterators::sequential::Index* idx = dynamic_cast<pdal::filters::iterators::sequential::Index*>(iter);
    if (!idx)
    {
        throw app_runtime_error("unable to cast iterator to Index iterator!");
    }
    
    idx->build();
    
    if (m_point.size())
    {
        boost::char_separator<char> sep(SEPARATORS);
        tokenizer tokens(m_point, sep);
        std::vector<double> values;
        for (tokenizer::iterator t = tokens.begin(); t != tokens.end(); ++t) {
            values.push_back(boost::lexical_cast<double>(*t));
        }
        
        if (values.size() < 2)
            throw app_runtime_error("--points must be two or three values");
        double x = values[0];
        double y = values[1];
        
        double z(0.0);
        if (values.size() > 2)
            z = values[2];
        std::vector<boost::uint32_t> ids = idx->query(x, y, z, 0.0, 1);
        
        if (ids.size())
        {
            PointBuffer data(reader->getSchema(), 1);
            StageRandomIterator* iterator = reader->createRandomIterator(data);
            iterator->seek(ids[0]);

            Schema const& schema = data.getSchema();
            Dimension const& dimX = schema.getDimension("X");
            Dimension const& dimY = schema.getDimension("Y");
            Dimension const& dimZ = schema.getDimension("Z");
            iterator->read(data);
            boost::int32_t xi = data.getField<boost::int32_t>(dimX, 0);
            boost::int32_t yi = data.getField<boost::int32_t>(dimY, 0);
            boost::int32_t zi = data.getField<boost::int32_t>(dimZ, 0);            
            double x = dimX.applyScaling<boost::int32_t>(xi);
            double y = dimY.applyScaling<boost::int32_t>(yi);
            double z = dimZ.applyScaling<boost::int32_t>(zi);
            std::cout.precision(8);
            std::cout << x << "," << y << "," << z << std::endl;
        }
        else
        {
            throw app_runtime_error("Candidate point not found!");
            
        }

    }

    std::cout << std::endl;
    
    delete filter;
    delete reader;


    
    return 0;
}

void PcQuery::readPoints(   StageSequentialIterator* iter,
                            PointBuffer& data)
{
    const Schema& schema = data.getSchema();
    
    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");

    boost::uint64_t id = 0;
    while (!iter->atEnd())
    {
        const boost::uint32_t numRead = iter->read(data);
        
        
    }

}

int main(int argc, char* argv[])
{
    PcQuery app(argc, argv);
    return app.run();
}

