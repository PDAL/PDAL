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



#include <map>
#include <vector>


using namespace pdal;

class Point
{
public:
    double x;
    double y;
    double z;
    boost::uint64_t id;
};

class PcEqual : public Application
{
public:
    PcEqual(int argc, char* argv[]);
    int execute(); // overrride
    
    
private:
    void addSwitches(); // overrride
    void validateSwitches(); // overrride
    
    void readPoints(std::vector<Point>* points,
                                boost::scoped_ptr<StageSequentialIterator>& iter,
                                PointBuffer& data);    
    std::string m_sourceFile;
    std::string m_candidateFile;
    std::string m_wkt;


	
    pdal::Options m_options;
};


PcEqual::PcEqual(int argc, char* argv[])
    : Application(argc, argv, "pcquery")
    , m_sourceFile("")
    , m_candidateFile("")
{
    return;
}


void PcEqual::validateSwitches()
{
    
    return;
}


void PcEqual::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");
    

    file_options->add_options()
        ("source", po::value<std::string>(&m_sourceFile), "source file name")
        ("candidate", po::value<std::string>(&m_candidateFile), "candidate file name")
        ;

    addSwitchSet(file_options);

    po::options_description* processing_options = new po::options_description("processing options");
    
    processing_options->add_options()

        ;
    
    addSwitchSet(processing_options);

    addPositionalSwitch("source", 1);
    addPositionalSwitch("candidate", 2);

    return;
}



void PcEqual::readPoints(   std::vector<Point>* points,
                            boost::scoped_ptr<StageSequentialIterator>& iter,
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
        
        for (boost::uint32_t i = 0; i < data.getNumPoints(); ++i)
        {
            boost::int32_t xi = data.getField<boost::uint32_t>(dimX, i);
            boost::int32_t yi = data.getField<boost::uint32_t>(dimY, i);
            boost::int32_t zi = data.getField<boost::uint32_t>(dimZ, i);
            
            Point p;
            p.x = dimX.applyScaling<boost::int32_t>(xi);
            p.y = dimY.applyScaling<boost::int32_t>(yi);
            p.z = dimZ.applyScaling<boost::int32_t>(zi);
            p.id = id;
            id += 1;
            
            points->push_back(p);
            
        }
        
    }

}

int PcEqual::execute()
{


    Options sourceOptions;
    {
        sourceOptions.add<std::string>("filename", m_sourceFile);
        sourceOptions.add<bool>("debug", isDebug());
        sourceOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    Stage* source = AppSupport::makeReader(sourceOptions);
    source->initialize();
    
    std::vector<Point>* source_points = new std::vector<Point>();
    source_points->reserve(source->getNumPoints());
    boost::uint32_t chunkSize = m_chunkSize != 0 ? m_chunkSize : 1048576; 
    PointBuffer source_data(source->getSchema(), chunkSize);
    boost::scoped_ptr<StageSequentialIterator> reader_iter(source->createSequentialIterator(source_data));

    readPoints(source_points, reader_iter, source_data);

    std::cout << "read " << source_points->size() << " source points" << std::endl;
    
    delete source;


    Options candidateOptions;
    {
        candidateOptions.add<std::string>("filename", m_candidateFile);
        candidateOptions.add<bool>("debug", isDebug());
        candidateOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    Stage* candidate = AppSupport::makeReader(candidateOptions);


    pdal::filters::Index* index_filter = new pdal::filters::Index(*candidate, candidateOptions);
    index_filter->initialize();    

    std::vector<Point>* candidate_points = new std::vector<Point>();
    candidate_points->reserve(candidate->getNumPoints());

    PointBuffer candidate_data(candidate->getSchema(), chunkSize);
    boost::scoped_ptr<StageSequentialIterator> index_iter(candidate->createSequentialIterator(candidate_data));
    readPoints(candidate_points, index_iter, candidate_data);

    std::cout << "read " << candidate_points->size() << " candidate points" << std::endl;
    
    delete index_filter;

    std::cout << std::endl;

    if (candidate_points->size() != source_points->size())
    {
        throw app_runtime_error("Source and candidate files do not have the same point count!");
    }
    
    
    delete candidate_points;
    delete source_points;
    
    return 0;
}


int main(int argc, char* argv[])
{
    PcEqual app(argc, argv);
    return app.run();
}

