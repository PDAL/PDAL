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
    
    bool equal(Point const& other)
    {
        return (Utils::compare_distance(x, other.x) && 
                Utils::compare_distance(y, other.y) && 
                Utils::compare_distance(z, other.z));
        
    }
    bool operator==(Point const& other)
    {
        return equal(other);
    }
    bool operator!=(Point const& other)
    {
        return !equal(other);
    }    
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
                                StageSequentialIterator* iter,
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
    if( m_chunkSize == 0)
        m_chunkSize = 1048576;     

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
                            StageSequentialIterator* iter,
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
            boost::int32_t xi = data.getField<boost::int32_t>(dimX, i);
            boost::int32_t yi = data.getField<boost::int32_t>(dimY, i);
            boost::int32_t zi = data.getField<boost::int32_t>(dimZ, i);
            
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

std::ostream& writeHeader(std::ostream& strm)
{
    strm << "\"ID\",\"DeltaX\",\"DeltaY\",\"DeltaZ\"" << std::endl;
    return strm;
    
}

int PcEqual::execute()
{

    std::vector<Point>* source_points = new std::vector<Point>();
    std::vector<Point>* candidate_points = new std::vector<Point>();
    
    {
        Options sourceOptions;
        {
            sourceOptions.add<std::string>("filename", m_sourceFile);
            sourceOptions.add<bool>("debug", isDebug());
            sourceOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
        }
        Stage* source = AppSupport::makeReader(sourceOptions);
        source->initialize();

        source_points->reserve(source->getNumPoints());

        PointBuffer source_data(source->getSchema(), m_chunkSize);
        StageSequentialIterator* reader_iter = source->createSequentialIterator(source_data);

        readPoints(source_points, reader_iter, source_data);

        delete reader_iter;
        delete source;
        
    }    

    Options candidateOptions;
    {
        candidateOptions.add<std::string>("filename", m_candidateFile);
        candidateOptions.add<bool>("debug", isDebug());
        candidateOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    Stage* candidate = AppSupport::makeReader(candidateOptions);
    pdal::filters::Index* index_filter = new pdal::filters::Index(*candidate, candidateOptions);
    index_filter->initialize();    


    candidate_points->reserve(candidate->getNumPoints());

    PointBuffer candidate_data(candidate->getSchema(), m_chunkSize);
    StageSequentialIterator* index_iter = index_filter->createSequentialIterator(candidate_data);
    readPoints(candidate_points, index_iter, candidate_data);



    if (candidate_points->size() != source_points->size())
    {
        throw app_runtime_error("Source and candidate files do not have the same point count!");
    }
    
    pdal::filters::iterators::sequential::Index* idx = dynamic_cast<pdal::filters::iterators::sequential::Index*>(index_iter);
    if (!idx)
    {
        throw app_runtime_error("unable to cast iterator to Index iterator!");
    }
    

    Stage* candidates = AppSupport::makeReader(candidateOptions);
    candidates->initialize();    

    PointBuffer candidates_data(candidates->getSchema(), 1);
    StageRandomIterator* random_iterator = candidates->createRandomIterator(candidates_data);
    
    
    Schema const& schema = candidates_data.getSchema();
    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");
    
    bool bWroteHeader(false);
    for (std::size_t i = 0; i <  source_points->size(); ++i)
    {
        Point& source = (*source_points)[i];
        
        std::vector<boost::uint32_t> ids = idx->query(source.x, source.y, source.z, 0.0, 1);

        if (ids.size())
            random_iterator->seek(ids[0]);
        else
            throw app_runtime_error("unable to find point for id" + i );
        
        random_iterator->read(candidates_data);
        boost::int32_t xi = candidates_data.getField<boost::int32_t>(dimX, 0);
        boost::int32_t yi = candidates_data.getField<boost::int32_t>(dimY, 0);
        boost::int32_t zi = candidates_data.getField<boost::int32_t>(dimZ, 0);
        
        Point p;
        p.x = dimX.applyScaling<boost::int32_t>(xi);
        p.y = dimY.applyScaling<boost::int32_t>(yi);
        p.z = dimZ.applyScaling<boost::int32_t>(zi);
        
        double xd = source.x - p.x;
        double yd = source.y - p.y;
        double zd = source.z - p.z;
        
        if (!bWroteHeader)
        {
            writeHeader(std::cout);
            bWroteHeader = true;
        }
        std::cout << i << ",";
        boost::uint32_t precision = Utils::getStreamPrecision(dimX.getNumericScale());
        std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
        std::cout.precision(precision);
        std::cout << xd << ",";

        precision = Utils::getStreamPrecision(dimY.getNumericScale());
        std::cout.precision(precision);
        std::cout << yd << ",";

        precision = Utils::getStreamPrecision(dimZ.getNumericScale());
        std::cout.precision(precision);
        std::cout << zd;
        
        std::cout << std::endl;

        
        
    }

    delete index_iter;
    delete index_filter;    
    delete candidate_points;
    delete source_points;
    
    return 0;
}


int main(int argc, char* argv[])
{
    PcEqual app(argc, argv);
    return app.run();
}

