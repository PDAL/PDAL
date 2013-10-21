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

#include <pdal/kernel/Query.hpp>

namespace pdal { namespace kernel {
    
Query::Query(int argc, const char* argv[])
    : Application(argc, argv, "query")
    , m_sourceFile("")
    , m_candidateFile("")
    , m_outputStream(0)
    , m_outputFileName("")
    , m_3d(true)
{
    return;
}


void Query::validateSwitches()
{
    m_chunkSize = 1048576;     

    return;
}


void Query::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");
    

    file_options->add_options()
        ("source", po::value<std::string>(&m_sourceFile), "source file name")
        ("candidate", po::value<std::string>(&m_candidateFile), "candidate file name")
        ("output", po::value<std::string>(&m_outputFileName), "output file name")
        ("2d", po::value<bool>(&m_3d)->zero_tokens()->implicit_value(false), "only 2D comparisons/indexing")
        ;

    addSwitchSet(file_options);

    po::options_description* processing_options = new po::options_description("processing options");
    
    processing_options->add_options()

        ;
    
    addSwitchSet(processing_options);

    addPositionalSwitch("source", 1);
    addPositionalSwitch("candidate", 2);
    addPositionalSwitch("output", 3);

    return;
}



void Query::readPoints(   StageSequentialIterator* iter,
                            PointBuffer& data)
{
    while (!iter->atEnd())
    {
        const boost::uint32_t numRead = iter->read(data);
    }

}

std::ostream& writeHeader(std::ostream& strm, bool b3D)
{
    strm << "\"ID\",\"DeltaX\",\"DeltaY\"";
    if (b3D)
        strm << ",\"DeltaZ\"";
    strm << std::endl;
    return strm;
    
}

int Query::execute()
{

    Options sourceOptions;
    {
        sourceOptions.add<std::string>("filename", m_sourceFile);
        sourceOptions.add<bool>("debug", isDebug());
        sourceOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }
    Stage* source = AppSupport::makeReader(sourceOptions);
    source->initialize();

    PointBuffer source_data(source->getSchema(), m_chunkSize);
    StageSequentialIterator* source_iter = source->createSequentialIterator(source_data);

    readPoints(source_iter, source_data);

    delete source_iter;
    delete source;



    Options candidateOptions;
    {
        candidateOptions.add<std::string>("filename", m_candidateFile);
        candidateOptions.add<bool>("debug", isDebug());
        candidateOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    Stage* candidate = AppSupport::makeReader(candidateOptions);
    // pdal::filters::Index* index_filter = new pdal::filters::Index(*candidate, candidateOptions);
    candidate->initialize();    


    IndexedPointBuffer candidate_data(candidate->getSchema(), m_chunkSize);
    StageSequentialIterator* candidate_iter = candidate->createSequentialIterator(candidate_data);
    readPoints(candidate_iter, candidate_data);
    delete candidate_iter;    


    if (source_data.getNumPoints() != candidate_data.getNumPoints())
    {
        std::cerr << "Source and candidate files do not have the same point count, testing each source point only!";
    }
    

    
    Schema const& candidate_schema = candidate_data.getSchema();
    Dimension const& cDimX = candidate_schema.getDimension("X");
    Dimension const& cDimY = candidate_schema.getDimension("Y");
    Dimension const& cDimZ = candidate_schema.getDimension("Z");

    Schema const& source_schema = source_data.getSchema();
    Dimension const& sDimX = source_schema.getDimension("X");
    Dimension const& sDimY = source_schema.getDimension("Y");
    Dimension const& sDimZ = source_schema.getDimension("Z");
    
    bool bWroteHeader(false);
    
    if (m_outputFileName.size())
    {
        m_outputStream = FileUtils::createFile(m_outputFileName);
    }
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;
    
    candidate_data.build(m_3d);
    for (boost::uint32_t i = 0; i < source_data.getNumPoints(); ++i)
    {
        double sx = source_data.applyScaling(sDimX, i);
        double sy = source_data.applyScaling(sDimY, i);
        double sz = source_data.applyScaling(sDimZ, i);                
        
        std::vector<std::size_t> ids = candidate_data.neighbors(sx, sy, sz, 1);

        if (!ids.size())
        {
			std::ostringstream oss;
			oss << "unable to find point for id '"  << i <<"'";
            throw app_runtime_error(oss.str() );
		}
        
        std::size_t id = ids[0];
        double cx = candidate_data.applyScaling(cDimX, id);
        double cy = candidate_data.applyScaling(cDimY, id);
        double cz = candidate_data.applyScaling(cDimZ, id);

        double xd = sx - cx;
        double yd = sy - cy;
        double zd = sz - cz;
        
        if (!bWroteHeader)
        {
            writeHeader(ostr, m_3d);
            bWroteHeader = true;
        }
        ostr << i << ",";
        boost::uint32_t precision = Utils::getStreamPrecision(cDimX.getNumericScale());
        ostr.setf(std::ios_base::fixed, std::ios_base::floatfield);
        ostr.precision(precision);
        ostr << xd << ",";

        precision = Utils::getStreamPrecision(cDimY.getNumericScale());
        ostr.precision(precision);
        ostr << yd;
        
        if (m_3d)
        {
            ostr << ",";
            precision = Utils::getStreamPrecision(cDimZ.getNumericScale());
            ostr.precision(precision);
            ostr << zd;
        }
        
        ostr << std::endl;

    }



    if (m_outputStream)
    {
        FileUtils::closeFile(m_outputStream);
    }
    
    return 0;
}

}} // pdal::kernel
