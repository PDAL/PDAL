/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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

#include <pdal/kernel/Diff.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <boost/scoped_ptr.hpp>

using boost::property_tree::ptree;

namespace pdal { namespace kernel {
    
Diff::Diff(int argc, const char* argv[])
    : Application(argc, argv, "dif")
    , m_sourceFile("")
    , m_candidateFile("")
{
    return;
}


void Diff::validateSwitches()
{
  
    
    if (!m_sourceFile.size())
        throw app_runtime_error("No source file given!");
    if (!m_candidateFile.size())
        throw app_runtime_error("No candidate file given!");
        
    return;
}


void Diff::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");
    

    file_options->add_options()
        ("source", po::value<std::string>(&m_sourceFile), "source file name")
        ("candidate", po::value<std::string>(&m_candidateFile), "candidate file name")
        ("xml", po::value<bool>(&m_useXML)->zero_tokens()->implicit_value(true), "dump XML")
        ("json", po::value<bool>(&m_useJSON)->zero_tokens()->implicit_value(true), "dump JSON")

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



void Diff::checkPoints(  StageSequentialIterator* source_iter,
                         PointBuffer& source_data,
                         StageSequentialIterator* candidate_iter,
                         PointBuffer& candidate_data,
                         ptree& errors)
{

    boost::uint32_t i(0);
    boost::uint32_t chunk(0);    
    boost::uint32_t MAX_BADBYTES(20);
    boost::uint32_t badbytes(0);
    while (!source_iter->atEnd())
    {
        const boost::uint32_t numSrcRead = source_iter->read(source_data);
        const boost::uint32_t numCandidateRead = candidate_iter->read(candidate_data);
        if (numSrcRead != numCandidateRead)
        {
            std::ostringstream oss;
        
            oss << "Unable to read same number of points for chunk number";
            errors.put<std::string>("points.error", oss.str());
            errors.put<boost::uint32_t>("points.candidate" , numCandidateRead);
            errors.put<boost::uint32_t>("points.source" , numSrcRead);         
        }
        
        chunk++;
        
        pdal::pointbuffer::PointBufferByteSize source_byte_length(0);
        pdal::pointbuffer::PointBufferByteSize candidate_byte_length(0);
        source_byte_length =  static_cast<pdal::pointbuffer::PointBufferByteSize>(source_data.getSchema().getByteSize()) * 
                              static_cast<pdal::pointbuffer::PointBufferByteSize>(source_data.getNumPoints());

        candidate_byte_length =  static_cast<pdal::pointbuffer::PointBufferByteSize>(candidate_data.getSchema().getByteSize()) * 
                                 static_cast<pdal::pointbuffer::PointBufferByteSize>(candidate_data.getNumPoints());
        
        if (source_byte_length != candidate_byte_length)
        {
            std::ostringstream oss;
        
            oss << "Source byte length != candidate byte length";
            errors.put<std::string>("buffer.error", oss.str());
            errors.put<boost::uint32_t>("buffer.candidate" , candidate_byte_length);
            errors.put<boost::uint32_t>("buffer.source" , source_byte_length);            
        }
        boost::uint8_t* s = source_data.getData(0);
        boost::uint8_t* c = candidate_data.getData(0);

        for (boost::uint32_t p = 0; p < std::min(source_byte_length, candidate_byte_length); ++p)
        {
            if (s[p] != c[p])
            {
                std::ostringstream oss;
        
                oss << "Byte number " << p << " is not equal for source and candidate";
                errors.put<std::string>("data.error", oss.str());
                badbytes++;                        
            }
            
        }
        
        if (badbytes > MAX_BADBYTES )
            break;
        
    }

}


int Diff::execute()
{
    PointContext sourceCtx;

    Options sourceOptions;
    {
        sourceOptions.add<std::string>("filename", m_sourceFile);
        sourceOptions.add<bool>("debug", isDebug());
        sourceOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }
    boost::scoped_ptr<Stage> source(AppSupport::makeReader(sourceOptions));
    source->prepare(sourceCtx);
    
    boost::uint32_t chunkSize(source->getNumPoints());
    if (m_chunkSize)
        chunkSize = m_chunkSize;
    PointBuffer source_data(source->getSchema(), chunkSize);
    boost::scoped_ptr<StageSequentialIterator> source_iter(source->createSequentialIterator(source_data));
    
    ptree errors;




    PointContext candidateCtx;
    Options candidateOptions;
    {
        candidateOptions.add<std::string>("filename", m_candidateFile);
        candidateOptions.add<bool>("debug", isDebug());
        candidateOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    boost::scoped_ptr<Stage> candidate(AppSupport::makeReader(candidateOptions));
    candidate->prepare(candidateCtx);


    PointBuffer candidate_data(candidate->getSchema(), chunkSize);
    boost::scoped_ptr<StageSequentialIterator> candidate_iter(candidate->createSequentialIterator(candidate_data));

    if (candidate->getNumPoints() != source->getNumPoints())
    {
        std::ostringstream oss;
        
        oss << "Source and candidate files do not have the same point count";
        errors.put<std::string>("count.error", oss.str());
        errors.put<boost::uint32_t>("count.candidate" , candidate->getNumPoints());
        errors.put<boost::uint32_t>("count.source" , source->getNumPoints());
        
    }
    
    pdal::Metadata source_metadata = source->collectMetadata();
    pdal::Metadata candidate_metadata = candidate->collectMetadata();
    
    if (source_metadata != candidate_metadata)
    {
        std::ostringstream oss;
        
        oss << "Source and candidate files do not have the same metadata count";
        errors.put<std::string>("metadata.error", oss.str());
        errors.put_child("metadata.source", source_metadata.toPTree());
        errors.put_child("metadata.candidate", candidate_metadata.toPTree());
    }

    
    Schema const& candidate_schema = candidate_data.getSchema();


    Schema const& source_schema = source_data.getSchema();
    
    if (! ( candidate_schema == source_schema))
    {
        std::ostringstream oss;
        
        oss << "Source and candidate files do not have the same schema";
        errors.put<std::string>("schema.error", oss.str());
        errors.put_child("schema.source", source_schema.toPTree());
        errors.put_child("schema.candidate", candidate_schema.toPTree());

    }

    

    if (errors.size())
    {
        write_json(std::cout, errors);
        return 1;
    } else
    {
        // If we made it this far with no errors, now we'll 
        // check the points.
        checkPoints(source_iter.get(), 
                    source_data, 
                    candidate_iter.get(), 
                    candidate_data, 
                    errors);
        if (errors.size())
        {
            write_json(std::cout, errors);
            return 1;
        }
    }

    return 0;

}

}} // pdal::kernel
