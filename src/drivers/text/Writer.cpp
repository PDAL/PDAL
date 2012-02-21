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

#include <pdal/drivers/text/Writer.hpp>
#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <algorithm>

#include <boost/algorithm/string.hpp>


PDAL_C_START

PDAL_DLL void PDALRegister_writer_text(void* factory)
{
    pdal::StageFactory& f = *(pdal::StageFactory*) factory;
    f.registerWriter(pdal::drivers::text::Writer::s_getName(), createTextWriter);
}

PDAL_C_END

pdal::Writer* createTextWriter(pdal::Stage& prevStage, const pdal::Options& options)
{
    return new pdal::drivers::text::Writer(prevStage, options);
}


namespace pdal { namespace drivers { namespace text {


struct FileStreamDeleter
{

       template <typename T>
       void operator()(T* ptr)
	{
		ptr->flush();
	    FileUtils::closeFile(ptr);
	}
};

Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
	, m_wrote_header(false)

{

    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();
	
	std::string filename = getOptions().getValueOrThrow<std::string>("filename");
	
	// This is so the stream gets closed down if we throw any sort of 
	// exception
	m_stream = FileStreamPtr(FileUtils::createFile(filename, true), FileStreamDeleter());
	
    return;
}



const Options Writer::getDefaultOptions() const
{
    Options options;
    
    Option delimiter("delimiter", ",", "Delimiter to use for writing text");
    Option newline("newline", "\n", "Newline character to use for additional lines");
    Option quote_header("quote_header", true, "Write dimension names in quotes");
	Option filename("filename", "", "Filename to write CSV file to");
	
	
	options.add(filename);
	options.add(delimiter);
	options.add(newline);
	options.add(quote_header);
	
    return options;
}


void Writer::writeBegin(boost::uint64_t targetNumPointsToWrite)
{

    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
   	m_stream.reset();
	m_stream = FileStreamPtr();
	m_wrote_header = false;
    return;
}

void Writer::WriteHeader(pdal::Schema const& schema)
{

    schema::index_by_index const& dims = schema.getDimensions().get<schema::index>(); 
	
	bool isQuoted = getOptions().getValueOrDefault<bool>("quote_header", true);
	std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
	std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");
	
    schema::index_by_index::const_iterator iter = dims.begin();
	while (iter != dims.end())
	{
		if (iter->isIgnored())
			continue;
		if (isQuoted)
			*m_stream << "\"";
		*m_stream << iter->getName();
		if (isQuoted)
			*m_stream<< "\"";
		iter++;
		if (iter != dims.end())
			*m_stream << delimiter;
	}
	*m_stream << newline;
    
    return;
}

std::string Writer::getStringRepresentation( PointBuffer const& data, 
                                    		 Dimension const& d, 
                                    		 std::size_t pointIndex) const
{
	std::ostringstream output;
        
    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);
    
    boost::uint32_t size = d.getByteSize();

	bool bHaveScaling = !Utils::compare_distance(d.getNumericScale(), 0.0);
	
	if (bHaveScaling)
	{
		output.setf(std::ios::fixed, std::ios::floatfield);
		output.precision(Utils::getStreamPrecision(d.getNumericScale()));
	}
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = data.getField<float>(d, pointIndex);
                output << static_cast<double>(flt);                
            }
            if (size == 8)
            {
                output << data.getField<double>(d, pointIndex);
            }
            break;

        case dimension::SignedInteger:
        case dimension::SignedByte:
            if (size == 1)
            {
                i8 = data.getField<boost::int8_t>(d, pointIndex);
                output << d.applyScaling<boost::int8_t>(i8);
            }
            if (size == 2)
            {
                i16 = data.getField<boost::int16_t>(d, pointIndex);
                output << d.applyScaling<boost::int16_t>(i16);
            }
            if (size == 4)
            {
                i32 = data.getField<boost::int32_t>(d, pointIndex);
                output << d.applyScaling<boost::int32_t>(i32);
            }
            if (size == 8)
            {
                i64 = data.getField<boost::int64_t>(d, pointIndex);
                output << d.applyScaling<boost::int64_t>(i64);
            }
            break;
            
        case dimension::UnsignedInteger:
        case dimension::UnsignedByte:
            if (size == 1)
            {
                u8 = data.getField<boost::uint8_t>(d, pointIndex);
                output << d.applyScaling<boost::uint8_t>(u8);
            }
            if (size == 2)
            {
                u16 = data.getField<boost::uint16_t>(d, pointIndex);
                output << d.applyScaling<boost::uint16_t>(u16);
            }
            if (size == 4)
            {
                u32 = data.getField<boost::uint32_t>(d, pointIndex);
                output << d.applyScaling<boost::uint32_t>(u32);
            }
            if (size == 8)
            {
                u64 = data.getField<boost::uint64_t>(d, pointIndex);
                output << d.applyScaling<boost::uint64_t>(u64);
            }
            break;

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
			break;
    }    
    
    return output.str();
}



boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{
	
	if (!m_wrote_header)
	{
		WriteHeader(data.getSchema());
		m_wrote_header = true;
	}

	std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
	std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");

	boost::uint32_t pointIndex(0);
	
	pdal::Schema const& schema = data.getSchema();
	schema::index_by_index const& dims = schema.getDimensions().get<schema::index>(); 
	
	while (pointIndex != data.getNumPoints())
	{

	    schema::index_by_index::const_iterator iter = dims.begin();
		while (iter != dims.end())
		{
			if (iter->isIgnored())
				continue;
				
			*m_stream << getStringRepresentation(data, *iter, pointIndex);

			iter++;
			if (iter != dims.end())
				*m_stream << delimiter;
		}
		*m_stream << newline;
		
		pointIndex++;

	}

	
 
    return data.getNumPoints();
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}


} } } // namespaces
