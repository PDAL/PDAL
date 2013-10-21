/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#ifndef INCLUDED_PDAL_KERNEL_QUERY_HPP
#define INCLUDED_PDAL_KERNEL_QUERY_HPP

#include <pdal/Stage.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "Application.hpp"

namespace pdal { namespace kernel {
    
class PDAL_DLL Point
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

class Query : public Application
{
public:
    Query(int argc, const char* argv[]);
    int execute(); // overrride
    
    
private:
    void addSwitches(); // overrride
    void validateSwitches(); // overrride
    
    void readPoints(    StageSequentialIterator* iter,
                        PointBuffer& data);    
    std::string m_sourceFile;
    std::string m_candidateFile;
    std::string m_wkt;

    std::ostream* m_outputStream;
    std::string m_outputFileName;
	
    bool m_3d;
};

}} // pdal::kernel

#endif