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

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <pdal/filters/Predicate.hpp>

#include <pdal/PointBuffer.hpp>

namespace pdal { namespace filters {


Predicate::Predicate(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    return;
}

void Predicate::initialize()
{
    Filter::initialize();
    
    m_expression = getOptions().getValueOrDefault<std::string>("expression", "") ;
    log()->get(logDEBUG)  << "expression " << m_expression << std::endl;

    assert(m_expression != "");

    return;
}


const Options Predicate::getDefaultOptions() const
{
    Options options;
    Option expression("expression", "");
    options.add(expression);
    return options;
}


boost::uint32_t Predicate::processBuffer(const PointBuffer& data, pdal::plang::PythonPDALMethod& parser) const
{
    //const Schema& schema = dstData.getSchema();
    //boost::uint32_t numSrcPoints = srcData.getNumPoints();
    //boost::uint32_t dstIndex = dstData.getNumPoints();
    //boost::uint32_t numPointsAdded = 0;

    //Dimension const& dimX = schema.getDimension("X");
    //Dimension const& dimY = schema.getDimension("Y");
    //Dimension const& dimZ = schema.getDimension("Z");
    //Dimension const& dimTime = schema.getDimension("Time");

    //for (boost::uint32_t srcIndex=0; srcIndex<numSrcPoints; srcIndex++)
    //{
    //    const double x = srcData.getField<double>(dimX, srcIndex);
    //    const double y = srcData.getField<double>(dimY, srcIndex);
    //    const double z = srcData.getField<double>(dimZ, srcIndex);
    //    const boost::uint64_t t = srcData.getField<boost::uint64_t>(dimTime, srcIndex);
    //    parser.setVariable("X", x);
    //    parser.setVariable("Y", y);
    //    parser.setVariable("Z", z);
    //    parser.setVariable("Time", t);
    //    bool ok = parser.evaluate();
    //    assert(ok);
    //    const bool predicate = parser.getVariable<bool>("result");

    //    if (predicate)
    //    {
    //        dstData.copyPointFast(dstIndex, srcIndex, srcData);
    //        dstData.setNumPoints(dstIndex+1);
    //        ++dstIndex;
    //        ++numPointsAdded;
    //    }
    //}

    //assert(dstIndex <= dstData.getCapacity());

    //return numPointsAdded;
    return 0;
}


pdal::StageSequentialIterator* Predicate::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Predicate(*this, buffer);
}


//---------------------------------------------------------------------------


namespace iterators { namespace sequential {

Predicate::Predicate(const pdal::filters::Predicate& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_predicateFilter(filter)
    , m_parser(NULL)
{
    return;
}


Predicate::~Predicate()
{
    delete m_parser;
}


void Predicate::createParser()
{
    const std::string expression = m_predicateFilter.getExpression();

    const std::string program = "float64 X; float64 Y; float64 Z; uint64 Time; bool result; result = " + expression + ";";

    ////m_parser = new pdal::plang::Parser(program);

    ////bool ok = m_parser->parse();
    ////assert(ok);

    return;
}


boost::uint32_t Predicate::readBufferImpl(PointBuffer& dstData)
{
    ////if (!m_parser)
    ////{
    ////    createParser();
    ////}

    ////// the following code taken from the Crop filter

    ////boost::uint32_t numPointsNeeded = dstData.getCapacity();
    ////assert(dstData.getNumPoints() == 0);

    ////while (numPointsNeeded > 0)
    ////{
    ////    if (getPrevIterator().atEnd()) break;

    ////    // set up buffer to be filled by prev stage
    ////    PointBuffer srcData(dstData.getSchema(), numPointsNeeded);

    ////    // read from prev stage
    ////    const boost::uint32_t numSrcPointsRead = getPrevIterator().read(srcData);
    ////    assert(numSrcPointsRead == srcData.getNumPoints());
    ////    assert(numSrcPointsRead <= numPointsNeeded);

    ////    // we got no data, and there is no more to get -- exit the loop
    ////    if (numSrcPointsRead == 0) break;

    ////    // copy points from src (prev stage) into dst (our stage), 
    ////    // based on the CropFilter's rules (i.e. its bounds)
    ////    const boost::uint32_t numPointsProcessed = m_predicateFilter.processBuffer(dstData, srcData, *m_parser);

    ////    numPointsNeeded -= numPointsProcessed;
    ////}

    ////const boost::uint32_t numPointsAchieved = dstData.getNumPoints();

    ////return numPointsAchieved;
    return 0;
}


boost::uint64_t Predicate::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Predicate::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

} } // iterators::sequential

} } // pdal::filters

#endif
