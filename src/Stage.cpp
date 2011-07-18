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

#include <pdal/Stage.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <iostream>

#include <pdal/exceptions.hpp>


namespace pdal
{

//--------------------------------------------------------------------------------

Stage::Stage(const Options& options)
    : m_options(options)
{
    return;
}


Stage::Stage(const DataStagePtr& prev, const Options& options)
    : m_options(options)
{
    m_prevStages.push_back(prev);
    return;
}


Stage::Stage(const std::vector<const DataStagePtr>& prevs, const Options& options)
    : m_options(options)
{
    for (boost::uint32_t i=0; i<prevs.size(); i++)
        m_prevStages.push_back(prevs[i]);
    return;
}


Stage::~Stage()
{
    return;
}

const Options& Stage::getOptions() const
{
    return m_options;
}


Options& Stage::getOptions()
{
    return m_options;
}


DataStagePtr Stage::getPrevStage() const
{
    if (m_prevStages.size() == 0)
        throw pdal_error("internal error - no prev stages");
    DataStagePtr ptr = m_prevStages[0];
    return ptr;
}


const std::vector<const DataStagePtr>& Stage::getPrevStages() const
{
    return m_prevStages;
}


//--------------------------------------------------------------------------------


DataStage::DataStage(const Options& options)
    : Stage(options)
    , m_numPoints(0)
    , m_pointCountType(PointCount_Fixed)
{
    return;
}


DataStage::DataStage(const DataStagePtr& prev, const Options& options)
    : Stage(prev, options)
    , m_numPoints(0)
    , m_pointCountType(PointCount_Fixed)
{
    return;
}


DataStage::DataStage(const std::vector<const DataStagePtr>& prevs, const Options& options)
    : Stage(prevs, options)
    , m_numPoints(0)
    , m_pointCountType(PointCount_Fixed)
{
    return;
}


DataStage::~DataStage()
{
    return;
}


const Bounds<double>& DataStage::getBounds() const
{
    return m_bounds;
}


void DataStage::setBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


const Schema& DataStage::getSchema() const
{
    return m_schema;
}


Schema& DataStage::getSchemaRef()
{
    return m_schema;
}


void DataStage::setSchema(const Schema& schema)
{
    m_schema = schema;
}


boost::uint64_t DataStage::getNumPoints() const
{
    return m_numPoints;
}


void DataStage::setNumPoints(boost::uint64_t numPoints)
{
    m_numPoints = numPoints;
}


PointCountType DataStage::getPointCountType() const
{
    return m_pointCountType;
}


void DataStage::setPointCountType(PointCountType pointCountType)
{
    m_pointCountType = pointCountType;
}


const SpatialReference& DataStage::getSpatialReference() const
{
    return m_spatialReference;
}


void DataStage::setSpatialReference(const SpatialReference& spatialReference)
{
    m_spatialReference = spatialReference;
}


int DataStage::getMetadataRecordCount() const
{
    return 0;
}


const MetadataRecord& DataStage::getMetadataRecord(int index) const
{
    // the default behaviour is to have no records at all...
    boost::ignore_unused_variable_warning(index);
    throw pdal_error("no such metadata record");
}


MetadataRecord& DataStage::getMetadataRecordRef(int index)
{
    // the default behaviour is to have no records at all...
    boost::ignore_unused_variable_warning(index);
    throw pdal_error("no such metadata record");
}


void DataStage::setCoreProperties(const DataStagePtr& stage)
{
    this->setSchema(stage->getSchema());
    this->setNumPoints(stage->getNumPoints());
    this->setPointCountType(stage->getPointCountType());
    this->setBounds(stage->getBounds());
    this->setSpatialReference(stage->getSpatialReference());

    return;
}


void DataStage::dump() const
{
    std::cout << *this;
}

std::ostream& operator<<(std::ostream& ostr, const Stage& stage)
{
    ostr << "  Name: " << stage.getName() << std::endl;
//    ostr << "  Num points: " << stage.getNumPoints() << std::endl;

//    ostr << "  Bounds:" << std::endl;
//    ostr << "    " << stage.getBounds() << std::endl;

//    ostr << "  Schema: " << std::endl;
//    ostr << "    Num dims: " << stage.getSchema().getDimensions().size() << std::endl;
//    ostr << "    Size in bytes: " << header.getSchema().getByteSize() << std::endl;

//    ostr << "  Spatial Reference:" << std::endl;
//    ostr << "    WKT: " << stage.getSpatialReference().getWKT() << std::endl;

    return ostr;
}


} // namespace pdal
