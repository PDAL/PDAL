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


namespace pdal
{


Stage::Stage(const std::vector<StageBase*>& prevs, const Options& options)
    : StageBase(prevs, options)
    , m_numPoints(0)
    , m_pointCountType(PointCount_Fixed)
{
    return;
}


Stage::~Stage()
{
    return;
}


void Stage::initialize()
{
    StageBase::initialize();
    
    // Try to fetch overridden options here    
    
    // If the user gave us an SRS via options, take that.
    try 
    {
        m_spatialReference = getOptions().getValueOrThrow<pdal::SpatialReference>("spatialreference");
        
    }
    catch (pdal_error const&) 
    {
        // If one wasn't set on the options, we'll ignore at this 
        // point.  Maybe another stage might forward/set it later.
    }

    return;
}


const Bounds<double>& Stage::getBounds() const
{
    return m_bounds;
}


void Stage::setBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


const Schema& Stage::getSchema() const
{
    return m_schema;
}


Schema& Stage::getSchemaRef()
{
    return m_schema;
}


void Stage::setSchema(const Schema& schema)
{
    m_schema = schema;
}


boost::uint64_t Stage::getNumPoints() const
{
    return m_numPoints;
}


void Stage::setNumPoints(boost::uint64_t numPoints)
{
    m_numPoints = numPoints;
}


PointCountType Stage::getPointCountType() const
{
    return m_pointCountType;
}


void Stage::setPointCountType(PointCountType pointCountType)
{
    m_pointCountType = pointCountType;
}


const SpatialReference& Stage::getSpatialReference() const
{
    return m_spatialReference;
}


void Stage::setSpatialReference(const SpatialReference& spatialReference)
{
    m_spatialReference = spatialReference;
}

std::vector<Metadata> Stage::getMetadata() const
{
    // the default behaviour is to have no records at all...
    throw pdal_error("no such metadata record");
}

void Stage::setCoreProperties(const Stage& stage)
{
    this->setSchema(stage.getSchema());
    this->setNumPoints(stage.getNumPoints());
    this->setPointCountType(stage.getPointCountType());
    this->setBounds(stage.getBounds());
    this->setSpatialReference(stage.getSpatialReference());

    return;
}

void Stage::addDefaultDimension(Dimension const& dim, std::string const& nmspace)
{
    m_defaultDimensions.push_back(dim);
    std::vector<Dimension>::reverse_iterator r = m_defaultDimensions.rbegin();
    if (r == m_defaultDimensions.rend())
    {
        throw pdal_error("Unable to get last default dimension!");
    }
    r->setNamespace(nmspace);
}

void Stage::addDefaultDimensions()
{
}

boost::property_tree::ptree Stage::toPTree() const
{
    boost::property_tree::ptree tree = StageBase::toPTree();

    tree.add("NumPoints", getNumPoints());
    tree.add("PointCountType", getPointCountType());
    tree.add("Bounds", getBounds());
    tree.add("SRS", getSpatialReference());

    return tree;
}


std::ostream& operator<<(std::ostream& ostr, const Stage& stage)
{
    ostr << "  Name: " << stage.getName() << std::endl;
    ostr << "  Num points: " << stage.getNumPoints() << std::endl;

    ostr << "  Bounds:" << std::endl;
    ostr << "    " << stage.getBounds() << std::endl;

    ostr << "  Schema: " << std::endl;
    ostr << "    Num dims: " << stage.getSchema().getDimensions().size() << std::endl;
//    ostr << "    Size in bytes: " << header.getSchema().getByteSize() << std::endl;

    ostr << "  Spatial Reference:" << std::endl;
    ostr << "    WKT: " << stage.getSpatialReference().getWKT() << std::endl;

    return ostr;
}


} // namespace pdal
