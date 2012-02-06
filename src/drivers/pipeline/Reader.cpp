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

#include <pdal/drivers/pipeline/Reader.hpp>

#include <pdal/PipelineReader.hpp>


namespace pdal { namespace drivers { namespace pipeline {


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_filename(options.getValueOrThrow<std::string>("filename"))
    , m_manager(NULL)
{
    return;
}


Reader::~Reader()
{
    m_manager.reset();
    return;
}


void Reader::initialize()
{
    pdal::Reader::initialize();

    boost::scoped_ptr<PipelineManager> tmp(new PipelineManager());
    m_manager.swap(tmp);

    PipelineReader xmlreader(*m_manager);
    bool isWriter = xmlreader.readPipeline(m_filename);
    if (isWriter)
    {
        throw pdal_error("pipeline file is not a Reader");
    }
    m_stage = m_manager->getStage();
    m_stage->initialize();

    setSchema( m_stage->getSchema() );

    setNumPoints(m_stage->getNumPoints());
    setPointCountType(m_stage->getPointCountType());
    setBounds(m_stage->getBounds());

    return;
}


const Options Reader::getDefaultOptions() const
{
    Options options;
    return options;
}

const SpatialReference& Reader::getSpatialReference() const
{
    return m_stage->getSpatialReference();
}


bool Reader::supportsIterator (StageIteratorType t) const
{   
    return m_stage->supportsIterator(t);
}


pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return m_stage->createSequentialIterator();
}


pdal::StageRandomIterator* Reader::createRandomIterator() const
{
    return m_stage->createRandomIterator();
}

void Reader::addDefaultDimensions()
{

}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    tree.add("filename", m_filename);

    return tree;
}

} } } // namespaces
