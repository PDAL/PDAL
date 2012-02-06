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

#ifndef INCLUDED_DRIVERS_PIPELINE_READER_HPP
#define INCLUDED_DRIVERS_PIPELINE_READER_HPP

#include <pdal/Reader.hpp>
#include <pdal/PipelineManager.hpp>
#include <boost/scoped_ptr.hpp>


namespace pdal { namespace drivers { namespace pipeline {


class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.pipeline.reader", "Pipeline Reader")

    Reader(const Options& options);
    ~Reader();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;
    virtual void addDefaultDimensions();
    
    bool supportsIterator (StageIteratorType t) const;
    pdal::StageSequentialIterator* createSequentialIterator() const;
    pdal::StageRandomIterator* createRandomIterator() const;

    // this is called by the stage's iterator
    boost::uint32_t processBuffer(PointBuffer& data, boost::uint64_t index) const;

    // for dumping
    virtual boost::property_tree::ptree toPTree() const;
    const SpatialReference& getSpatialReference() const;

    PipelineManager const& getManager() { return *m_manager.get(); }

private:
    std::string m_filename;
    boost::scoped_ptr<PipelineManager> m_manager;
    Stage* m_stage; // owned by m_manager

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented
};


} } } // namespaces


#endif
