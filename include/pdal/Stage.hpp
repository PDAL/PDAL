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

#ifndef INCLUDED_STAGE_HPP
#define INCLUDED_STAGE_HPP

#include <pdal/pdal_internal.hpp>

#include <pdal/StageBase.hpp>
#include <pdal/Schema.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/SpatialReference.hpp>

namespace pdal
{

class Iterator;
class StageSequentialIterator;
class StageRandomIterator;
class StageBlockIterator;
class PointBuffer;
//
// supported options:
//   <uint32>id
//   <bool>debug
//   <uint32>verbose
//

class PDAL_DLL Stage : public StageBase
{
public:
    Stage(const std::vector<StageBase*>& prevs, const Options& options);
    virtual ~Stage();

    virtual void initialize();

    inline Schema const& getSchema() const
    {
        return m_schema;
    }
    
    virtual boost::uint64_t getNumPoints() const;
    PointCountType getPointCountType() const;
    const Bounds<double>& getBounds() const;
    const SpatialReference& getSpatialReference() const;

    virtual bool supportsIterator(StageIteratorType) const = 0;

    virtual StageSequentialIterator* createSequentialIterator(PointBuffer&) const
    {
        return NULL;
    }
    virtual StageRandomIterator* createRandomIterator(PointBuffer&) const
    {
        return NULL;
    }

protected:
    // setters for the core properties
    Schema& getSchemaRef();


    void setSchema(Schema const&);
    void setNumPoints(boost::uint64_t);
    void setPointCountType(PointCountType);
    void setBounds(Bounds<double> const&);
    void setSpatialReference(SpatialReference const&);

    // convenience function, for doing a "copy ctor" on all the core props
    // (used by the Filter stage, for example)
    void setCoreProperties(const Stage&);

private:
    Schema m_schema;
    mutable boost::uint64_t m_numPoints;
    PointCountType m_pointCountType;
    Bounds<double> m_bounds;
    SpatialReference m_spatialReference;

    Stage& operator=(const Stage&); // not implemented
    Stage(const Stage&); // not implemented
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Stage&);

} // namespace pdal

#endif
