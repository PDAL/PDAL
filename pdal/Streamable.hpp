/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#pragma once

#include <pdal/pdal_internal.hpp>
#include <pdal/Stage.hpp>

namespace pdal
{

class StreamableWrapper;

class PDAL_EXPORT Streamable : public virtual Stage
{
    friend class StreamableWrapper;
public:
    Streamable();

    /**
      Execute a prepared pipeline (linked set of stages) in streaming mode.

      This performs the action associated with the stage by executing the
      \ref processOne function of each stage in depth first order.  Points
      are processed up to the capacity of the provided StreamPointTable.
      Not all stages support streaming mode and an exception will be thrown
      when attempting to \ref execute an unsupported stage.

      Streaming points can reduce memory consumption, but will limit access
      to algorithms that need to operate on full point sets.

      \param table  Streaming point table used for stage pipeline.  This must be
        the same \ref table used in the \ref prepare function.

    */
    virtual void execute(StreamPointTable& table);
    using Stage::execute;

    /**
      Determine if a pipeline is streamable.

      \return Whether the pipeline is streamable.
    */
    virtual bool pipelineStreamable() const;

protected:
    Streamable& operator=(const Streamable&) = delete;
    Streamable(const Streamable&); // not implemented

    using SrsMap = std::map<Streamable *, SpatialReference>;

    void execute(StreamPointTable& table, std::list<Streamable *>& stages,
        SrsMap& srsMap);

    /**
      Process a single point (streaming mode).  Implement in subclass.

      \param point  Point to process.
      \return  Readers return false when no more points are to be read.
        Filters return false if a point is to be filtered-out (not passed
        to subsequent stages).
    */
    virtual bool processOne(PointRef& /*point*/) = 0;
    /**
    {
        throwStreamingError();
        return false;
    }
    **/

    /**
      Notification that the points that will follow in processing are from
      a spatial reference different than the previous spatial reference.

       \param srs  New spatial reference.
    */
    virtual void spatialReferenceChanged(const SpatialReference& /*srs*/)
    {}

    /**
      Find the first nonstreamable stage in a pipeline.

      \return  NULL if the pipeline is streamable, otherwise return
        a pointer to the first found stage that's not streamable.
    */
    const Stage *findNonstreamable() const;
};

} // namespace pdal
