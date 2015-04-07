/******************************************************************************
* Copyright (c) 2013, Andrew Bell (andrew.bell.ia@gmail.com)
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

#include <pdal/Filter.hpp>

#include <hexer/Mathpair.hpp>
#include <hexer/HexGrid.hpp>
#include <hexer/Processor.hpp>

namespace pdal
{

class PDAL_DLL HexBin : public Filter
{
public:
    HexBin() : Filter()
        {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const { return "filters.hexbin"; }

private:

    std::unique_ptr<hexer::HexGrid> m_grid;
    std::string m_xDimName;
    std::string m_yDimName;
    uint32_t m_sampleSize;
    int32_t m_density;
    double m_edgeLength;
    bool m_outputTesselation;

    virtual void processOptions(const Options& options);
    virtual void ready(PointTableRef table);
    virtual void filter(PointView& view);
    virtual void done(PointTableRef table);

    HexBin& operator=(const HexBin&); // not implemented
    HexBin(const HexBin&); // not implemented
};

} // namespace pdal
