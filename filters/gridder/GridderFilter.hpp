/******************************************************************************
 * Copyright (c) 2015, Oscar Martinez Rubi (o.rubi@esciencecenter.nl)
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

extern "C" int32_t GridderFilter_ExitFunc();
extern "C" PF_ExitFunc GridderFilter_InitPlugin();

namespace pdal
{

class PDAL_DLL GridderFilter : public pdal::Filter
{
public:
    GridderFilter() : Filter()
        {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    uint32_t num_x;
    uint32_t num_y;
    double min_x;
    double max_x;
    double min_y;
    double max_y;

    virtual void processOptions(const Options& options);
    virtual PointViewSet run(PointViewPtr view);

    GridderFilter& operator=(const GridderFilter&); // not implemented
    GridderFilter(const GridderFilter&); // not implemented
};

} // namespace pdal
