/******************************************************************************
 * Copyright (c) 2017, Peter J. Gadomski <pete@gadom.ski>
 *
 * All rights reserved
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

namespace pdal
{

class PDAL_EXPORT CpdFilter : public Filter
{
  public:
    static std::string defaultMethod();

    CpdFilter() : Filter(), m_fixed(nullptr), m_method(""), m_complete(false)
    {
    }
    std::string getName() const;

    virtual PointViewSet run(PointViewPtr view);

  private:
    virtual void addArgs(ProgramArgs& args);
    virtual void done(PointTableRef _);

    PointViewPtr change(PointViewPtr fixed, PointViewPtr moving);
    void cpd_rigid(PointViewPtr fixed, PointViewPtr moving);
    void cpd_affine(PointViewPtr fixed, PointViewPtr moving);
    void cpd_nonrigid(PointViewPtr fixed, PointViewPtr moving);

    PointViewPtr m_fixed;
    std::string m_method;
    bool m_complete;

    CpdFilter& operator=(const CpdFilter&); // not implemented
    CpdFilter(const CpdFilter&);            // not implemented
};
} // namespace pdal
