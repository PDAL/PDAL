/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include "StageRunner.hpp"

#include <pdal/Filter.hpp>

namespace pdal
{

StageRunner::StageRunner(Stage *s, PointViewPtr view) : m_stage(s)
{
    m_keeps = view->makeNew();
    m_skips = view->makeNew();
    m_stage->splitView(view, m_keeps, m_skips);
}

PointViewPtr StageRunner::keeps()
{
    return m_keeps;
}

// For now this is all synchronous
void StageRunner::run()
{
    point_count_t keepSize = m_keeps->size();
    m_viewSet = m_stage->run(m_keeps);

    if (m_skips->size() == 0)
        return;

    if (m_stage->mergeMode() == Filter::WhereMergeMode::True)
    {
        if (m_viewSet.size())
            (*m_viewSet.begin())->append(*m_skips);
                return;
    }
    else if (m_stage->mergeMode() == Filter::WhereMergeMode::Auto)
    {
        if (m_viewSet.size() == 1)
        {
            PointViewPtr keeps = *m_viewSet.begin();
            if (keeps.get() == m_keeps.get() && keepSize == keeps->size())
            {
                keeps->append(*m_skips);
                return;
            }
        }
    }
    m_viewSet.insert(m_skips);
}

PointViewSet StageRunner::wait()
{
    return m_viewSet;
}

} // namespace pdal

