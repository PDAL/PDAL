/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

class ProgramArgs;

/**
  Scaling provides support for transforming X/Y/Z values from double to
  scaled integers and vice versa.
*/
class PDAL_EXPORT Scaling
{
public:
    XForm m_xXform;          ///< X-dimension transform (scale/offset)
    XForm m_yXform;          ///< Y-dimension transform (scale/offset)
    XForm m_zXform;          ///< Z-dimension transform (scale/offset)
    Arg *m_xOffArg;
    Arg *m_yOffArg;
    Arg *m_zOffArg;
    Arg *m_xScaleArg;
    Arg *m_yScaleArg;
    Arg *m_zScaleArg;

    /**
       Determine if any of the transformations are non-standard.

       \return  Whether any transforms are non-standard.
    */
    bool nonstandard() const
    {
        return m_xXform.nonstandard() || m_yXform.nonstandard() ||
            m_zXform.nonstandard();
    }

    /**
      Compute an automatic scale/offset for points in the PointView.

      \param view  PointView on which scale should be computed.
    */
    virtual void setAutoXForm(const PointViewSet& pvSet);

    /**
      Add option/command-line arguments for transform variables.

      \param args  Argument set to add to.
    */
    void addArgs(ProgramArgs& args);
};

} // namespace pdal

