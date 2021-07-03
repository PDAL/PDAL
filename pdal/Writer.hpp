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

#pragma once

#include <memory>

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>

namespace pdal
{

class UserCallback;

namespace expr
{
    class ConditionalExpression;
}

/**
  A Writer is a terminal stage for a PDAL pipeline.  It usually writes output
  to a file, but this isn't a requirement.  The class provides support for
  some operations common for producing point output.
*/
class PDAL_DLL Writer : public virtual Stage
{
    friend class WriterWrapper;
    friend class DbWriter;
    friend class FlexWriter;

    struct Args;

public:
    Writer();
    ~Writer();
    Writer& operator=(const Writer&) = delete;
    Writer(const Writer&) = delete;

    /**
      Locate template placeholder ('#') and validate filename with respect
      to placeholder.
    */
    static std::string::size_type
        handleFilenameTemplate(const std::string& filename);

private:
    virtual PointViewSet run(PointViewPtr view)
    {
        PointViewSet viewSet;
        write(view);
        viewSet.insert(view);
        return viewSet;
    }
    virtual void l_addArgs(ProgramArgs& args) final;
    virtual void l_initialize(PointTableRef table);
    virtual void l_prepared(PointTableRef table) final;

    virtual const expr::ConditionalExpression *whereExpr() const;
    virtual WhereMergeMode mergeMode() const;
    /**
      Write the point in a PointView.  This is a simplification of the
      \ref run() interface for convenience.  Impelment in subclass if desired.
    */
    virtual void write(const PointViewPtr /*view*/)
        { std::cerr << "Can't write with stage = " << getName() << "!\n"; }

    std::unique_ptr<Args> m_args;
};

} // namespace pdal

