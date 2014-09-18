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

#include <pdal/pdal_internal.hpp>
#include <pdal/Options.hpp>
#include <pdal/UserCallback.hpp>
#include <pdal/Stage.hpp>

#include <string>

namespace pdal
{

class PointBuffer;
class UserCallback;

/// End-stage consumer of PDAL pipeline
class PDAL_DLL Writer : public Stage
{
public:
    /// Constructs an end-stage consumer of a pipeline of data -- a writer
    /// @param options options to be passed into the writer.
    Writer(Options const& options) : Stage(options),
            m_callback(new UserCallback)
        {}

    Writer() : m_callback(new UserCallback)
        {}
    
    /// Serialize the pipeline to a boost::property_tree::ptree
    /// @return boost::property_tree::ptree with xml attributes
    virtual boost::property_tree::ptree serializePipeline() const;
    
    /// Sets the UserCallback to manage progress/cancel operations
    void setUserCallback(UserCallback* userCallback)
        { m_callback.reset(userCallback); }

protected:
    std::unique_ptr<UserCallback> m_callback;
    XForm m_xXform;
    XForm m_yXform;
    XForm m_zXform;

private:
    virtual PointBufferSet run(PointBufferPtr buffer)
    {
        PointBufferSet pbSet;
        write(*buffer);
        pbSet.insert(buffer);
        return pbSet;
    }
    virtual void writerProcessOptions(const Options& options);
    virtual void write(const PointBuffer& /*buffer*/)
        { std::cerr << "Can't write with stage = " << getName() << "!\n"; }

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented
};

} // namespace pdal

