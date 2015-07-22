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

#include <pdal/Stage.hpp>
#include <pdal/Options.hpp>

namespace pdal
{

class Reader;

class PDAL_DLL Reader : public Stage
{
public:
    typedef std::function<void(PointView&, PointId)> PointReadFunc;

    Reader() : m_count(std::numeric_limits<point_count_t>::max())
    {}

    void setReadCb(PointReadFunc cb)
        { m_cb = cb; }

protected:
    std::string m_filename;
    point_count_t m_count;
    PointReadFunc m_cb;

private:
    virtual PointViewSet run(PointViewPtr view)
    {
        PointViewSet viewSet;

        view->clearTemps();
        read(view, m_count);
        viewSet.insert(view);
        return viewSet;
    }
    virtual void readerProcessOptions(const Options& options);
    virtual point_count_t read(PointViewPtr /*view*/, point_count_t /*num*/)
        { return 0; }
    virtual boost::property_tree::ptree serializePipeline() const;
};

} // namespace pdal

