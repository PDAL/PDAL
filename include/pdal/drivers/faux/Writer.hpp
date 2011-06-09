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

#ifndef INCLUDED_DRIVERS_FAUX_WRITER_HPP
#define INCLUDED_DRIVERS_FAUX_WRITER_HPP

#include <pdal/pdal.hpp>

#include <pdal/Writer.hpp>


namespace pdal { namespace drivers { namespace faux {


//
// The FauxWriter doesn't actually write to disk -- instead, it just
// record some summary stats about the data it is given.
//
// This writer knows only about three dimensions: X,Y,Z (as doubles).
//
class LIBPC_DLL Writer : public pdal::Writer
{
public:
    Writer(Stage& prevStage);

    const std::string& getDescription() const;
    const std::string& getName() const;
    
    // retrieve the summary info
    double getMinX() const { return m_minimumX; }
    double getMinY() const { return m_minimumY; }
    double getMinZ() const { return m_minimumZ; }
    double getMaxX() const { return m_maximumX; }
    double getMaxY() const { return m_maximumY; }
    double getMaxZ() const { return m_maximumZ; }
    double getAvgX() const { return m_averageX; }
    double getAvgY() const { return m_averageY; }
    double getAvgZ() const { return m_averageZ; }

private:
    double m_minimumX;
    double m_minimumY;
    double m_minimumZ;
    double m_maximumX;
    double m_maximumY;
    double m_maximumZ;
    double m_averageX;
    double m_averageY;
    double m_averageZ;

    void writeBegin();
    boost::uint32_t writeBuffer(const PointBuffer&);
    void writeEnd();

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented
};


} } } // namespaces


#endif
