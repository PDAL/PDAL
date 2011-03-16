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

#ifndef INCLUDED_FAUXWRITER_HPP
#define INCLUDED_FAUXWRITER_HPP

#include <string>

#include <libpc/Writer.hpp>

namespace libpc
{

//
// The FauxWriter doesn't actually write to disk -- instead, it just
// record some summary stats about the data it is given.
//
// This writer knows only about three dimensions: X,Y,Z (as floats).
//
class LIBPC_DLL FauxWriter : public Writer
{
public:
    FauxWriter(Stage& prevStage);

    const std::string& getName() const;

    // retrieve the summary info
    float getMinX() const { return m_minimumX; }
    float getMinY() const { return m_minimumY; }
    float getMinZ() const { return m_minimumZ; }
    float getMaxX() const { return m_maximumX; }
    float getMaxY() const { return m_maximumY; }
    float getMaxZ() const { return m_maximumZ; }
    float getAvgX() const { return m_averageX; }
    float getAvgY() const { return m_averageY; }
    float getAvgZ() const { return m_averageZ; }

private:
    float m_minimumX;
    float m_minimumY;
    float m_minimumZ;
    float m_maximumX;
    float m_maximumY;
    float m_maximumZ;
    float m_averageX;
    float m_averageY;
    float m_averageZ;

    void writeBegin();
    boost::uint32_t writeBuffer(const PointData&);
    void writeEnd();

    FauxWriter& operator=(const FauxWriter&); // not implemented
    FauxWriter(const FauxWriter&); // not implemented
};

} // namespace libpc

#endif
