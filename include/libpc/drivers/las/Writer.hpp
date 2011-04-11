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

#ifndef INCLUDED_DRIVERS_LAS_WRITER_HPP
#define INCLUDED_DRIVERS_LAS_WRITER_HPP

#include <libpc/libpc.hpp>

#include <libpc/Writer.hpp>
#include <libpc/drivers/las/Header.hpp>

namespace libpc { namespace drivers { namespace las {


class LIBPC_DLL LasWriter : public Writer
{
public:
    LasWriter(Stage& prevStage, std::ostream&);

    const std::string& getName() const;

protected:
    // this is called once before the loop with the writeBuffer calls
    virtual void writeBegin();

    // called repeatedly, until out of data
    virtual boost::uint32_t writeBuffer(const PointBuffer&);

    // called once, after the writeBuffer calls
    virtual void writeEnd();

private:
    std::ostream& m_ostream;
    LasHeader m_lasHeader;
    boost::uint32_t m_numPointsWritten;

    LasWriter& operator=(const LasWriter&); // not implemented
    LasWriter(const LasWriter&); // not implemented
};

} } } // namespaces

#endif
