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

#ifndef INCLUDED_DRIVERS_LAS_READER_HPP
#define INCLUDED_DRIVERS_LAS_READER_HPP

#include <libpc/libpc.hpp>

#include <libpc/Stage.hpp>
#include <libpc/drivers/las/Header.hpp>


namespace libpc
{
    class PointBuffer;
}

namespace libpc { namespace drivers { namespace las {

class LasHeader;

class LIBPC_DLL LasReader : public Stage
{
public:
    LasReader(const std::string& filename);

    const std::string& getName() const;

    const std::string& getFileName() const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        if (t == StageIterator_Random ) return true;
        
        return false;
    }

    libpc::SequentialIterator* createSequentialIterator() const;
    libpc::RandomIterator* createRandomIterator() const;

    // this is called by the stage's iterator
    boost::uint32_t processBuffer(PointBuffer& PointBuffer, std::istream& stream) const;

    const LasHeader& getLasHeader() const { return m_lasHeader; }

protected:
    LasHeader& getLasHeaderRef() { return m_lasHeader; }

private:
    const std::string m_filename;
    LasHeader m_lasHeader;

    LasReader& operator=(const LasReader&); // not implemented
    LasReader(const LasReader&); // not implemented
};


} } } // namespaces

#endif
