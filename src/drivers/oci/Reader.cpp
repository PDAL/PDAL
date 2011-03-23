/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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


#include <cassert>
#include <sstream>


#include <libpc/drivers/oci/Reader.hpp>
#include <libpc/drivers/oci/Header.hpp>
#include <libpc/drivers/oci/Common.hpp>


#include <libpc/exceptions.hpp>

#include <cstdlib>
#include <iostream>

#include <boost/make_shared.hpp>

namespace libpc { namespace driver { namespace oci {


Reader::Reader(Options& options)
    : libpc::Stage()
    , m_options(options)
{

}    


const std::string& Reader::getName() const
{
    static std::string name("OCI Reader");
    return name;
}

Reader::~Reader()
{

    return;
}

void Reader::seekToPoint(boost::uint64_t pointNum)
{
    throw not_yet_implemented("oci seekToPoint");

    ////if (pointNum == getIndex())
    ////{
    ////    return;
    ////}

    ////setCurrentPointIndex(0);
    ////
    ////// we read the points only to get to the right place -- we
    ////// will just drop the points on the floor and return
    ////boost::uint32_t pointNumX = (boost::uint32_t)pointNum; // BUG
    ////assert(pointNumX == pointNum);
    ////PointBuffer PointBuffer(getHeader().getSchema(), pointNumX);
    ////read(PointBuffer);

    ////return;
}

}}} // namespace libpc::driver::oci
