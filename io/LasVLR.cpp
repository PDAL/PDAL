/******************************************************************************
* Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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

#include "LasVLR.hpp"

#include <limits>
#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include <io/private/las/Vlr.hpp>

namespace pdal
{

struct LasVLR::Private
{
    Private(las::Vlr *v) : v(v)
    {}

    las::Vlr *v;
};

LasVLR::LasVLR(las::Vlr *v) : d(std::make_unique<Private>(v))
{}

LasVLR::LasVLR(const LasVLR& src) : d(std::make_unique<Private>(src.d->v))
{}

LasVLR::LasVLR(LasVLR&& v) : d(std::move(v.d))
{}

LasVLR& LasVLR::operator=(const LasVLR& src)
{
    d->v = src.d->v;
    return *this;
}

LasVLR& LasVLR::operator=(LasVLR&& src)
{
    d->v = src.d->v;
    return *this;
}

LasVLR::~LasVLR()
{}

std::string LasVLR::userId() const
{
    return d->v->userId;
}

uint16_t LasVLR::recordId() const
{
    return d->v->recordId;
}

std::string LasVLR::description() const
{
    return d->v->description;
}

bool LasVLR::matches(const std::string& u) const
{
    return u == userId();
}

bool LasVLR::matches(const std::string& u, uint16_t r) const
{
    return u == userId() && r == recordId();
}

const char *LasVLR::data() const
{
    return d->v->data();
}

char *LasVLR::data()
{
    return nullptr;
}

bool LasVLR::isEmpty() const
{
    return d->v->empty();
}

uint64_t LasVLR::dataLen() const
{
    return (uint64_t)d->v->dataSize();
}

void LasVLR::setDataLen(uint64_t size)
{
    (void)size;
}

void LasVLR::write(OLeStream& out, uint16_t recordSig)
{
    (void)out;
    (void)recordSig;
}

bool LasVLR::read(ILeStream& in, size_t limit)
{
    (void)in;
    (void)limit;
    return false;
}

OLeStream& operator<<(OLeStream& out, const LasVLR& v)
{
    (void)v;
    return out;
}

std::istream& operator>>(std::istream& in, LasVLR& v)
{
    (void)v;
    return in;
}

std::ostream& operator<<(std::ostream& out,  const LasVLR& v)
{
    (void)v;
    return out;
}

} // namespace pdal
