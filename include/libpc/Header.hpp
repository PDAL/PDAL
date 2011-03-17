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

#ifndef INCLUDED_HEADER_HPP
#define INCLUDED_HEADER_HPP

#include <iostream>
#include <boost/cstdint.hpp>

#include <libpc/libpc.hpp>
#include <libpc/Schema.hpp>
#include <libpc/Bounds.hpp>
#include <libpc/SpatialReference.hpp>
#include <libpc/Metadata.hpp>

namespace libpc
{

class LIBPC_DLL Header
{
public:
    Header();
    Header(const Header&);
    virtual ~Header();

    const Schema& getSchema() const;
    Schema& getSchema();
    void setSchema(const Schema&);

    boost::uint64_t getNumPoints() const;
    void setNumPoints(boost::uint64_t);

    PointCountType getPointCountType() const;
    void setPointCountType(PointCountType);

    const Bounds<double>& getBounds() const;
    void setBounds(const Bounds<double>&);

    const SpatialReference& getSpatialReference() const;
    void setSpatialReference(const SpatialReference&);

    const Metadata::Array& getMetadata() const;
    Metadata::Array& getMetadata();

    void dump() const;

private:
    Schema m_schema;
    boost::uint64_t m_numPoints;
    PointCountType m_pointCountType;
    Bounds<double> m_bounds;
    SpatialReference m_spatialReference;
    Metadata::Array m_metadataArray;

    Header& operator=(const Header&); // nope
};


LIBPC_DLL std::ostream& operator<<(std::ostream& ostr, const Header&);


} // namespace libpc

#endif
