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

#ifndef INCLUDED_POINTLAYOUT_HPP
#define INCLUDED_POINTLAYOUT_HPP

#include <vector>
#include <string>

#include <boost/cstdint.hpp>

#include "libpc/export.hpp"
#include "libpc/Dimension.hpp"

namespace libpc
{

class LIBPC_DLL PointLayout
{
public:
    PointLayout();
    PointLayout(const PointLayout&);
    PointLayout& operator=(const PointLayout& other);

    bool operator==(const PointLayout& other) const;

    // adds a field to the end of the layout
    void addField(const Dimension& field);

    // returns a given field
    const Dimension& getField(std::size_t fieldIndex) const
    {
        return m_fields[fieldIndex];
    }
    Dimension& getField(std::size_t fieldIndex)
    {
        return m_fields[fieldIndex];
    }

    // total num bytes for all fields in the point
    std::size_t getSizeInBytes() const;

    // number of fields in the point
    std::size_t getNumFields() const;

    void dump(std::string indent="") const;

    // returns false if not found, otherwise sets index
    bool findFieldIndex(std::string item, std::size_t& index) const;
    bool hasField(std::string item) const;

private:
    std::vector<Dimension> m_fields; // each of the fields

    std::size_t m_numBytes; // num bytes required to store all fields
};

} // namespace libpc

#endif
