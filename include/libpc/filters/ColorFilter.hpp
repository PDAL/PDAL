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

#ifndef INCLUDED_COLORFILTER_HPP
#define INCLUDED_COLORFILTER_HPP

#include <boost/cstdint.hpp>

#include <libpc/export.hpp>
#include <libpc/Filter.hpp>
#include <libpc/FilterIterator.hpp>

namespace libpc { namespace filters {

class ColorFilterIterator;

// adds three new u8 fields (R,G,B) for the colourization of the Z axis
// the color is done as a ramp from the declared Z min/max values in the header
class LIBPC_DLL ColorFilter : public Filter
{
public:
    ColorFilter(Stage& prevStage);

    const std::string& getName() const;

    void getColor(float value, boost::uint8_t& red, boost::uint8_t& green, boost::uint8_t& blue) const;

    Iterator* createIterator();

private:
    void checkImpedance();

    ColorFilter& operator=(const ColorFilter&); // not implemented
    ColorFilter(const ColorFilter&); // not implemented
};


} } // namespaces

#endif
