/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS color class
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2008, Howard Butler
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <libpc/Color.hpp>

// boost
//#include <boost/cstdint.hpp>
// std
//#include <stdexcept>
//#include <limits>

namespace libpc
{

Color::Color()
{
    m_color.assign(0);
}

Color::Color(boost::uint32_t red, boost::uint32_t green, boost::uint32_t blue)
{
    if (red > (std::numeric_limits<boost::uint16_t>::max()) ||
            green > (std::numeric_limits<boost::uint16_t>::max()) ||
            blue > (std::numeric_limits<boost::uint16_t>::max()))
        throw_invalid_color_component();

    using boost::uint16_t;

    m_color[0] = static_cast<uint16_t>(red);
    m_color[1] = static_cast<uint16_t>(green);
    m_color[2] = static_cast<uint16_t>(blue);
}

Color::Color(boost::array<value_type, 3> const& color)
{
    m_color = color;
}

Color::Color(Color const& other)
    : m_color(other.m_color)
{
}

Color& Color::operator=(Color const& rhs)
{
    if (&rhs != this)
    {
        m_color = rhs.m_color;
    }
    return *this;
}

void Color::throw_index_out_of_range() const
{
    throw std::out_of_range("subscript out of range");
}

void Color::throw_invalid_color_component() const
{
    throw std::invalid_argument("Color component value too large.  Each must be less than 65536");
}

} // namespace liblas
