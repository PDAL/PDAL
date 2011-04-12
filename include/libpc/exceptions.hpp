/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Exception subclasses for C++ libLAS
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
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

#ifndef LIBPC_EXCEPTION_HPP_INCLUDED
#define LIBPC_EXCEPTION_HPP_INCLUDED

#include <libpc/libpc.hpp>
#include <stdexcept>

namespace libpc
{


// base class for all libpc exceptions
class libpc_error : public std::runtime_error
{
public:
    libpc_error(std::string const& msg)
        : std::runtime_error(msg)
    {}
};


/// Exception reporting invalid point data.
/// It's usually thrown by Point::Validate function.
class invalid_point_data : public libpc_error
{
public:
    invalid_point_data(std::string const& msg, unsigned int who)
        : libpc_error(msg), m_who(who)
    {}

    /// Return flags identifying invalid point data members.
    /// Flags are composed with composed with Point::DataMemberFlag.
    /// Testing flags example: bool timeValid = e.who() & Point::eTime;
    unsigned int who() const
   {
        return m_who;
    }

private:
    unsigned int m_who;
};


class invalid_expression : public libpc_error
{
public:

    invalid_expression(std::string const& msg)
        : libpc_error(msg)
    {}
};


class invalid_format : public libpc_error
{
public:

    invalid_format(std::string const& msg)
        : libpc_error(msg)
    {}
};

// for when a stage doesn't get the schema it expects
class impedance_invalid : public libpc_error
{
public:

    impedance_invalid(std::string const& msg)
        : libpc_error(msg)
    {}
};

// use this for attempts to use a feature not compiled in, e.g. laszip or gdal
class configuration_error : public libpc_error
{
public:
    configuration_error(std::string const& msg)
        : libpc_error(msg)
    {}
};

// use this for situations where indeterminate point counts prevent some 
// operation from happening
class indeterminate_count_error : public libpc_error
{
public:
    indeterminate_count_error(std::string const& msg)
        : libpc_error(msg)
    {}
};

class dimension_not_found : public libpc_error
{
public:

    dimension_not_found(std::string const& msg)
        : libpc_error(msg)
    {}
};


// use this for code still under development
class not_yet_implemented : public libpc_error
{
public:
    not_yet_implemented(std::string const& msg)
        : libpc_error(msg)
    {}
};

} // namespace libpc

#endif // LIBPC_EXCEPTION_HPP_INCLUDED
