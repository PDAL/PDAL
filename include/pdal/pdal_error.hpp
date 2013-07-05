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

#ifndef PDAL_EXCEPTION_HPP_INCLUDED
#define PDAL_EXCEPTION_HPP_INCLUDED

#include <stdexcept>

namespace pdal
{


// base class for all pdal exceptions
class pdal_error : public std::runtime_error
{
public:
    pdal_error(std::string const& msg)
        : std::runtime_error(msg)
    {}
};


/// Exception reporting invalid point data.
/// It's usually thrown by Point::Validate function.
class invalid_point_data : public pdal_error
{
public:
    invalid_point_data(std::string const& msg, unsigned int who)
        : pdal_error(msg), m_who(who)
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


class invalid_expression : public pdal_error
{
public:

    invalid_expression(std::string const& msg)
        : pdal_error(msg)
    {}
};


// use this for conditions that indicate a program bug -- things
// that we'd assert can't happen
class internal_error : public pdal_error
{
public:

    internal_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


class invalid_format : public pdal_error
{
public:

    invalid_format(std::string const& msg)
        : pdal_error(msg)
    {}
};


// for when a stage doesn't get the schema it expects
class impedance_invalid : public pdal_error
{
public:

    impedance_invalid(std::string const& msg)
        : pdal_error(msg)
    {}
};

// for when a stage doesn't provide the requested iterator
class iterator_not_found : public pdal_error
{
public:

    iterator_not_found(std::string const& msg)
        : pdal_error(msg)
    {}
};


// use this for attempts to use a feature not compiled in, e.g. laszip or gdal
class configuration_error : public pdal_error
{
public:
    configuration_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

// thrown when seeking off the end of a stage
class invalid_seek_error : public pdal_error
{
public:
    invalid_seek_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


// use this for situations where indeterminate point counts prevent some
// operation from happening
class indeterminate_count_error : public pdal_error
{
public:
    indeterminate_count_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


class dimension_not_found : public pdal_error
{
public:

    dimension_not_found(std::string const& msg)
        : pdal_error(msg)
    {}
};

class multiple_parent_dimensions : public pdal_error
{
public:

    multiple_parent_dimensions(std::string const& msg)
        : pdal_error(msg)
    {}
};

class duplicate_dimension_id : public pdal_error
{
public:

    duplicate_dimension_id(std::string const& msg)
        : pdal_error(msg)
    {}
};

class metadata_not_found : public pdal_error
{
public:

    metadata_not_found(std::string const& msg)
        : pdal_error(msg)
    {}
};

class metadata_error : public pdal_error
{
public:

    metadata_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class multiple_parent_metadata : public pdal_error
{
public:

    multiple_parent_metadata(std::string const& msg)
        : pdal_error(msg)
    {}
};

// use this for code still under development
class not_yet_implemented : public pdal_error
{
public:
    not_yet_implemented(std::string const& msg)
        : pdal_error(msg)
    {}
};


// for Option processing
class option_not_found : public pdal_error
{
public:
    option_not_found(std::string const& msg)
        : pdal_error(msg)
    {}
};


// for errors when reading pipeline XML
class pipeline_xml_error : public pdal_error
{
public:
    pipeline_xml_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


// use this when an executing pipeline needs to abort per user request
class pipeline_interrupt : public pdal_error
{
public:
    pipeline_interrupt(std::string const& msg)
        : pdal_error(msg)
    {}
};

/// Used for pdal::Bounds
class bounds_error : public pdal_error
{
public:

    bounds_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class gdal_error : public pdal_error
{
public:
    gdal_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class schema_error : public std::runtime_error
{
public:
    schema_error(std::string const& msg)
        : std::runtime_error(msg)
    {}
};

class buffer_error : public std::runtime_error
{
public:
    buffer_error(std::string const& msg)
        : std::runtime_error(msg)
    {}
};

class python_error : public std::runtime_error
{
public:
    python_error(std::string const& msg)
        : std::runtime_error(msg)
    {}
};

class plugin_error : public std::runtime_error
{
public:
    plugin_error(std::string const& msg)
        : std::runtime_error(msg)
    {}
};


} // namespace pdal

#endif // PDAL_EXCEPTION_HPP_INCLUDED
