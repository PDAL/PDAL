/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

#pragma once

#include "PcdHeader.hpp"

#include <pdal/Writer.hpp>

namespace pdal
{

class PDAL_DLL PcdWriter : public Writer
{
    struct DimSpec
    {
        DimSpec() : m_field(PcdField()), m_precision(3)
        {
        }

        DimSpec(PcdField field, uint32_t precision)
        {
            m_field = field;
            m_precision = precision;
        }
        PcdField m_field;
        uint32_t m_precision;
    };

public:
    std::string getName() const;

    PcdWriter();
    PcdWriter& operator=(const PcdWriter&) = delete;
    PcdWriter(const PcdWriter&) = delete;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);

    DimSpec extractDim(std::string dim, PointTableRef table);
    bool findDim(Dimension::Id id, DimSpec& ds);

    PcdHeader m_header;
    std::ostream* m_ostream;
    std::string m_filename;
    std::string m_compression_string;
    bool m_writeAllDims;
    std::string m_dimOrder;
    uint32_t m_precision;

    std::vector<DimSpec> m_dims;
    DimSpec m_xDim;
    DimSpec m_yDim;
    DimSpec m_zDim;
};

} // namespaces
