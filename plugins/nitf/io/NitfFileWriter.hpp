/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <pdal/Options.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wunused-private-field"

#include <nitro/c++/import/nitf.hpp>

#pragma GCC diagnostic pop

namespace pdal
{

//
// all the processing that is NITF-file specific goes in here
//
class PDAL_DLL NitfFileWriter
{
public:
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

    NitfFileWriter()
    {}
    NitfFileWriter(const NitfFileWriter&) = delete;
    NitfFileWriter& operator=(const NitfFileWriter&) = delete;

    void initialize();
    void setFilename(const std::string& filename)
        { m_filename = filename; }
    void wrapData(const char *buf, size_t size);
    void wrapData(const std::string& filename);
    void addArgs(ProgramArgs& args);
    void setBounds(const BOX3D& bounds);
    void write();

    std::unique_ptr<nitf::DataSource> m_source;
    std::unique_ptr<nitf::IOHandle> m_inputHandle;

    std::string m_cLevel;
    std::string m_sType;
    std::string m_oStationId;
    std::string m_fileTitle;
    std::string m_fileClass;
    std::string m_origName;
    std::string m_origPhone;
    std::string m_securityControlAndHandling;
    std::string m_securityClassificationSystem;
    std::string m_imgSecurityClass;
    std::string m_imgDate;
    StringList m_aimidb;
    StringList m_acftb;
    std::string m_imgIdentifier2;
    std::string m_sic;
    BOX3D m_bounds;

    std::string m_filename;
};


} // namespaces
