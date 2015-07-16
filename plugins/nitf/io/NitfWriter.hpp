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

#include <pdal/StageFactory.hpp>
#include <las/LasWriter.hpp>

namespace pdal
{


class PDAL_DLL NitfWriter : public LasWriter
{
public:
    NitfWriter();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

private:
    virtual void processOptions(const Options& options);
    virtual void readyFile(const std::string& filename);
    virtual void doneFile();
    virtual void writeView(const PointViewPtr view);

    std::string m_nitfFilename;
    BOX3D m_bounds;
    std::string m_cLevel;
    std::string m_sType;
    std::string m_oStationId;
    std::string m_fileTitle;
    std::string m_fileClass;
    std::string m_origName;
    std::string m_origPhone;
    std::string m_securityClass;
    std::string m_securityControlAndHandling;
    std::string m_securityClassificationSystem;
    std::string m_imgSecurityClass;
    std::string m_imgDate;
    pdal::Option m_aimidb;
    pdal::Option m_acftb;
    std::string m_imgIdentifier2;
    std::string m_sic;
    std::stringstream m_oss;

    NitfWriter& operator=(const NitfWriter&); // not implemented
    NitfWriter(const NitfWriter&); // not implemented
};

} // namespace pdal
