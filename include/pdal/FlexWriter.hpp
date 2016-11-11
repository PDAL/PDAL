/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (hobu@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <pdal/PDALUtils.hpp>
#include <pdal/Scaling.hpp>
#include <pdal/Writer.hpp>

namespace pdal
{

class PDAL_DLL FlexWriter : public Writer
{
protected:
    FlexWriter() : m_filenum(1)
    {}

    std::string m_filename;
    Scaling m_scaling;

    void validateFilename(PointTableRef table)
    {
        if (!table.supportsView() &&
            (m_filename.find('#') != std::string::npos))
        {
            std::ostringstream oss;
            oss << getName() << ": Can't write with template-based "
                "filename using streaming point table.";
            throw oss.str();
        }
    }

private:
    std::string::size_type m_hashPos;

    virtual void writerInitialize(PointTableRef table)
    {
        Writer::writerInitialize(table);
        m_hashPos = handleFilenameTemplate(m_filename);
    }

    std::string generateFilename()
    {
        std::string filename = m_filename;
        if (m_hashPos != std::string::npos) {
            std::string fileCount = std::to_string(m_filenum++);
            filename.replace(m_hashPos, 1, fileCount);
        }
        return filename;
    }

#if (__GNUG__ < 4 || (__GNUG__ == 4 && __GNUG_MINOR__ < 7))
#define final
#endif

    virtual void ready(PointTableRef table) final
    {
        readyTable(table);
        if (m_hashPos == std::string::npos)
        {
            if (!table.spatialReferenceUnique())
            {
                std::ostringstream oss;
                oss << getName() << ": Attempting to write '" << m_filename <<
                    "' with multiple spatial references.";
                Utils::printError(oss.str());
            }
            readyFile(generateFilename(), table.spatialReference());
        }
    }

    // This essentially moves ready() and done() into write(), which means
    // that they get executed once for each view.  The check for m_hashPos
    // is a test to see if the filename specification is a template.  If it's
    // not a template, ready() and done() are taken care of in the ready()
    // and done() functions in this class.
    virtual void write(const PointViewPtr view) final
    {
        if (m_hashPos != std::string::npos)
            readyFile(generateFilename(), view->spatialReference());
        writeView(view);
        if (m_hashPos != std::string::npos)
            doneFile();
    }

    virtual void done(PointTableRef table) final
    {
        if (m_hashPos == std::string::npos)
            doneFile();
        doneTable(table);
    }

#undef final

    virtual void readyTable(PointTableRef table)
    {}

    virtual void doneTable(PointTableRef table)
    {}

    virtual void readyFile(const std::string& filename,
        const SpatialReference& srs) = 0;
    virtual void writeView(const PointViewPtr view) = 0;
    virtual void doneFile()
    {}

    size_t m_filenum;

    FlexWriter& operator=(const FlexWriter&); // not implemented
    FlexWriter(const FlexWriter&); // not implemented
};

} // namespace pdal

