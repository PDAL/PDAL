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

#include <pdal/Writer.hpp>

namespace pdal
{

class PDAL_DLL FlexWriter : public Writer
{
protected:
    FlexWriter() : m_hashPos(std::string::npos), m_filenum(1)
    {}

private:
    virtual void writerProcessOptions(const Options& options)
    {
        Writer::writerProcessOptions(options);
        if (m_filename.empty())
        {
            std::ostringstream oss;
            oss << "Can't write with " << getName() << " without filename.";
            throw pdal_error(oss.str());
        }
        std::string::size_type suffixPos = m_filename.find_last_of('.');
        m_hashPos = m_filename.find_first_of('#');
        if (m_hashPos != std::string::npos)
        {
            if (m_hashPos > suffixPos)
            {
                throw pdal_error("File number placeholder ('#') is not "
                    "allowed in filename suffix.");
            }
            if (m_filename.find_first_of('#', m_hashPos + 1) !=
                std::string::npos)
            {
                std::ostringstream oss;
                oss << getName() << " filename specification can only contain "
                    "a single '#' placeholder.";
                throw pdal_error(oss.str());
            }
        }
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
            readyFile(generateFilename());
    }

    virtual void write(const PointViewPtr view) final
    {
        if (m_hashPos != std::string::npos)
            readyFile(generateFilename());
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

    virtual void readyFile(const std::string& filename) = 0;
    virtual void writeView(const PointViewPtr view) = 0;
    virtual void doneFile()
    {}

    std::string::size_type m_hashPos;
    size_t m_filenum;

    FlexWriter& operator=(const FlexWriter&); // not implemented
    FlexWriter(const FlexWriter&); // not implemented
};

} // namespace pdal

