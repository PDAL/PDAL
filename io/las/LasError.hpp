/******************************************************************************
 * Copyright (c) 2014, Hobu Inc., hobu@hobu.co
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
 *     * Neither the name of the Hobu Inc. nor the names of contributors
 *       to this software may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include <vector>

#include <pdal/util/Utils.hpp>
#include <pdal/Log.hpp>

namespace pdal
{

class LasError
{
public:
    LasError() : m_filename("<unknown>")
    {}

    void setFilename(const std::string& filename)
        { m_filename = filename; }

    void setLog(LogPtr log) { m_log = log; }

    void returnNumWarning(int returnNum)
    {
        static std::vector<int> warned;

        if (!Utils::contains(warned, returnNum))
        {
            warned.push_back(returnNum);
            m_log->get(LogLevel::Warning) << m_filename << ": Found invalid value of '" <<
                returnNum << "' for point's return number.\n";
        }
    }

    void numReturnsWarning(int numReturns)
    {
        static std::vector<int> warned;

        if (!Utils::contains(warned, numReturns))
        {
            warned.push_back(numReturns);
            m_log->get(LogLevel::Warning) << m_filename << ": Found invalid value "
                "of '" << numReturns << "' for point's number of returns.\n";
        }
    }

private:
    std::string m_filename;
    LogPtr m_log;
};

} // namespace pdal
