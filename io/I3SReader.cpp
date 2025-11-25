/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include <arbiter/arbiter.hpp>
#include "I3SReader.hpp"
#include "private/esri/EsriUtil.hpp"
#include "private/esri/Interface.hpp"

#include <thread>

namespace pdal
{

using namespace i3s;

static PluginInfo const i3sInfo
{
    "readers.i3s",
    "I3S Reader",
    "https://pdal.org/stages/readers.i3s.html"
};

CREATE_STATIC_STAGE(I3SReader, i3sInfo)

namespace
{

struct I3SInterface : public i3s::Interface
{
    I3SInterface(const std::string& filename) : m_filename(filename)
    {}
    ~I3SInterface() override
    {}

    void initInfo() override;
    std::vector<char> fetchBinary(std::string url, std::string attNum,
        std::string ext) const override;
    std::string fetchJson(std::string) override;
    NL::json getInfo() override
        { return m_info; }

    NL::json m_info;
    const std::string& m_filename;
    arbiter::Arbiter m_arbiter;
};

}

std::string I3SReader::getName() const { return i3sInfo.name; }

I3SReader::I3SReader() : EsriReader(std::make_unique<I3SInterface>(m_filename))
{}

void I3SInterface::initInfo()
{
    try
    {
        std::string s = m_arbiter.get(m_filename);
        NL::json info = i3s::parse(s, "Invalid JSON in file '" + m_filename + "'.");

        if (info.empty())
            throw pdal_error(std::string("Incorrect Json object"));
        if (!info.contains("layers"))
            throw pdal_error(std::string("Json object contains no layers"));

        m_info = info["layers"][0];
    }
    catch(i3s::EsriError& e)
    {
        throw pdal_error(std::string("Error parsing Json object: ") + e.what());
    }
}


std::string I3SInterface::fetchJson(std::string filepath)
{
    filepath = m_filename + "/layers/0/" + filepath;
    return m_arbiter.get(filepath);
}


std::vector<char> I3SInterface::fetchBinary(std::string url,
    std::string attNum, std::string ext) const
{
    const int NumRetries(5);
    int retry = 0;

    std::string filepath = m_filename + "/layers/0/" + url + attNum;
    // For the REST I3S endpoint there are no file extensions.
    std::vector<char> result;
    while (true)
    {
        auto data = m_arbiter.tryGetBinary(filepath);
        if (data)
        {
            result = std::move(*data);
            break;
        }
        if (++retry == NumRetries)
            throw EsriError(std::string("Failed to fetch: " + filepath));
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    return result;
}

} //namespace pdal
