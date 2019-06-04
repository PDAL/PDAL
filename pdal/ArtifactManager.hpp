/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pdal/Artifact.hpp>

namespace pdal
{

class ArtifactManager
{
public:
    ArtifactManager() = default;
    ArtifactManager(const ArtifactManager&) = delete;
    ArtifactManager& operator=(const ArtifactManager&) = delete;

    bool put(const std::string& name, ArtifactPtr artifact)
    {
        return m_storage.insert(std::make_pair(name, artifact)).second;
    }

    template <typename T>
    bool replace(const std::string& name, std::shared_ptr<T> art)
    {
        auto it = m_storage.find(name);
        if (it == m_storage.end())
            return false;

        if (!std::dynamic_pointer_cast<T>(it->second))
            return false;
        it->second = art;
        return true;
    }

    template<typename T>
    bool replaceOrPut(const std::string& name, std::shared_ptr<T> art)
    {
        if (!replace(name, art))
            return put(name, art);
        return true;
    }

    bool erase(const std::string& name)
    {
        return m_storage.erase(name);
    }

    bool exists(const std::string& name)
    {
        return (m_storage.find(name) != m_storage.end());
    }

    std::vector<std::string> keys() const
    {
        std::vector<std::string> ks;
        for (auto e : m_storage)
            ks.push_back(e.first);
        return ks;
    }

    template <typename T>
    std::shared_ptr<T> get(const std::string& name)
    {
        std::shared_ptr<T> art;
        try
        {
            art = std::dynamic_pointer_cast<T>(m_storage.at(name));
        }
        catch (...)
        {}
        return art;
    }
private:
    std::map<std::string, ArtifactPtr> m_storage;
};

} // namespace pdal
