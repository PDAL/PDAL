/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/KernelFactory.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/Utils.hpp>

#include <delta/DeltaKernel.hpp>
#include <diff/DiffKernel.hpp>
#include <info/InfoKernel.hpp>
#include <pipeline/PipelineKernel.hpp>
#include <random/RandomKernel.hpp>
#include <sort/SortKernel.hpp>
#include <translate/TranslateKernel.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include <sstream>
#include <stdio.h> // for funcptr
#include <string>
#include <vector>

namespace pdal
{

KernelFactory::KernelFactory(bool no_plugins)
{
    PluginManager & pm = GlobalEnvironment::get().getPluginManager();
    if (!no_plugins) { pm.loadAll(PF_PluginType_Kernel); }
    PluginManager::initializePlugin(DeltaKernel_InitPlugin);
    PluginManager::initializePlugin(DiffKernel_InitPlugin);
    PluginManager::initializePlugin(InfoKernel_InitPlugin);
    PluginManager::initializePlugin(PipelineKernel_InitPlugin);
    PluginManager::initializePlugin(RandomKernel_InitPlugin);
    PluginManager::initializePlugin(SortKernel_InitPlugin);
    PluginManager::initializePlugin(TranslateKernel_InitPlugin);
}

std::unique_ptr<Kernel> KernelFactory::createKernel(std::string const& kernel_name)
{
    PluginManager & pm = GlobalEnvironment::get().getPluginManager();

    void * kernel = pm.createObject(kernel_name);
    if (!kernel)
    {
        int32_t res = pm.guessLoadByPath(kernel_name);
        if (res == 0)
            kernel = pm.createObject(kernel_name);
    }
    Kernel *k = (Kernel*)kernel;
    std::unique_ptr<Kernel> retKernel(k);
    return retKernel;
}

std::vector<std::string> KernelFactory::getKernelNames()
{
    PluginManager & pm = GlobalEnvironment::get().getPluginManager();
    PluginManager::RegistrationMap rm = pm.getRegistrationMap();
    std::vector<std::string> nv;
    for (auto r : rm)
    {
        if (r.second.pluginType == PF_PluginType_Kernel)
            nv.push_back(r.first);
    }
    return nv;
}

} // namespace pdal
