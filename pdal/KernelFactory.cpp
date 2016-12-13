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
#include <pdal/PluginManager.hpp>

#include <kernels/DeltaKernel.hpp>
#include <kernels/DiffKernel.hpp>
#include <kernels/GroundKernel.hpp>
#include <kernels/HausdorffKernel.hpp>
#include <kernels/InfoKernel.hpp>
#include <kernels/MergeKernel.hpp>
#include <kernels/PipelineKernel.hpp>
#include <kernels/RandomKernel.hpp>
#include <kernels/SortKernel.hpp>
#include <kernels/SplitKernel.hpp>
#include <kernels/TIndexKernel.hpp>
#include <kernels/TranslateKernel.hpp>

namespace pdal
{

KernelFactory::KernelFactory(bool no_plugins)
{
    if (!no_plugins)
        PluginManager::loadAll(PF_PluginType_Kernel);

    PluginManager::initializePlugin(DeltaKernel_InitPlugin);
    PluginManager::initializePlugin(DiffKernel_InitPlugin);
    PluginManager::initializePlugin(GroundKernel_InitPlugin);
    PluginManager::initializePlugin(HausdorffKernel_InitPlugin);
    PluginManager::initializePlugin(InfoKernel_InitPlugin);
    PluginManager::initializePlugin(MergeKernel_InitPlugin);
    PluginManager::initializePlugin(PipelineKernel_InitPlugin);
    PluginManager::initializePlugin(RandomKernel_InitPlugin);
    PluginManager::initializePlugin(SortKernel_InitPlugin);
    PluginManager::initializePlugin(SplitKernel_InitPlugin);
    PluginManager::initializePlugin(TIndexKernel_InitPlugin);
    PluginManager::initializePlugin(TranslateKernel_InitPlugin);
}

} // namespace pdal
