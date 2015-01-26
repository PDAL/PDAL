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
#include <pdal/Kernels.hpp>

#include <pdal/Utils.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include <sstream>
#include <stdio.h> // for funcptr

namespace pdal
{

//
// define the functions to create the kernels
//
MAKE_KERNEL_CREATOR(delta, pdal::DeltaKernel)
MAKE_KERNEL_CREATOR(diff, pdal::DiffKernel)
MAKE_KERNEL_CREATOR(info, pdal::InfoKernel)
MAKE_KERNEL_CREATOR(pipeline, pdal::PipelineKernel)
MAKE_KERNEL_CREATOR(random, pdal::RandomKernel)
MAKE_KERNEL_CREATOR(sort, pdal::SortKernel)
MAKE_KERNEL_CREATOR(translate, pdal::TranslateKernel)

KernelFactory::KernelFactory()
{
    registerKnownKernels();

    loadPlugins();
    return;
}


std::unique_ptr<Kernel> KernelFactory::createKernel(const std::string& type)
{
    KernelCreator* f = getKernelCreator(type);
    if (!f)
    {
        std::ostringstream oss;
        oss << "Unable to create kernel for type '" << type << "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }
    std::unique_ptr<Kernel> kernel(f());
    return kernel;
}


template<typename T>
static T* findFirst(const std::string& type, std::map<std::string, T*> list)
{
    typename std::map<std::string, T*>::const_iterator iter = list.find(type);
    if (iter == list.end())
        return NULL;
    return (*iter).second;
}


KernelFactory::KernelCreator* KernelFactory::getKernelCreator(const std::string& type) const
{
    return findFirst<KernelCreator>(type, m_kernelCreators);
}


void KernelFactory::registerKernel(const std::string& type, KernelCreator* f)
{
    std::pair<std::string, KernelCreator*> p(type, f);
    m_kernelCreators.insert(p);
}


void KernelFactory::registerKnownKernels()
{
    REGISTER_KERNEL(delta, pdal::DeltaKernel);
    REGISTER_KERNEL(diff, pdal::DiffKernel);
    REGISTER_KERNEL(info, pdal::InfoKernel);
    REGISTER_KERNEL(pipeline, pdal::PipelineKernel);
    REGISTER_KERNEL(random, pdal::RandomKernel);
    REGISTER_KERNEL(sort, pdal::SortKernel);
    REGISTER_KERNEL(translate, pdal::TranslateKernel);
}


void KernelFactory::loadPlugins()
{
    using namespace boost::filesystem;

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{kerneltype}_{name}

    // For example, libpdal_plugin_kernel_ground


    // If we don't have a driver path, we're not loading anything

    if (pluginDir.size() == 0)
    {
        pluginDir = "/usr/local/lib:./lib:../lib:../bin";
    }

    std::vector<std::string> pluginPathVec;
    boost::algorithm::split(pluginPathVec, pluginDir, boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (const auto& pluginPath : pluginPathVec)
    {
        if (!boost::filesystem::is_directory(pluginPath))
            continue;
        directory_iterator dir(pluginPath), it, end;

        std::map<path, path> pluginFilenames;

        // Collect candidate filenames in the above form. Prefer symlink files
        // over hard files if their basenames are the same.
        for (it = dir; it != end; ++it)
        {
            path p = it->path();

            if (boost::algorithm::istarts_with(p.filename().string(), "libpdal_plugin"))
            {
                path extension = p.extension();
                if (boost::algorithm::iends_with(extension.string(), "DLL") ||
                        boost::algorithm::iends_with(extension.string(), "DYLIB") ||
                        boost::algorithm::iends_with(extension.string(), "SO"))
                {
                    std::string basename;

                    // Step through the stems until the extension of the stem
                    // is empty. This is our basename.  For example,
                    // libpdal_plugin_writer_text.0.dylib will basename down to
                    // libpdal_plugin_writer_text and so will
                    // libpdal_plugin_writer_text.dylib
                    // copy the path so we can modify in place
                    path t = p;
                    for (; !t.extension().empty(); t = t.stem())
                    {
                        if (t.stem().extension().empty())
                        {
                            basename = t.stem().string();
                        }
                    }

                    if (pluginFilenames.find(basename) == pluginFilenames.end())
                    {
                        // We haven't already loaded a plugin with this basename,
                        // load it.
                        pluginFilenames.insert(std::pair<path, path>(basename, p));
                    }
                    else
                    {
                        // We already have a filename with the basename of this
                        // file.  If the basename of our current file is a symlink
                        // we're going to replace what's in the map with ours because
                        // we are going to presume that a symlink'd file is more
                        // cannonical than a hard file of the same name.
                        std::map<path, path>::iterator i = pluginFilenames.find(basename);
                        if (it->symlink_status().type() == symlink_file)
                        {
                            // Take the symlink over a hard SO
                            i->second = p;
                        }
                    }
                }
            }
        }

        std::map<std::string, std::string> registerMethods;

        for (std::map<path, path>::iterator t = pluginFilenames.begin();
                t!= pluginFilenames.end(); t ++)
        {
            // Basenames must be in the following form:
            // libpdal_plugin_writer_text or libpdal_plugin_filter_color
            // The last two tokens are the kernel type and the kernel name.
            path basename = t->first;
            path filename = t->second;

            registerPlugin(filename.string());
            // std::string methodName = "PDALRegister_" + boost::algorithm::ireplace_first_copy(basename.string(), "libpdal_plugin_", "");
            // Utils::registerPlugin((void*)this, filename.string(), methodName);

        }
    }
}


void KernelFactory::registerPlugin(std::string const& filename)
{
    using namespace boost::filesystem;
    path basename;

    path t = path(filename);
    for (; !t.extension().empty(); t = t.stem())
    {
        if (t.stem().extension().empty())
        {
            basename = t.stem().string();
        }
    }

    std::string base = basename.string();

    std::string pluginName = boost::algorithm::ireplace_first_copy(base, "libpdal_plugin_", "");
    std::string pluginType = pluginName.substr(0, pluginName.find_last_of("_"));
    if (boost::iequals(pluginType, "kernel"))
    {
        std::string registerMethodName = "PDALRegister_" + pluginName;

        std::string versionMethodName = "PDALRegister_version_" + pluginName;

        Utils::registerPlugin((void*)this, filename, registerMethodName, versionMethodName);
    }
}


std::map<std::string, pdal::KernelInfo> const& KernelFactory::getKernelInfos() const
{
    return m_driver_info;
}


} // namespace pdal
