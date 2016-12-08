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

#include <cctype>
#include <iostream>

#include <pdal/Kernel.hpp>
#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/pdal_config.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <pdal/pdal_config.hpp>

#include <io/BufferReader.hpp>

#include <memory>
#include <vector>

namespace pdal
{

namespace
{

bool parseOption(std::string o, std::string& stage, std::string& option,
    std::string& value)
{
    value.clear();
    if (o.size() < 2)
        return false;
    if (o[0] != '-' || o[1] != '-')
        return false;

    o = o.substr(2);

    // Options are stage_type.stage_name.option_name
    // stage_type is always lowercase stage_names start with lowercase and
    // then are lowercase or digits.  Option names start with lowercase and
    // then contain lowercase, digits or underscore.

    // This awfulness is to work around the multiply-defined islower.  Seems
    // a bit better than the cast solution.
    auto islc = [](char c)
        { return std::islower(c); };
    auto islcOrDigit = [](char c)
        { return std::islower(c) || std::isdigit(c); };

    std::string::size_type pos = 0;
    std::string::size_type count = 0;

    // Get stage_type.
    count = Utils::extract(o, pos, islc);
    pos += count;
    std::string stage_type = o.substr(0, pos);
    if (stage_type != "readers" && stage_type != "writers" &&
        stage_type != "filters")
        return false;
    if (pos >= o.length() || o[pos++] != '.')
        return false;

    // Get stage_name.
    count = Utils::extract(o, pos, islcOrDigit);
    if (std::isdigit(o[pos]))
        return false;
    pos += count;
    stage = o.substr(0, pos);
    if (pos >= o.length() || o[pos++] != '.')
        return false;

    // Get option name.
    std::string::size_type optionStart = pos;
    count = Option::parse(o, pos);
    pos += count;
    option = o.substr(optionStart, count);

    // We've gotten a good option name, so return true, even if the value
    // is missing.  The caller can handle the missing value if desired.
    if (pos >= o.length() || o[pos++] != '=')
        return true;

    // The command-line parser takes care of quotes around an argument
    // value and such.  May want to do something to handle escaped characters?
    value = o.substr(pos);
    return true;
}

} // unnamed namespace


Kernel::Kernel() :
    m_showTime(false)
    , m_hardCoreDebug(false)
    , m_visualize(false)
{}


std::ostream& operator<<(std::ostream& ostr, const Kernel& kernel)
{
    ostr << "  Name: " << kernel.getName() << std::endl;
    return ostr;
}


void Kernel::doSwitches(const StringList& cmdArgs, ProgramArgs& args)
{
    OptionsMap& stageOptions = m_manager.stageOptions();
    StringList stringArgs;

    // Scan the argument vector for extra stage options.  Pull them out and
    // stick them in the list.  Let the ProgramArgs handle everything else.
    // NOTE: This depends on the format being "option=value" rather than
    //   "option value".  This is what we've always expected, so no problem,
    //   but it would be better to be more flexible.
    for (size_t i = 0; i < cmdArgs.size(); ++i)
    {
        std::string stageName, opName, value;

        if (parseOption(cmdArgs[i], stageName, opName, value))
        {
            if (value.empty())
            {
                std::ostringstream oss;
                oss << "Stage option '" << stageName << "." << opName <<
                    "' must be specified " << " as --" << stageName << "." <<
                    opName << "=<value>" << ".";
                throw pdal_error(oss.str());
            }
            Option op(opName, value);
            stageOptions[stageName].add(op);
        }
        else
            stringArgs.push_back(cmdArgs[i]);
    }

    try
    {
        // parseSimple allows us to scan for the help option without
        // raising exception about missing arguments and so on.
        // It also removes consumed args from the arg list, so for now,
        // parse a copy that will be ignored by parse().
        ProgramArgs hargs;
        hargs.add("help,h", "Print help message", m_showHelp);
        hargs.parseSimple(stringArgs);

        addBasicSwitches(args);
        addSwitches(args);
        if (!m_showHelp)
            args.parse(stringArgs);
    }
    catch (arg_error& e)
    {
        throw pdal_error(getName() + ": " + e.m_error);
    }
}


int Kernel::doStartup()
{
    return 0;
}


int Kernel::doExecution(ProgramArgs& args)
{
    if (m_hardCoreDebug)
    {
        int status = innerRun(args);
        return status;
    }

    int status = 1;

    try
    {
        status = innerRun(args);
    }
    catch (pdal::pdal_error const& e)
    {
        Utils::printError(e.what());
        return 1;
    }
    catch (std::exception const& e)
    {
        Utils::printError(e.what());
        return 1;
    }
    catch (...)
    {
        Utils::printError("Caught unexpected exception.");
        return 1;
    }

    return status;
}


// this just wraps ALL the code in total catch block
int Kernel::run(const StringList& cmdArgs, LogPtr& log)
{
    m_log = log;
    m_manager.setLog(m_log);

    ProgramArgs args;

    try
    {
        doSwitches(cmdArgs, args);
    }
    catch (const pdal_error& e)
    {
        Utils::printError(e.what());
        return 1;
    }

    if (m_showHelp)
    {
        outputHelp(args);
        return 0;
    }

    int startup_status = doStartup();
    if (startup_status)
        return startup_status;

    return doExecution(args);
}


int Kernel::innerRun(ProgramArgs& args)
{
    try
    {
        // do any user-level sanity checking
        validateSwitches(args);
    }
    catch (pdal_error e)
    {
        Utils::printError(e.what());
        outputHelp(args);
        return -1;
    }

    parseCommonOptions();
    return execute();
}


bool Kernel::isVisualize() const
{
    return m_visualize;
}


void Kernel::visualize(PointViewPtr view)
{
    PipelineManager manager;

    manager.commonOptions() = m_manager.commonOptions();
    manager.stageOptions() = m_manager.stageOptions();

    BufferReader& reader =
        static_cast<BufferReader&>(manager.makeReader("", "readers.buffer"));
    reader.addView(view);

    Stage& writer = manager.makeWriter("", "writers.pclvisualizer", reader);

    PointTable table;
    writer.prepare(table);
    writer.execute(table);
}

/*
void Kernel::visualize(PointViewPtr input_view, PointViewPtr output_view) const
{
#ifdef PDAL_HAVE_PCL_VISUALIZE
    int viewport = 0;

    // Determine XYZ bounds
    BOX3D const& input_bounds = input_view->calculateBounds();
    BOX3D const& output_bounds = output_view->calculateBounds();

    // Convert PointView to a PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pclsupport::PDALtoPCD(
        const_cast<PointViewPtr>(*input_view), *input_cloud, input_bounds);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pclsupport::PDALtoPCD(
        const_cast<PointViewPtr>(*output_view), *output_cloud, output_bounds);

    // Create PCLVisualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> p(
        new pcl::visualization::PCLVisualizer("3D Viewer"));

    // Set background to black
    p->setBackgroundColor(0, 0, 0);

    // Use Z dimension to colorize points
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
        input_color(input_cloud, "z");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
        output_color(output_cloud, "z");

    // Add point cloud to the viewer with the Z dimension color handler
    p->createViewPort(0, 0, 0.5, 1, viewport);
    p->addPointCloud<pcl::PointXYZ> (input_cloud, input_color, "cloud");
    p->createViewPort(0.5, 0, 1, 1, viewport);
    p->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "cloud1");

    p->resetCamera();

    while (!p->wasStopped())
    {
        p->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
#endif
}
*/


void Kernel::parseCommonOptions()
{
    Options& options = m_manager.commonOptions();

    if (m_visualize)
        options.add("visualize", m_visualize);
}


void Kernel::outputHelp(ProgramArgs& args)
{
    std::cout << "usage: " << "pdal " << getShortName() << " [options] " <<
        args.commandLine() << std::endl;

    std::cout << "options:" << std::endl;
    args.dump(std::cout, 2, Utils::screenWidth());

    //ABELL - Fix me.

    std::cout <<"\nFor more information, see the full documentation for "
        "PDAL at http://pdal.io/\n" << std::endl;
}


void Kernel::addBasicSwitches(ProgramArgs& args)
{
    args.add("developer-debug",
        "Enable developer debug (don't trap exceptions)", m_hardCoreDebug);
    args.add("label", "A string to label the process with", m_label);

    args.add("visualize", "Visualize result", m_visualize);
    args.add("driver", "Override reader driver", m_driverOverride, "");
}

Stage& Kernel::makeReader(const std::string& inputFile, std::string driver)
{
    return m_manager.makeReader(inputFile, driver);
}


Stage& Kernel::makeReader(const std::string& inputFile, std::string driver,
    Options options)
{
    return m_manager.makeReader(inputFile, driver, options);
}


Stage& Kernel::makeFilter(const std::string& driver)
{
    return m_manager.makeFilter(driver);
}


Stage& Kernel::makeFilter(const std::string& driver, Stage& parent)
{
    return m_manager.makeFilter(driver, parent);
}


Stage& Kernel::makeFilter(const std::string& driver, Stage& parent,
    Options options)
{
    return m_manager.makeFilter(driver, parent, options);
}


Stage& Kernel::makeWriter(const std::string& outputFile, Stage& parent,
    std::string driver)
{
    return m_manager.makeWriter(outputFile, driver, parent);
}


Stage& Kernel::makeWriter(const std::string& outputFile, Stage& parent,
    std::string driver, Options options)
{
    return m_manager.makeWriter(outputFile, driver, parent, options);
}


bool Kernel::test_parseOption(std::string o, std::string& stage,
    std::string& option, std::string& value)
{
    return parseOption(o, stage, option, value);
}

} // namespace pdal
