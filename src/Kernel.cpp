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

#include <pdal/pdal_config.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <pdal/pdal_config.hpp>

#include <buffer/BufferReader.hpp>

#include <memory>
#include <vector>

namespace pdal
{

namespace
{

bool parseOption(std::string o, std::string& stage, std::string& option,
    std::string& value)
{
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
    if (o[pos++] != '.')
        return false;

    // Get stage_name.
    count = Utils::extract(o, pos, islcOrDigit);
    if (std::isdigit(o[pos]))
        return false; 
    pos += count;
    stage = o.substr(0, pos);
    if (o[pos++] != '.')
        return false;

    // Get option name.
    std::string::size_type optionStart = pos;
    count = Option::parse(o, pos);
    pos += count;
    option = o.substr(optionStart, count);

    if (o[pos++] != '=')
        return false;

    // The command-line parser takes care of quotes around an argument
    // value and such.  May want to do something to handle escaped characters?
    value = o.substr(pos);
    return true;
}

} // unnamed namespace


Kernel::Kernel()
    : m_usestdin(false)
    , m_log("pdal", "stderr")
    , m_isDebug(false)
    , m_verboseLevel(0)
    , m_showVersion(false)
    , m_showTime(false)
    , m_hardCoreDebug(false)
    , m_reportDebug(false)
    , m_visualize(false)
{}


std::ostream& operator<<(std::ostream& ostr, const Kernel& kernel)
{
    ostr << "  Name: " << kernel.getName() << std::endl;
    return ostr;
}


void Kernel::doSwitches(int argc, const char *argv[], ProgramArgs& args)
{
    StringList stringArgs;

    // Scan the argument vector for extra stage options.  Pull them out and
    // stick them in the list.  Let the ProgramArgs handle everything else.
    // NOTE: This depends on the format being "option=value" rather than
    //   "option value".  This is what we've always expected, so no problem,
    //   but it would be better to be more flexible.
    for (int i = 0; i < argc; ++i)
    {
        std::string stageName, opName, value;

        if (parseOption(argv[i], stageName, opName, value))
        {
            Option op(opName, value);
            m_extraStageOptions[stageName].add(op);
        }
        else
            stringArgs.push_back(argv[i]);
    }

    try
    {
        addBasicSwitches(args);

        // parseSimple allows us to scan for the help option without
        // raising exception about missing arguments and so on.
        args.parseSimple(stringArgs);
        addSwitches(args);
        if (!m_showHelp)
            args.parse(stringArgs);
    }
    catch (arg_error& e)
    {
        throw pdal_error(e.m_error);
    }
}


int Kernel::doStartup()
{
    try
    {
        pdal::GlobalEnvironment::startup();
    }
    catch (std::exception const& e)
    {
        const std::string s("Caught exception in initialization: ");
        Utils::printError(s + e.what());
        return 1;
    }
    catch (...)
    {
        Utils::printError("Caught unknown exception in initialization");
        return 1;
    }

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


int Kernel::doShutdown()
{
    try
    {
        pdal::GlobalEnvironment::shutdown();
    }
    catch (std::exception const& e)
    {
        const std::string s("Caught exception during shutdown: ");
        Utils::printError(s + e.what());
        return 1;
    }
    catch (...)
    {
        Utils::printError("Caught unknown exception during shutdown.");
        return 1;
    }

    return 0;
}


// this just wraps ALL the code in total catch block
int Kernel::run(int argc, char const * argv[], const std::string& appName)
{
    m_appName = appName;

    ProgramArgs args;

    try
    {
        doSwitches(argc, argv, args);
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

    int execution_status = doExecution(args);

    // note we will try to shutdown cleanly even if we got an error condition
    // in the execution phase

    int shutdown_status = doShutdown();

    if (execution_status)
        return execution_status;

    return shutdown_status;
}


void Kernel::collectExtraOptions()
{
    for (const auto& o : m_extra_options)
    {
        std::string stageName, opName, value;

        if (parseOption(o, stageName, opName, value))
        {
            Option op(opName, value);
            m_extraStageOptions[stageName].add(op);
        }
    }
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

    collectExtraOptions();
    return execute();
}


bool Kernel::isDebug() const
{
    return m_isDebug;
}


uint32_t Kernel::getVerboseLevel() const
{
    return m_verboseLevel;
}


bool Kernel::isVisualize() const
{
    return m_visualize;
}


void Kernel::visualize(PointViewPtr view)
{
    BufferReader bufferReader;
    bufferReader.addView(view);

    StageFactory f;
    Stage& writer = ownStage(f.createStage("writers.pclvisualizer"));
    writer.setInput(bufferReader);

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclsupport::PDALtoPCD(const_cast<PointViewPtr>(*input_view), *input_cloud, input_bounds);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclsupport::PDALtoPCD(const_cast<PointViewPtr>(*output_view), *output_cloud, output_bounds);

    // Create PCLVisualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> p(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // Set background to black
    p->setBackgroundColor(0, 0, 0);

    // Use Z dimension to colorize points
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> input_color(input_cloud, "z");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> output_color(output_cloud, "z");

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


void Kernel::setCommonOptions(Options &options)
{
    options.add("visualize", m_visualize);

    if (m_isDebug)
    {
        options.add("debug", true);
        uint32_t verbosity(m_verboseLevel);
        if (!verbosity)
            verbosity = 1;

        options.add("verbose", verbosity);
        options.add("log", "STDERR");
    }

    auto pred = [](char c){ return (bool)strchr(",| ", c); };

    if (!m_scales.empty())
    {
        std::vector<double> scales;
        StringList scaleTokens = Utils::split2(m_scales, pred);
        for (std::string s : scaleTokens)
        {
            double val;

            if (Utils::fromString(s, val))
                scales.push_back(val);
            else
            {
                std::ostringstream oss;
                oss << getName() << ": Invalid scale value '" << s << "'." <<
                    std::endl;
                throw pdal_error(oss.str());
            }
        }
        if (scales.size() > 0)
            options.add("scale_x", scales[0]);
        if (scales.size() > 1)
            options.add("scale_y", scales[1]);
        if (scales.size() > 2)
            options.add("scale_z", scales[2]);
    }

    if (!m_offsets.empty())
    {
        std::vector<double> offsets;
        StringList offsetTokens = Utils::split2(m_offsets, pred);
        for (std::string o : offsetTokens)
        {
            double val;

            if (Utils::fromString(o, val))
                offsets.push_back(val);
            else
            {
                std::ostringstream oss;
                oss << getName() << ": Invalid offset value '" << o << "'." <<
                    std::endl;
                throw pdal_error(oss.str());
            }
        }
        if (offsets.size() > 0)
            options.add("offset_x", offsets[0]);
        if (offsets.size() > 1)
            options.add("offset_y", offsets[1]);
        if (offsets.size() > 2)
            options.add("offset_z", offsets[2]);
    }
}


void Kernel::outputHelp(ProgramArgs& args)
{
    std::cout << "usage: " << "pdal " << m_appName << " [options] " <<
        args.commandLine() << std::endl;

    std::cout << "options:" << std::endl;
    args.dump(std::cout, 2, Utils::screenWidth());

    //ABELL - Fix me.

    std::cout <<"\nFor more information, see the full documentation for "
        "PDAL at http://pdal.io/\n" << std::endl;
}


void Kernel::addBasicSwitches(ProgramArgs& args)
{
    args.add("help,h", "Print help message", m_showHelp);

    args.add("debug,d", "Enable debug mode", m_isDebug);
    args.add("developer-debug",
        "Enable developer debug (don't trap exceptions)", m_hardCoreDebug);
    args.add("label", "A string to label the process with", m_label);
    args.add("verbose,v", "Set verbose message level", m_verboseLevel);

    args.add("visualize", "Visualize result", m_visualize);
    args.add("stdin,s", "Read pipeline XML from stdin", m_usestdin);
    /**
    args.add("heartbeat", "Shell command to run for every progress heartbeat",
        m_heartbeat_shell_command);
    **/
    args.add("scale",
         "A comma-separated or quoted, space-separated list of scales to "
         "set on the output file: \n--scale 0.1,0.1,0.00001\n--scale \""
         "0.1 0.1 0.00001\"", m_scales);
    args.add("offset",
         "A comma-separated or quoted, space-separated list of offsets to "
         "set on the output file: \n--offset 0,0,0\n--offset "
         "\"1234 5678 91011\"", m_offsets);
}


Stage& Kernel::makeReader(const std::string& inputFile)
{
    if (!FileUtils::fileExists(inputFile))
        throw app_runtime_error("file not found: " + inputFile);

    StageFactory factory;
    std::string driver = factory.inferReaderDriver(inputFile);
    if (driver.empty())
        throw app_runtime_error("Cannot determine input file type of " +
            inputFile);

    Stage *stage = factory.createStage(driver);
    if (!stage)
        throw app_runtime_error("reader creation failed");
    ownStage(stage);
    return *stage;
}


Stage& Kernel::makeWriter(const std::string& outputFile, Stage& parent)
{
    pdal::StageFactory factory;

    std::string driver = factory.inferWriterDriver(outputFile);
    if (driver.empty())
        throw pdal_error("Cannot determine output file type of " +
            outputFile);
    Options options = factory.inferWriterOptionsChanges(outputFile);

    Stage *writer = factory.createStage(driver);
    if (!writer)
    {
        std::ostringstream ss;
        ss << "Error creating writer stage for file '" << outputFile << "'.";
        throw pdal_error(ss.str());
    }
    ownStage(writer);
    writer->setInput(parent);
    writer->addOptions(options);

    return *writer;
}


bool Kernel::test_parseOption(std::string o, std::string& stage,
    std::string& option, std::string& value)
{
    return parseOption(o, stage, option, value);
}

} // namespace pdal
