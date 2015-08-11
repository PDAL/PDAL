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

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/PDALUtils.hpp>
#include <iostream>

#include <boost/algorithm/string.hpp>

#include <pdal/pdal_config.hpp>
#include <pdal/StageFactory.hpp>

#include <pdal/pdal_config.hpp>

#include <pdal/BufferReader.hpp>

#include <memory>
#include <vector>

#include <boost/tokenizer.hpp>
typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

namespace po = boost::program_options;

namespace pdal
{

Kernel::Kernel()
    : m_usestdin(false)
    , m_log("pdal", "stderr")
    , m_isDebug(false)
    , m_verboseLevel(0)
    , m_showHelp(false)
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


int Kernel::do_switches()
{
    try
    {
        // add -h, -v, etc
        addBasicSwitchSet();

        // add the options for the derived application
        addSwitches();

        // parse the command line
        parseSwitches();
    }
    catch (std::exception const& e)
    {
        Utils::printError(e.what());
        return 1;
    }
    catch (...)
    {
        Utils::printError("Caught unknown exception handling switches");
        return 1;
    }
    return 0;
}


int Kernel::do_startup()
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


int Kernel::do_execution()
{

    if (m_reportDebug)
    {
        std::cout << getPDALDebugInformation() << std::endl;
        return 0;
    }

    if (m_hardCoreDebug)
    {
        int status = innerRun();
        return status;
    }

    int status = 1;

    try
    {
        status = innerRun();
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


int Kernel::do_shutdown()
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
int Kernel::run(int argc, const char* argv[], const std::string& appName)
{
    m_argc = argc;
    m_argv = argv;
    m_appName = appName;

    int switches_status = do_switches();
    if (switches_status)
        return switches_status;

    int startup_status = do_startup();
    if (startup_status)
        return startup_status;

    int execution_status = do_execution();

    // note we will try to shutdown cleanly even if we got an error condition
    // in the execution phase

    int shutdown_status = do_shutdown();

    if (execution_status)
        return execution_status;

    return shutdown_status;
}


void Kernel::collectExtraOptions()
{
    for (const auto& o : m_extra_options)
    {
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

        // if we don't have --, we're not an option we
        // even care about
        if (!boost::algorithm::find_first(o, "--"))
            continue;

        // Find the dimensions listed and put them on the id list.
        boost::char_separator<char> equal("=");
        boost::char_separator<char> dot(".");
        tokenizer option_tokens(o, equal);
        std::vector<std::string> option_split;
        for (auto const& ti : option_tokens)
            option_split.push_back(boost::lexical_cast<std::string>(ti));
        if (!(option_split.size() == 2))
        {
//             std::ostringstream oss;
//             oss << "option '" << o << "' did not split correctly. Is it "
//                 "in the form --readers.las.option=foo?";
//             throw app_usage_error(oss.str());
            continue;
        }

        std::string option_value(option_split[1]);
        std::string stage_value(option_split[0]);
        boost::algorithm::erase_all(stage_value, "--");

        tokenizer name_tokens(stage_value, dot);
        std::vector<std::string> stage_values;
        for (auto const& ti : name_tokens)
            stage_values.push_back(ti);

        std::string option_name = *stage_values.rbegin();
        std::ostringstream stage_name_ostr;
        bool bFirst(true);
        for (auto s = stage_values.begin(); s != stage_values.end() - 1; ++s)
        {
            auto s2 = boost::algorithm::erase_all_copy(*s, " ");

            if (bFirst)
            {
                bFirst = false;
            }
            else
                stage_name_ostr <<".";
            stage_name_ostr << s2;
        }
        std::string stageName(stage_name_ostr.str());

        Option op(option_name, option_value);
        m_extraStageOptions[stageName].add(op);
    }
}


bool Kernel::argumentSpecified(const std::string& name)
{
    auto ai = m_variablesMap.find(name);
    if (ai == m_variablesMap.end())
        return false;
    return !(ai->second.defaulted());
}


int Kernel::innerRun()
{
    // handle the well-known options
    if (m_showVersion)
    {
        outputVersion();
        return 0;
    }

    if (m_showHelp)
    {
        outputHelp();
        return 0;
    }

    if (!m_showOptions.empty())
    {
        return 0;
    }

    try
    {
        // do any user-level sanity checking
        validateSwitches();
        collectExtraOptions();
    }
    catch (app_usage_error e)
    {
        Utils::printError(std::string("Usage error: ") + e.what());
        outputHelp();
        return 1;
    }

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


void Kernel::addSwitchSet(po::options_description* options)
{
    if (options)
        m_public_options.push_back(options);
}


void Kernel::addHiddenSwitchSet(po::options_description* options)
{
    if (options)
        m_hidden_options.push_back(options);
}


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

    boost::char_separator<char> sep(",| ");

    if (argumentExists("scale"))
    {
        std::vector<double> scales;
        tokenizer scale_tokens(m_scales, sep);
        for (auto const& t : scale_tokens)
            scales.push_back(boost::lexical_cast<double>(t));
        if (scales.size())
        {
            if (scales.size() <= 1)
            {
                options.add<double >("scale_x", scales[0]);
            }
            else if (scales.size() <= 2)
            {
                options.add<double >("scale_x", scales[0]);
                options.add<double >("scale_y", scales[1]);
            }
            else if (scales.size() <= 3)
            {
                options.add<double >("scale_x", scales[0]);
                options.add<double >("scale_y", scales[1]);
                options.add<double >("scale_z", scales[2]);
            }
        }
    }

    if (argumentExists("offset"))
    {
        std::vector<std::string> offsets;
        tokenizer offset_tokens(m_offsets, sep);
        for (auto const& t : offset_tokens)
            offsets.push_back(boost::lexical_cast<std::string>(t));
        if (offsets.size())
        {
            if (offsets.size() <= 1)
            {
                options.add<std::string>("offset_x", offsets[0]);
            }
            else if (offsets.size() <= 2)
            {
                options.add<std::string>("offset_x", offsets[0]);
                options.add<std::string>("offset_y", offsets[1]);
            }
            else if (offsets.size() <= 3)
            {
                options.add<std::string>("offset_x", offsets[0]);
                options.add<std::string>("offset_y", offsets[1]);
                options.add<std::string>("offset_z", offsets[2]);
            }
        }
    }
}


void Kernel::addPositionalSwitch(const char* name, int max_count)
{
    m_positionalOptions.add(name, max_count);
}


void Kernel::outputHelp()
{
    outputVersion();

    for (auto const& iter : m_public_options)
    {
        std::cout << *iter;
        std::cout << std::endl;
    }

    std::cout <<"\nFor more information, see the full documentation for "
        "PDAL at http://pdal.io/\n" << std::endl << std::endl;
}


void Kernel::outputVersion()
{
    std::cout << "pdal " << m_appName << " (" <<
        GetFullVersionString() << ")\n";
    std::cout << std::endl;
}


void Kernel::addBasicSwitchSet()
{
    po::options_description* basic_options =
        new po::options_description("basic options");

    basic_options->add_options()
    ("help,h",
        po::value<bool>(&m_showHelp)->zero_tokens()->implicit_value(true),
        "Print help message")
    ("options", po::value<std::string>(&m_showOptions)->implicit_value("all"),
        "Show available options for a driver")
    ("debug,d",
        po::value<bool>(&m_isDebug)->zero_tokens()->implicit_value(true),
        "Enable debug mode")
    ("report-debug",
        po::value<bool>(&m_reportDebug)->zero_tokens()->implicit_value(true),
        "Report PDAL compilation DEBUG status")
    ("developer-debug",
        po::value<bool>(&m_hardCoreDebug)->zero_tokens()->implicit_value(true),
        "Enable developer debug mode (don't trap exceptions so segfaults "
        "are thrown)")
    ("label",
        po::value<std::string>(&m_label)->default_value(""),
        "A string to label the process with")
    ("verbose,v", po::value<uint32_t>(&m_verboseLevel)->default_value(0),
        "Set verbose message level")
    ("version",
        po::value<bool>(&m_showVersion)->zero_tokens()->implicit_value(true),
        "Show version info")
    ("visualize",
        po::value<bool>(&m_visualize)->zero_tokens()->implicit_value(true),
        "Visualize result")
    ("stdin,s",
        po::value<bool>(&m_usestdin)->zero_tokens()->implicit_value(true),
        "Read pipeline XML from stdin")
    ("heartbeat",
        po::value< std::vector<std::string> >(&m_heartbeat_shell_command),
        "Shell command to run for every progress heartbeat")
    ("scale", po::value< std::string >(&m_scales),
         "A comma-separated or quoted, space-separated list of scales to "
         "set on the output file: \n--scale 0.1,0.1,0.00001\n--scale \""
         "0.1 0.1 0.00001\"")
    ("offset", po::value< std::string >(&m_offsets),
         "A comma-separated or quoted, space-separated list of offsets to "
         "set on the output file: \n--offset 0,0,0\n--offset "
         "\"1234 5678 91011\"")
    ;

    addSwitchSet(basic_options);
}


void Kernel::parseSwitches()
{
    po::options_description options;

    for (auto const& iter : m_public_options)
        options.add(*iter);
    for (auto const& iter : m_hidden_options)
        options.add(*iter);

    try
    {
        auto parsed = po::command_line_parser(m_argc, m_argv).
            options(options).allow_unregistered().
            positional(m_positionalOptions).run();
        m_extra_options = po::collect_unrecognized(parsed.options,
            po::include_positional);

        po::store(parsed, m_variablesMap);
    }
    catch (boost::program_options::unknown_option e)
    {
        throw app_usage_error("unknown option: " + e.get_option_name());
    }
    po::notify(m_variablesMap);
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
    writer->setOptions(options + writer->getOptions());

    return *writer;
}

} // namespace pdal

