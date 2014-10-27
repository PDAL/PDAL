/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/kernel/Translate.hpp>

#include <pdal/drivers/buffer/BufferReader.hpp>
#include <pdal/filters/Crop.hpp>
#include <pdal/filters/Decimation.hpp>
#include <pdal/filters/Reprojection.hpp>
#include <pdal/kernel/Support.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{
namespace kernel
{

Translate::Translate() :
    Kernel(), m_bCompress(false),
    m_input_srs(pdal::SpatialReference()),
    m_output_srs(pdal::SpatialReference()), m_bForwardMetadata(false),
    m_decimation_step(1), m_decimation_offset(0), m_decimation_leaf_size(1), m_decimation_limit(0)
{}


void Translate::validateSwitches()
{
    if (m_inputFile == "")
        throw app_usage_error("--input/-i required");
    if (m_outputFile == "")
        throw app_usage_error("--output/-o required");
    //
    // auto options = getExtraOptions();
    //
    // for (auto o: options)
    // {
    //
    //     typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
    //
    //     // if we don't have --, we're not an option we
    //     // even care about
    //     if (!boost::algorithm::find_first(o, "--")) continue;
    //
    //     // Find the dimensions listed and put them on the id list.
    //     boost::char_separator<char> equal("=");
    //     boost::char_separator<char> dot(".");
    //     // boost::erase_all(o, " "); // Wipe off spaces
    //     tokenizer option_tokens(o, equal);
    //     std::vector<std::string> option_split;
    //     for (auto ti = option_tokens.begin(); ti != option_tokens.end(); ++ti)
    //         option_split.push_back(boost::lexical_cast<std::string>(*ti));
    //     if (! (option_split.size() == 2))
    //     {
    //         std::ostringstream oss;
    //         oss << "option '" << o << "' did not split correctly. Is it in the form --drivers.las.reader.option=foo?";
    //         throw app_usage_error(oss.str());
    //     }
    //
    //     std::string option_value(option_split[1]);
    //     std::string stage_value(option_split[0]);
    //     boost::algorithm::erase_all(stage_value, "--");
    //
    //     tokenizer name_tokens(stage_value, dot);
    //     std::vector<std::string> stage_values;
    //     for (auto ti = name_tokens.begin(); ti != name_tokens.end(); ++ti)
    //     {
    //         stage_values.push_back(*ti);
    //     }
    //
    //     std::string option_name = *stage_values.rbegin();
    //     std::ostringstream stage_name_ostr;
    //     bool bFirst(true);
    //     for (auto s = stage_values.begin(); s != stage_values.end()-1; ++s)
    //     {
    //         auto s2 = boost::algorithm::erase_all_copy(*s, " ");
    //
    //         if (bFirst)
    //         {
    //             bFirst = false;
    //         } else
    //             stage_name_ostr <<".";
    //         stage_name_ostr << s2;
    //     }
    //     std::string stage_name(stage_name_ostr.str());
    //     std::cout << "stage name: '" << stage_name << "' option_name: '" << option_name << "' option value: '" << option_value <<"'"<<std::endl;
    //
    //     auto found = m_stage_options.find(stage_name);
    //     if (found == m_stage_options.end())
    //         m_stage_options.insert(std::make_pair(stage_name, Option(option_name, option_value, "")));
    //     else
    //         found->second.add(Option(option_name, option_value, ""));
    // }

}


void Translate::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
         "input file name")
        ("output,o", po::value<std::string>(&m_outputFile)->default_value(""),
         "output file name")
        ("a_srs", po::value<pdal::SpatialReference>(&m_input_srs),
         "Assign input coordinate system (if supported by output format)")
        ("t_srs", po::value<pdal::SpatialReference>(&m_output_srs),
         "Transform to output coordinate system (if supported by "
         "output format)")
        ("compress,z",
         po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true),
         "Compress output data (if supported by output format)")
        ("bounds", po::value<BOX3D >(&m_bounds),
         "Extent (in XYZ to clip output to)")
        ("polygon", po::value<std::string >(&m_wkt),
         "POLYGON WKT to use for precise crop of data (2d or 3d)")
        ("metadata,m",
         po::value< bool >(&m_bForwardMetadata)->implicit_value(true),
         "Forward metadata (VLRs, header entries, etc) from previous stages")
        ("d_step",
         po::value<uint32_t>(&m_decimation_step)->default_value(1),
         "Decimation filter step")
        ("d_offset",
         po::value<uint32_t>(&m_decimation_offset)->default_value(0),
         "Decimation filter offset")
        ("d_leaf_size",
         po::value<double>(&m_decimation_leaf_size)->default_value(1),
         "Decimation filter leaf size")
        ("d_method",
         po::value<std::string>(&m_decimation_method)->default_value("RankOrder"),
         "Decimation filter method (RankOrder, VoxelGrid)")
        ("d_limit",
         po::value<point_count_t>(&m_decimation_limit)->default_value(0),
         "Decimation limit")
        ;

    addSwitchSet(file_options);
    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
}

std::unique_ptr<Stage> Translate::makeReader(Options readerOptions)
{
    if (isDebug())
    {
        readerOptions.add("debug", true);
        uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
            verbosity = 1;

        readerOptions.add("verbose", verbosity);
        readerOptions.add("log", "STDERR");
    }

    Stage* stage = AppSupport::makeReader(m_inputFile);
    stage->setOptions(readerOptions);
    std::unique_ptr<Stage> reader_stage(stage);

    return reader_stage;
}


Stage* Translate::makeTranslate(Options translateOptions, Stage* reader_stage)
{
    Stage* final_stage = reader_stage;
    Options readerOptions = reader_stage->getOptions();
    std::map<std::string, Options> extra_opts = getExtraStageOptions();
    if (!m_bounds.empty() || !m_wkt.empty() || !m_output_srs.empty() || extra_opts.size() > 0)
    {
        Stage* next_stage = reader_stage;
        Stage* crop_stage(0);
        Stage* reprojection_stage(0);


        bool bHaveReprojection = extra_opts.find("filters.reprojection") != extra_opts.end();
        bool bHaveCrop = extra_opts.find("filters.crop") != extra_opts.end();

        if (!m_output_srs.empty())
        {
            translateOptions.add("out_srs", m_output_srs.getWKT());
            reprojection_stage =
                new filters::Reprojection();
            reprojection_stage->setInput(next_stage);
            reprojection_stage->setOptions(readerOptions);
            next_stage = reprojection_stage;
        } else if (bHaveReprojection)
        {
            reprojection_stage =
                new filters::Reprojection();
            reprojection_stage->setInput(next_stage);
            reprojection_stage->setOptions(extra_opts.find("filters.reprojection")->second);
            next_stage = reprojection_stage;
        }

        if ((!m_bounds.empty() && m_wkt.empty()))
        {
            readerOptions.add("bounds", m_bounds);
            crop_stage = new pdal::filters::Crop();
            crop_stage->setInput(next_stage);
            crop_stage->setOptions(readerOptions);
            next_stage = crop_stage;
        }
        else if (m_bounds.empty() && !m_wkt.empty())
        {
            std::istream* wkt_stream;
            try
            {
                wkt_stream = FileUtils::openFile(m_wkt);
                std::stringstream buffer;
                buffer << wkt_stream->rdbuf();

                m_wkt = buffer.str();
                FileUtils::closeFile(wkt_stream);

            }
            catch (pdal::pdal_error const&)
            {
                // If we couldn't open the file given in m_wkt because it
                // was likely actually wkt, leave it alone
            }
            readerOptions.add("polygon", m_wkt);
            crop_stage = new pdal::filters::Crop();
            crop_stage->setInput(next_stage);
            crop_stage->setOptions(readerOptions);
            next_stage = crop_stage;
        } else if (bHaveCrop)
        {
            crop_stage = new pdal::filters::Crop();
            crop_stage->setInput(next_stage);
            crop_stage->setOptions(extra_opts.find("filters.crop")->second);
            next_stage = crop_stage;
        }
        final_stage = next_stage;
    }

    if (boost::iequals(m_decimation_method, "VoxelGrid"))
    {
        StageFactory f;
        StageFactory::FilterCreator* fc = f.getFilterCreator("filters.pclblock");
        if (fc)
        {
            Stage* decimation_stage = fc();

            Options decimationOptions;
            std::ostringstream ss;
            ss << "{";
            ss << "  \"pipeline\": {";
            ss << "    \"filters\": [{";
            ss << "      \"name\": \"VoxelGrid\",";
            ss << "      \"setLeafSize\": {";
            ss << "        \"x\": " << m_decimation_leaf_size << ",";
            ss << "        \"y\": " << m_decimation_leaf_size << ",";
            ss << "        \"z\": " << m_decimation_leaf_size;
            ss << "        }";
            ss << "      }]";
            ss << "    }";
            ss << "}";
            std::string json = ss.str();
            decimationOptions.add<std::string>("json", json);
            decimationOptions.add<bool>("debug", isDebug());
            decimationOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
            decimation_stage->setOptions(decimationOptions);
            decimation_stage->setInput(final_stage);
            final_stage = decimation_stage;
        }
        else
        {
            std::ostringstream oss;
            oss << "Unable to create filter for type 'drivers.pclblock.filter'. Does a driver with this type name exist?";
            throw pdal_error(oss.str());
        }
    }
    else if (m_decimation_step > 1 || m_decimation_limit > 0)
    {
        Options decimationOptions;
        decimationOptions.add("debug", isDebug());
        decimationOptions.add("verbose", getVerboseLevel());
        decimationOptions.add("step", m_decimation_step);
        decimationOptions.add("offset", m_decimation_offset);
        decimationOptions.add("limit", m_decimation_limit);
        Stage *decimation_stage = new filters::Decimation();
        decimation_stage->setInput(final_stage);
        decimation_stage->setOptions(decimationOptions);
        final_stage = decimation_stage;
    }

    return final_stage;
}

int Translate::execute()
{
    PointContext ctx;

    Options readerOptions;
    readerOptions.add("filename", m_inputFile);
    readerOptions.add("debug", isDebug());
    readerOptions.add("verbose", getVerboseLevel());
    if (!m_input_srs.empty())
        readerOptions.add("spatialreference", m_input_srs.getWKT());

    std::unique_ptr<Stage> readerStage = makeReader(readerOptions);

    // go ahead and prepare/execute on reader stage only to grab input
    // PointBufferSet, this makes the input PointBuffer available to both the
    // processing pipeline and the visualizer
    readerStage->prepare(ctx);
    PointBufferSet pbSetIn = readerStage->execute(ctx);

    // the input PointBufferSet will be used to populate a BufferReader that is
    // consumed by the processing pipeline
    PointBufferPtr input_buffer = *pbSetIn.begin();
    drivers::buffer::BufferReader bufferReader;
    bufferReader.setOptions(readerOptions);
    bufferReader.addBuffer(input_buffer);

    // the translation consumes the BufferReader rather than the readerStage
    Stage* finalStage = makeTranslate(readerOptions, &bufferReader);

    Options writerOptions;
    writerOptions.add("filename", m_outputFile);
    setCommonOptions(writerOptions);

    if (!m_input_srs.empty())
        writerOptions.add("spatialreference", m_input_srs.getWKT());

    if (m_bCompress)
        writerOptions.add("compression", true);
    if (m_bForwardMetadata)
        writerOptions.add("forward_metadata", true);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

    std::unique_ptr<Writer> writer( AppSupport::makeWriter(m_outputFile, finalStage));
    if (!m_output_srs.empty())
        writer->setSpatialReference(m_output_srs);

    // Some options are inferred by makeWriter based on filename
    // (compression, driver type, etc).
    writer->setOptions(writerOptions+writer->getOptions());

    writer->setUserCallback(callback);

    for (auto pi: getExtraStageOptions())
    {
        std::string name = pi.first;
        Options options = pi.second;
        std::vector<Stage*> stages = writer->findStage(name);
        for (auto s: stages)
        {
            Options opts = s->getOptions();
            for (auto o: options.getOptions())
                opts.add(o);
            s->setOptions(opts);
        }
    }
    writer->prepare(ctx);

    // process the data, grabbing the PointBufferSet for visualization of the
    PointBufferSet pbSetOut = writer->execute(ctx);

    if (isVisualize())
        visualize(*pbSetOut.begin());
    //visualize(*pbSetIn.begin(), *pbSetOut.begin());

    return 0;
}

} // namespace kernel
} // namespace pdal
