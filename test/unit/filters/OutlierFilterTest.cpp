/******************************************************************************
 * Copyright (c) 2021 Kenneth Tussey (kenneth.tussey@scientiallc.com)
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
 *     * Neither the name of Scientia LLC nor the
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

#include <pdal/pdal_test_main.hpp>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include <thread>

#include "Support.hpp"

namespace pdal
{

// Run outlier list with previous serial outlier filter
// for a given faux reader seed, and save values.
const std::vector<PointId> truth_radius_outliers =
{
    1, 2, 4, 5, 6, 7, 9, 14, 15, 17, 18, 22, 23,
    25, 26, 28, 34, 36, 37, 38, 39, 43
};

const std::vector<PointId> truth_statistical_outliers =
{
    2, 6, 7, 17, 28, 36, 41, 44, 47
};

// Faux reader ops used to generate above truth points.
Options default_reader_ops() {
    Options ops;
    ops.add("mode", "normal");
    ops.add("count", 50);
    ops.add("seed", 0);
    return ops;
}


TEST(OutlierFilterTest, serial_radius)
{
    using namespace Dimension;

    PipelineManager pipeline;
    Stage &reader = pipeline.addReader("readers.faux");
    reader.setOptions(default_reader_ops());

    Options filterOps;
    filterOps.add("method", "radius");
    filterOps.add("radius", 0.75);
    Stage &outlier_filter = pipeline.makeFilter("filters.outlier", filterOps);
    outlier_filter.setInput(reader);

    pipeline.prepare();
    PointViewSet viewSet = outlier_filter.execute(pipeline.pointTable());
    PointViewPtr outView = *viewSet.begin();

    std::vector<PointId> found_outliers;
    for (PointRef p : *outView) {
        if (p.getFieldAs<uint8_t>(Id::Classification) == ClassLabel::LowPoint) {
            found_outliers.push_back(p.pointId());
        }
    }
    ASSERT_EQ(found_outliers.size(), truth_radius_outliers.size());
    for (PointId truth : truth_radius_outliers) {
        ASSERT_TRUE(std::count(found_outliers.begin(), found_outliers.end(), truth));
    }
}

TEST(OutlierFilterTest, parallel_radius)
{
    using namespace Dimension;

    PipelineManager pipeline;
    Stage &reader = pipeline.addReader("readers.faux");
    reader.setOptions(default_reader_ops());

    Options filterOps;
    filterOps.add("method", "radius");
    filterOps.add("radius", 0.75);
    filterOps.add("threads", std::thread::hardware_concurrency());
    Stage &outlier_filter = pipeline.makeFilter("filters.outlier", filterOps);
    outlier_filter.setInput(reader);

    pipeline.prepare();
    PointViewSet viewSet = outlier_filter.execute(pipeline.pointTable());
    PointViewPtr outView = *viewSet.begin();

    std::vector<PointId> found_outliers;
    for (PointRef p : *outView) {
        if (p.getFieldAs<uint8_t>(Id::Classification) == ClassLabel::LowPoint) {
            found_outliers.push_back(p.pointId());
        }
    }
    ASSERT_EQ(found_outliers.size(), truth_radius_outliers.size());
    for (PointId truth : truth_radius_outliers) {
        ASSERT_TRUE(std::count(found_outliers.begin(), found_outliers.end(), truth));
    }
}

TEST(OutlierFilterTest, serial_statistical)
{
    using namespace Dimension;

    PipelineManager pipeline;
    Stage &reader = pipeline.addReader("readers.faux");
    reader.setOptions(default_reader_ops());

    Options filterOps;
    filterOps.add("method", "statistical");
    filterOps.add("multiplier", 1.0);
    Stage &outlier_filter = pipeline.makeFilter("filters.outlier", filterOps);
    outlier_filter.setInput(reader);

    pipeline.prepare();
    PointViewSet viewSet = outlier_filter.execute(pipeline.pointTable());
    PointViewPtr outView = *viewSet.begin();

    std::vector<PointId> found_outliers;
    for (PointRef p : *outView) {
        if (p.getFieldAs<uint8_t>(Id::Classification) == ClassLabel::LowPoint) {
            found_outliers.push_back(p.pointId());
        }
    }
    ASSERT_EQ(found_outliers.size(), truth_statistical_outliers.size());
    for (PointId truth : truth_statistical_outliers) {
        ASSERT_TRUE(std::count(found_outliers.begin(), found_outliers.end(), truth));
    }
}

TEST(OutlierFilterTest, parallel_statistical)
{
    using namespace Dimension;

    PipelineManager pipeline;
    Stage &reader = pipeline.addReader("readers.faux");
    Options readerOps;
    reader.setOptions(default_reader_ops());

    Options filterOps;
    filterOps.add("method", "statistical");
    filterOps.add("multiplier", 1.0);
    filterOps.add("threads", std::thread::hardware_concurrency());
    Stage &outlier_filter = pipeline.makeFilter("filters.outlier", filterOps);
    outlier_filter.setInput(reader);

    pipeline.prepare();
    PointViewSet viewSet = outlier_filter.execute(pipeline.pointTable());
    PointViewPtr outView = *viewSet.begin();

    std::vector<PointId> found_outliers;
    for (PointRef p : *outView) {
        if (p.getFieldAs<uint8_t>(Id::Classification) == ClassLabel::LowPoint) {
            found_outliers.push_back(p.pointId());
        }
    }
    ASSERT_EQ(found_outliers.size(), truth_statistical_outliers.size());
    for (PointId truth : truth_statistical_outliers) {
        ASSERT_TRUE(std::count(found_outliers.begin(), found_outliers.end(), truth));
    }
}

TEST(OutlierFilterTest, time_comparisons) {
    using namespace Dimension;
    using namespace std::chrono;

    const size_t point_count = 10000;
    const unsigned int hw_threads = std::thread::hardware_concurrency();

    PointViewPtr serial_radius_outliers, parallel_radius_outliers,
        serial_statistical_outliers, parallel_statistical_outliers;

    PipelineManager serial_radius_pipeline, parallel_radius_pipeline,
        serial_statistical_pipeline, parallel_statistical_pipeline;

    Options reader_ops;
    reader_ops.add("mode", "normal");
    reader_ops.add("count", point_count);
    reader_ops.add("seed", 0);

    Options serial_radius_ops;
    serial_radius_ops.add("method", "radius");
    serial_radius_ops.add("radius", 0.75);

    Options parallel_radius_ops = serial_radius_ops;
    parallel_radius_ops.add("threads", hw_threads);

    Options serial_stats_ops;
    serial_stats_ops.add("method", "statistical");
    serial_stats_ops.add("multiplier", 1.0);

    Options parallel_stats_ops = serial_stats_ops;
    parallel_stats_ops.add("threads", hw_threads);

    // Serial radius processing
    {
        Stage &reader = serial_radius_pipeline.addReader("readers.faux");
        reader.setOptions(reader_ops);

        Stage &outlier_filter =
            serial_radius_pipeline.makeFilter("filters.outlier", serial_radius_ops);
        outlier_filter.setInput(reader);

        serial_radius_pipeline.prepare();
        time_point<system_clock> tick = system_clock::now();
        PointViewSet viewSet =
            outlier_filter.execute(serial_radius_pipeline.pointTable());
        time_point<system_clock> tock = system_clock::now();

        auto diff = tock - tick;
        std::cout
            << "Serial radius search (including faux point initialization) "
            << point_count << " points: "
            << duration_cast<milliseconds>(diff).count()
            << " ms. " << std::endl;

        serial_radius_outliers = *viewSet.begin();
    }

    // Parallel radius processing
    {

        Stage &reader = parallel_radius_pipeline.addReader("readers.faux");
        reader.setOptions(reader_ops);

        Stage &outlier_filter =
            parallel_radius_pipeline.makeFilter("filters.outlier", parallel_radius_ops);
        outlier_filter.setInput(reader);

        parallel_radius_pipeline.prepare();
        time_point<system_clock> tick = system_clock::now();
        PointViewSet viewSet =
            outlier_filter.execute(parallel_radius_pipeline.pointTable());
        time_point<system_clock> tock = system_clock::now();

        auto diff = tock - tick;
        std::cout
            << "Parallel radius search (including faux point initialization) "
            << point_count << " points: "
            << duration_cast<milliseconds>(diff).count()
            << " ms using " << hw_threads
            << " threads" << std::endl;

        parallel_radius_outliers = *viewSet.begin();
    }

    // Serial stats processing
    {
        Stage &reader = serial_statistical_pipeline.addReader("readers.faux");
        reader.setOptions(reader_ops);

        Stage &outlier_filter =
            serial_statistical_pipeline.makeFilter("filters.outlier", serial_stats_ops);
        outlier_filter.setInput(reader);

        serial_statistical_pipeline.prepare();
        time_point<system_clock> tick = system_clock::now();
        PointViewSet viewSet =
            outlier_filter.execute(serial_statistical_pipeline.pointTable());
        time_point<system_clock> tock = system_clock::now();

        auto diff = tock - tick;
        std::cout
            << "Serial statistical search (including faux point initialization) "
            << point_count << " points: "
            << duration_cast<milliseconds>(diff).count()
            << " ms. " << std::endl;

        serial_statistical_outliers = *viewSet.begin();
    }

    // Parallel radius processing
    {

        Stage &reader = parallel_statistical_pipeline.addReader("readers.faux");
        reader.setOptions(reader_ops);

        Stage &outlier_filter =
            parallel_statistical_pipeline.makeFilter("filters.outlier", parallel_stats_ops);
        outlier_filter.setInput(reader);

        parallel_statistical_pipeline.prepare();
        time_point<system_clock> tick = system_clock::now();
        PointViewSet viewSet =
            outlier_filter.execute(parallel_statistical_pipeline.pointTable());
        time_point<system_clock> tock = system_clock::now();

        auto diff = tock - tick;
        std::cout
            << "Parallel statistical search (including faux point initialization) "
            << point_count << " points: "
            << duration_cast<milliseconds>(diff).count()
            << " ms using " << hw_threads
            << " threads" << std::endl;

        parallel_statistical_outliers = *viewSet.begin();
    }

    ASSERT_EQ(parallel_radius_outliers->size(), serial_radius_outliers->size());
    ASSERT_EQ(parallel_statistical_outliers->size(), serial_statistical_outliers->size());

}

TEST(OutlierFilterTest, thread_count_validation) {
    using namespace Dimension;
    using namespace std::chrono;

    const size_t point_count = 100;
    auto log = Log::makeLog("outlier_filter_test", "stdout");

    // if threads are > hardware_concurrency, warn but accept.
    {
        PipelineManager pipeline;
        Options reader_ops;
        reader_ops.add("mode", "normal");
        reader_ops.add("count", point_count);
        Stage &reader = pipeline.addReader("readers.faux");
        reader.setOptions(reader_ops);

        Options filter_ops;
        filter_ops.add("method", "statistical");
        filter_ops.add("threads", std::thread::hardware_concurrency() + 1);
        Stage &filter = pipeline.makeFilter("filters.outlier", filter_ops);
        filter.setLog(log);
        filter.setInput(reader);

        pipeline.prepare();
        pipeline.execute();
    }

    // if threads are set to < 1, warn set to 1 and run.
    {
        PipelineManager pipeline;
        Options reader_ops;
        reader_ops.add("mode", "normal");
        reader_ops.add("count", point_count);
        Stage &reader = pipeline.addReader("readers.faux");
        reader.setOptions(reader_ops);

        Options filter_ops;
        filter_ops.add("method", "statistical");
        filter_ops.add("threads", 0);
        Stage &filter = pipeline.makeFilter("filters.outlier", filter_ops);
        filter.setLog(log);
        filter.setInput(reader);

        pipeline.prepare();
        pipeline.execute();
    }


}


} // namespace pdal
