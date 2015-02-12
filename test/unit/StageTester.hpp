#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>

namespace pdal
{

// Provide access to private members of stage.
class StageTester
{
public:
    static void initialize(std::shared_ptr<Stage> s, PointContext ctx)
    {
        s->l_initialize(ctx);
        s->initialize();
    }
    static void processOptions(std::shared_ptr<Stage> s, const Options& options)
        { s->processOptions(options); }
    static void addDimensions(std::shared_ptr<Stage> s, PointContext ctx)
        { s->addDimensions(ctx); }
    static void ready(std::shared_ptr<Stage> s, PointContext ctx)
        { s->ready(ctx); }
    static void done(std::shared_ptr<Stage> s, PointContext ctx)
    {
        s->l_done(ctx);
        s->done(ctx);
    }
    static PointBufferSet run(std::shared_ptr<Stage> s, PointBufferPtr buffer)
        { return s->run(buffer); }
};

// Provide access to private members of Filter.
class FilterTester : public StageTester
{
public:
    static void filter(std::shared_ptr<Filter> f, PointBuffer& buffer)
        { f->filter(buffer); }
};

//
// Provide access to private members of Writer.
class WriterTester : public StageTester
{
public:
    static void write(std::shared_ptr<Writer> w, PointBuffer& buffer)
        { w->write(buffer); }
};

} //namespace pdal
