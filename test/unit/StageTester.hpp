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
    static void initialize(Stage *s, PointContext ctx)
    {
        s->l_initialize(ctx);
        s->initialize();
    }
    static void processOptions(Stage *s, const Options& options)
        { s->processOptions(options); }
    static void addDimensions(Stage *s, PointContext ctx)
        { s->addDimensions(ctx); }
    static void ready(Stage *s, PointContext ctx)
        { s->ready(ctx); }
    static void done(Stage *s, PointContext ctx)
    {
        s->l_done(ctx);
        s->done(ctx);
    }
    static PointBufferSet run(Stage *s, PointBufferPtr buffer)
        { return s->run(buffer); }
};

// Provide access to private members of Filter.
class FilterTester : public StageTester
{
public:
    static void filter(Filter *f, PointBuffer& buffer)
        { f->filter(buffer); }
};

//
// Provide access to private members of Writer.
class WriterTester : public StageTester
{
public:
    static void write(Writer *w, PointBuffer& buffer)
        { w->write(buffer); }
};

} //namespace pdal
