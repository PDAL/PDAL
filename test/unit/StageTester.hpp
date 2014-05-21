#pragma once

namespace pdal
{

// Provide access to private members of stage.
class StageTester
{
public:
    static void initialize(Stage *s)
        { s->initialize(); }
    static void processOptions(Stage *s, const Options& options)
        { s->processOptions(options); }
    static void buildSchema(Stage *s, Schema *schema)
        { s->buildSchema(schema); }
    static void ready(Stage *s, PointContext ctx)
        { s->ready(ctx); }
    static void done(Stage *s, PointContext ctx)
        { s->done(ctx); }
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

} //namespace pdal
