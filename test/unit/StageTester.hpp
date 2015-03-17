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
    static void initialize(std::shared_ptr<Stage> s, PointTablePtr table)
    {
        s->l_initialize(table);
        s->initialize();
    }
    static void processOptions(Stage& s, const Options& options)
        { s.processOptions(options); }
    static void addDimensions(Stage& s, PointLayoutPtr layout)
        { s.addDimensions(layout); }
    static void ready(Stage& s, PointTablePtr table)
        { s.ready(table); }
    static void done(Stage& s, PointTablePtr table)
    {
        s.l_done(table);
        s.done(table);
    }
    static PointViewSet run(Stage& s, PointViewPtr view)
        { return s.run(view); }
};

// Provide access to private members of Filter.
class FilterTester : public StageTester
{
public:
    static void filter(Filter& f, PointViewPtr view)
        { f.filter(view); }
};

//
// Provide access to private members of Writer.
class WriterTester : public StageTester
{
public:
    static void write(Writer& w, PointViewPtr view)
        { w.write(view); }
};

} //namespace pdal
