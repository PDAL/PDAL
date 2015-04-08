#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>

namespace pdal
{

// Provide access to private members of stage.
class StageWrapper
{
public:
    static void initialize(std::shared_ptr<Stage> s, PointTableRef table)
    {
        s->l_initialize(table);
        s->initialize();
    }
    static void processOptions(Stage& s, const Options& options)
        { s.processOptions(options); }
    static void addDimensions(Stage& s, PointLayoutPtr layout)
        { s.addDimensions(layout); }
    static void ready(Stage& s, PointTableRef table)
        { s.ready(table); }
    static void done(Stage& s, PointTableRef table)
    {
        s.l_done(table);
        s.done(table);
    }
    static PointViewSet run(Stage& s, PointViewPtr view)
        { return s.run(view); }
};

// Provide access to private members of Filter.
class FilterWrapper : public StageWrapper
{
public:
    static void filter(Filter& f, PointView& view)
        { f.filter(view); }
};

// Provide access to private members of Writer.
class WriterWrapper : public StageWrapper
{
public:
    static void write(Writer& w, PointViewPtr view)
        { w.write(view); }
};

} //namespace pdal
