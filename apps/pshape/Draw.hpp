#ifndef INCLUDED_PSHAPE_DRAW_HPP
#define INCLUDED_PSHAPE_DRAW_HPP

#include <cairo/cairo.h>
#include <cairo/cairo-svg.h> 

#include "Mathpair.hpp"

namespace Pshape
{

class HexGrid;
class Hexagon;
class Segment;

struct Color
{
    Color(double red, double blue, double green) :
        m_red(red), m_blue(blue), m_green(green)
    {}

    double m_red;
    double m_blue;
    double m_green;
};

class Draw
{
public:
    Draw(HexGrid *grid_p);
    ~Draw();

    void drawHexagon(Hexagon *hex_p, bool fill = false);
    void drawSegment(Segment s, Color c = Color(0, 0, 1));
    void drawPoint(Point p);

private:
    HexGrid *m_grid_p;
    // Display *m_dpy_p;
    // Window m_window;
    cairo_surface_t *m_surface_p;
    cairo_t *m_cairo_p;
};

} // namespace

#endif // file guard
