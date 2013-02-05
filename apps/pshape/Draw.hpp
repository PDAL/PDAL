#ifndef INCLUDED_PSHAPE_DRAW_HPP
#define INCLUDED_PSHAPE_DRAW_HPP

#include <cairo/cairo-xlib.h>
#include <X11/Xlib.h>

#include "Mathpair.hpp"

namespace Pshape
{

class HexGrid;
class Hexagon;
class Segment;

class Draw
{
public:
    Draw(HexGrid *grid_p);
    ~Draw();

    void drawHexagon(Hexagon *hex_p, bool fill = false);
    void drawSegment(Segment s);
    void drawPoint(Point p);

private:
    HexGrid *m_grid_p;
    Display *m_dpy_p;
    Window m_window;
    cairo_surface_t *m_surface_p;
    cairo_t *m_cairo_p;
};

} // namespace

#endif // file guard
