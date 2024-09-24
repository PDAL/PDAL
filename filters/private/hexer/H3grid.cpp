#include <assert.h>
#include <sstream>

#include <h3api.h>

#include "BaseGrid.hpp"
#include "H3grid.hpp"
#include "Processor.hpp"

namespace hexer
{

H3Grid::~H3Grid()
{}

void H3Grid::processHeight(double height)
{
    // bins for H3 auto resolution:
    // - H3 level roughly equivalent to hexer hexagon size at same edge value
    //     - (since our coords are in degrees, the appropriate values will vary based on
    //       location. Some way of scaling this by latitude would be more accurate)
    // - does not automatically make very large (>1km^2) or very small (<6m^2) hexagons
    // We ignore resolutions 1 through 7, so add 8 to the entry we find..
    static const double resHeights[] { 2.0, 2.62e-4, 6.28e-5, 2.09e-5, 
                                        8.73e-6, 3.32e-6, 1.4e-6 };

    for (size_t i = 0; i < 6; ++i) {
        if (height < resHeights[i])
            m_res = i + 8;
    }
    if (m_res == -1)
        throw hexer_error("unable to calculate H3 grid size!");
    //std::cout << "H3 resolution: " << m_res << std::endl;
}

HexId H3Grid::findHexagon(Point p)
{
    H3Index index(0);
    LatLng ll{p.m_y, p.m_x};
    if (PDALH3latLngToCell(&ll, m_res, &index) != E_SUCCESS) {
            std::ostringstream oss;
            oss << "Can't convert LatLng (" << ll.lat <<
                ", " << ll.lng <<") to H3Index.";
            throw hexer_error(oss.str());
    }
    if (m_origin == 0) {
        m_origin = index;
        m_minI = h32ij(index).i;
    }
    HexId ij = h32ij(index);

    // minimum i value, used in inGrid() for finding root/child paths in parentOrChild();
    // set as i - 1 to account for m_hexPaths containing hexagons across edge 3 (-i direction)
    m_minI = std::min(m_minI, ij.i - 1);
    return ij;
}

Point H3Grid::findPoint(Segment& s)
{
    DirEdge dir_edge;
    if (PDALH3cellsToDirectedEdge(ij2h3(s.hex), ij2h3(edgeHex(s.hex, s.edge)), &dir_edge) != E_SUCCESS) {
        std::ostringstream oss;
        oss << "Can't get directed edge between hexagons (" << s.hex.i <<
            ", " << s.hex.j <<") and (" << edgeHex(s.hex, s.edge).i <<", " <<
            edgeHex(s.hex, s.edge).j << ").";
        throw hexer_error(oss.str());
    }

    CellBoundary edge_bound;

    if (PDALH3directedEdgeToBoundary(dir_edge, &edge_bound) != E_SUCCESS)
        throw hexer_error("unable to get cell boundary from directed edge!");
    double x = PDALH3radsToDegs(edge_bound.verts[1].lng);
    double y = PDALH3radsToDegs(edge_bound.verts[1].lat);

    return Point{x, y};
}

HexId H3Grid::edgeHex(HexId hex, int edge) const
{
    // Relative to H3 IJ coordinates, hexagon sides are labeled:
    //
    //               (+ I)
    //                __0_
    // (+ I, + J)  5 /    \ 1  (- J)
    //              /      \
    //              \      /
    //      (+ J)  4 \____/ 2   (- I, - J)
    //                  3
    //               (- I)
    //
    static const HexId offsets[] {{1, 0}, {0, -1}, {-1, -1}, {-1, 0}, {0, 1}, {1, 1}};
    return hex + offsets[edge];
}

} // namespace hexer
