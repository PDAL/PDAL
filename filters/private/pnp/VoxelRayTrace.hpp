#pragma once

#include <functional>
#include <vector>
#include <utility>
#include <cmath>

#include "Comparison.hpp"

// Amanatides and Woo algorithm to find cells traversed by a segment (2D).
class VoxelRayTrace
{
public:
    using CellList = std::vector<std::pair<int, int>>;
    using CellCb = std::function<bool (int, int)>;

    // Think parametric equations:
    //  x = xstart + (xend - xstart) * t
    //  y = ystart + (yend - ystart) * t
    // OR
    //  x = xstart + xvec * t
    //  y = ystart + tvec * t
    VoxelRayTrace(double cellWidth, double cellHeight,
        double xstart, double ystart, double xend, double yend) :
        m_cellWidth(cellWidth), m_cellHeight(cellHeight),
        m_xCellOrigin(0), m_yCellOrigin(0),
        m_xstart(xstart), m_ystart(ystart), m_xend(xend), m_yend(yend)
    {
        initialize();
    }

    VoxelRayTrace(double cellWidth, double cellHeight,
        double xCellOrigin, double yCellOrigin,
        double xstart, double ystart, double xend, double yend) :
        m_cellWidth(cellWidth), m_cellHeight(cellHeight),
        m_xCellOrigin(xCellOrigin), m_yCellOrigin(yCellOrigin),
        m_xstart(xstart), m_ystart(ystart), m_xend(xend), m_yend(yend)
    {
        initialize();
    }

    void initialize()
    {
        // Direction from start to end.
        m_xvec = m_xend - m_xstart;
        m_yvec = m_yend - m_ystart;

        // Cell of start point.
        int gridX = xcell(m_xstart);
        int gridY = ycell(m_ystart);

        // X and Y direction (either 1 or -1) we move as we move from start
        // point to end point.
        m_stepX = (m_xvec >= 0 ? 1 : -1);
        m_stepY = (m_yvec >= 0 ? 1 : -1);

        // The first GRID x/y we'll hit as we move from start point to
        // the end point.  The grid x/y is designated by it's lower left corner.
        int gridNextX = m_stepX > 0 ? gridX + 1 : gridX;
        int gridNextY = m_stepY > 0 ? gridY + 1 : gridY;

        // The x/y coordinates of the X and Y grid lines we'll cross next
        // as we move from the start point to the end point.
        double xNextCell = m_xCellOrigin + ((gridNextX) * m_cellWidth);
        double yNextCell = m_yCellOrigin + ((gridNextY) * m_cellHeight);

        // These come straight from the equations shown above.
        // X/Y component of the vector from start to the intersection of
        // the "next" cell. Set the value to max if we're not moving in that
        // direction to force the emit loop to only consider movement in one
        // direction.
        m_tMaxX = m_xvec ?
            ((xNextCell - m_xstart) / m_xvec) :
            (std::numeric_limits<double>::max)();
        m_tMaxY = m_yvec ?
            ((yNextCell - m_ystart) / m_yvec) :
            (std::numeric_limits<double>::max)();

        // Amount t changes to move across a cell.
        m_tDeltaX = m_xvec ?
            std::abs(m_cellWidth / m_xvec) :
            (std::numeric_limits<double>::max)();
        m_tDeltaY = m_yvec ?
            std::abs(m_cellHeight / m_yvec) :
            (std::numeric_limits<double>::max)();
    }

    // Return the list of cells crossed when moving from the start point
    // to the end point.
    CellList emit()
    {
        CellList cells;

        int gridX = xcell(m_xstart);
        int gridY = ycell(m_ystart);

        int xlast = xcell(m_xend);
        int ylast = ycell(m_yend);

        cells.push_back( {gridX, gridY} );

        // Stop when we hit either the X or Y end position. This allows us to properly
        // exit the loop when m_tMaxX == t_MaxY at the end, which can happen if an
        // endpoint is exactly on a grid vertex.
        while (gridX != xlast && gridY != ylast)
        {
            if (m_tMaxX < m_tMaxY)
            {
                m_tMaxX += m_tDeltaX;
                gridX += m_stepX;
            }
            else
            {
                m_tMaxY += m_tDeltaY;
                gridY += m_stepY;
            }
            cells.push_back( { gridX, gridY } );
        }

        // Once we have hit the endpoint in one direction, add cells in the other
        // direction until we're done.
        while (gridX != xlast)
        {
            gridX += m_stepX;
            cells.push_back( { gridX, gridY } );
        }
        while (gridY != ylast)
        {
            gridY += m_stepY;
            cells.push_back( { gridX, gridY } );
        }
        return cells;
    }

    // Determine all cells traversed when moving along the ray from the
    // start point in the direction of the endpoint.  Keep emitting cells
    // via the callback until "false" is returned.
    void emit(const CellCb& cb)
    {
        int gridX = xcell(m_xstart);
        int gridY = ycell(m_ystart);

        bool more = cb(gridX, gridY);
        while (more)
        {
            if (m_tMaxX < m_tMaxY)
            {
                m_tMaxX += m_tDeltaX;
                gridX += m_stepX;
            }
            else
            {
                m_tMaxY += m_tDeltaY;
                gridY += m_stepY;
            }
            more = cb(gridX, gridY);
        }
    }

private:
    double m_cellWidth, m_cellHeight;
    double m_xCellOrigin, m_yCellOrigin;
    double m_xstart, m_ystart;
    double m_xend, m_yend;
    double m_xvec, m_yvec;
    int m_stepX, m_stepY;
    double m_tMaxX, m_tMaxY;
    double m_tDeltaX, m_tDeltaY;

    // Determine X cell index from the external X position
    int xcell(double xpos)
    {
        xpos = (xpos - m_xCellOrigin) / m_cellWidth;
        if (Comparison::closeEnough(xpos, std::ceil(xpos)))
            return (int)std::ceil(xpos);
        return (int)std::floor(xpos);
    }

    // Determine Y cell index from the external Y position
    int ycell(double ypos)
    {
        ypos = (ypos - m_yCellOrigin) / m_cellHeight;
        if (Comparison::closeEnough(ypos, std::ceil(ypos)))
            return (int)std::ceil(ypos);
        return (int)std::floor(ypos);
    }
};
