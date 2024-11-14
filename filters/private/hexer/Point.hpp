#pragma once

namespace hexer
{

struct Point
{
    Point() : m_x(0.0), m_y(0.0)
    {}

    Point(double x, double y) : m_x(x), m_y(y)
    {}

    double m_x;
    double m_y;
        
    void operator -= (const Point& p)
    {
       m_x -= p.m_x;
       m_y -= p.m_y;
    }

    Point& operator += (const Point& p)
    {
        m_x += p.m_x;
        m_y += p.m_y;
        return *this;
    }
    
    friend Point operator - (Point p1, const Point& p2)
    {
        p1 -= p2;
        return p1;
    }

    friend Point operator + (Point p1, const Point& p2)
    {
        p1 += p2;
        return p1;
    }
};

} // namespace hexer;
